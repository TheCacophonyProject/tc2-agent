use crate::dbus_attiny_i2c::{
    ATTINY_REG_TC2_AGENT_STATE, dbus_write_attiny_command, read_tc2_agent_state,
};
use crate::dbus_managementd::TestRecordingStatus;
use log::{error, info};
use rustbus::DuplexConn;
use std::process;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, AtomicU8, AtomicU16, AtomicU32, AtomicU64, Ordering};
use std::thread::sleep;
use std::time::Duration;
// mod tc2_agent_state {
//     pub const NOT_READY: u8 = 0b0000_0000;
//     /// tc2-agent is ready to accept files or recording streams from the rp2040
//     pub const READY: u8 = 0b0000_0010;
//     /// taking an audio or thermal recording, not safe to reboot rp2040
//     pub const RECORDING: u8 = 0b0000_0100;
//     /// Requested test audio recording.  Cleared by rp2040 when test audio recording
//     /// is completed
//     pub const REQUESTED_TEST_AUDIO_RECORDING: u8 = 0b0000_1000;
//
//     #[allow(unused)]
//     /// This is set by the thermal process on the rp2040 when it wants to reset to take
//     /// a scheduled audio recording
//     pub const TAKE_AUDIO: u8 = 0b0001_0000;
//
//     #[allow(unused)]
//     /// Set by the rp2040 when offloading files, and unset at the end of the offload.
//     /// Can be unset during offload by tc2-agent to interrupt the upload and start getting
//     /// thermal frames delivered
//     pub const OFFLOAD: u8 = 0b0010_0000;
//
//     #[allow(unused)]
//     /// Set by the audio process on the rp2040 when it wants to restart back into thermal mode
//     /// after taking a scheduled audio recording during a thermal recording window when in
//     /// `AudioMode::AudioAndThermal`
//     pub const THERMAL_MODE: u8 = 0b0100_0000;
//
//     pub const REQUESTED_LONG_AUDIO_RECORDING: u8 = 0b1000_0000;
//
//     // TODO: What if we could do test recordings in both modes, and we had to set the MODE bit as well
//     //  as the test recording bit? Test and Long test would have different hard-coded meanings for each.
//
//     // FIXME: Can you currently take a test audio recording when you have audio disabled?
//     //  Answer - no, and this is by design.
// }

pub mod tc2_agent_state {
    pub const NOT_READY: u8 = 0x00;
    pub const READY: u8 = 1 << 1;
    pub const RECORDING: u8 = 1 << 2;
    pub const SHORT_TEST_RECORDING: u8 = 1 << 3;
    pub const AUDIO_MODE: u8 = 1 << 4;
    pub const OFFLOAD: u8 = 1 << 5;
    pub const THERMAL_MODE: u8 = 1 << 6;
    pub const LONG_TEST_RECORDING: u8 = 1 << 7;
}

#[derive(Default, Copy, Clone)]
pub struct Tc2AgentState(u8);

impl From<u8> for Tc2AgentState {
    fn from(value: u8) -> Self {
        Tc2AgentState(value)
    }
}

impl From<Tc2AgentState> for u8 {
    fn from(value: Tc2AgentState) -> Self {
        value.0
    }
}

impl Tc2AgentState {
    fn flag_is_set(&self, flag: u8) -> bool {
        self.0 & flag != 0
    }

    pub fn set_flag(&mut self, flag: u8) {
        self.0 |= flag;
    }

    pub fn unset_flag(&mut self, flag: u8) {
        self.0 &= !flag;
    }

    pub fn is_not_ready(&self) -> bool {
        self.0 == 0
    }

    pub fn is_ready(&self) -> bool {
        self.flag_is_set(tc2_agent_state::READY)
    }

    pub fn recording_in_progress(&self) -> bool {
        self.flag_is_set(tc2_agent_state::RECORDING)
    }

    pub fn audio_recording_in_progress(&self) -> bool {
        self.flag_is_set(tc2_agent_state::RECORDING)
            && self.flag_is_set(tc2_agent_state::AUDIO_MODE)
    }

    pub fn thermal_recording_in_progress(&self) -> bool {
        self.flag_is_set(tc2_agent_state::RECORDING)
            && self.flag_is_set(tc2_agent_state::AUDIO_MODE)
    }

    pub fn test_audio_recording_requested(&self) -> bool {
        self.short_test_audio_recording_requested() || self.long_test_audio_recording_requested()
    }

    pub fn test_thermal_recording_requested(&self) -> bool {
        self.short_test_thermal_recording_requested()
            || self.long_test_thermal_recording_requested()
    }

    pub fn short_test_audio_recording_requested(&self) -> bool {
        self.requested_audio_mode() && self.flag_is_set(tc2_agent_state::SHORT_TEST_RECORDING)
    }

    pub fn long_test_audio_recording_requested(&self) -> bool {
        self.requested_audio_mode() && self.flag_is_set(tc2_agent_state::LONG_TEST_RECORDING)
    }

    pub fn short_test_thermal_recording_requested(&self) -> bool {
        self.requested_thermal_mode() && self.flag_is_set(tc2_agent_state::SHORT_TEST_RECORDING)
    }

    pub fn long_test_thermal_recording_requested(&self) -> bool {
        self.requested_thermal_mode() && self.flag_is_set(tc2_agent_state::LONG_TEST_RECORDING)
    }

    pub fn test_recording_requested(&self) -> bool {
        self.test_audio_recording_requested() || self.test_thermal_recording_requested()
    }

    pub fn is_offloading_files(&self) -> bool {
        self.flag_is_set(tc2_agent_state::OFFLOAD)
    }

    pub fn requested_thermal_mode(&self) -> bool {
        self.flag_is_set(tc2_agent_state::THERMAL_MODE)
    }
    pub fn requested_audio_mode(&self) -> bool {
        self.flag_is_set(tc2_agent_state::AUDIO_MODE)
    }
}

#[repr(u8)]
#[allow(clippy::enum_variant_names)]
enum TestRecordingState {
    NotRequested = 0,
    ShortTestRecordingRequested = 1,
    LongTestRecordingRequested = 2,
    Rp2040Requested = 3,
}

#[repr(u8)]
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum RecordingMode {
    Thermal = 0,
    Audio = 1,
}

#[derive(Clone)]
struct RecordingModeState {
    inner: Arc<AtomicU8>,
}

impl RecordingModeState {
    pub fn new() -> Self {
        Self { inner: Arc::new(AtomicU8::new(RecordingMode::Thermal as u8)) }
    }

    pub fn is_in_audio_mode(&self) -> bool {
        self.inner.load(Ordering::Relaxed) == RecordingMode::Audio as u8
    }

    pub fn is_in_thermal_mode(&self) -> bool {
        self.inner.load(Ordering::Relaxed) == RecordingMode::Thermal as u8
    }

    pub fn set_mode(&mut self, mode: RecordingMode) {
        self.inner.store(mode as u8, Ordering::Relaxed);
    }
}

struct OffloadProgress {
    remaining_offload_bytes: AtomicU32,
    total_offload_bytes: AtomicU32,
    start_time: AtomicU64,
    total_files: AtomicU16,
    remaining_files: AtomicU16,
    total_events: AtomicU16,
    remaining_events: AtomicU32,
}

#[derive(Clone)]
pub struct RecordingState {
    // This is our local copy of the tc2-agent state register on the attiny,
    // which we sync back periodically.
    rp2040_recording_state_inner: Arc<AtomicU8>,

    // This stores the user-requested test audio recording status
    audio_test_recording_state_inner: Arc<AtomicU8>,

    // This stores the user-requested test low power thermal recording status
    lp_thermal_test_recording_state_inner: Arc<AtomicU8>,

    // This reflects the current mode that the rp2040 has booted in, which is
    // set by the initial camera connection handshake
    recording_mode_state: RecordingModeState,

    // This tracks offload state when an offload is in progress in the camera loop
    offload_state: Arc<OffloadProgress>,

    // Set whenever a forced offload of files from the rp2040 is requested.
    force_offload_request_state: Arc<AtomicBool>,

    // Set whenever a managementd wants us to defer file offloads and prioritise frame serving.
    prioritise_frames_request_state: Arc<AtomicBool>,

    // Set whenever a managementd wants to cancel an in-progress file offload session.
    cancel_offload_request_state: Arc<AtomicBool>,
}

impl RecordingState {
    pub fn new() -> Self {
        Self {
            rp2040_recording_state_inner: Arc::new(AtomicU8::new(tc2_agent_state::NOT_READY)),
            audio_test_recording_state_inner: Arc::new(AtomicU8::new(0)),
            lp_thermal_test_recording_state_inner: Arc::new(AtomicU8::new(0)),
            recording_mode_state: RecordingModeState::new(),
            offload_state: Arc::new(OffloadProgress {
                remaining_offload_bytes: AtomicU32::new(0),
                total_offload_bytes: AtomicU32::new(0),
                start_time: AtomicU64::new(0),
                total_files: AtomicU16::new(0),
                remaining_files: AtomicU16::new(0),
                total_events: AtomicU16::new(0),
                remaining_events: AtomicU32::new(0),
            }),
            force_offload_request_state: Arc::new(AtomicBool::new(false)),
            prioritise_frames_request_state: Arc::new(AtomicBool::new(false)),
            cancel_offload_request_state: Arc::new(AtomicBool::new(false)),
        }
    }

    pub fn set_mode(&mut self, mode: RecordingMode) {
        self.recording_mode_state.set_mode(mode);
    }

    pub fn is_recording(&self) -> bool {
        Tc2AgentState::from(self.rp2040_recording_state_inner.load(Ordering::Relaxed))
            .recording_in_progress()
    }

    pub fn is_offloading(&self) -> bool {
        self.offload_state.start_time.load(Ordering::Relaxed) != 0
    }

    pub fn request_forced_file_offload(&self) {
        self.force_offload_request_state.store(true, Ordering::Relaxed);
    }

    pub fn request_offload_cancellation(&self) {
        self.cancel_offload_request_state.store(true, Ordering::Relaxed);
    }

    pub fn forced_file_offload_request_sent(&self) {
        self.force_offload_request_state.store(false, Ordering::Relaxed);
    }

    pub fn forced_file_offload_requested(&self) -> bool {
        self.force_offload_request_state.load(Ordering::Relaxed)
    }

    pub fn request_prioritise_frames(&self) {
        self.prioritise_frames_request_state.store(true, Ordering::Relaxed);
    }

    pub fn prioritise_frames_request_sent(&self) {
        self.prioritise_frames_request_state.store(false, Ordering::Relaxed);
    }

    pub fn prioritise_frames_requested(&self) -> bool {
        self.prioritise_frames_request_state.load(Ordering::Relaxed)
    }

    pub fn get_offload_status(&self) -> (bool, u32, u32, u16, u16, u16, u32) {
        let total_files = self.offload_state.total_files.load(Ordering::Relaxed);
        let remaining_files = self.offload_state.remaining_files.load(Ordering::Relaxed);
        let total_events = self.offload_state.total_events.load(Ordering::Relaxed);
        let remaining_events = self.offload_state.remaining_events.load(Ordering::Relaxed);
        if self.is_offloading() {
            // Return estimated time remaining in seconds
            let transfer_start_time = self.offload_state.start_time.load(Ordering::Relaxed);
            let now_ms = chrono::Local::now().timestamp_millis() as u64;
            let elapsed = now_ms - transfer_start_time;
            let remaining_bytes =
                self.offload_state.remaining_offload_bytes.load(Ordering::Relaxed);
            let total_bytes = self.offload_state.total_offload_bytes.load(Ordering::Relaxed);
            let bytes_transferred = total_bytes - remaining_bytes;
            let bytes_per_second = bytes_transferred as f32 / ((elapsed as f32) / 1000.0);
            let remaining_seconds = remaining_bytes as f32 / bytes_per_second;
            let percent_complete = (remaining_bytes as f32 / total_bytes as f32) * 100.0;
            (
                true,
                percent_complete as u32,
                remaining_seconds as u32,
                total_files,
                remaining_files,
                total_events,
                remaining_events,
            )
        } else {
            (false, 0, 0, total_files, remaining_files, total_events, remaining_events)
        }
    }

    pub fn completed_file_offload(&mut self) {
        let remaining = self.offload_state.remaining_files.load(Ordering::Relaxed);
        self.offload_state.remaining_files.store(remaining.saturating_sub(1), Ordering::Relaxed);
    }

    pub fn completed_event_offload(&mut self) {
        let remaining = self.offload_state.remaining_events.load(Ordering::Relaxed);
        self.offload_state.remaining_events.store(remaining.saturating_sub(1), Ordering::Relaxed);
    }

    pub fn set_offload_totals(
        &mut self,
        num_files_to_offload: u16,
        num_blocks_to_offload: u16,
        num_events_to_offload: u16,
    ) {
        self.offload_state.total_files.store(num_files_to_offload, Ordering::Relaxed);
        self.offload_state.remaining_files.store(num_files_to_offload, Ordering::Relaxed);
        self.offload_state
            .total_offload_bytes
            .store(u32::from(num_blocks_to_offload) * 64 * 2048, Ordering::Relaxed);
        self.offload_state.total_events.store(num_events_to_offload, Ordering::Relaxed);
    }

    pub fn update_offload_progress(&mut self, block: u16) {
        // Block is from 0..2047
        // Page is from 0..63
        // Page size is 2048 bytes
        let pages_remaining = (block as u32 + 1) * 64;
        let bytes_remaining = pages_remaining * 2048;
        if self.offload_state.start_time.load(Ordering::Relaxed) == 0 {
            self.offload_state
                .start_time
                .store(chrono::Local::now().timestamp_millis() as u64, Ordering::Relaxed);
            self.offload_state.total_offload_bytes.store(bytes_remaining, Ordering::Relaxed);
        }
        let pages_remaining = block as u32 * 64;
        let bytes_remaining = pages_remaining * 2048;
        let last_remaining = self.offload_state.remaining_offload_bytes.load(Ordering::Relaxed);
        if bytes_remaining < last_remaining && last_remaining != 0 {
            // Make sure the number can only go down once offload has started.
            self.offload_state.remaining_offload_bytes.store(bytes_remaining, Ordering::Relaxed);
        }
    }

    pub fn end_offload(&mut self) {
        self.offload_state.start_time.store(0, Ordering::Relaxed);
        self.offload_state.total_offload_bytes.store(0, Ordering::Relaxed);
        self.offload_state.remaining_offload_bytes.store(0, Ordering::Relaxed);
        self.offload_state.remaining_files.store(0, Ordering::Relaxed);
        self.offload_state.total_files.store(0, Ordering::Relaxed);
        self.offload_state.total_events.store(0, Ordering::Relaxed);
    }

    pub fn sync_state_from_attiny(&mut self, conn: &mut DuplexConn) -> u8 {
        let state = read_tc2_agent_state(conn);
        if let Ok(state) = state {
            self.set_state(state);
            state
        } else {
            error!("Failed reading ready state from attiny");
            process::exit(1);
        }
    }

    pub fn set_is_recording(&mut self, is_recording: bool) {
        let old_state =
            Tc2AgentState::from(self.rp2040_recording_state_inner.load(Ordering::Relaxed));
        let mut new_state = old_state;
        if is_recording {
            new_state.set_flag(tc2_agent_state::RECORDING);
        } else {
            new_state.unset_flag(tc2_agent_state::RECORDING);
        }
        while self
            .rp2040_recording_state_inner
            .compare_exchange(
                old_state.into(),
                new_state.into(),
                Ordering::Relaxed,
                Ordering::Relaxed,
            )
            .is_err()
        {
            sleep(Duration::from_micros(1));
        }
    }

    pub fn is_in_audio_mode(&self) -> bool {
        self.recording_mode_state.is_in_audio_mode()
    }

    pub fn is_in_thermal_mode(&self) -> bool {
        self.recording_mode_state.is_in_thermal_mode()
    }

    pub fn recording_mode(&self) -> RecordingMode {
        if self.recording_mode_state.is_in_audio_mode() {
            RecordingMode::Audio
        } else {
            RecordingMode::Thermal
        }
    }

    pub fn is_taking_test_audio_recording(&self) -> bool {
        self.get_test_audio_recording_status() == TestRecordingStatus::TakingTestRecording
    }

    pub fn is_taking_user_requested_audio_recording(&self) -> bool {
        let status = self.get_test_audio_recording_status();
        matches!(
            status,
            TestRecordingStatus::TakingTestRecording | TestRecordingStatus::TakingLongRecording
        )
    }

    pub fn is_taking_user_requested_thermal_recording(&self) -> bool {
        let status = self.get_test_thermal_recording_status();
        matches!(
            status,
            TestRecordingStatus::TakingTestRecording | TestRecordingStatus::TakingLongRecording
        )
    }

    pub fn is_taking_test_thermal_recording(&self) -> bool {
        self.get_test_thermal_recording_status() == TestRecordingStatus::TakingTestRecording
    }

    pub fn is_taking_long_thermal_recording(&self) -> bool {
        self.get_test_thermal_recording_status() == TestRecordingStatus::TakingLongRecording
    }

    pub fn is_taking_long_audio_recording(&self) -> bool {
        self.get_test_audio_recording_status() == TestRecordingStatus::TakingLongRecording
    }

    #[allow(unused)]
    pub fn is_waiting_to_take_test_audio_recording(&self) -> bool {
        self.get_test_audio_recording_status() == TestRecordingStatus::WaitingToTakeTestRecording
    }

    pub fn finished_taking_user_requested_audio_recording(&mut self) {
        self.audio_test_recording_state_inner
            .store(TestRecordingState::NotRequested as u8, Ordering::Relaxed);
        let old_state =
            Tc2AgentState::from(self.rp2040_recording_state_inner.load(Ordering::Relaxed));
        let mut new_state = old_state;
        new_state.unset_flag(tc2_agent_state::RECORDING);
        new_state.unset_flag(tc2_agent_state::AUDIO_MODE);
        new_state.unset_flag(tc2_agent_state::LONG_TEST_RECORDING);
        new_state.unset_flag(tc2_agent_state::SHORT_TEST_RECORDING);
        while self
            .rp2040_recording_state_inner
            .compare_exchange(
                old_state.into(),
                new_state.into(),
                Ordering::Relaxed,
                Ordering::Relaxed,
            )
            .is_err()
        {
            sleep(Duration::from_micros(1));
        }
    }

    pub fn finished_taking_user_requested_thermal_recording(&mut self) {
        self.lp_thermal_test_recording_state_inner
            .store(TestRecordingState::NotRequested as u8, Ordering::Relaxed);
        let old_state =
            Tc2AgentState::from(self.rp2040_recording_state_inner.load(Ordering::Relaxed));
        let mut new_state = old_state;
        new_state.unset_flag(tc2_agent_state::RECORDING);
        new_state.unset_flag(tc2_agent_state::THERMAL_MODE);
        new_state.unset_flag(tc2_agent_state::LONG_TEST_RECORDING);
        new_state.unset_flag(tc2_agent_state::SHORT_TEST_RECORDING);
        while self
            .rp2040_recording_state_inner
            .compare_exchange(
                old_state.into(),
                new_state.into(),
                Ordering::Relaxed,
                Ordering::Relaxed,
            )
            .is_err()
        {
            sleep(Duration::from_micros(1));
        }
    }

    pub fn get_test_audio_recording_status(&self) -> TestRecordingStatus {
        let state = Tc2AgentState::from(self.rp2040_recording_state_inner.load(Ordering::Relaxed));
        if state.long_test_audio_recording_requested() && state.recording_in_progress() {
            TestRecordingStatus::TakingLongRecording
        } else if state.short_test_audio_recording_requested() && state.recording_in_progress() {
            TestRecordingStatus::TakingTestRecording
        } else if state.short_test_audio_recording_requested() {
            TestRecordingStatus::WaitingToTakeTestRecording
        } else if state.long_test_audio_recording_requested() {
            TestRecordingStatus::WaitingToTakeLongRecording
        } else if state.recording_in_progress() {
            TestRecordingStatus::Recording
        } else if self.user_requested_audio_recording() {
            TestRecordingStatus::WaitingToTakeTestRecording
        } else {
            TestRecordingStatus::Ready
        }
    }

    pub fn get_test_thermal_recording_status(&self) -> TestRecordingStatus {
        let state = Tc2AgentState::from(self.rp2040_recording_state_inner.load(Ordering::Relaxed));
        if state.long_test_thermal_recording_requested() && state.recording_in_progress() {
            TestRecordingStatus::TakingLongRecording
        } else if state.short_test_thermal_recording_requested() && state.recording_in_progress() {
            TestRecordingStatus::TakingTestRecording
        } else if state.short_test_thermal_recording_requested() {
            TestRecordingStatus::WaitingToTakeTestRecording
        } else if state.long_test_thermal_recording_requested() {
            TestRecordingStatus::WaitingToTakeLongRecording
        } else if state.recording_in_progress() {
            TestRecordingStatus::Recording
        } else if self.user_requested_thermal_recording() {
            TestRecordingStatus::WaitingToTakeTestRecording
        } else {
            TestRecordingStatus::Ready
        }
    }

    pub(crate) fn set_state(&mut self, new_state: u8) {
        self.rp2040_recording_state_inner.store(new_state, Ordering::Relaxed);
    }

    pub fn request_long_audio_recording(&mut self) {
        self.audio_test_recording_state_inner
            .store(TestRecordingState::LongTestRecordingRequested as u8, Ordering::Relaxed);
    }

    pub fn request_long_thermal_recording(&mut self) {
        self.lp_thermal_test_recording_state_inner
            .store(TestRecordingState::LongTestRecordingRequested as u8, Ordering::Relaxed);
    }

    pub fn request_test_audio_recording(&mut self) {
        self.audio_test_recording_state_inner
            .store(TestRecordingState::ShortTestRecordingRequested as u8, Ordering::Relaxed);
    }
    pub fn request_test_thermal_recording(&mut self) {
        self.lp_thermal_test_recording_state_inner
            .store(TestRecordingState::ShortTestRecordingRequested as u8, Ordering::Relaxed);
    }

    pub fn user_requested_audio_recording(&self) -> bool {
        let state: u8 = self.audio_test_recording_state_inner.load(Ordering::Relaxed);
        state == TestRecordingState::ShortTestRecordingRequested as u8
            || state == TestRecordingState::LongTestRecordingRequested as u8
    }

    pub fn user_requested_thermal_recording(&self) -> bool {
        let state: u8 = self.lp_thermal_test_recording_state_inner.load(Ordering::Relaxed);
        state == TestRecordingState::ShortTestRecordingRequested as u8
            || state == TestRecordingState::LongTestRecordingRequested as u8
    }

    pub fn merge_state_to_attiny(
        &mut self,
        state_bits_to_set: Option<u8>,
        state_bits_to_unset: Option<u8>,
        conn: &mut DuplexConn,
    ) {
        let mut state = self.sync_state_from_attiny(conn);
        if let Some(state_bits_to_set) = state_bits_to_set {
            state |= state_bits_to_set
        }
        if let Some(state_bits_to_unset) = state_bits_to_unset {
            state &= !state_bits_to_unset;
        }
        self.set_state(state);
        dbus_write_attiny_command(conn, ATTINY_REG_TC2_AGENT_STATE, state)
            .map_err(|msg: &str| -> Result<(), String> {
                error!("{msg}");
                process::exit(1);
            })
            .ok();
        self.sync_state_from_attiny(conn);
    }

    pub fn set_ready(&mut self, conn: &mut DuplexConn) {
        self.merge_state_to_attiny(Some(tc2_agent_state::READY), None, conn);
    }

    pub fn safe_to_restart_rp2040(&mut self, conn: &mut DuplexConn) -> bool {
        self.sync_state_from_attiny(conn);
        !self.is_recording()
    }

    pub fn request_audio_recording_from_rp2040(&mut self, conn: &mut DuplexConn) -> bool {
        self.sync_state_from_attiny(conn);
        if self.is_recording() {
            info!("Requested audio test recording, but rp2040 is already recording");
            false
        } else {
            let state: u8 = self.audio_test_recording_state_inner.load(Ordering::Relaxed);
            if state == TestRecordingState::ShortTestRecordingRequested as u8 {
                self.merge_state_to_attiny(
                    Some(tc2_agent_state::AUDIO_MODE | tc2_agent_state::SHORT_TEST_RECORDING),
                    None,
                    conn,
                );
            } else if state == TestRecordingState::LongTestRecordingRequested as u8 {
                self.merge_state_to_attiny(
                    Some(tc2_agent_state::AUDIO_MODE | tc2_agent_state::LONG_TEST_RECORDING),
                    None,
                    conn,
                );
            }
            self.audio_test_recording_state_inner
                .store(TestRecordingState::Rp2040Requested as u8, Ordering::Relaxed);
            true
        }
    }

    pub fn cancel_offload_session(&mut self, conn: &mut DuplexConn) {
        if self.cancel_offload_request_state.load(Ordering::Relaxed) {
            self.sync_state_from_attiny(conn);
            if self.is_recording() {
                info!("Requested offload cancellation, but rp2040 is already recording");
            } else {
                info!("Cancel offload");
                self.merge_state_to_attiny(None, Some(tc2_agent_state::OFFLOAD), conn);
            }
            self.cancel_offload_request_state.store(false, Ordering::Relaxed);
        }
    }

    pub fn request_thermal_recording_from_rp2040(&mut self, conn: &mut DuplexConn) -> bool {
        self.sync_state_from_attiny(conn);
        if self.is_recording() {
            info!("Requested thermal test recording, but rp2040 is already recording");
            false
        } else {
            let state: u8 = self.lp_thermal_test_recording_state_inner.load(Ordering::Relaxed);
            if state == TestRecordingState::ShortTestRecordingRequested as u8 {
                info!("Request short test thermal recording");
                self.merge_state_to_attiny(
                    Some(tc2_agent_state::THERMAL_MODE | tc2_agent_state::SHORT_TEST_RECORDING),
                    None,
                    conn,
                );
            } else if state == TestRecordingState::LongTestRecordingRequested as u8 {
                info!("Request long test thermal recording");
                self.merge_state_to_attiny(
                    Some(tc2_agent_state::THERMAL_MODE | tc2_agent_state::LONG_TEST_RECORDING),
                    None,
                    conn,
                );
            }
            self.lp_thermal_test_recording_state_inner
                .store(TestRecordingState::Rp2040Requested as u8, Ordering::Relaxed);
            true
        }
    }
}
