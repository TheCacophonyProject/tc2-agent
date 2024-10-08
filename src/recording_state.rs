use crate::dbus_attiny_i2c::{dbus_write_attiny_command, read_tc2_agent_state};
use crate::dbus_audio::AudioStatus;
use log::error;
use rustbus::DuplexConn;
use std::process;
use std::sync::atomic::{AtomicU8, Ordering};
use std::sync::Arc;
use std::thread::sleep;
use std::time::Duration;

mod tc2_agent_state {
    pub const NOT_READY: u8 = 0b0000_0000;
    /// tc2-agent is ready to accept files or recording streams from the rp2040
    pub const READY: u8 = 0b0000_0010;
    /// taking an audio or thermal recording, not safe to reboot rp2040
    pub const RECORDING: u8 = 0b0000_0100;
    /// Requested test audio recording.  Cleared by rp2040 when test audio recording
    /// is completed
    pub const REQUESTED_TEST_AUDIO_RECORDING: u8 = 0b0000_1000;

    /// FIXME: Unclear what this is for, or if it is really used?
    #[allow(unused)]
    pub const TAKE_AUDIO: u8 = 0b0001_0000;

    /// FIXME: Not used anywhere in either tc2-agent or tc2-firmware: remove?
    #[allow(unused)]
    pub const OFFLOAD: u8 = 0b0010_0000;

    #[allow(unused)]
    pub const THERMAL_MODE: u8 = 0b0100_0000;
}

#[repr(u8)]
enum TestRecordingState {
    NotRequested = 0,
    UserRequested = 1,
    Rp2040Requested = 2,
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

    #[allow(unused)]
    pub fn is_in_thermal_mode(&self) -> bool {
        self.inner.load(Ordering::Relaxed) == RecordingMode::Thermal as u8
    }

    pub fn set_mode(&mut self, mode: RecordingMode) {
        self.inner.store(mode as u8, Ordering::Relaxed);
    }
}

#[derive(Clone)]
pub struct RecordingState {
    rp2040_recording_state_inner: Arc<AtomicU8>,
    test_recording_state_inner: Arc<AtomicU8>,
    recording_mode_state: RecordingModeState,
}

impl RecordingState {
    pub fn new() -> Self {
        Self {
            rp2040_recording_state_inner: Arc::new(AtomicU8::new(tc2_agent_state::NOT_READY)),
            test_recording_state_inner: Arc::new(AtomicU8::new(0)),
            recording_mode_state: RecordingModeState::new(),
        }
    }

    pub fn set_mode(&mut self, mode: RecordingMode) {
        self.recording_mode_state.set_mode(mode);
    }

    pub fn is_recording(&self) -> bool {
        self.rp2040_recording_state_inner.load(Ordering::Relaxed) & tc2_agent_state::RECORDING
            == tc2_agent_state::RECORDING
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
        let state = self.rp2040_recording_state_inner.load(Ordering::Relaxed);
        let new_state =
            if is_recording { tc2_agent_state::RECORDING } else { !tc2_agent_state::RECORDING };
        while !self
            .rp2040_recording_state_inner
            .compare_exchange(state, state & new_state, Ordering::Relaxed, Ordering::Relaxed)
            .is_ok()
        {
            sleep(Duration::from_micros(1));
        }
    }

    pub fn is_in_audio_mode(&self) -> bool {
        self.recording_mode_state.is_in_audio_mode()
    }

    pub fn recording_mode(&self) -> RecordingMode {
        if self.recording_mode_state.is_in_audio_mode() {
            RecordingMode::Audio
        } else {
            RecordingMode::Thermal
        }
    }

    pub fn is_taking_test_audio_recording(&self) -> bool {
        self.get_audio_status() == AudioStatus::TakingTestRecording
    }

    #[allow(unused)]
    pub fn is_waiting_to_take_test_audio_recording(&self) -> bool {
        self.get_audio_status() == AudioStatus::WaitingToTakeTestRecording
    }

    pub fn finished_taking_test_recording(&mut self) {
        self.test_recording_state_inner
            .store(TestRecordingState::NotRequested as u8, Ordering::Relaxed);
        let state = self.rp2040_recording_state_inner.load(Ordering::Relaxed);
        while !self
            .rp2040_recording_state_inner
            .compare_exchange(
                state,
                state
                    & !(tc2_agent_state::RECORDING
                        | tc2_agent_state::REQUESTED_TEST_AUDIO_RECORDING),
                Ordering::Relaxed,
                Ordering::Relaxed,
            )
            .is_ok()
        {
            sleep(Duration::from_micros(1));
        }
    }

    pub fn get_audio_status(&self) -> AudioStatus {
        let state = self.rp2040_recording_state_inner.load(Ordering::Relaxed);
        if state & (tc2_agent_state::REQUESTED_TEST_AUDIO_RECORDING | tc2_agent_state::RECORDING)
            == (tc2_agent_state::REQUESTED_TEST_AUDIO_RECORDING | tc2_agent_state::RECORDING)
        {
            AudioStatus::TakingTestRecording
        } else if state & tc2_agent_state::REQUESTED_TEST_AUDIO_RECORDING
            == tc2_agent_state::REQUESTED_TEST_AUDIO_RECORDING
        {
            AudioStatus::WaitingToTakeTestRecording
        } else if state & tc2_agent_state::RECORDING == tc2_agent_state::RECORDING {
            AudioStatus::Recording
        } else if self.user_requested_test_audio_recording() {
            AudioStatus::WaitingToTakeTestRecording
        } else {
            AudioStatus::Ready
        }
    }

    pub(crate) fn set_state(&mut self, new_state: u8) {
        self.rp2040_recording_state_inner.store(new_state, Ordering::Relaxed);
    }

    pub fn request_test_audio_recording(&mut self) {
        self.test_recording_state_inner
            .store(TestRecordingState::UserRequested as u8, Ordering::Relaxed);
    }

    pub fn user_requested_test_audio_recording(&self) -> bool {
        self.test_recording_state_inner.load(Ordering::Relaxed)
            == TestRecordingState::UserRequested as u8
    }

    pub fn merge_state_to_attiny(&mut self, state_bits_to_set: u8, conn: &mut DuplexConn) {
        let state = self.sync_state_from_attiny(conn);
        let new_state = state | state_bits_to_set;
        dbus_write_attiny_command(conn, 0x07, new_state)
            .map(|_| ())
            .or_else(|msg: &str| -> Result<(), String> {
                error!("{}", msg);
                process::exit(1);
            })
            .ok();
        self.set_state(new_state);
    }

    pub fn set_ready(&mut self, conn: &mut DuplexConn) {
        self.merge_state_to_attiny(tc2_agent_state::READY, conn);
    }

    pub fn safe_to_restart_rp2040(&mut self, conn: &mut DuplexConn) -> bool {
        self.sync_state_from_attiny(conn);
        !self.is_recording()
    }

    pub fn request_test_audio_recording_from_rp2040(&mut self, conn: &mut DuplexConn) -> bool {
        self.sync_state_from_attiny(conn);
        if self.is_recording() {
            false
        } else {
            self.merge_state_to_attiny(tc2_agent_state::REQUESTED_TEST_AUDIO_RECORDING, conn);
            self.test_recording_state_inner
                .store(TestRecordingState::Rp2040Requested as u8, Ordering::Relaxed);
            true
        }
    }
}
