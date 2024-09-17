extern crate core;

mod camera_transfer_state;
mod cptv_frame_dispatch;
mod cptv_header;
mod dbus_attiny;
mod dbus_audio;
mod detection_mask;
mod device_config;
mod double_buffer;
mod event_logger;
mod frame_socket_server;
mod mode_config;
mod program_rp2040;
mod save_audio;
mod save_cptv;
mod service;
mod socket_stream;
mod telemetry;
mod utils;

use rppal::gpio::Gpio;

use std::fs;
use std::process;
use std::sync::atomic::{AtomicBool, AtomicU8, Ordering};
use std::sync::mpsc::channel;
use std::sync::Arc;
use std::thread::sleep;

use std::{thread, time::Duration};
use thread_priority::ThreadBuilderExt;
use thread_priority::*;

use log::{error, info};
use rustbus::connection::Timeout;
use rustbus::{get_system_bus_path, DuplexConn};
use simplelog::*;

use crate::camera_transfer_state::enter_camera_transfer_loop;
use crate::dbus_attiny::exit_cleanly;
use crate::dbus_attiny::read_attiny_firmware_version;
use crate::dbus_attiny::safe_to_restart_rp2040;
use crate::dbus_attiny::set_attiny_tc2_agent_ready;
use crate::dbus_audio::setup_dbus_test_audio_recording_service;
use crate::dbus_audio::AudioStatus;
use crate::device_config::watch_local_config_file_changes;
use crate::device_config::DeviceConfig;
use crate::frame_socket_server::spawn_frame_socket_server_thread;
use crate::mode_config::ModeConfig;
use crate::program_rp2040::check_if_rp2040_needs_programming;

pub mod tc2_agent_state {
    pub const NOT_READY: u8 = 0b0000_0000;

    /// tc2-agent is ready to accept files or recording streams from the rp2040
    pub const READY: u8 = 0b0000_0010;
    //taking an audio or thermal recording
    pub const RECORDING: u8 = 0b0000_0100;
    pub const TAKING_TEST_AUDIO_RECORDING: u8 = 0b0000_1000;
    //an audio recording has been requested from thermal mode
    pub const SHOULD_TAKE_TEST_AUDIO_RECORDING: u8 = 0b0001_0000;

    // FIXME: Unused?
    pub const OFFLOAD: u8 = 0b0010_0000;
    //requested to boot into thermal mode from audio mode
    pub const PENDING_BOOT_INTO_THERMAL_MODE: u8 = 0b0100_0000;
}

const AUDIO_SHEBANG: u16 = 1;

const EXPECTED_RP2040_FIRMWARE_HASH: &str = include_str!("../_releases/tc2-firmware.sha256");
const EXPECTED_RP2040_FIRMWARE_VERSION: u32 = 14;
const EXPECTED_ATTINY_FIRMWARE_VERSION: u8 = 1;

const SEGMENT_LENGTH: usize = 9760;
const FRAME_LENGTH: usize = SEGMENT_LENGTH * 4;

#[repr(u8)]
#[derive(Copy, Clone, Debug)]
enum RecordingMode {
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
struct RecordingState {
    rp2040_recording_state_inner: Arc<AtomicU8>,
    test_recording_state_inner: Arc<AtomicBool>,
}

impl RecordingState {
    pub fn new() -> Self {
        Self {
            rp2040_recording_state_inner: Arc::new(AtomicU8::new(tc2_agent_state::NOT_READY)),
            test_recording_state_inner: Arc::new(AtomicBool::new(false)),
        }
    }

    pub fn is_recording(&self) -> bool {
        self.rp2040_recording_state_inner.load(Ordering::Relaxed) & tc2_agent_state::RECORDING == 0
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

    pub fn is_taking_test_audio_recording(&self) -> bool {
        self.get_audio_status() == AudioStatus::TakingTestRecording
    }

    #[allow(unused)]
    pub fn is_waiting_to_take_test_audio_recording(&self) -> bool {
        self.get_audio_status() == AudioStatus::WaitingToTakeTestRecording
    }

    pub fn finished_taking_test_recording(&mut self) {
        self.test_recording_state_inner.store(false, Ordering::Relaxed);
        let state = self.rp2040_recording_state_inner.load(Ordering::Relaxed);
        while !self
            .rp2040_recording_state_inner
            .compare_exchange(
                state,
                state
                    & !(tc2_agent_state::RECORDING | tc2_agent_state::TAKING_TEST_AUDIO_RECORDING),
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
        if state & (tc2_agent_state::TAKING_TEST_AUDIO_RECORDING | tc2_agent_state::RECORDING)
            == (tc2_agent_state::TAKING_TEST_AUDIO_RECORDING | tc2_agent_state::RECORDING)
        {
            AudioStatus::TakingTestRecording
        } else if state & tc2_agent_state::TAKING_TEST_AUDIO_RECORDING
            == tc2_agent_state::TAKING_TEST_AUDIO_RECORDING
        {
            AudioStatus::WaitingToTakeTestRecording
        } else if state & tc2_agent_state::RECORDING == tc2_agent_state::RECORDING {
            AudioStatus::Recording
        } else if self.should_take_test_audio_recording() {
            AudioStatus::WaitingToTakeTestRecording
        } else {
            AudioStatus::Ready
        }
    }

    pub fn set_state(&mut self, state: u8) {
        self.rp2040_recording_state_inner.store(state, Ordering::Relaxed);
    }

    pub fn set_should_take_test_audio_recording(&mut self, state: bool) {
        self.test_recording_state_inner.store(state, Ordering::Relaxed);
    }

    pub fn should_take_test_audio_recording(&self) -> bool {
        self.test_recording_state_inner.load(Ordering::Relaxed)
    }
}

const VERSION: &str = env!("CARGO_PKG_VERSION");

fn main() {
    let log_config = ConfigBuilder::default().set_time_level(LevelFilter::Off).build();
    TermLogger::init(LevelFilter::Info, log_config, TerminalMode::Mixed, ColorChoice::Auto)
        .unwrap();

    println!(
        "\n=========\nStarting thermal camera 2 agent {}, run with --help to see options.\n",
        VERSION
    );
    check_if_rp2040_needs_programming();

    let (serve_frames_via_wifi, spi_speed_mhz) = {
        let startup_mode_config: ModeConfig = argh::from_env();
        let serve_frames_via_wifi = startup_mode_config.use_wifi;
        let spi_speed = startup_mode_config.spi_speed_mhz;
        (serve_frames_via_wifi, spi_speed)
    };

    let device_config = DeviceConfig::load_from_fs();
    if device_config.is_err() {
        error!("Load config error: {}", device_config.err().unwrap());
        process::exit(1);
    }

    let session_path = get_system_bus_path().unwrap_or_else(|e| {
        error!("Error getting system DBus: {}", e);
        process::exit(1);
    });
    let mut dbus_conn = DuplexConn::connect_to_bus(session_path, true).unwrap_or_else(|e| {
        error!("Error connecting to system DBus: {}", e);
        process::exit(1);
    });
    let _unique_name: String = dbus_conn.send_hello(Timeout::Infinite).unwrap_or_else(|e| {
        error!("Error getting handshake with system DBus: {}", e);
        process::exit(1);
    });

    let recording_mode_state = RecordingModeState::new();
    let mut recording_state = RecordingState::new();
    let _dbus_audio_thread =
        setup_dbus_test_audio_recording_service(&recording_mode_state, &recording_state);

    let current_config = device_config.unwrap();
    let initial_config = current_config.clone();
    let device_config_change_channel_rx = watch_local_config_file_changes(current_config);

    // NOTE: This handles gracefully exiting the process if ctrl-c etc is pressed
    //  while running in an interactive terminal.
    let sig_term_state = Arc::new(AtomicBool::new(false));
    signal_hook::flag::register(signal_hook::consts::SIGTERM, sig_term_state.clone()).unwrap();
    signal_hook::flag::register(signal_hook::consts::SIGINT, sig_term_state.clone()).unwrap();
    exit_if_attiny_version_is_not_as_expected(&mut dbus_conn);

    // We want real-time priority for all the work we do.
    let handle = thread::Builder::new()
        .name("frame-acquire".to_string())
        .spawn_with_priority(ThreadPriority::Max, move |result| {
            assert!(
                result.is_ok(),
                "Thread must have permissions to run with realtime priority, run as root user"
            );

            // For some reason when running periph.io host.ini function, needed to use the I2C in the attiny-controller,
            // it 'holds/exports' some of the GPIO pins, so we manually 'release/unexport' them with the following.
            let gpio_number = "7";
            let _ = fs::write("/sys/class/gpio/unexport", gpio_number);

            let gpio = Gpio::new().unwrap();
            let mut run_pin = gpio.get(23).unwrap().into_output_high();
            if !run_pin.is_set_high() {
                info!("Setting run pin high to enable rp2040");
                run_pin.set_high();
                sleep(Duration::from_millis(1000));
            }

            let (camera_handshake_channel_tx, camera_handshake_channel_rx) = channel();
            let (restart_rp2040_channel_tx, restart_rp2040_channel_rx) = channel();

            // Used to indicate that a reset request was received and processed by the frame-socket thread.
            let restart_rp2040_ack = Arc::new(AtomicBool::new(false));

            spawn_frame_socket_server_thread(
                restart_rp2040_channel_rx,
                camera_handshake_channel_rx,
                serve_frames_via_wifi,
                run_pin,
                restart_rp2040_ack.clone(),
            );
            set_attiny_tc2_agent_ready(&mut dbus_conn)
                .or_else(|msg: &str| -> Result<(), String> {
                    error!("{}", msg);
                    process::exit(1);
                })
                .ok();
            recording_state.set_state(tc2_agent_state::READY);
            if !initial_config.use_low_power_mode() || safe_to_restart_rp2040(&mut dbus_conn) {
                // NOTE: Always reset rp2040 on startup if it's safe to do so.
                let _ = restart_rp2040_channel_tx.send(true);
            }
            enter_camera_transfer_loop(
                initial_config,
                dbus_conn,
                spi_speed_mhz,
                device_config_change_channel_rx,
                restart_rp2040_channel_tx,
                sig_term_state,
                camera_handshake_channel_tx,
                restart_rp2040_ack,
                recording_mode_state,
                recording_state,
            );
            info!("Exiting gracefully");
            Ok::<(), Error>(())
        })
        .unwrap();

    if let Err(e) = handle.join() {
        error!("Thread panicked: {:?}", e);
        process::exit(1);
    }
}

fn exit_if_attiny_version_is_not_as_expected(dbus_conn: &mut DuplexConn) {
    let version = read_attiny_firmware_version(dbus_conn);
    match version {
        Ok(version) => match version {
            EXPECTED_ATTINY_FIRMWARE_VERSION => {}
            _ => {
                error!(
                    "Mismatched attiny firmware version, expected {}, got {}",
                    EXPECTED_ATTINY_FIRMWARE_VERSION, version
                );
                exit_cleanly(dbus_conn);
                process::exit(1);
            }
        },
        Err(msg) => {
            error!("{}", msg);
            exit_cleanly(dbus_conn);
            process::exit(1);
        }
    }
}
