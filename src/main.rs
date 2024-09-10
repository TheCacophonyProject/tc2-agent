mod camera_transfer_state;
mod cptv_frame_dispatch;
mod cptv_header;
mod dbus_attiny;
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

use byteorder::{BigEndian, ByteOrder, LittleEndian};
use chrono::{DateTime, NaiveDateTime, Utc};
use rppal::spi::BitOrder;
use rppal::{
    gpio::{Gpio, Trigger},
    spi::{Bus, Mode, Polarity, SlaveSelect, Spi},
};
use rustbus::connection::dispatch_conn::DispatchConn;

use std::fs;
use std::io;
use std::ops::Not;
use std::path::Path;
use std::process;
use std::process::Command;
use std::sync::atomic::{AtomicBool, AtomicU8, Ordering};
use std::sync::mpsc::{channel, TryRecvError};
use std::sync::{mpsc, Arc};
use std::thread::sleep;
use std::time::Instant;

use std::{thread, time::Duration};
use thread_priority::ThreadBuilderExt;
use thread_priority::*;

use crate::camera_transfer_state::ExtTransferMessage::{
    BeginAndEndFileTransfer, BeginFileTransfer, CameraConnectInfo, CameraRawFrameTransfer,
    EndFileTransfer, GetMotionDetectionMask, ResumeFileTransfer, SendLoggerEvent,
};
use crate::cptv_header::{decode_cptv_header_streaming, CptvHeader};
use crate::device_config::{watch_local_config_file_changes, DeviceConfig};
use crate::double_buffer::DoubleBuffer;
use crate::event_logger::{LoggerEvent, LoggerEventKind, WakeReason};
use crate::mode_config::ModeConfig;
use crate::service::AgentService;
use crate::socket_stream::{get_socket_address, SocketStream};
use crate::telemetry::{read_telemetry, Telemetry};
use crate::utils::u8_slice_as_u16_slice;
use chrono::format::Pad;
use crc::{Algorithm, Crc, CRC_16_XMODEM};
use log::{error, info, warn};
use notify::event::{AccessKind, AccessMode};
use notify::{Event, EventKind, RecursiveMode, Watcher};
use rustbus::connection::dispatch_conn::HandleEnvironment;
use rustbus::connection::dispatch_conn::HandleResult;
use rustbus::connection::dispatch_conn::Matches;
use rustbus::connection::Timeout;
use rustbus::message_builder::MarshalledMessage;
use rustbus::{get_system_bus_path, DuplexConn, MessageBuilder, MessageType};
use simplelog::*;
use std::io::Write;

pub mod tc2_agent_state {
    pub const NOT_READY: u8 = 0x00;
    pub const READY: u8 = 1 << 1;
    //taking an audio or thermal recording
    pub const RECORDING: u8 = 1 << 2;
    pub const TEST_AUDIO_RECORDING: u8 = 1 << 3;
    //an audio recording has been requested from thermal mode
    pub const TAKE_AUDIO: u8 = 1 << 4;
    pub const OFFLOAD: u8 = 1 << 5;
    //requested to boot into thermal mode from audio mode
    pub const THERMAL_MODE: u8 = 1 << 6;
    //is no audio mode flag as it isn't needed at this point
    //    pub const THERMAL_MODE: u8 = 1 << 7;
}

const AUDIO_SHEBANG: u16 = 1;

const EXPECTED_RP2040_FIRMWARE_HASH: &str = include_str!("../_releases/tc2-firmware.sha256");
const EXPECTED_RP2040_FIRMWARE_VERSION: u32 = 14;
const EXPECTED_ATTINY_FIRMWARE_VERSION: u8 = 1;

const SEGMENT_LENGTH: usize = 9760;
const FRAME_LENGTH: usize = SEGMENT_LENGTH * 4;

// TC2-Agent dbus audio service
type MyHandleEnv<'a, 'b> = HandleEnvironment<&'b mut AgentService, ()>;

fn default_handler(
    _c: &mut AgentService,
    _matches: Matches,
    _msg: &MarshalledMessage,
    _env: &mut MyHandleEnv,
) -> HandleResult<()> {
    Ok(None)
}

fn audio_handler(
    ctx: &mut AgentService,
    _matches: Matches,
    msg: &MarshalledMessage,
    _env: &mut MyHandleEnv,
) -> HandleResult<()> {
    if msg.dynheader.member.as_ref().unwrap() == "testaudio" {
        let message = if !ctx.making_test_audio_recording() {
            ctx.recording_state.set_should_take_test_audio_recording(true);
            "Asked for a test recording"
        } else {
            "Already making a test recording"
        };
        let mut resp = msg.dynheader.make_response();
        resp.body.push_param(message)?;
        Ok(Some(resp))
    } else if msg.dynheader.member.as_ref().unwrap() == "audiostatus" {
        let mut response = msg.dynheader.make_response();
        let status = ctx.recording_state.get_audio_status();
        response.body.push_param(if ctx.recording_mode_state.is_in_audio_mode() {
            1
        } else {
            0
        })?;
        response.body.push_param(status as u8)?;
        Ok(Some(response))
    } else {
        Ok(None)
    }
}
pub enum AudioStatus {
    Ready = 1,
    WaitingToRecord = 2,
    TakingTestRecoding = 3,
    Recording = 4,
}

use crate::camera_transfer_state::{
    enter_camera_transfer_loop, CAMERA_BEGIN_FILE_TRANSFER, CAMERA_CONNECT_INFO,
    CAMERA_GET_MOTION_DETECTION_MASK, CAMERA_RAW_FRAME_TRANSFER, CAMERA_SEND_LOGGER_EVENT,
};
use crate::cptv_frame_dispatch::FRAME_BUFFER;
use crate::dbus_attiny::{
    dbus_write_attiny_command, exit_cleanly, process_interrupted, read_attiny_firmware_version,
    read_attiny_recording_flag, read_tc2_agent_state, safe_to_restart_rp2040,
    set_attiny_tc2_agent_ready, set_attiny_tc2_agent_test_audio_rec,
};
use crate::frame_socket_server::spawn_frame_socket_server_thread;
use crate::program_rp2040::{check_if_rp2040_needs_programming, program_rp2040};
use crate::save_audio::save_audio_file_to_disk;
use crate::save_cptv::save_cptv_file_to_disk;

#[repr(u8)]
#[derive(Copy, Clone)]
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
        Self { inner: Arc::new(AtomicU8::new(0)) }
    }

    pub fn is_in_audio_mode(&self) -> bool {
        self.inner.load(Ordering::Relaxed) == 1
    }

    pub fn is_in_thermal_mode(&self) -> bool {
        self.inner.load(Ordering::Relaxed) == 0
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
        // TODO: Why initialised to 2?
        Self {
            rp2040_recording_state_inner: Arc::new(AtomicU8::new(2)),
            test_recording_state_inner: Arc::new(AtomicBool::new(false)),
        }
    }

    pub fn is_recording(&self) -> bool {
        self.rp2040_recording_state_inner.load(Ordering::Relaxed)
            & tc2_agent_state::TEST_AUDIO_RECORDING
            == 0
    }

    pub fn is_making_test_audio_recording(&self) -> bool {
        self.rp2040_recording_state_inner.load(Ordering::Relaxed)
            & tc2_agent_state::TEST_AUDIO_RECORDING
            == 0
    }

    pub fn get_audio_status(&self) -> AudioStatus {
        let state = self.rp2040_recording_state_inner.load(Ordering::Relaxed);
        if state & (tc2_agent_state::TEST_AUDIO_RECORDING | tc2_agent_state::RECORDING)
            == (tc2_agent_state::TEST_AUDIO_RECORDING | tc2_agent_state::RECORDING)
        {
            AudioStatus::TakingTestRecoding
        } else if state & tc2_agent_state::TEST_AUDIO_RECORDING
            == tc2_agent_state::TEST_AUDIO_RECORDING
        {
            AudioStatus::WaitingToRecord
        } else if state & tc2_agent_state::RECORDING == tc2_agent_state::RECORDING {
            AudioStatus::Recording
        } else if self.should_take_test_audio_recording() {
            AudioStatus::WaitingToRecord
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
        let spi_speed = startup_mode_config.spi_speed;
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

    let mut recording_mode_state = RecordingModeState::new();
    let agent_service_state = recording_mode_state.clone();
    let mut recording_state = RecordingState::new();
    let agent_recording_state = recording_state.clone();

    // set up dbus service for handling messages between managementd and tc2-agent about when
    // to make test audio recordings.
    let dbus_thread = thread::Builder::new().name("dbus-service".to_string()).spawn_with_priority(
        ThreadPriority::Max,
        move |_| {
            let mut dbus_conn =
                DuplexConn::connect_to_bus(session_path, false).unwrap_or_else(|e| {
                    error!("Error connecting to system DBus: {}", e);
                    process::exit(1);
                });
            let _name = dbus_conn.send_hello(Timeout::Infinite).unwrap_or_else(|e| {
                error!("Error getting handshake with system DBus: {}", e);
                process::exit(1);
            });
            dbus_conn
                .send
                .send_message(&mut rustbus::standard_messages::request_name(
                    "org.cacophony.TC2Agent".into(),
                    rustbus::standard_messages::DBUS_NAME_FLAG_REPLACE_EXISTING,
                ))
                .unwrap()
                .write_all()
                .unwrap();

            let mut dispatch_conn = DispatchConn::new(
                dbus_conn,
                AgentService {
                    recording_mode_state: agent_service_state,
                    recording_state: agent_recording_state,
                },
                Box::new(default_handler),
            );
            dispatch_conn.add_handler("/org/cacophony/TC2Agent", Box::new(audio_handler));
            dispatch_conn.run().unwrap();
        },
    );

    let current_config = device_config.unwrap();
    let initial_config = current_config.clone();
    let (device_config_change_channel_tx, device_config_change_channel_rx) =
        watch_local_config_file_changes(current_config);

    // NOTE: This handles gracefully exiting the process if ctrl-C etc is pressed
    // while running in an interactive terminal.
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

            // TODO: Handle errors on inner threads
            spawn_frame_socket_server_thread(
                restart_rp2040_channel_rx,
                camera_handshake_channel_rx,
                serve_frames_via_wifi,
                run_pin,
                restart_rp2040_ack.clone(),
            );

            info!("Waiting to for messages from rp2040");
            // Poke register 0x07 of the attiny letting the rp2040 know that we're ready:
            set_attiny_tc2_agent_ready(&mut dbus_conn)
                .or_else(|msg| {
                    error!("{}", msg);
                    process::exit(1);
                })
                .ok();

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
        eprintln!("Thread panicked: {:?}", e);
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
