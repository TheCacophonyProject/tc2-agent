#![warn(clippy::all)]
extern crate core;

mod camera_transfer_state;
mod cptv_frame_dispatch;
mod cptv_header;
mod dbus_attiny_i2c;
mod dbus_managementd;
mod detection_mask;
mod device_config;
mod double_buffer;
mod event_logger;
mod frame_socket_server;
mod mode_config;
mod program_rp2040;
mod recording_state;
mod save_audio;
mod save_cptv;
mod socket_stream;
mod telemetry;
mod utils;

use rppal::gpio::Gpio;

use std::fs;
use std::process;
use std::sync::Arc;
use std::sync::atomic::AtomicBool;
use std::sync::mpsc::channel;
use std::thread::sleep;

use std::{thread, time::Duration};
use thread_priority::ThreadBuilderExt;
use thread_priority::*;

use log::{error, info, warn};
use rustbus::connection::Timeout;
use rustbus::{DuplexConn, get_system_bus_path};
use simplelog::*;
use sysinfo::Disks;

use crate::camera_transfer_state::enter_camera_transfer_loop;
use crate::dbus_attiny_i2c::exit_if_attiny_version_is_not_as_expected;
use crate::dbus_managementd::setup_dbus_managementd_recording_service;
use crate::device_config::DeviceConfig;
use crate::device_config::watch_local_config_file_changes;
use crate::frame_socket_server::spawn_frame_socket_server_thread;
use crate::mode_config::ModeConfig;
use crate::program_rp2040::check_if_rp2040_needs_programming;
use crate::recording_state::RecordingState;

const AUDIO_SHEBANG: u16 = 1;

const EXPECTED_RP2040_FIRMWARE_HASH: &str = include_str!("../_releases/tc2-firmware.sha256");
const EXPECTED_RP2040_FIRMWARE_VERSION: u32 = 26;
const EXPECTED_ATTINY_FIRMWARE_VERSION: u8 = 1;

const SEGMENT_LENGTH: usize = 9760;
const FRAME_LENGTH: usize = SEGMENT_LENGTH * 4;

const VERSION: &str = env!("CARGO_PKG_VERSION");

fn mb_disk_space_remaining() -> (usize, usize) {
    let disks = Disks::new_with_refreshed_list();
    let mut available_space_bytes = 0;
    let mut total_space_bytes = 0;
    const ROOT_MOUNT_POINT: &str = "/";
    let main_partition =
        disks.list().iter().find(|disk| disk.mount_point().to_str().unwrap() == ROOT_MOUNT_POINT);
    if let Some(main_partition) = main_partition {
        available_space_bytes += main_partition.available_space();
        total_space_bytes += main_partition.total_space();
    } else {
        println!("Partition mounted at '{ROOT_MOUNT_POINT}' not found");
        process::exit(1);
    }
    let available_space = available_space_bytes / (1024 * 1024);
    let total_space = total_space_bytes / (1024 * 1024);
    (available_space as usize, total_space as usize)
}

fn check_for_sufficient_free_disk_space() {
    // Check for enough available disk space to make/offload recordings
    let mut warned_once = false;
    loop {
        let (mb_remaining, mb_total) = mb_disk_space_remaining();
        if mb_remaining < 1000 {
            if !warned_once {
                warn!(
                    "Insufficient disk space remaining: ({mb_remaining}MB/{mb_total}MB), \
                    sleeping 1 minute before trying again.",
                );
                warned_once = true;
            }
            sleep(Duration::from_secs(60));
        } else {
            break;
        }
    }
}

fn main() {
    let log_config = ConfigBuilder::default().set_time_level(LevelFilter::Off).build();
    TermLogger::init(LevelFilter::Info, log_config, TerminalMode::Mixed, ColorChoice::Auto)
        .unwrap();

    println!(
        "\n=========\nStarting thermal camera 2 agent {VERSION}, run with --help to see options.\n"
    );

    check_for_sufficient_free_disk_space();
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
        error!("Error getting system DBus: {e}");
        process::exit(1);
    });
    let mut dbus_conn = DuplexConn::connect_to_bus(session_path, true).unwrap_or_else(|e| {
        error!("Error connecting to system DBus: {e}");
        process::exit(1);
    });
    let _unique_name: String = dbus_conn.send_hello(Timeout::Infinite).unwrap_or_else(|e| {
        error!("Error getting handshake with system DBus: {e}");
        process::exit(1);
    });

    let mut recording_state = RecordingState::new();
    let _dbus_audio_thread = setup_dbus_managementd_recording_service(&recording_state);

    let current_config = device_config.unwrap();
    let initial_config = current_config.clone();
    let (device_config_change_channel_tx, device_config_change_channel_rx) = channel();
    let _file_watcher =
        watch_local_config_file_changes(current_config, &device_config_change_channel_tx);

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

            // For some reason when running periph.io host.ini function,
            // needed to use the I2C in the attiny-controller, it 'holds/exports' some of the
            // GPIO pins, so we manually 'release/unexport' them with the following.
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

            // Used to indicate that a reset request was received and processed by the
            // frame-socket thread.
            let restart_rp2040_ack = Arc::new(AtomicBool::new(false));

            // The frame socket server takes frames from the main camera transfer loop,
            // and serves them to various consumers of frames.
            // For mostly historical reasons, it's also the thread that handles actually restarting
            // the rp2040 – but it would perhaps be cleaner to handle this in a separate thread?
            spawn_frame_socket_server_thread(
                restart_rp2040_channel_rx,
                camera_handshake_channel_rx,
                serve_frames_via_wifi,
                run_pin,
                restart_rp2040_ack.clone(),
                &recording_state,
            );
            recording_state.set_ready(&mut dbus_conn);
            if initial_config.use_high_power_mode() || !recording_state.is_recording() {
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
                recording_state,
            );
            info!("Exiting gracefully");
            Ok::<(), Error>(())
        })
        .unwrap();

    if let Err(e) = handle.join() {
        error!("Thread panicked: {e:?}");
        process::exit(1);
    }
}
