mod cptv_header;
mod detection_mask;
mod device_config;
mod double_buffer;
mod event_logger;
mod mode_config;
mod socket_stream;
mod telemetry;
mod utils;

use byteorder::{BigEndian, ByteOrder, LittleEndian};
use chrono::NaiveDateTime;
use rppal::spi::BitOrder;
use rppal::{
    gpio::{Gpio, Trigger},
    spi::{Bus, Mode, Polarity, SlaveSelect, Spi},
};
use std::fs;
use std::ops::Not;
use std::path::Path;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::{channel, TryRecvError};
use std::sync::Arc;
use std::thread::sleep;
use std::time::Instant;
use std::{thread, time::Duration};
use thread_priority::ThreadBuilderExt;
use thread_priority::*;

use crate::cptv_header::{decode_cptv_header_streaming, CptvHeader};
use crate::device_config::DeviceConfig;
use crate::double_buffer::DoubleBuffer;
use crate::event_logger::{LoggerEvent, LoggerEventKind};
use crate::mode_config::ModeConfig;
use crate::socket_stream::{get_socket_address, SocketStream};
use crate::telemetry::{read_telemetry, Telemetry};
use crate::utils::u8_slice_as_u16_slice;
use crate::ExtTransferMessage::{
    BeginAndEndFileTransfer, BeginFileTransfer, CameraConnectInfo, CameraRawFrameTransfer,
    EndFileTransfer, GetMotionDetectionMask, ResumeFileTransfer, SendLoggerEvent,
};
use crc::{Algorithm, Crc, CRC_16_XMODEM};
use log::{error, info, warn};
use notify::event::{AccessKind, AccessMode};
use notify::{Event, EventKind, RecursiveMode, Watcher};
use rppal::i2c::I2c;
use rustbus::connection::Timeout;
use rustbus::{get_system_bus_path, DuplexConn};
use simplelog::*;

const EXPECTED_RP2040_FIRMWARE_VERSION: u32 = 9;
const EXPECTED_ATTINY_FIRMWARE_VERSION: u8 = 11;
const SEGMENT_LENGTH: usize = 9760;
const FRAME_LENGTH: usize = SEGMENT_LENGTH * 4;
pub type Frame = [u8; FRAME_LENGTH];
pub static FRAME_BUFFER: DoubleBuffer = DoubleBuffer::new();
fn send_frame(stream: &mut SocketStream, is_recording: bool) -> (Option<Telemetry>, bool) {
    let fb = { FRAME_BUFFER.get_front().lock().unwrap().take() };
    if let Some(mut fb) = fb {
        if is_recording {
            // Write recording flag into unused telemetry.
            fb[639] = 1;
        } else {
            fb[638] = 0;
        }

        // Make sure there are no zero/bad pixels:
        //let pixels = u8_slice_as_u16_slice(&fb[640..]);
        //let pos = pixels.iter().position(|&px| px == 0);
        // let mut found_bad_pixels = false;
        // if let Some(pos) = pos {
        //     let y = pos / 160;
        //     let x = pos - (y * 160);
        //     info!("Zero px found at ({}, {}) not sending", x, y);
        //     //found_bad_pixels = true;
        // }
        // Make sure each frame number is ascending correctly.
        let telemetry = read_telemetry(&fb);
        //info!("Got frame {:?}", telemetry.frame_num);
        if !telemetry.ffc_in_progress {
            //  && !found_bad_pixels
            if let Err(_) = stream.write_all(&fb) {
                return (None, false);
            }
            (Some(telemetry), stream.flush().is_ok())
        } else {
            (Some(telemetry), true)
        }
    } else {
        return (None, true);
    }
}

fn save_cptv_file_to_disk(cptv_bytes: Vec<u8>, output_dir: &str) {
    let output_dir = String::from(output_dir);
    thread::spawn(move || match decode_cptv_header_streaming(&cptv_bytes) {
        Ok(header) => match header {
            CptvHeader::V2(header) => {
                info!("Saving CPTV file with header {:?}", header);
                let recording_date_time =
                    NaiveDateTime::from_timestamp_millis(header.timestamp as i64 / 1000)
                        .unwrap_or(chrono::Local::now().naive_local());
                if fs::metadata(&output_dir).is_err() {
                    fs::create_dir(&output_dir)
                        .expect(&format!("Failed to create output directory {}", output_dir));
                }
                let path = format!(
                    "{}/{}.cptv",
                    output_dir,
                    recording_date_time.format("%Y-%m-%d--%H-%M-%S")
                );
                // If the file already exists, don't re-save it.
                let is_existing_file = match fs::metadata(&path) {
                    Ok(metadata) => metadata.len() as usize == cptv_bytes.len(),
                    Err(_) => false,
                };
                if !is_existing_file {
                    match fs::write(&path, &cptv_bytes) {
                        Ok(()) => {
                            info!("Saved CPTV file {}", path);
                        }
                        Err(e) => {
                            error!(
                                "Failed writing CPTV file to storage at {}, reason: {}",
                                path, e
                            );
                        }
                    }

                    // NOTE: For debug purposes, we may want to also save the CPTV file locally for inspection.
                    // let path = format!(
                    //     "{}/{}.cptv",
                    //     "/home/pi",
                    //     recording_date_time.format("%Y-%m-%d--%H-%M-%S")
                    // );
                    // match fs::write(&path, &cptv_bytes) {
                    //     Ok(()) => {
                    //         info!("Saved CPTV file {}", path);
                    //     }
                    //     Err(e) => {
                    //         error!(
                    //             "Failed writing CPTV file to storage at {}, reason: {}",
                    //             path, e
                    //         );
                    //     }
                    // }
                } else {
                    error!("File {} already exists, discarding duplicate", path);
                }
            }
            _ => error!("Unsupported CPTV file format, discarding file"),
        },
        Err(e) => {
            error!("Invalid CPTV file: ({:?}), discarding", e);
        }
    });
}

const CAMERA_CONNECT_INFO: u8 = 0x1;

const CAMERA_RAW_FRAME_TRANSFER: u8 = 0x2;

const CAMERA_BEGIN_FILE_TRANSFER: u8 = 0x3;

const CAMERA_RESUME_FILE_TRANSFER: u8 = 0x4;

const CAMERA_END_FILE_TRANSFER: u8 = 0x5;

const CAMERA_BEGIN_AND_END_FILE_TRANSFER: u8 = 0x6;

const CAMERA_GET_MOTION_DETECTION_MASK: u8 = 0x7;
const CAMERA_SEND_LOGGER_EVENT: u8 = 0x8;

#[repr(u8)]
#[derive(Copy, Clone, PartialEq, Debug)]
pub enum ExtTransferMessage {
    CameraConnectInfo = 0x1,
    CameraRawFrameTransfer = 0x2,
    BeginFileTransfer = 0x3,
    ResumeFileTransfer = 0x4,
    EndFileTransfer = 0x5,
    BeginAndEndFileTransfer = 0x6,
    GetMotionDetectionMask = 0x7,
    SendLoggerEvent = 0x8,
}

impl TryFrom<u8> for ExtTransferMessage {
    type Error = ();

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            0x1 => Ok(CameraConnectInfo),
            0x2 => Ok(CameraRawFrameTransfer),
            0x3 => Ok(BeginFileTransfer),
            0x4 => Ok(ResumeFileTransfer),
            0x5 => Ok(EndFileTransfer),
            0x6 => Ok(BeginAndEndFileTransfer),
            0x7 => Ok(GetMotionDetectionMask),
            0x8 => Ok(SendLoggerEvent),
            _ => Err(()),
        }
    }
}

fn read_tc2_agent_state(attiny_i2c: &mut I2c) -> Result<u8, &'static str> {
    read_attiny_reg(attiny_i2c, 0x07)
}

fn read_attiny_recording_flag(attiny_i2c: &mut I2c) -> bool {
    read_tc2_agent_state(attiny_i2c).map_or(false, |x| x & 0x04 == 0x04)
}
fn safe_to_restart_rp2040(attiny_interface: &mut I2c) -> bool {
    !read_attiny_recording_flag(attiny_interface)
}

fn read_attiny_firmware_version(attiny_i2c: &mut I2c) -> Result<u8, &'static str> {
    read_attiny_reg(attiny_i2c, 0x01)
}

fn set_attiny_tc2_agent_ready(attiny_i2c: &mut I2c) -> Result<(), &'static str> {
    let state = read_tc2_agent_state(attiny_i2c);
    if let Ok(state) = state {
        write_attiny_command(attiny_i2c, 0x07, state | 0x02)
    } else {
        Err("Failed reading ready state from attiny")
    }
}

fn main() {
    let log_config = ConfigBuilder::default()
        .set_time_level(LevelFilter::Off)
        .build();
    TermLogger::init(
        LevelFilter::Info,
        log_config,
        TerminalMode::Mixed,
        ColorChoice::Auto,
    )
    .unwrap();
    println!("\n=========\nStarting thermal camera 2 agent, run with --help to see options.\n");
    let config: ModeConfig = argh::from_env();
    let device_config = DeviceConfig::load_from_fs();
    if device_config.is_err() {
        error!("Load config error: {}", device_config.err().unwrap());
        std::process::exit(1);
    }

    let session_path = get_system_bus_path().unwrap_or_else(|e| {
        error!("Error getting system DBus: {}", e);
        std::process::exit(1);
    });
    let mut dbus_conn = DuplexConn::connect_to_bus(session_path, true).unwrap_or_else(|e| {
        error!("Error connecting to system DBus: {}", e);
        std::process::exit(1);
    });
    let _unique_name: String = dbus_conn.send_hello(Timeout::Infinite).unwrap_or_else(|e| {
        error!("Error getting handshake with system DBus: {}", e);
        std::process::exit(1);
    });

    let mut current_config = device_config.unwrap();
    let initial_config = current_config.clone();
    let (config_tx, config_rx) = channel();
    let mut watcher = notify::recommended_watcher(move |res| match res {
        Ok(Event { kind, .. }) => {
            match kind {
                EventKind::Access(AccessKind::Close(AccessMode::Write)) => {
                    // File got written to
                    // Send event to
                    match DeviceConfig::load_from_fs() {
                        Ok(config) => {
                            // Send to rp2040 to write via a channel somehow.  Maybe we have to restart the rp2040 here so that it re-handshakes and
                            // picks up the new info
                            if config != current_config {
                                current_config = config;
                                warn!("Config updated");
                                let _ = config_tx.send(current_config.clone());
                            }
                        }
                        Err(msg) => {
                            error!("Load config error: {}", msg);
                            std::process::exit(1);
                        }
                    }
                }
                _ => {}
            }
        }
        Err(e) => error!("file watch error for /etc/cacophony/config.toml: {:?}", e),
    })
    .unwrap();
    // Add a path to be watched. All files and directories at that path and
    // below will be monitored for changes.
    watcher
        .watch(
            Path::new("/etc/cacophony/config.toml"),
            RecursiveMode::NonRecursive,
        )
        .unwrap();

    let term = Arc::new(AtomicBool::new(false));
    signal_hook::flag::register(signal_hook::consts::SIGTERM, Arc::clone(&term)).unwrap();
    signal_hook::flag::register(signal_hook::consts::SIGINT, Arc::clone(&term)).unwrap();

    // We want real-time priority for all the work we do.
    let _ = thread::Builder::new().name("frame-acquire".to_string()).spawn_with_priority(ThreadPriority::Max, move |result| {
        assert!(result.is_ok(), "Thread must have permissions to run with realtime priority, run as root user");

        // 65K buffer that we won't fully use at the moment.
        let mut raw_read_buffer = [0u8; 65535];

        //let spi_speed = 30_000_000; // rPi4 can handle this in PIO mode
        let spi_speed = config.spi_speed * 1_000_000;
        // rPi3 can handle 12Mhz (@600Mhz), may need to back it off a little to have some slack.
        info!("Initialising SPI at {}Mhz", config.spi_speed);
        let mut spi = Spi::new(Bus::Spi0, SlaveSelect::Ss0, spi_speed, Mode::Mode3).unwrap();
        spi.set_bits_per_word(8).unwrap();
        spi.set_bit_order(BitOrder::MsbFirst).unwrap();
        spi.set_ss_polarity(Polarity::ActiveLow).unwrap();

        let address = get_socket_address(&config);

        // For some reason when running periph.io host.ini function, needed to use the I2C in the attiny-controller,
        // it 'holds/exports' some of the GPIO pins, so we manually 'release/unexport' them with the following.
        let gpio_number = "7";
        let _ = fs::write("/sys/class/gpio/unexport", gpio_number);

        let gpio = Gpio::new().unwrap();
        let mut pin = gpio.get(7).expect("Failed to get pi ping interrupt pin, is 'dtoverlay=spi0-1cs,cs0_pin=8' set in your config.txt?").into_input();
        let mut run_pin = gpio.get(23).unwrap().into_output_high();
        if !run_pin.is_set_high() {
            info!("Setting run pin high to enable rp2040");
            run_pin.set_high();
            sleep(Duration::from_millis(1000));
        }

        pin.clear_interrupt().expect("Unable to clear pi ping interrupt pin");
        pin.set_interrupt(Trigger::RisingEdge).expect("Unable to set pi ping interrupt");
        let (tx, rx) = channel();
        let (restart_tx, restart_rx) = channel();

        let _ = thread::Builder::new().name("frame-socket".to_string()).spawn_with_priority(ThreadPriority::Max, move |result| {
            // Spawn a thread which can output the frames, converted to rgb grayscale
            // This is printed out from within the spawned thread.
            assert!(result.is_ok(), "Thread must have permissions to run with realtime priority, run as root user");

            info!("Connecting to frame socket {}", address);
            let mut reconnects = 0;
            let mut prev_frame_num = None;

            loop {
                info!("Entering frame-socket loop");
                if let Ok(_) = restart_rx.try_recv() {
                    info!("Restarting rp2040");
                    if !run_pin.is_set_high() {
                        run_pin.set_high();
                        sleep(Duration::from_millis(1000));
                    }

                    run_pin.set_low();
                    sleep(Duration::from_millis(1000));
                    run_pin.set_high();
                }
                match SocketStream::from_address(&address, *&config.use_wifi) {
                    Ok(mut stream) => {
                        info!("Connected to {}, waiting for frames from rp2040", if config.use_wifi { &"tc2-frames server"} else { &"thermal-recorder unix socket"});
                        let mut sent_header = false;
                        let mut ms_elapsed = 0;
                        'send_loop: loop {
                            // Check if we need to reset rp2040 because of a config change
                            if let Ok(_) = restart_rx.try_recv() {
                                loop {
                                    info!("Restarting rp2040");
                                    if !run_pin.is_set_high() {
                                        run_pin.set_high();
                                        sleep(Duration::from_millis(1000));
                                    }

                                    run_pin.set_low();
                                    sleep(Duration::from_millis(1000));
                                    run_pin.set_high();
                                    break;
                                }
                            }

                            let recv_timeout_ms = 10;
                            let result = rx.recv_timeout(Duration::from_millis(recv_timeout_ms));
                            match result {
                                Ok((Some((radiometry_enabled, is_recording, firmware_version, camera_serial)), None)) => {
                                    if !config.use_wifi && !sent_header {
                                        // Send the header info here:
                                        let model = if radiometry_enabled { "lepton3.5" } else { "lepton3" };
                                        let header = format!("ResX: 160\nResX: 160\nResY: 120\nFrameSize: 39040\nModel: {}\nBrand: flir\nFPS: 9\nFirmware: DOC-AI-v0.{}\nCameraSerial: {}\n\n", model, firmware_version, camera_serial);
                                        if let Err(_) = stream.write_all(header.as_bytes()) {
                                            warn!("Failed sending header info");
                                        }

                                        // Clear existing
                                        if let Err(_) = stream.write_all(b"clear") {
                                            warn!("Failed clearing buffer");
                                        }
                                        let _ = stream.flush();
                                        sent_header = true;
                                    }

                                    if reconnects > 0 {
                                        info!("Got frame connection");
                                        prev_frame_num = None;
                                        reconnects = 0;
                                    }
                                    let s = Instant::now();
                                    let (telemetry, sent) = send_frame(&mut stream, is_recording);
                                    let e = s.elapsed().as_secs_f32();
                                    if e > 0.1 {
                                        info!("socket send took {}s", e);
                                    }
                                    if !sent {
                                        warn!("Send to {} failed", if config.use_wifi {&"tc2-frames server"} else {&"thermal-recorder unix socket"});
                                        let _ = stream.shutdown().is_ok();
                                        break 'send_loop;
                                    } else {
                                        if let Some(telemetry) = telemetry {
                                            if let Some(prev_frame_num) = prev_frame_num {
                                                if !telemetry.ffc_in_progress {
                                                    if telemetry.frame_num != prev_frame_num + 1 {
                                                        // NOTE: Frames can be missed when the raspberry pi blocks the thread with the unix socket in `thermal-recorder`.
                                                        // println!("====");
                                                        // println!("Missed {} frames after {}s on", telemetry.frame_num - (prev_frame_num + 1), telemetry.msec_on as f32 / 1000.0);
                                                    }
                                                }
                                            }
                                            prev_frame_num = Some(telemetry.frame_num);
                                            if telemetry.frame_num % 2700 == 0 {
                                                info!("Got frame #{}", telemetry.frame_num);
                                            }
                                        }
                                    }
                                    ms_elapsed = 0;
                                },
                                Ok((None, Some(_transfer_in_progress))) => {
                                    ms_elapsed = 0;
                                },
                                _ => {
                                    const NUM_ATTEMPTS_BEFORE_RESET: usize = 10;
                                    ms_elapsed += recv_timeout_ms;
                                    if ms_elapsed > 5000 {
                                        ms_elapsed = 0;
                                        reconnects += 1;
                                        if reconnects == NUM_ATTEMPTS_BEFORE_RESET {
                                            let date = chrono::Local::now();
                                            warn!("Resetting rp2040 at {}", date.format("%Y-%m-%d--%H:%M:%S"));
                                            reconnects = 0;
                                            prev_frame_num = None;

                                            if !run_pin.is_set_high() {
                                                run_pin.set_high();
                                                sleep(Duration::from_millis(1000));
                                            }

                                            run_pin.set_low();
                                            sleep(Duration::from_millis(1000));
                                            run_pin.set_high();
                                        } else {
                                            info!("-- #{reconnects} waiting for frames from rp2040 (resetting rp2040 after {} more attempts)", NUM_ATTEMPTS_BEFORE_RESET - reconnects);
                                        }
                                    }
                                }
                            }
                        }
                    }
                    Err(e) => warn!("{} not found: {}", if config.use_wifi {&"tc2-frames server"} else {&"thermal-recorder unix socket"}, e.to_string())
                }
                // Wait 1 second between attempts to connect to the frame socket
                sleep(Duration::from_millis(1000));
            }
        });
        info!("Waiting to acquire frames from rp2040");
        // Poke register 0x07 of the attiny letting the rp2040 know that we're ready:
        let mut attiny_i2c_interface = None;
        if let Ok(mut attiny_i2c) = I2c::new() {
            if attiny_i2c.set_slave_address(0x25).is_ok() {
                match set_attiny_tc2_agent_ready(&mut attiny_i2c) {
                    Ok(_) => attiny_i2c_interface = Some(attiny_i2c),
                    Err(msg) => {
                        error!("{}", msg);
                        std::process::exit(1);
                    }
                }
                let version = read_attiny_firmware_version(attiny_i2c_interface.as_mut().unwrap());
                match version {
                    Ok(version) => match version {
                        EXPECTED_ATTINY_FIRMWARE_VERSION => {},
                        _ => {
                            error!("Mismatched attiny firmware version, expected {}, got {}", EXPECTED_ATTINY_FIRMWARE_VERSION, version);
                            exit_cleanly(&mut attiny_i2c_interface);
                            std::process::exit(1);
                        }
                    },
                    Err(msg) => {
                        error!("{}", msg);
                        exit_cleanly(&mut attiny_i2c_interface);
                        std::process::exit(1);
                    }
                }

                if let Some(ref mut attiny_i2c_interface) = attiny_i2c_interface {
                    if !initial_config.use_low_power_mode() || safe_to_restart_rp2040(attiny_i2c_interface) {
                        // NOTE: Always reset rp2040 on startup if it's safe to do so.
                        let _ = restart_tx.send(true);
                    }
                }

            }
        } else {
            error!("Error communicating to attiny");
            std::process::exit(1);
        }

        let mut got_first_frame = false;
        let mut file_download: Option<Vec<u8>> = None;
        let header_length = 18;
        let mut return_payload_buf = [0u8; 32 + 104];

        // This sequence is used to synchronise the return payload start on the rp2040, since
        // it seems to have a fair bit of slop/offsetting.
        return_payload_buf[0..4].copy_from_slice(&[1, 2, 3, 4]);

        let mut part_count = 0;
        let mut start = Instant::now();
        let crc_check = Crc::<u16>::new(&CRC_16_XMODEM);
        let max_size: usize = raw_read_buffer.len();
        let mut device_config: DeviceConfig = initial_config;
        let mut rp2040_needs_reset = false;
        let mut sent_reset_request = false;
        let mut has_failed = false;
        let mut got_startup_info = false;

        let mut radiometry_enabled = false;
        let mut firmware_version = 0;
        let mut lepton_serial_number = String::from("");

        'transfer: loop {
            // Check once per frame to see if the config file may have been changed
            let updated_config = config_rx.try_recv();
            match updated_config {
                Ok(config) => {
                    // NOTE: Defer this till a time we know the rp2040 isn't writing to flash memory.
                    if device_config != config {
                        info!("Config updated, should update rp2040");
                        device_config = config;
                    }
                    rp2040_needs_reset = true;
                }
                Err(kind) => {
                    match kind {
                        TryRecvError::Empty => {},
                        TryRecvError::Disconnected => {
                            warn!("Disconnected from config file watcher channel");
                        },
                    }
                }
            }

            // // We might have to abandon using pin interrupt to trigger SPI, and just constantly poll for a pattern to align to.
            // // Align our SPI reads to the start of the sequence 1, 2, 3, 4, 1, 2, 3, 4
            // // Can it also be related to our DMA transfers?
            let poll_result = pin.poll_interrupt(true, Some(Duration::from_millis(2000)));
            if let Ok(_pin_level) = poll_result {
                if _pin_level.is_some() {
                    {
                        drop(pin);
                        let output_pin = gpio.get(7).expect("Failed to get pi ping interrupt pin, is 'dtoverlay=spi0-1cs,cs0_pin=8' set in your config.txt?").into_output_low();
                        drop(output_pin);
                        pin = gpio.get(7).expect("Failed to get pi ping interrupt pin, is 'dtoverlay=spi0-1cs,cs0_pin=8' set in your config.txt?").into_input();
                        pin.clear_interrupt().expect("Unable to clear pi ping interrupt pin");
                        pin.set_interrupt(Trigger::RisingEdge).expect("Unable to set pi ping interrupt");
                    }

                    spi.read(&mut raw_read_buffer[..2066]).unwrap();
                    {
                        let header_slice = &raw_read_buffer[..header_length];
                        let transfer_type = header_slice[0];
                        let transfer_type_dup = header_slice[1];

                        let mut num_bytes = LittleEndian::read_u32(&header_slice[2..6]) as usize;
                        let num_bytes_dup = LittleEndian::read_u32(&header_slice[6..10]) as usize;
                        num_bytes = num_bytes.min(max_size);
                        let crc_from_remote = LittleEndian::read_u16(&header_slice[10..12]);
                        let crc_from_remote_dup = LittleEndian::read_u16(&header_slice[12..14]);
                        let crc_from_remote_inv = LittleEndian::read_u16(&header_slice[14..16]);
                        let crc_from_remote_inv_dup = LittleEndian::read_u16(&header_slice[16..=17]);

                        let num_bytes_check = num_bytes == num_bytes_dup;
                        let header_crc_check = crc_from_remote == crc_from_remote_dup && crc_from_remote_inv_dup == crc_from_remote_inv && crc_from_remote_inv.not() == crc_from_remote;
                        let transfer_type_check = transfer_type == transfer_type_dup;
                        if !num_bytes_check || !header_crc_check || !transfer_type_check {
                            if !has_failed {
                                has_failed = true;
                                warn!("Header integrity check failed {:?}", &header_slice[..]);
                                // We still need to make sure we read out all the bytes?
                            }

                            // Well, this is super tricky if this is a raw frame transfer that got garbled.
                            // Do we care about this case?  In the case where the pi wants frames, it should just restart
                            // the rp2040 on startup.
                            LittleEndian::write_u16(&mut return_payload_buf[4..6], 0);
                            LittleEndian::write_u16(&mut return_payload_buf[6..8], 0);
                            spi.write(&return_payload_buf).unwrap();
                            if process_interrupted(&term, &mut attiny_i2c_interface) {
                                break 'transfer;
                            }
                            continue 'transfer;
                        }

                        if num_bytes == 0 {
                            //warn!("zero-sized payload");
                            LittleEndian::write_u16(&mut return_payload_buf[4..6], 0);
                            LittleEndian::write_u16(&mut return_payload_buf[6..8], 0);
                            spi.write(&return_payload_buf).unwrap();
                            if process_interrupted(&term, &mut attiny_i2c_interface) {
                                break 'transfer;
                            }
                            continue 'transfer;
                        }

                        if transfer_type < CAMERA_CONNECT_INFO || transfer_type > CAMERA_SEND_LOGGER_EVENT {
                            warn!("unknown transfer type {}", transfer_type);
                            LittleEndian::write_u16(&mut return_payload_buf[4..6], 0);
                            LittleEndian::write_u16(&mut return_payload_buf[6..8], 0);
                            spi.write(&return_payload_buf).unwrap();
                            if process_interrupted(&term, &mut attiny_i2c_interface) {
                                break 'transfer;
                            }
                            continue 'transfer;
                        }

                        if transfer_type != CAMERA_RAW_FRAME_TRANSFER {
                            if transfer_type == CAMERA_BEGIN_FILE_TRANSFER {
                                start = Instant::now();
                            }
                            let chunk = &raw_read_buffer[header_length..header_length + num_bytes];
                            // Write back the crc we calculated.
                            let crc = crc_check.checksum(chunk);
                            LittleEndian::write_u16(&mut return_payload_buf[4..6], crc);
                            LittleEndian::write_u16(&mut return_payload_buf[6..8], crc);

                            if transfer_type == CAMERA_CONNECT_INFO {
                                info!("Got camera connect info {:?}", chunk);
                                // Write all the info we need about the device:
                                device_config.write_to_slice(&mut return_payload_buf[8..]);
                                info!("Sending camera device config to rp2040");
                            }
                            else if transfer_type == CAMERA_GET_MOTION_DETECTION_MASK {
                                let piece_number = &chunk[0];
                                //info!("Got request for mask piece number {}", piece_number);
                                let piece = device_config.mask_piece(*piece_number as usize);

                                let mut piece_with_length = [0u8; 101];
                                piece_with_length[0] = piece.len() as u8;
                                piece_with_length[1..1 + piece.len()].copy_from_slice(piece);
                                let crc = crc_check.checksum(&piece_with_length[0..1 + piece.len()]);
                                LittleEndian::write_u16(&mut return_payload_buf[8..10], crc);
                                //info!("Sending piece({}) {:?}", piece.len(), piece_with_length);
                                return_payload_buf[10..10 + piece.len() + 1].copy_from_slice(&piece_with_length);
                            }
                            // Always write the return buffer
                            spi.write(&return_payload_buf).unwrap();
                            if crc == crc_from_remote {
                                match transfer_type {
                                    CAMERA_CONNECT_INFO => {
                                        radiometry_enabled = LittleEndian::read_u32(&chunk[0..4]) == 2;
                                        firmware_version = LittleEndian::read_u32(&chunk[4..8]);
                                        lepton_serial_number = format!("{}", LittleEndian::read_u32(&chunk[8..12]));
                                        got_startup_info = true;
                                        info!("Got startup info: radiometry enabled: {}, firmware version: {}, lepton serial #{}", radiometry_enabled, firmware_version, lepton_serial_number);
                                        if firmware_version != EXPECTED_RP2040_FIRMWARE_VERSION {
                                            exit_cleanly(&mut attiny_i2c_interface);
                                            error!("Unsupported firmware version, expected {}, got {}", EXPECTED_RP2040_FIRMWARE_VERSION, firmware_version);
                                            panic!("Exit");
                                        }
                                        if device_config.use_low_power_mode() && !radiometry_enabled {
                                            exit_cleanly(&mut attiny_i2c_interface);
                                            error!("Low power mode is currently only supported on lepton sensors with radiometry, exiting.");
                                            panic!("Exit");
                                        }
                                        // Terminate any existing file download.
                                        let in_progress_file_transfer = file_download.take();
                                        if let Some(file) = in_progress_file_transfer {
                                            warn!("Aborting in progress file transfer with {} bytes", file.len());
                                        }
                                        let _ = tx.send((None, Some(false)));
                                    }
                                    CAMERA_SEND_LOGGER_EVENT => {
                                        let event_kind = LittleEndian::read_u16(&chunk[0..2]);
                                        let event_timestamp = LittleEndian::read_u64(&chunk[2..2 + 8]);
                                        let event_payload = LittleEndian::read_u64(&chunk[10..18]);
                                        if let Ok(mut event_kind) = LoggerEventKind::try_from(event_kind) {
                                            if let Some(time) = NaiveDateTime::from_timestamp_micros(event_timestamp as i64) {
                                                if let LoggerEventKind::SetAlarm(alarm_time) = &mut event_kind {
                                                    if NaiveDateTime::from_timestamp_micros(event_payload as i64).is_some() {
                                                        *alarm_time = event_payload;
                                                    } else {
                                                        warn!("Wakeup alarm from event was invalid {}", event_payload);
                                                    }
                                                }
                                                let payload_json = if let LoggerEventKind::SavedNewConfig = event_kind {
                                                    // If we get saved new config, the rp2040 would have just been
                                                    // restarted after the config change, so we can log the current
                                                    // config in relation to that event.
                                                    let json_inner = format!(r#""continuous-recorder": {}, "use-low-power-mode": {}, "start-recording": "{:?}", "stop-recording": "{:?}", "location": "({}, {}, {})""#,
                                                                                         device_config.is_continuous_recorder(),
                                                                                         device_config.use_low_power_mode(),
                                                                                         device_config.recording_window().0,
                                                                                         device_config.recording_window().1,
                                                                                         device_config.lat_lng().0,
                                                                                         device_config.lat_lng().1,
                                                                                         device_config.location_altitude().unwrap_or(0.0));
                                                    let json = String::from(format!("{{{}}}", json_inner));
                                                    Some(json)
                                                } else {
                                                    None
                                                };
                                                info!("Got logger event {:?} at {}", event_kind, time);
                                                let event = LoggerEvent::new(event_kind, event_timestamp);
                                                event.log(&mut dbus_conn, payload_json);
                                            } else {
                                                warn!("Event had invalid timestamp {}", event_timestamp);
                                            }
                                        } else {
                                            warn!("Unknown logger event kind {}", event_kind);
                                        }
                                    }
                                    CAMERA_BEGIN_FILE_TRANSFER => {
                                        if file_download.is_some() {
                                            warn!("Trying to begin file without ending current");
                                        }
                                        info!("Begin file transfer");
                                        // Open new file transfer

                                        part_count += 1;
                                        // If we have to grow this Vec once it gets big it can be slow and interrupt the transfer.
                                        // TODO: Should really be able to recover from that though!
                                        let mut file = Vec::with_capacity(150_000_000);
                                        file.extend_from_slice(&chunk);
                                        file_download = Some(file);
                                        let _ = tx.send((None, Some(true)));
                                    }
                                    CAMERA_RESUME_FILE_TRANSFER => {
                                        if let Some(file) = &mut file_download {
                                            // Continue current file transfer
                                            //println!("Continue file transfer");
                                            if part_count % 100 == 0 {
                                                let megabytes_per_second = (file.len() + chunk.len()) as f32 / Instant::now().duration_since(start).as_secs_f32() / (1024.0 * 1024.0);
                                                info!("Transferring part #{} {:?} for {} bytes, {}MB/s", part_count, Instant::now().duration_since(start), file.len() + chunk.len(), megabytes_per_second);
                                            }
                                            part_count += 1;
                                            file.extend_from_slice(&chunk);
                                            let _ = tx.send((None, Some(true)));
                                        } else {
                                            warn!("Trying to continue file with no open file");
                                            if !got_startup_info {
                                                if let Some(ref mut attiny_i2c_interface) = &mut attiny_i2c_interface {
                                                    if safe_to_restart_rp2040(attiny_i2c_interface) {
                                                        let date = chrono::Local::now();
                                                        error!("1) Requesting reset of rp2040 to force handshake, {}", date.format("%Y-%m-%d--%H:%M:%S"));
                                                        let _ = restart_tx.send(true);
                                                    }
                                                }
                                            }
                                        }
                                    }
                                    CAMERA_END_FILE_TRANSFER => {
                                        // End current file transfer
                                        if !file_download.is_some() {
                                            warn!("Trying to end file with no open file");
                                        }
                                        if let Some(mut file) = file_download.take() {
                                            // Continue current file transfer
                                            let megabytes_per_second = (file.len() + chunk.len()) as f32 / Instant::now().duration_since(start).as_secs_f32() / (1024.0 * 1024.0);
                                            info!("End file transfer, took {:?} for {} bytes, {}MB/s", Instant::now().duration_since(start), file.len() + chunk.len(), megabytes_per_second);
                                            part_count = 0;
                                            file.extend_from_slice(&chunk);
                                            save_cptv_file_to_disk(file, device_config.output_dir());
                                            let _ = tx.send((None, Some(false)));
                                        } else {
                                            warn!("Trying to end file with no open file");
                                        }
                                    }
                                    CAMERA_BEGIN_AND_END_FILE_TRANSFER => {
                                        if file_download.is_some() {
                                            info!("Trying to begin (and end) file without ending current");
                                        }
                                        // Open and end new file transfer
                                        part_count = 0;
                                        let mut file = Vec::new();
                                        file.extend_from_slice(&chunk);
                                        save_cptv_file_to_disk(file, device_config.output_dir());
                                        let _ = tx.send((None, Some(false)));
                                    }
                                    CAMERA_GET_MOTION_DETECTION_MASK => {
                                        // Already handled
                                    }
                                    _ => if num_bytes != 0 { warn!("Unhandled transfer type, {:#x}", transfer_type) }
                                }
                            } else {
                                warn!("Crc check failed, remote was notified and will re-transmit");
                            }
                        } else {
                            spi.read(&mut raw_read_buffer[2066..num_bytes + header_length]).unwrap();
                            // Frame
                            let mut frame = [0u8; FRAME_LENGTH];
                            BigEndian::write_u16_into(u8_slice_as_u16_slice(&raw_read_buffer[header_length..header_length + FRAME_LENGTH]), &mut frame);

                            // let mask = device_config.mask();
                            // for y in 0..120 {
                            //     for x in 0..160 {
                            //         let idx = y * 160 + x;
                            //         if mask.is_masked_at_index(idx) {
                            //             u8_slice_as_u16_slice_mut(&mut frame)[idx] = 10;
                            //         }
                            //     }
                            // }

                            let back = FRAME_BUFFER.get_back().lock().unwrap();
                            back.replace(Some(frame));
                            if !got_first_frame {
                                got_first_frame = true;
                                info!("Got first frame from rp2040, got startup info {}", got_startup_info);
                            }
                            let is_recording = crc_from_remote == 1;
                            if !is_recording && (rp2040_needs_reset || !got_startup_info) {
                                let date = chrono::Local::now();
                                if !got_startup_info {
                                    error!("2) Requesting reset of rp2040 to force handshake, {}", date.format("%Y-%m-%d--%H:%M:%S"));
                                } else if rp2040_needs_reset {
                                    error!("3) Requesting reset of rp2040 due to config change, {}", date.format("%Y-%m-%d--%H:%M:%S"));
                                    rp2040_needs_reset = false;
                                    got_startup_info = false;
                                }
                                if !sent_reset_request {
                                    sent_reset_request = true;
                                    let _ = restart_tx.send(true);
                                }
                            }
                            FRAME_BUFFER.swap();
                            let _ = tx.send((Some((radiometry_enabled, is_recording, firmware_version, lepton_serial_number.clone())), None));
                        }
                    }
                }
            }
            if process_interrupted(&term, &mut attiny_i2c_interface) {
                break 'transfer;
            }
        }
        info!("Exiting gracefully");
        Ok::<(), Error>(())
    }).unwrap().join();
}

pub const CRC_AUG_CCITT: Algorithm<u16> = Algorithm {
    width: 16,
    poly: 0x1021,
    init: 0x1D0F,
    refin: false,
    refout: false,
    xorout: 0x0000,
    check: 0x0000,
    residue: 0x0000,
};

fn write_attiny_command(attiny_i2c: &mut I2c, command: u8, value: u8) -> Result<(), &'static str> {
    let mut payload = [command, value, 0x00, 0x00];
    let crc = Crc::<u16>::new(&CRC_AUG_CCITT).checksum(&payload[0..=1]);
    BigEndian::write_u16(&mut payload[2..=3], crc);
    if attiny_i2c.write(&payload).is_err() {
        Err("Failed setting state on attiny")
    } else {
        // Check the state on the attiny is what we expected
        if let Ok(val) = read_attiny_reg(attiny_i2c, command) {
            if val != value {
                Err("Failed setting state on attiny")
            } else {
                Ok(())
            }
        } else {
            Err("Failed setting state on attiny")
        }
    }
}

fn read_attiny_reg(attiny_i2c: &mut I2c, command: u8) -> Result<u8, &'static str> {
    let mut payload = [command, 0x00, 0x00];

    let crc = Crc::<u16>::new(&CRC_AUG_CCITT).checksum(&payload[0..1]);
    let mut response = [0u8; 3];
    BigEndian::write_u16(&mut payload[1..=2], crc);
    if attiny_i2c.write_read(&payload, &mut response).is_err() {
        Err("Failed setting state on attiny")
    } else {
        let crc = Crc::<u16>::new(&CRC_AUG_CCITT).checksum(&response[0..1]);
        let received_crc = BigEndian::read_u16(&response[1..=2]);
        if received_crc != crc {
            Err("CRC Mismatch")
        } else {
            Ok(response[0])
        }
    }
}

fn exit_cleanly(attiny_i2c_interface: &mut Option<I2c>) {
    if let Some(attiny_i2c) = attiny_i2c_interface {
        let _ = write_attiny_command(attiny_i2c, 0x07, 0x00);
    }
}

fn process_interrupted(term: &Arc<AtomicBool>, attiny_i2c_interface: &mut Option<I2c>) -> bool {
    if term.load(Ordering::Relaxed) {
        // We got terminated - before we exit, clean up the tc2-agent ready state register
        exit_cleanly(attiny_i2c_interface);
        true
    } else {
        false
    }
}
