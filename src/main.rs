mod cptv_header;
mod device_config;
mod double_buffer;
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
use crate::mode_config::ModeConfig;
use crate::socket_stream::{get_socket_address, SocketStream};
use crate::telemetry::{read_telemetry, Telemetry};
use crate::utils::u8_slice_as_u16_slice;
use crc::{Crc, CRC_16_XMODEM};
use log::{debug, error, info, warn};
use notify::event::{AccessKind, AccessMode};
use notify::{Event, EventKind, RecursiveMode, Watcher};
use rppal::i2c::I2c;
use simplelog::*;

const EXPECTED_FIRMWARE_VERSION: u32 = 5;
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
        let pixels = u8_slice_as_u16_slice(&fb[640..]);
        let pos = pixels.iter().position(|&px| px == 0);
        let mut found_bad_pixels = false;
        if let Some(pos) = pos {
            let y = pos / 160;
            let x = pos - (y * 160);
            info!("Zero px found at ({}, {}) not sending", x, y);
            //found_bad_pixels = true;
        }
        // Make sure each frame number is ascending correctly.
        let telemetry = read_telemetry(&fb);
        if !telemetry.ffc_in_progress && !found_bad_pixels {
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
                    /*
                    let path = format!(
                        "{}/{}.cptv",
                        "/home/pi",
                        recording_date_time.format("%Y-%m-%d--%H-%M-%S")
                    );
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
                     */
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
    if device_config.is_none() {
        error!("Config not found at /etc/cacophony/config.toml");
        std::process::exit(1);
    }
    let mut current_config = device_config.unwrap();
    let initial_config = current_config.clone();
    let (config_tx, config_rx) = channel();
    let debug_mode = false;
    // if !debug_mode && !current_config.use_low_power_mode() {
    //     // Restart rp2040 on startup in non-low-power-mode
    //     let _ = config_tx.send(current_config.clone());
    // }
    let mut watcher = notify::recommended_watcher(move |res| match res {
        Ok(Event { kind, .. }) => {
            match kind {
                EventKind::Access(AccessKind::Close(AccessMode::Write)) => {
                    // File got written to
                    // Send event to
                    match DeviceConfig::load_from_fs() {
                        Some(config) => {
                            // Send to rp2040 to write via a channel somehow.  Maybe we have to restart the rp2040 here so that it re-handshakes and
                            // picks up the new info
                            if config != current_config {
                                current_config = config;
                                warn!("Config updated");
                                let _ = config_tx.send(current_config.clone());
                            }
                        }
                        None => {
                            error!("Config not found at /etc/cacophony/config.toml");
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

        // 1MB buffer that we won't fully use at the moment.
        let mut raw_read_buffer = [0u8; 1024 * 1024];

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
        let mut pin = gpio.get(7).expect("Failed to get pi ping interrupt pin, is 'dtoverlay=spi0-1cs,cs0_pin=8' set in your config.txt?").into_input_pulldown();
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
            let mut prev_time_on_msec = 0u32;

            loop {
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
                                info!("Restarting rp2040");
                                if !run_pin.is_set_high() {
                                    run_pin.set_high();
                                    sleep(Duration::from_millis(1000));
                                }

                                run_pin.set_low();
                                sleep(Duration::from_millis(1000));
                                run_pin.set_high();
                            }

                            let recv_timeout_ms = 10;
                            let result = rx.recv_timeout(Duration::from_millis(recv_timeout_ms));
                            match result {
                                Ok((Some((radiometry_enabled, is_recording, firmware_version, camera_serial)), None)) => {
                                    if !config.use_wifi && !sent_header {
                                        // Send the header info here:
                                        //let model = if radiometry_enabled { "lepton3.5" } else { "lepton3" };
                                        let model = "lepton3.5";
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
                                        reconnects = 0;
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
                                        ms_elapsed = 0;
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
                                            prev_time_on_msec = telemetry.msec_on;
                                        }
                                    }
                                    ms_elapsed = 0;
                                },
                                Ok((None, Some(transfer_in_progress))) => {
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
                                            {
                                                if !run_pin.is_set_high() {
                                                    run_pin.set_high();
                                                    sleep(Duration::from_millis(1000));
                                                }

                                                run_pin.set_low();
                                                sleep(Duration::from_millis(1000));
                                                run_pin.set_high();
                                            }
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
        if let Ok(mut attiny_i2c) = rppal::i2c::I2c::new() {
            if attiny_i2c.set_slave_address(0x25).is_ok() {
                attiny_i2c.write(&[0x07, 0x02]).expect("Failed writing ready state to attiny");
                attiny_i2c_interface = Some(attiny_i2c);
            }
        } else {
            error!("Error communicating to attiny");
        }

        let mut got_first_frame = false;
        let mut file_download: Option<Vec<u8>> = None;
        let mut transfer_count = 0;
        let mut header = [0u8; 8 + 26];
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
        let mut has_failed = false;
        let mut got_startup_info = false;

        'transfer: loop {
            // Check once per frame to see if the config file may have been changed
            let updated_config = config_rx.try_recv();
            match updated_config {
                Ok(config) => {
                    // NOTE: Defer this till a time we know the rp2040 isn't writing to flash memory.
                    if device_config != config {
                        info!("Config updated, should update pi");
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

            let mut radiometry_enabled = false;
            let mut firmware_version = 0;
            let mut lepton_serial_number = String::from("");

            // // We might have to abandon using pin interrupt to trigger SPI, and just constantly poll for a pattern to align to.
            // // Align our SPI reads to the start of the sequence 1, 2, 3, 4, 1, 2, 3, 4
            // // Can it also be related to our DMA transfers?
            if let Ok(_pin_level) = pin.poll_interrupt(true, Some(Duration::from_millis(2000))) {
                if _pin_level.is_some() {
                    header.fill(0);
                    spi.read(&mut header).unwrap();
                    {
                        //info!("Read {} bytes of header", header.len());
                        //info!("Header {:?}", header);
                        // For some reason, on some hardware, the first SPI transfer from the rp2040 has the first 8 bytes
                        // wrong, and in these instances they should be skipped.  If they are correct we expect to see 2
                        // repeated redundant u32s representing the length of the payload to read.
                        let offset = if &header[2..6] != &header[6..10] {
                            8
                        } else {
                            0
                        };
                        //info!("Got offset {}", offset);
                        let header_slice = &header[offset..];
                        //info!("Header {:?}", header);
                        // NOTE: We should be aligned now.
                        let transfer_type = header_slice[0];
                        let transfer_type_dup = header_slice[1];

                        let mut num_bytes = LittleEndian::read_u32(&header_slice[2..6]) as usize;
                        let num_bytes_dup = LittleEndian::read_u32(&header_slice[6..10]) as usize;
                        num_bytes = num_bytes.min(max_size);
                        let crc_from_remote = LittleEndian::read_u16(&header_slice[10..12]);
                        let crc_from_remote_dup = LittleEndian::read_u16(&header_slice[12..14]);
                        let crc_from_remote_inv = LittleEndian::read_u16(&header_slice[14..16]);
                        let crc_from_remote_inv_dup = LittleEndian::read_u16(&header_slice[16..=17]);

                        let actual_num_bytes = LittleEndian::read_u32(&header_slice[18..=21]) as usize;
                        let actual_num_bytes_dup = LittleEndian::read_u32(&header_slice[22..=25]) as usize;

                        let num_bytes_check = num_bytes == num_bytes_dup;
                        let header_crc_check = crc_from_remote == crc_from_remote_dup && crc_from_remote_inv_dup == crc_from_remote_inv && crc_from_remote_inv.reverse_bits() == crc_from_remote;
                        let transfer_type_check = transfer_type == transfer_type_dup;
                        let actual_num_bytes_check = actual_num_bytes == actual_num_bytes_dup;
                        //info!("Got num bytes {}/{}, {:?}", num_bytes, actual_num_bytes, &header[..]);

                        if num_bytes != 0 {
                            //info!("Header {:?}, tt {}, crc {}, num_bytes {}, actual_num_bytes {}", &header, transfer_type, crc_from_remote, num_bytes, actual_num_bytes);
                            if num_bytes > actual_num_bytes {
                                continue 'transfer;
                            }
                        }

                        if !num_bytes_check || !header_crc_check || !transfer_type_check || !actual_num_bytes_check {
                            if !has_failed {
                                has_failed = true;
                                warn!("Header integrity check failed {:?}", &header_slice[..]);
                            }

                            // Well, this is super tricky if this is a raw frame transfer that got garbled.
                            // Do we care about this case?  In the case where the pi wants frames, it should just restart
                            // the rp2040 on startup.
                            LittleEndian::write_u16(&mut return_payload_buf[4..6], 0);
                            LittleEndian::write_u16(&mut return_payload_buf[6..8], 0);
                            //spi.write(&return_payload_buf).unwrap();
                            if process_interrupted(&term, &mut attiny_i2c_interface) {
                                break 'transfer;
                            }
                            continue 'transfer;
                        }

                        if num_bytes == 0 {
                            //warn!("zero-sized payload");
                            LittleEndian::write_u16(&mut return_payload_buf[4..6], 0);
                            LittleEndian::write_u16(&mut return_payload_buf[6..8], 0);
                            //spi.write(&return_payload_buf).unwrap();
                            if process_interrupted(&term, &mut attiny_i2c_interface) {
                                break 'transfer;
                            }
                            continue 'transfer;
                        }

                        if transfer_type < CAMERA_CONNECT_INFO || transfer_type > CAMERA_BEGIN_AND_END_FILE_TRANSFER {
                            warn!("unknown transfer type {}", transfer_type);
                            LittleEndian::write_u16(&mut return_payload_buf[4..6], 0);
                            LittleEndian::write_u16(&mut return_payload_buf[6..8], 0);
                            //spi.write(&return_payload_buf).unwrap();
                            if process_interrupted(&term, &mut attiny_i2c_interface) {
                                break 'transfer;
                            }
                            continue 'transfer;
                        }

                        if transfer_type > 1 {
                            transfer_count += 1;
                        }
                        if transfer_type == CAMERA_BEGIN_FILE_TRANSFER {
                            start = Instant::now();
                        }
                        // let now = chrono::Local::now();
                        if num_bytes != 0 && transfer_type > 0 && transfer_type < 6 {
                            //info!("Reading {} bytes from payload", actual_num_bytes);
                            /*
                            let chunk = if offset == 8 {
                                raw_read_buffer[0..8].copy_from_slice(&header_slice[header_slice.len() - 8..]);
                                &mut raw_read_buffer[8..actual_num_bytes]
                            } else {
                                &mut raw_read_buffer[0..actual_num_bytes]
                            };
                             */
                            let chunk = &mut raw_read_buffer[0..actual_num_bytes];
                            spi.read(chunk).unwrap();
                            //info!("Read {} bytes of payload", chunk.len());
                            let chunk = &raw_read_buffer[..num_bytes];
                            //let remainder = &chunk[num_bytes..actual_num_bytes];
                            ///info!("Took {} actual bytes", chunk.len());
                            //info!("-- [{}] SPI got transfer type {}, #{} of {} bytes", now.format("%H:%M:%S:%.3f"), transfer_type, transfer_count, num_bytes);

                            if transfer_type != CAMERA_RAW_FRAME_TRANSFER {
                                // Write back the crc we calculated.
                                let crc = crc_check.checksum(&chunk);
                                LittleEndian::write_u16(&mut return_payload_buf[4..6], crc);
                                LittleEndian::write_u16(&mut return_payload_buf[6..8], crc);
                                if transfer_type == CAMERA_CONNECT_INFO {
                                    // Write all the info we need about the device:
                                    device_config.write_to_slice(&mut return_payload_buf[8..]);
                                }

                                //if let Ok(_pin_level) = pin.poll_interrupt(true, Some(Duration::from_millis(1000))) {
                                    if transfer_type == CAMERA_CONNECT_INFO {
                                        info!("Sending camera device config to rp2040");
                                        spi.write(&return_payload_buf).unwrap();
                                    } else {
                                        spi.write(&return_payload_buf).unwrap();
                                    }
                                    if crc == crc_from_remote {
                                        match transfer_type {
                                            CAMERA_CONNECT_INFO => {
                                                radiometry_enabled = LittleEndian::read_u32(&chunk[0..4]) == 1;
                                                firmware_version = LittleEndian::read_u32(&chunk[4..8]);
                                                lepton_serial_number = format!("{}", LittleEndian::read_u32(&chunk[8..12]));
                                                got_startup_info = true;
                                                info!("Got startup info: radiometry enabled: {}, firmware version: {}, lepton serial #{}", radiometry_enabled, firmware_version, lepton_serial_number);
                                                if firmware_version != EXPECTED_FIRMWARE_VERSION {
                                                    exit_cleanly(&mut attiny_i2c_interface);
                                                    error!("Unsupported firmware version, expected {}, got {}", EXPECTED_FIRMWARE_VERSION, firmware_version);
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
                                            CAMERA_RAW_FRAME_TRANSFER => {
                                                // Frame
                                                let mut frame = [0u8; FRAME_LENGTH];
                                                frame.copy_from_slice(&chunk[0..FRAME_LENGTH]);
                                                let back = FRAME_BUFFER.get_back().lock().unwrap();
                                                back.replace(Some(frame));
                                                if !got_first_frame {
                                                    got_first_frame = true;
                                                    info!("Got first frame from rp2040");
                                                }
                                                FRAME_BUFFER.swap();
                                                let is_recording = crc_from_remote == 1;
                                                let _ = tx.send((Some((radiometry_enabled, is_recording, firmware_version, lepton_serial_number)), None));
                                            }
                                            CAMERA_BEGIN_FILE_TRANSFER => {
                                                if file_download.is_some() {
                                                    warn!("Trying to begin file without ending current");
                                                }
                                                info!("Begin file transfer");
                                                // Open new file transfer

                                                part_count += 1;
                                                let mut file = Vec::with_capacity(10_000_000);
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
                                                        let date = chrono::Local::now();
                                                        error!("Requesting reset of rp2040 to force handshake, {}", date.format("%Y-%m-%d--%H:%M:%S"));
                                                        let _ = restart_tx.send(true);
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
                                            _ => if num_bytes != 0 { warn!("Unhandled transfer type, {:#x}", transfer_type) }
                                        }
                                    } else {
                                        warn!("Crc check failed, remote was notified and will re-transmit");
                                    }
                                //}
                            } else {
                                // Frame
                                let mut frame = [0u8; FRAME_LENGTH];
                                BigEndian::write_u16_into(u8_slice_as_u16_slice(&chunk[0..FRAME_LENGTH]), &mut frame);
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
                                        error!("Requesting reset of rp2040 to force handshake, {}", date.format("%Y-%m-%d--%H:%M:%S"));
                                    } else if rp2040_needs_reset {
                                        error!("Requesting reset of rp2040 due to config change, {}", date.format("%Y-%m-%d--%H:%M:%S"));
                                        rp2040_needs_reset = false;
                                        got_startup_info = false;
                                    }
                                    let _ = restart_tx.send(true);
                                }
                                FRAME_BUFFER.swap();
                                let _ = tx.send((Some((radiometry_enabled, is_recording, firmware_version, lepton_serial_number)), None));
                            }
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

fn exit_cleanly(attiny_i2c_interface: &mut Option<I2c>) {
    if let Some(attiny_i2c) = attiny_i2c_interface {
        attiny_i2c
            .write(&[0x07, 0x00])
            .expect("Failed writing ready state to attiny");
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
