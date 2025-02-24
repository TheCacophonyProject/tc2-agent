use crate::cptv_frame_dispatch::FRAME_BUFFER;
use crate::dbus_attiny_i2c::{exit_cleanly, process_interrupted};

use crate::device_config::{check_for_device_config_changes, DeviceConfig};
use crate::event_logger::{LoggerEvent, LoggerEventKind, WakeReason};
use crate::frame_socket_server::FrameSocketServerMessage;
use crate::program_rp2040::program_rp2040;
use crate::recording_state::RecordingMode;
use crate::save_audio::save_audio_file_to_disk;
use crate::save_cptv::save_cptv_file_to_disk;
use crate::utils::u8_slice_as_u16_slice;
use crate::{RecordingState, AUDIO_SHEBANG, EXPECTED_RP2040_FIRMWARE_VERSION, FRAME_LENGTH};
use byteorder::{BigEndian, ByteOrder, LittleEndian};
use chrono::{DateTime, Utc};
use chrono_tz::Tz::Pacific__Auckland;
use crc::{Crc, CRC_16_XMODEM};
use log::{error, info, warn};
use rppal::gpio::Trigger;
use rppal::spi::{BitOrder, Bus, Mode, Polarity, SlaveSelect, Spi};
use rustbus::connection::Timeout;
use rustbus::{get_system_bus_path, DuplexConn};
use std::ops::Not;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::{Receiver, Sender};
use std::sync::Arc;
use std::thread::{sleep, JoinHandle};
use std::time::{Duration, Instant};
use std::{process, thread};

pub const CAMERA_CONNECT_INFO: u8 = 0x1;
pub const CAMERA_RAW_FRAME_TRANSFER: u8 = 0x2;
pub const CAMERA_BEGIN_FILE_TRANSFER: u8 = 0x3;
pub const CAMERA_RESUME_FILE_TRANSFER: u8 = 0x4;
pub const CAMERA_END_FILE_TRANSFER: u8 = 0x5;
pub const CAMERA_BEGIN_AND_END_FILE_TRANSFER: u8 = 0x6;
pub const CAMERA_GET_MOTION_DETECTION_MASK: u8 = 0x7;
pub const CAMERA_SEND_LOGGER_EVENT: u8 = 0x8;

pub struct CameraHandshakeInfo {
    pub radiometry_enabled: bool,
    pub is_recording: bool,
    pub firmware_version: u32,
    pub camera_serial: String,
}

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
        use ExtTransferMessage::*;
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

fn  wait_for_threads_to_finish(save_threads: Vec<JoinHandle<()>>){
    for handle in save_threads{
        info!("Waiting on thread to finish");
        handle.join();
    }
}


fn reset_pin(mut pin: rppal::gpio::InputPin,gpio: &rppal::gpio::Gpio)-> Result<rppal::gpio::InputPin,()>{
    drop(pin);
    let output_pin = match gpio.get(7) {
        Ok(pin) => pin.into_output_low(),
        Err(e) => {
            error!(
                "Failed to get pi ping interrupt pin ({e}), \
is 'dtoverlay=spi0-1cs,cs0_pin=8' set in your config.txt?"
            );
            return Err(());
        }
    };
    drop(output_pin);
    pin = match gpio.get(7) {
        Ok(pin) => pin.into_input(),
        Err(e) => {
            error!(
                "Failed to get pi ping interrupt pin ({e}), \
is 'dtoverlay=spi0-1cs,cs0_pin=8' set in your config.txt?"
            );
            return Err(());
        }
    };

    if let Err(e) = pin.clear_interrupt(){
        error!("Unable to clear pi ping interrupt pin: {e}");
        return Err(());
    }

     // this seems to happen when save_audio fails...
     let attempts = 100;
     for i in 0..attempts{
         let res = pin.set_interrupt(Trigger::RisingEdge, None);
         match res{
             Ok(())=> break,
             Err(e)=>{
                 if i == attempts-1 {
                     error!("Unable to set pi ping interrupt: {e}");
                     return Err(());
                 }
                 info!("Pin interrupts failed,trying again {}",e);
                 sleep(Duration::from_millis(100));
             }
         }
     }
    return Ok(pin)
}

pub fn enter_camera_transfer_loop(
    initial_config: DeviceConfig,
    mut dbus_conn: DuplexConn,
    spi_speed_mhz: u32,
    device_config_change_channel_rx: Receiver<DeviceConfig>,
    restart_rp2040_channel_tx: Sender<bool>,
    sig_term: Arc<AtomicBool>,
    camera_handshake_channel_tx: Sender<FrameSocketServerMessage>,
    restart_rp2040_ack: Arc<AtomicBool>,
    mut recording_state: RecordingState,
) {
    let mut save_threads: Vec<JoinHandle<()>> = Vec::with_capacity(3);
    let spi_speed = spi_speed_mhz * 1_000_000;
    // rPi3 can handle 12Mhz (@600Mhz), may need to back it off a little to have some slack.
    info!("Initialising SPI at {}Mhz", spi_speed_mhz);
    let mut spi = match Spi::new(Bus::Spi0, SlaveSelect::Ss0, spi_speed, Mode::Mode3) {
        Ok(spi) => spi,
        Err(e) => {
            error!("Failed to get SPI0: {e}");
            process::exit(1);
        }
    };
    if spi.set_bits_per_word(8).is_err() {
        error!("Failed to set SPI bits per word");
        process::exit(1);
    };
    if spi.set_bit_order(BitOrder::MsbFirst).is_err() {
        error!("Failed to set SPI output bit order");
        process::exit(1);
    };
    if spi.set_ss_polarity(Polarity::ActiveLow).is_err() {
        error!("Failed to set SPI SS polarity");
        process::exit(1);
    }
    let gpio = match rppal::gpio::Gpio::new() {
        Err(e) => {
            error!("Failed to get GPIO: {e}");
            process::exit(1);
        }
        Ok(gpio) => gpio,
    };
    let mut pin = match gpio.get(7) {
        Ok(pin) => pin.into_input(),
        Err(e) => {
            error!(
                "Failed to get pi ping interrupt pin ({e}), \
        is 'dtoverlay=spi0-1cs,cs0_pin=8' set in your config.txt?"
            );
            process::exit(1);
        }
    };
    pin.clear_interrupt()
        .map_err(|e| {
            error!("Unable to clear pi ping interrupt pin: {e}");
            process::exit(1);
        })
        .unwrap();

    // NOTE: `rppal` now has a `debounce` option here which may be worth exploring.
    pin.set_interrupt(Trigger::RisingEdge, None)
        .map_err(|e| {
            error!("Unable to set pi ping interrupt: {e}");
            process::exit(1);
        })
        .unwrap();
    // 65K buffer that we won't fully use at the moment.
    let mut raw_read_buffer = [0u8; 65535];
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
    let mut header_integrity_check_has_failed = false;
    let mut got_startup_info = false;
    let mut radiometry_enabled = false;
    let mut firmware_version = 0;
    let mut lepton_serial_number = String::from("");
    let mut is_audio_device = device_config.is_audio_device();

    info!("Waiting for messages from rp2040");
    'transfer: loop {
        save_threads.retain(|handle|!handle.is_finished());
      
        check_for_device_config_changes(
            &device_config_change_channel_rx,
            &mut device_config,
            &mut is_audio_device,
            &mut rp2040_needs_reset,
        );
        if is_audio_device {
            maybe_make_test_audio_recording(
                &mut dbus_conn,
                &restart_rp2040_channel_tx,
                &mut recording_state,
            );
        }
        if !recording_state.is_recording() && rp2040_needs_reset {
            let date = chrono::Local::now();
            warn!(
                "Requesting reset of rp2040 at {}",
                date.with_timezone(&Pacific__Auckland)
            );
            rp2040_needs_reset = false;
            got_startup_info = false;
            is_audio_device = device_config.is_audio_device();

            if !sent_reset_request {
                sent_reset_request = true;
                let _ = restart_rp2040_channel_tx.send(true);
            }

            if restart_rp2040_ack.load(Ordering::Relaxed) {
                // Reset this atomic bool if the reset has been processed by the frame server thread.
                sent_reset_request = false;
                restart_rp2040_ack.store(false, Ordering::Relaxed);
            }
        }
        let poll_result = pin.poll_interrupt(false, Some(Duration::from_millis(2000)));
        if let Ok(_pin_level) = poll_result {
            if _pin_level.is_some() {
                let res = reset_pin(pin,&gpio);
                match res{
                    Ok((res))=> pin = res,
                    Err(())=>{
                        wait_for_threads_to_finish(save_threads);
                        process::exit(1);
                    }
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
                    let transfer_type_check = transfer_type == transfer_type_dup;
                    let mut transfer_block = 0;
                    let is_file_transfer_message = transfer_type_check
                        && transfer_type >= CAMERA_BEGIN_FILE_TRANSFER
                        && transfer_type <= CAMERA_BEGIN_AND_END_FILE_TRANSFER;
                    let is_file_transfer_progress_message = transfer_type_check
                        && (transfer_type == CAMERA_BEGIN_FILE_TRANSFER
                            || transfer_type == CAMERA_END_FILE_TRANSFER);
                    let header_crc_check = if is_file_transfer_progress_message {
                        transfer_block = crc_from_remote_dup;

                        crc_from_remote_inv_dup == crc_from_remote_inv
                            && crc_from_remote_inv.not() == crc_from_remote
                    } else {
                        crc_from_remote == crc_from_remote_dup
                            && crc_from_remote_inv_dup == crc_from_remote_inv
                            && crc_from_remote_inv.not() == crc_from_remote
                    };
                    let num_bytes_check = num_bytes == num_bytes_dup;
                    if !num_bytes_check || !header_crc_check || !transfer_type_check {
                        // Just log the *first* time the header integrity check fails in a session.
                        if !header_integrity_check_has_failed {
                            header_integrity_check_has_failed = true;
                            warn!("Header integrity check failed {:?}", &header_slice[..]);
                            // We still need to make sure we read out all the bytes?
                        }

                        LittleEndian::write_u16(&mut return_payload_buf[4..6], 0);
                        LittleEndian::write_u16(&mut return_payload_buf[6..8], 0);
                        spi.write(&return_payload_buf).unwrap();
                        if process_interrupted(&sig_term, &mut dbus_conn) {
                            break 'transfer;
                        }
                        continue 'transfer;
                    }
                    if num_bytes == 0 {
                        // warn!("zero-sized payload");
                        LittleEndian::write_u16(&mut return_payload_buf[4..6], 0);
                        LittleEndian::write_u16(&mut return_payload_buf[6..8], 0);
                        spi.write(&return_payload_buf).unwrap();
                        if process_interrupted(&sig_term, &mut dbus_conn) {
                            break 'transfer;
                        }
                        continue 'transfer;
                    }
                    if transfer_type < CAMERA_CONNECT_INFO
                        || transfer_type > CAMERA_SEND_LOGGER_EVENT
                    {
                        warn!("unknown transfer type {}", transfer_type);
                        LittleEndian::write_u16(&mut return_payload_buf[4..6], 0);
                        LittleEndian::write_u16(&mut return_payload_buf[6..8], 0);
                        spi.write(&return_payload_buf).unwrap();
                        if process_interrupted(&sig_term, &mut dbus_conn) {
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
                        } else if transfer_type == CAMERA_GET_MOTION_DETECTION_MASK {
                            let piece_number = &chunk[0];
                            //info!("Got request for mask piece number {}", piece_number);
                            let piece = device_config.mask_piece(*piece_number as usize);

                            let mut piece_with_length = [0u8; 101];
                            piece_with_length[0] = piece.len() as u8;
                            piece_with_length[1..1 + piece.len()].copy_from_slice(piece);
                            let crc = crc_check.checksum(&piece_with_length[0..1 + piece.len()]);
                            LittleEndian::write_u16(&mut return_payload_buf[8..10], crc);
                            //info!("Sending piece({}) {:?}", piece.len(), piece_with_length);
                            return_payload_buf[10..10 + piece.len() + 1]
                                .copy_from_slice(&piece_with_length);
                        }
                        // Always write the return buffer
                        spi.write(&return_payload_buf).unwrap();
                        if crc == crc_from_remote {
                            if is_file_transfer_progress_message {
                                recording_state.update_offload_progress(transfer_block);
                            } else if !is_file_transfer_message && recording_state.is_offloading() {
                                recording_state.end_offload();
                            }

                            match transfer_type {
                                CAMERA_CONNECT_INFO => {
                                    radiometry_enabled = LittleEndian::read_u32(&chunk[0..4]) == 2;
                                    firmware_version = LittleEndian::read_u32(&chunk[4..8]);
                                    lepton_serial_number =
                                        format!("{}", LittleEndian::read_u32(&chunk[8..12]));
                                    got_startup_info = true;
                                    if firmware_version != EXPECTED_RP2040_FIRMWARE_VERSION {
                                        exit_cleanly(&mut dbus_conn);
                                        info!(
                                            "Unsupported firmware version, expected {}, got {}. Will reprogram RP2040.",
                                            EXPECTED_RP2040_FIRMWARE_VERSION, firmware_version
                                        );
                                        let e = program_rp2040();
                                        if e.is_err() {
                                            warn!("Failed to reprogram rp2040: {}", e.unwrap_err());
                                            panic!("Exit");
                                        }
                                        process::exit(0);
                                    }

                                    let recording_mode =
                                        if LittleEndian::read_u32(&chunk[12..16]) != 0 {
                                            RecordingMode::Audio
                                        } else {
                                            RecordingMode::Thermal
                                        };
                                    recording_state.set_mode(recording_mode);
                                    info!(
                                        "Got startup info: \
                                    radiometry enabled: {radiometry_enabled}, \
                                    firmware version: {firmware_version}, \
                                    lepton serial #{lepton_serial_number} \
                                    recording mode {:?}",
                                        recording_mode
                                    );

                                    if device_config.use_low_power_mode()
                                        && !radiometry_enabled
                                        && !device_config.is_audio_device()
                                    {
                                        exit_cleanly(&mut dbus_conn);
                                        error!(
                                            "Low power mode is currently only supported on \
                                        lepton sensors with radiometry or audio device, exiting."
                                        );
                                        panic!("Exit");
                                    }
                                    // Terminate any existing file download.
                                    let in_progress_file_transfer = file_download.take();
                                    if let Some(file) = in_progress_file_transfer {
                                        warn!(
                                            "Aborting in progress file transfer with {} bytes",
                                            file.len()
                                        );
                                    }
                                    let _ = camera_handshake_channel_tx.send(
                                        FrameSocketServerMessage {
                                            camera_handshake_info: None,
                                            camera_file_transfer_in_progress: false,
                                        },
                                    );
                                }
                                CAMERA_SEND_LOGGER_EVENT => {
                                    let event_kind = LittleEndian::read_u16(&chunk[0..2]);
                                    let mut event_timestamp =
                                        LittleEndian::read_u64(&chunk[2..2 + 8]);
                                    let event_payload = LittleEndian::read_u64(&chunk[10..18]);
                                    if let Ok(mut event_kind) =
                                        LoggerEventKind::try_from(event_kind)
                                    {
                                        if let Some(mut time) =
                                            DateTime::from_timestamp_micros(event_timestamp as i64)
                                        {
                                            if let LoggerEventKind::SetAlarm(alarm_time) =
                                                &mut event_kind
                                            {
                                                if DateTime::from_timestamp_micros(
                                                    event_payload as i64,
                                                )
                                                .is_some()
                                                {
                                                    *alarm_time = event_payload;
                                                } else {
                                                    warn!(
                                                        "Wakeup alarm from event was invalid {}",
                                                        event_payload
                                                    );
                                                }
                                            } else if let LoggerEventKind::Rp2040MissedAudioAlarm(
                                                alarm_time,
                                            ) = &mut event_kind
                                            {
                                                if DateTime::from_timestamp_micros(
                                                    event_payload as i64,
                                                )
                                                .is_some()
                                                {
                                                    *alarm_time = event_payload;
                                                } else {
                                                    warn!(
                                                        "Missed alarm from event was invalid {}",
                                                        event_payload
                                                    );
                                                }
                                            } else if let LoggerEventKind::ToldRpiToWake(reason) =
                                                &mut event_kind
                                            {
                                                if let Ok(wake_reason) =
                                                    WakeReason::try_from(event_payload as u8)
                                                {
                                                    *reason = wake_reason;
                                                } else {
                                                    warn!(
                                                        "Told rpi to wake invalid reason {}",
                                                        event_payload
                                                    );
                                                }
                                            } else if let LoggerEventKind::RtcCommError =
                                                &mut event_kind
                                            {
                                                if event_timestamp == 0 {
                                                    time = chrono::Local::now().with_timezone(&Utc);
                                                    event_timestamp =
                                                        time.timestamp_micros() as u64;
                                                }
                                            } else if let LoggerEventKind::LostFrames(lost_frames) =
                                                &mut event_kind
                                            {
                                                *lost_frames = event_payload as u64;
                                            }
                                            let payload_json =
                                                if let LoggerEventKind::SavedNewConfig = event_kind
                                                {
                                                    // If we get saved new config, the rp2040 would have just been
                                                    // restarted after the config change, so we can log the current
                                                    // config in relation to that event.
                                                    let json_inner = format!(
                                                        r#""continuous-recorder": {},
                                                        "use-low-power-mode": {},
                                                        "start-recording": "{:?}",
                                                        "stop-recording": "{:?}",
                                                        "location": "({}, {}, {})"
                                                        "#,
                                                        device_config.is_continuous_recorder(),
                                                        device_config.use_low_power_mode(),
                                                        device_config.recording_window().0,
                                                        device_config.recording_window().1,
                                                        device_config.lat_lng().0,
                                                        device_config.lat_lng().1,
                                                        device_config
                                                            .location_altitude()
                                                            .unwrap_or(0.0)
                                                    );
                                                    let json =
                                                        String::from(format!("{{{}}}", json_inner));
                                                    Some(json)
                                                } else {
                                                    None
                                                };
                                            info!(
                                                "Got logger event {:?} at {}",
                                                event_kind,
                                                time.with_timezone(&Pacific__Auckland)
                                            );
                                            let event =
                                                LoggerEvent::new(event_kind, event_timestamp);
                                            event.log(&mut dbus_conn, payload_json);
                                        } else {
                                            warn!(
                                                "Event had invalid timestamp {}",
                                                event_timestamp
                                            );
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
                                    let _ = camera_handshake_channel_tx.send(
                                        FrameSocketServerMessage {
                                            camera_handshake_info: None,
                                            camera_file_transfer_in_progress: true,
                                        },
                                    );
                                }
                                CAMERA_RESUME_FILE_TRANSFER => {
                                    if let Some(file) = &mut file_download {
                                        // Continue current file transfer
                                        //println!("Continue file transfer");
                                        if part_count % 100 == 0 {
                                            let megabytes_per_second = (file.len() + chunk.len())
                                                as f32
                                                / Instant::now()
                                                    .duration_since(start)
                                                    .as_secs_f32()
                                                / (1024.0 * 1024.0);
                                            info!(
                                                "Transferring part #{} {:?} for {} bytes, {}MB/s",
                                                part_count,
                                                Instant::now().duration_since(start),
                                                file.len() + chunk.len(),
                                                megabytes_per_second
                                            );
                                        }
                                        part_count += 1;
                                        file.extend_from_slice(&chunk);
                                        let _ = camera_handshake_channel_tx.send(
                                            FrameSocketServerMessage {
                                                camera_handshake_info: None,
                                                camera_file_transfer_in_progress: true,
                                            },
                                        );
                                    } else {
                                        warn!("Trying to continue file with no open file");
                                        if !got_startup_info {
                                            if recording_state
                                                .safe_to_restart_rp2040(&mut dbus_conn)
                                            {
                                                let date = chrono::Local::now();
                                                error!(
                                                    "1) Requesting reset of rp2040 to \
                                                force handshake, {}",
                                                    date.format("%Y-%m-%d--%H:%M:%S")
                                                );
                                                let _ = restart_rp2040_channel_tx.send(true);
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
                                        let megabytes_per_second = (file.len() + chunk.len())
                                            as f32
                                            / Instant::now().duration_since(start).as_secs_f32()
                                            / (1024.0 * 1024.0);
                                        info!(
                                            "End file transfer, took {:?} for {} bytes, {}MB/s",
                                            Instant::now().duration_since(start),
                                            file.len() + chunk.len(),
                                            megabytes_per_second
                                        );
                                        part_count = 0;
                                        file.extend_from_slice(&chunk);
                                        let shebang = LittleEndian::read_u16(&file[0..2]);
                                        let thread_res = if shebang == AUDIO_SHEBANG {
                                            save_audio_file_to_disk(file, device_config.clone())
                                        } else {
                                           save_cptv_file_to_disk(file, device_config.output_dir())
                                        };

                                        match thread_res {
                                            Ok(join_handle)=>{
                                                save_threads.push(join_handle);
                                            },
                                            Err(e)=>error!("Couldn't spawn save thread {}",e)
                                        }
                                        let _ = camera_handshake_channel_tx.send(
                                            FrameSocketServerMessage {
                                                camera_handshake_info: None,
                                                camera_file_transfer_in_progress: false,
                                            },
                                        );
                                    } else {
                                        warn!("Trying to end file with no open file");
                                    }
                                }
                                CAMERA_BEGIN_AND_END_FILE_TRANSFER => {
                                    if file_download.is_some() {
                                        info!(
                                            "Trying to begin (and end) file without ending current"
                                        );
                                    }
                                    // Open and end new file transfer
                                    part_count = 0;
                                    let mut file = Vec::new();
                                    file.extend_from_slice(&chunk);
                                    let shebang = LittleEndian::read_u16(&file[0..2]);
                                    let thread_res = if shebang == AUDIO_SHEBANG {
                                        save_audio_file_to_disk(file, device_config.clone())
                                    } else {
                                        save_cptv_file_to_disk(file, device_config.output_dir())
                                    };

                                    match thread_res {
                                        Ok(join_handle)=>{
                                            save_threads.push(join_handle);
                                        },
                                        Err(e)=>error!("Couldn't spawn save thread {}",e)
                                    }
                                    let _ = camera_handshake_channel_tx.send(
                                        FrameSocketServerMessage {
                                            camera_handshake_info: None,
                                            camera_file_transfer_in_progress: false,
                                        },
                                    );
                                }
                                CAMERA_GET_MOTION_DETECTION_MASK => {
                                    // Already handled
                                }
                                _ => {
                                    if num_bytes != 0 {
                                        warn!("Unhandled transfer type, {:#x}", transfer_type)
                                    }
                                }
                            }
                        } else {
                            warn!("Crc check failed, remote was notified and will re-transmit");
                        }
                    } else {
                        spi.read(&mut raw_read_buffer[2066..num_bytes + header_length])
                            .map_err(|e| {
                                error!("SPI read error: {:?}", e);
                                process::exit(1);
                            })
                            .unwrap();
                        // Frame
                        let mut frame = [0u8; FRAME_LENGTH];
                        BigEndian::write_u16_into(
                            u8_slice_as_u16_slice(
                                &raw_read_buffer[header_length..header_length + FRAME_LENGTH],
                            ),
                            &mut frame,
                        );
                        let back = FRAME_BUFFER.get_back().lock().unwrap();
                        back.replace(Some(frame));
                        if !got_first_frame {
                            got_first_frame = true;
                            info!(
                                "Got first frame from rp2040, got startup info {}",
                                got_startup_info
                            );
                        }
                        // FIXME: Should is_recording bit only be set in high power mode?
                        let is_recording =
                            crc_from_remote == 1 && !device_config.use_low_power_mode();
                        recording_state.set_is_recording(is_recording);
                        if !got_startup_info {
                            warn!("Requesting reset of rp2040 to force handshake");
                            rp2040_needs_reset = true;
                        } else if !rp2040_needs_reset {
                            FRAME_BUFFER.swap();
                            let _ = camera_handshake_channel_tx.send(FrameSocketServerMessage {
                                camera_handshake_info: Some(CameraHandshakeInfo {
                                    radiometry_enabled,
                                    is_recording: recording_state.is_recording(),
                                    firmware_version,
                                    camera_serial: lepton_serial_number.clone(),
                                }),
                                camera_file_transfer_in_progress: false,
                            });
                        }
                    }
                }
            }
        }
        if process_interrupted(&sig_term, &mut dbus_conn) {
            break 'transfer;
        }
    }
}
fn maybe_make_test_audio_recording(
    dbus_conn: &mut DuplexConn,
    restart_rp2040_channel_tx: &Sender<bool>,
    recording_state: &mut RecordingState,
) {
    // If the rp2040 is making a recording, and a user test audio recording was requested,
    // do nothing.

    // If the user requested a test audio recording, trigger the test audio recording, and
    // launch a thread to track when that recording has completed.
    if recording_state.user_requested_audio_recording() {
        recording_state.sync_state_from_attiny(dbus_conn);
        if !recording_state.is_recording()
            && recording_state.request_audio_recording_from_rp2040(dbus_conn)
        {
            let _ = restart_rp2040_channel_tx.send(true);
            info!("Telling rp2040 to take test recording and restarting");
            let mut inner_recording_state = recording_state.clone();
            let _ = thread::spawn(move || {
                let dbus_session_path = get_system_bus_path();
                if dbus_session_path.is_err(){
                    error!("Error getting system DBus: {}", dbus_session_path.err().unwrap());
                    return;
                }
                let dbus_session_path = dbus_session_path.unwrap();
                match DuplexConn::connect_to_bus(dbus_session_path, true) {
                    Err(e) => {
                        error!("Error connecting to system DBus: {}", e);
                        // process::exit(1);
                    }
                    Ok(mut conn) => {
                        let _unique_name = conn.send_hello(Timeout::Infinite);
                        if _unique_name.is_err() {
                            error!(
                                "Error getting handshake with system DBus: {}",
                                _unique_name.err().unwrap()
                            );
                            return;
                        } else {
                            loop {
                                // Re-sync our internal rp2040 state once every 1-2 seconds until
                                // we see that the state has entered taking_test_audio_recording.
                                inner_recording_state.sync_state_from_attiny(&mut conn);
                                let sleep_duration_ms = if inner_recording_state.is_recording() {
                                    2000
                                } else {
                                    1000
                                };
                                if inner_recording_state.is_taking_test_audio_recording() {
                                    break;
                                }
                                sleep(Duration::from_millis(sleep_duration_ms));
                            }
                            loop {
                                // Now wait until we've exited taking_test_audio_recording.
                                inner_recording_state.sync_state_from_attiny(&mut conn);
                                let sleep_duration_ms = if inner_recording_state.is_recording() {
                                    2000
                                } else {
                                    1000
                                };
                                if !inner_recording_state.is_taking_test_audio_recording() {
                                    inner_recording_state.finished_taking_test_recording();
                                    break;
                                }
                                sleep(Duration::from_millis(sleep_duration_ms));
                            }
                        }
                    }
                }
            });
        }
    }
}
