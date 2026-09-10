use crate::camera_transfer_state::CameraHandshakeInfo;
use crate::program_rp2040::program_rp2040;
use crate::recording_state::{RecordingMode, RecordingState};
use crate::socket_stream::{SocketStream, get_socket_address};
use crate::telemetry::{Telemetry, read_telemetry};
use crate::{FRAME_LENGTH, cptv_frame_dispatch};
use log::{debug, error, info, warn};
use rppal::gpio::OutputPin;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::{Receiver, RecvTimeoutError};
use std::thread::sleep;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};
use std::{fs, process, thread};
use thread_priority::{ThreadBuilderExt, ThreadPriority};

pub struct FrameSocketServerMessage {
    pub(crate) camera_handshake_info: Option<CameraHandshakeInfo>,
    pub(crate) camera_file_transfer_in_progress: bool,
    pub(crate) file_offload: Option<FileOffloadInfo>,
    pub(crate) frame_message: bool,
}
pub struct FileOffloadInfo {
    pub(crate) frame_bytes: usize,
    pub(crate) is_last_part: bool,
    pub(crate) data: Vec<u8>,
    pub(crate) package_num: u8,
}

fn restart_rp2040_if_requested(
    restart_rp2040_channel_rx: &Receiver<bool>,
    run_pin: &mut OutputPin,
    restart_rp2040_ack: &mut Arc<AtomicBool>,
) {
    // FIXME: Check if the rp2040 recording flag is set before restarting.

    // Check if we need to reset rp2040 because of a config change
    if restart_rp2040_channel_rx.try_recv().is_ok() {
        restart_rp2040(run_pin, restart_rp2040_ack);
        // restart_rp2040_ack.store(true, Ordering::Relaxed);
        // info!("Restarting rp2040");
        // if !run_pin.is_set_high() {
        //     run_pin.set_high();
        //     sleep(Duration::from_millis(1000));
        // }
        // run_pin.set_low();
        // sleep(Duration::from_millis(1000));
        // run_pin.set_high();
    }
}

fn restart_rp2040(run_pin: &mut OutputPin, restart_rp2040_ack: &mut Arc<AtomicBool>) {
    restart_rp2040_ack.store(true, Ordering::Relaxed);
    info!("Restarting rp2040");
    if !run_pin.is_set_high() {
        run_pin.set_high();
        sleep(Duration::from_millis(1000));
    }
    run_pin.set_low();
    sleep(Duration::from_millis(1000));
    run_pin.set_high();
}

pub fn spawn_frame_socket_server_thread(
    restart_rp2040_channel_rx: Receiver<bool>,
    camera_handshake_channel_rx: Receiver<FrameSocketServerMessage>,
    serve_frames_via_wifi: bool,
    mut run_pin: OutputPin,
    mut restart_rp2040_ack: Arc<AtomicBool>,
    recording_state: &RecordingState,
    medium_power_mode: bool,
) {
    let recording_state = recording_state.clone();
    let _ = thread::Builder::new().name("frame-socket".to_string()).spawn_with_priority(
        ThreadPriority::Max,
        move |result| {
            info!("Frame socket started");

            let mut file_download: Option<Vec<u8>> = None;
            let gzip_header: [u8; 10] = [0x1f, 0x8b, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff];
            let mut frame_i = 0;
            // let mut was_recording = false;
            let address = get_socket_address(serve_frames_via_wifi);
            let management_address = "/var/spool/managementd".to_string();
            // Spawn a thread which can output the frames, converted to rgb grayscale
            // This is printed out from within the spawned thread.
            if result.is_err() {
                error!(
                    "Thread must have permissions to run with realtime priority, \
                    run as root user"
                );
                process::exit(1);
            }

            let mut reconnects = 0;
            let mut prev_frame_num = None;
            let mut sockets: [(String, bool, Option<SocketStream>); 2] =
                [(address, serve_frames_via_wifi, None), (management_address, false, None)];

            restart_rp2040_if_requested(
                &restart_rp2040_channel_rx,
                &mut run_pin,
                &mut restart_rp2040_ack,
            );
            let mut recv_timeout_ms = 10;
            info!("Connecting to frame sockets");
            let mut ms_elapsed = 0;
            let mut last_package_num = 255u8;
            let mut sent_end = false;
            let mut message_handled = false;
            let mut received_end = false;
            let address = &get_socket_address(serve_frames_via_wifi);
            loop {
                restart_rp2040_if_requested(
                    &restart_rp2040_channel_rx,
                    &mut run_pin,
                    &mut restart_rp2040_ack,
                );
                if recording_state.recording_mode() == RecordingMode::Thermal {
                    for (address, use_wifi, stream) in
                        sockets.iter_mut().filter(|(_, _, stream)| stream.is_none())
                    {
                        let stream_connection: Option<SocketStream> =
                            SocketStream::from_address(address, *use_wifi).ok();
                        if stream_connection.is_some() {
                            println!("Connected to {address}");                            
                        }
                        *stream = stream_connection;
                    }

                    let connections =
                        sockets.iter().filter(|(_, _, stream)| stream.is_some()).count();
                    if connections == 0 && !medium_power_mode {
                        sleep(Duration::from_millis(1000));
                        continue;
                    }
                }


                let message = camera_handshake_channel_rx
                    .recv_timeout(Duration::from_millis(recv_timeout_ms));
                if medium_power_mode && let Ok(FrameSocketServerMessage {
                        camera_handshake_info:
                            Some(CameraHandshakeInfo {radiometry_enabled,firmware_version,
                                camera_serial,
                                is_recording,
                            }),
                        camera_file_transfer_in_progress: false,
                        file_offload,
                        frame_message: true,
                    }) = message.as_ref()
                    {
                        message_handled = *is_recording;
                        if *is_recording{
                            let file_info = file_offload.as_ref().expect("Data should always be there if recording");

                            let frame_bytes = file_info.frame_bytes;
                            let is_last_part = file_info.is_last_part;
                            let frame_data = &file_info.data;
                            let package_num = file_info.package_num;
                            received_end = is_last_part;
                            if frame_i ==0  {
                                info!("Reset file download as have new recording");
                                file_download = None;
                                last_package_num = 255;
                                sent_end = false;
                            }
                            let mut was_sent = false;
                            if package_num == last_package_num{
                                // could happen if rp2040 thinks we didnt receive the last packet
                                warn!("Received the same package twice ignoring the second one");
                                continue
                            }

                            last_package_num = package_num;
                            // first 16 bytes are timestamp, serial and firmware
                            if frame_i ==0 && frame_data[16..16+10]!= gzip_header{
                            // ensure is a gzip
                                error!("New file is missing the GZIP header {:?} restart rp2040",&frame_data[..16+10]);
                                restart_rp2040(&mut run_pin, &mut restart_rp2040_ack);
                                // what do we do here???
                                // force rp2040 to offload last file??
                                // if we could send a message ask for the start again
                                continue;
                            }
                            frame_i+=1;

                            let socket = sockets.iter_mut().find(|(sock_address, _, stream)| {
                                stream.is_some() && sock_address == address
                            });
                            if let Some(sock) = socket
                            {
                                    was_sent = handle_medium_power(
                                        sock,
                                        radiometry_enabled,
                                        firmware_version,
                                        camera_serial,
                                        &mut file_download,
                                        &mut ms_elapsed,
                                        Some(frame_data),
                                        is_last_part,
                                        frame_i == 1,
                                    );
                                    if is_last_part && was_sent{
                                        sent_end = true;
                                        frame_i = 0;
                                    }
                            }
                            // store file to send later
                            if !was_sent {
                                if let Some(file) = &mut file_download {
                                    info!("Adding bytes {} to memory file", frame_bytes);
                                    file.extend_from_slice(frame_data);
                                } else if frame_i > 1{
                                        error!("Lost socket connection part way through medium power offload (Something must have gone wrong), Restarting RP2040");
                                        // restart rp2040 and offload the file
                                        restart_rp2040(&mut run_pin, &mut restart_rp2040_ack);
                                        frame_i = 0;
                                        continue

                                }else{
                                    info!("Starting new file");
                                    let mut file: Vec<u8> = Vec::with_capacity(50_000_000);
                                    file.extend_from_slice(frame_data);
                                    file_download = Some(file);
                                }
                            }
                        }else if !sent_end && (frame_i >0 || file_download.is_some()) {
                            // have a frame message and not recording so need to finish any open recordings, generally this is done as soon as we receive the last packet
                            // but there are some edge cases (abort and socket wasnt connect yet)
                            let socket = sockets.iter_mut().find(|(sock_address, _, stream)| {
                                stream.is_some() && sock_address == address
                            });
                            if  let Some(sock) = socket{
                                if received_end && file_download.is_some(){
                                    // handles edge case that the whole recording was received and is in file_download before anything was sent for processing
                                    // very unlikely as generally the python code is waiting prior to tc2-agent starting
                                    if handle_medium_power(
                                                sock,
                                                &true, //we may not have received the radiometry info since we don't always handshake first, so just say its on always
                                                firmware_version,
                                                camera_serial,
                                                &mut file_download,
                                                &mut ms_elapsed,
                                                None,
                                                true,
                                                true,
                                    ){
                                        sent_end = true;
                                    }
                                }else{
                                    // send abort when recording was discarded on rp2040
                                    send_abort(sock);
                                    file_download = None;
                                    sent_end = true;
                                }
                            }
                            frame_i =0;
                        }

                }
                if !message_handled {
                    // dont send normal frame messages to medium power socket, we may want to change this and send the message type
                    let sub_sockets: Vec<&mut (String, bool, Option<SocketStream>)>= sockets.iter_mut().filter(|(sock_address, _, stream)| { stream.is_some() && (!medium_power_mode || sock_address != address) }).collect();
                    handle_payload_from_frame_acquire_thread(
                        message,
                        sub_sockets,
                        &mut ms_elapsed,
                        &mut reconnects,
                        &mut prev_frame_num,
                        &recording_state,
                        &mut recv_timeout_ms,
                    );
                }
            }
        },
    );
}

fn handle_payload_from_frame_acquire_thread(
    result: Result<FrameSocketServerMessage, RecvTimeoutError>,
    mut sockets: Vec<&mut (String, bool, Option<SocketStream>)>,
    ms_elapsed: &mut u64,
    reconnects: &mut usize,
    prev_frame_num: &mut Option<u32>,
    recording_state: &RecordingState,
    recv_timeout_ms: &mut u64,
) {
    match result {
        Ok(FrameSocketServerMessage {
            camera_handshake_info:
                Some(CameraHandshakeInfo {
                    radiometry_enabled,
                    is_recording,
                    firmware_version,
                    camera_serial,
                }),
            camera_file_transfer_in_progress: false,
            file_offload: None,
            frame_message: true,
        }) => {
            let model = if radiometry_enabled { "lepton3.5" } else { "lepton3" };
            let header = format!(
                "ResX: 160\n\
                        ResX: 160\n\
                        ResY: 120\n\
                        FrameSize: 39040\n\
                        Model: {model}\n\
                        Brand: flir\n\
                        FPS: 9\n\
                        Firmware: DOC-AI-v0.{firmware_version}\n\
                        CameraSerial: {camera_serial}\n\n",
            );
            for (_, _, stream) in sockets.iter_mut().filter(|(_, use_wifi, stream)| {
                stream.is_some() && !use_wifi && !stream.as_ref().unwrap().sent_header
            }) {
                let stream = stream.as_mut().expect("Never fails, because we filtered already.");

                if stream.write_all(header.as_bytes()).is_err() {
                    warn!("Failed sending header info");
                }
                // Clear existing
                if stream.write_all(b"clear").is_err() {
                    warn!("Failed clearing buffer");
                }
                let _ = stream.flush();
                stream.sent_header = true;
            }

            if *reconnects > 0 {
                info!("Got frame connection");
                *prev_frame_num = None;
                *reconnects = 0;
            }
            let s = Instant::now();
            let mut telemetry: Option<Telemetry> = None;
            let frame_data = cptv_frame_dispatch::get_frame(is_recording);
            if let Some(fb) = frame_data {
                telemetry = Some(read_telemetry(&fb));
                for (address, use_wifi, stream) in
                    sockets.iter_mut().filter(|(_, _, stream)| stream.is_some())
                {
                    let sent =
                        cptv_frame_dispatch::send_frame(&fb, stream.as_mut().expect("Never fails"));
                    if !sent {
                        warn!(
                            "Send to {} failed",
                            if *use_wifi { "tc2-frames server" } else { address }
                        );
                        let _ = stream.take().expect("Never fails").shutdown().is_ok();
                    }
                }
            }
            let e = s.elapsed().as_secs_f32();
            if e > 0.1 {
                info!("socket send took {e}s");
            }
            if let Some(telemetry) = telemetry {
                if let Some(prev_frame_num) = prev_frame_num
                    && !telemetry.ffc_in_progress
                    && telemetry.frame_num != *prev_frame_num + 1
                {
                    // NOTE: Frames can be missed when the raspberry pi
                    //  blocks the thread with the
                    //  unix socket in `thermal-recorder`.
                    debug!(
                        "Missed {} frames after {}s on",
                        telemetry.frame_num - (*prev_frame_num + 1),
                        telemetry.msec_on as f32 / 1000.0
                    );
                }

                *prev_frame_num = Some(telemetry.frame_num);
                if telemetry.frame_num % 2700 == 0 {
                    info!("Got frame #{}", telemetry.frame_num);
                }
            }

            *ms_elapsed = 0;
        }
        Ok(FrameSocketServerMessage {
            camera_handshake_info: None,
            camera_file_transfer_in_progress: true,
            ..
        }) => {
            // There's a file transfer in progress, and we got a recording mode change?
            *ms_elapsed = 0;
            match recording_state.recording_mode() {
                RecordingMode::Audio => {
                    *recv_timeout_ms = 1000;
                    for (address, use_wifi, stream) in
                        sockets.iter_mut().filter(|(_, _, stream)| stream.is_some())
                    {
                        info!(
                            "Shutting down socket '{}'",
                            if *use_wifi { "tc2-frames server" } else { address }
                        );
                        let _ = stream.take().unwrap().shutdown().is_ok();
                    }
                }
                RecordingMode::Thermal => {
                    *recv_timeout_ms = 10;
                }
            }
        }
        _ => {
            if recording_state.recording_mode() == RecordingMode::Thermal {
                const NUM_ATTEMPTS_BEFORE_RESTART_OR_REPROGRAM: usize = 20;
                *ms_elapsed += *recv_timeout_ms;
                if *ms_elapsed > 10_000 {
                    *ms_elapsed = 0;
                    *reconnects += 1;
                    if *reconnects == NUM_ATTEMPTS_BEFORE_RESTART_OR_REPROGRAM {
                        let reprogram_file = "/home/pi/last-rp2040-reprogram";
                        let last_reprogram_over_1hr_ago = match fs::exists(reprogram_file) {
                            Ok(true) => {
                                let metadata = fs::metadata(reprogram_file)
                                    .expect("Failed reading file metadata");
                                let created = metadata.modified().unwrap_or_else(|_| {
                                    let timestamp_str = fs::read(reprogram_file)
                                        .map(|file_contents| {
                                            String::from_utf8(file_contents)
                                                .unwrap_or(String::from("0"))
                                        })
                                        .expect("Failed getting timestamp string");
                                    let timestamp_seconds =
                                        timestamp_str.parse::<u64>().unwrap_or(0);
                                    let timestamp = Duration::from_secs(timestamp_seconds);
                                    UNIX_EPOCH + timestamp
                                });
                                SystemTime::now()
                                    .duration_since(created)
                                    .is_ok_and(|duration| duration > Duration::from_secs(60 * 60))
                            }
                            Ok(false) | Err(_) => true,
                        };
                        if last_reprogram_over_1hr_ago {
                            let _ = fs::remove_file(reprogram_file);
                            match program_rp2040() {
                                Ok(()) => {
                                    let now = SystemTime::now();
                                    let timestamp_seconds = now
                                        .duration_since(UNIX_EPOCH)
                                        .expect("Time went backwards")
                                        .as_secs();
                                    fs::write(reprogram_file, format!("{timestamp_seconds}"))
                                        .expect("Failed writing reprogram placeholder file");
                                    process::exit(0)
                                }
                                Err(e) => {
                                    error!("Failed to reprogram RP2040: {e}");
                                    process::exit(1);
                                }
                            }
                        } else {
                            error!(
                                "Failed to connect to rp2040 frame serving, restarting tc2-agent"
                            );
                            process::exit(0);
                        }
                    } else {
                        info!(
                            "-- #{reconnects} waiting to connect to rp2040 \
                        (will restart tc2-agent after {} more attempts)",
                            NUM_ATTEMPTS_BEFORE_RESTART_OR_REPROGRAM - *reconnects
                        );
                    }
                }
            }
        }
    }
}

fn send_abort(socket: &mut (String, bool, Option<SocketStream>)) -> bool {
    info!("Aborted recording");
    let (_, _, og_stream) = socket;
    let stream = og_stream.as_mut().expect("Never fails, because we filtered already.");

    if stream.write_all(b"abort").is_err() {
        let _ = stream.shutdown().is_ok();
        return false;
    }
    true
}
fn handle_medium_power(
    socket: &mut (String, bool, Option<SocketStream>),
    radiometry_enabled: &bool,
    firmware_version: &u32,
    camera_serial: &String,
    file_download: &mut Option<Vec<u8>>,
    ms_elapsed: &mut u64,
    frame_data: Option<&[u8]>,
    is_last_part: bool,
    first_part: bool,
) -> bool {
    let (address, use_wifi, og_stream) = socket;
    let stream = og_stream.as_mut().expect("Never fails, because we filtered already.");
    if !stream.sent_header {
        info!("Sending header");
        let _ = stream.flush();
        let model = if *radiometry_enabled { "lepton3.5" } else { "lepton3" };
        let header = format!(
            "ResX: 160\n\
                ResX: 160\n\
                ResY: 120\n\
                FrameSize: 39040\n\
                Model: {model}\n\
                Brand: flir\n\
                FPS: 9\n\
                Firmware: DOC-AI-v0.{firmware_version}\n\
                CameraSerial: {camera_serial}\n\n",
        );

        if stream.write_all(header.as_bytes()).is_err() {
            warn!("Failed sending header info");
        }
        // Clear existing
        if stream.write_all(b"clear").is_err() {
            warn!("Failed clearing buffer");
        }
        let sent: bool = stream.flush().is_ok();
        if !sent {
            info!("Shutting down socket");
            let _ = og_stream.take().expect("Never fails").shutdown().is_ok();
            return false;
        }
        stream.sent_header = true;
    }
    let s = Instant::now();

    if first_part || file_download.is_some() {
        info!("Sending start");
        if stream.write_all(b"start\n\n").is_err() {
            warn!("Failed sending start rec");
        }
        let sent: bool = stream.flush().is_ok();
        if !sent {
            info!("Shutting down socket");
            let _ = og_stream.take().expect("Never fails").shutdown().is_ok();
            return false;
        }
    }

    if file_download.is_some() {
        info!("Thermal is ready and have some file so send it....");

        let data: &mut Vec<u8> = file_download.as_mut().unwrap();
        info!("Sending file to thermal {} first 10 {:?}", data.len(), &data[..10]);
        for chunk in data.chunks(FRAME_LENGTH) {
            info!("Sending chunk {}", chunk.len());
            let sent = cptv_frame_dispatch::send_frame(chunk, stream);

            if !sent {
                let _ = og_stream.take().expect("Never fails").shutdown().is_ok();
                return false;
            }
        }
        // only remove if sent all
        file_download.take();
        info!("Sent all of file download");
    }
    // info!("THermal ready? {} was reco {} is_rec {} bytes {}", thermal_ready,was_recording,is_recording,frame_bytes);

    if let Some(fb) = frame_data {
        let sent = cptv_frame_dispatch::send_frame(fb, stream);
        if !sent {
            warn!(
                "Medium Power Send to {} failed",
                if *use_wifi { "tc2-frames server" } else { address }
            );
            let _ = og_stream.take().expect("Never fails").shutdown().is_ok();
            return false;
        }
    }
    if is_last_part {
        info!("Ending recording");
        if stream.write_all(b"clear").is_err() {
            let _ = stream.shutdown().is_ok();
            return false;
        }
    }
    let e = s.elapsed().as_secs_f32();
    if e > 0.1 {
        info!("socket send took {e}s");
    }
    *ms_elapsed = 0;
    true
}
