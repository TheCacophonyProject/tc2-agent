use crate::camera_transfer_state::CameraHandshakeInfo;
use crate::cptv_frame_dispatch;
use crate::program_rp2040::program_rp2040;
use crate::recording_state::{RecordingMode, RecordingState};
use crate::socket_stream::{SocketStream, get_socket_address};
use crate::telemetry::{Telemetry, read_telemetry};
use log::{debug, error, info, warn};
use rppal::gpio::OutputPin;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::{Receiver, RecvTimeoutError};
use std::thread::sleep;
use std::time::{Duration, Instant};
use std::{process, thread};
use thread_priority::{ThreadBuilderExt, ThreadPriority};

pub struct FrameSocketServerMessage {
    pub(crate) camera_handshake_info: Option<CameraHandshakeInfo>,
    pub(crate) camera_file_transfer_in_progress: bool,
}

fn restart_rp2040_if_requested(
    restart_rp2040_channel_rx: &Receiver<bool>,
    run_pin: &mut OutputPin,
    restart_rp2040_ack: &mut Arc<AtomicBool>,
) {
    // FIXME: Check if the rp2040 recording flag is set before restarting.

    // Check if we need to reset rp2040 because of a config change
    if restart_rp2040_channel_rx.try_recv().is_ok() {
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
}

pub fn spawn_frame_socket_server_thread(
    restart_rp2040_channel_rx: Receiver<bool>,
    camera_handshake_channel_rx: Receiver<FrameSocketServerMessage>,
    serve_frames_via_wifi: bool,
    mut run_pin: OutputPin,
    mut restart_rp2040_ack: Arc<AtomicBool>,
    recording_state: &RecordingState,
) {
    let recording_state = recording_state.clone();
    let _ = thread::Builder::new().name("frame-socket".to_string()).spawn_with_priority(
        ThreadPriority::Max,
        move |result| {
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
                    if connections == 0 {
                        sleep(Duration::from_millis(1000));
                        continue;
                    }
                }

                handle_payload_from_frame_acquire_thread(
                    camera_handshake_channel_rx
                        .recv_timeout(Duration::from_millis(recv_timeout_ms)),
                    &mut sockets,
                    &mut ms_elapsed,
                    &mut reconnects,
                    &mut prev_frame_num,
                    &recording_state,
                    &mut recv_timeout_ms,
                );
            }
        },
    );
}

fn handle_payload_from_frame_acquire_thread(
    result: Result<FrameSocketServerMessage, RecvTimeoutError>,
    sockets: &mut [(String, bool, Option<SocketStream>); 2],
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
                        cptv_frame_dispatch::send_frame(fb, stream.as_mut().expect("Never fails"));
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
                const NUM_ATTEMPTS_BEFORE_REPROGRAM: usize = 20;
                *ms_elapsed += *recv_timeout_ms;
                if *ms_elapsed > 10_000 {
                    *ms_elapsed = 0;
                    *reconnects += 1;
                    if *reconnects == NUM_ATTEMPTS_BEFORE_REPROGRAM {
                        match program_rp2040() {
                            Ok(()) => process::exit(0),
                            Err(e) => {
                                error!("Failed to reprogram RP2040: {e}");
                                process::exit(1);
                            }
                        }
                    } else {
                        info!(
                            "-- #{reconnects} waiting to connect to rp2040 \
                        (reprogram RP2040 after {} more attempts)",
                            NUM_ATTEMPTS_BEFORE_REPROGRAM - *reconnects
                        );
                    }
                }
            }
        }
    }
}
