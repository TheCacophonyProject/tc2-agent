use crate::camera_transfer_state::CameraHandshakeInfo;
use crate::medium_power_offloads;
use crate::program_rp2040::program_rp2040;
use crate::recording_state::{RecordingMode, RecordingState};
use crate::socket_stream::{ClientMsg, SocketStream, get_socket_address};
use crate::telemetry::{Telemetry, read_telemetry};
use crate::{FRAME_LENGTH, cptv_frame_dispatch};
use byteorder::{ByteOrder, LittleEndian};
use log::{debug, error, info, warn};
use rppal::gpio::OutputPin;
use std::collections::VecDeque;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::{Receiver, RecvTimeoutError};
use std::thread::sleep;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};
use std::{fs, process, thread};
use thread_priority::{ThreadBuilderExt, ThreadPriority};

/// After restarting the rp2040 because a recording didn't start properly, how long to drop
/// medium power packets while waiting for it to start sending a new recording, before allowing
/// another restart.
const RESTART_WAIT: Duration = Duration::from_secs(30);

/// How many medium power packets to send between checks for status messages from the client.
const CLIENT_POLL_INTERVAL: i32 = 10;

/// Bytes at the start of the first packet of a medium power file, before the gzipped file data:
/// timestamp (i64), lepton serial (u32), firmware version (u32), file start block (u16).
const FIRST_PACKET_HEADER_LENGTH: usize = 8 + 4 + 4 + 2;

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
    pub(crate) packet_num: u8,
    // Whether this packet's data matched the crc the rp2040 sent with it.
    pub(crate) crc_ok: bool,
}

/// A recording being received by medium power streaming.
struct StreamedRecording {
    // When the recording started, from the first packet. The same as the CPTV header timestamp.
    timestamp: u64,
    // Whether every packet so far passed its crc check.
    intact: bool,
}

/// Most streamed recordings to keep waiting for the client to finish. The client should answer
/// each one, so this only stops the list growing if it doesn't.
const MAX_AWAITING_DONE: usize = 100;

/// Called once a streamed recording has been passed on in full. If it was intact, it waits for
/// the client to say it finished the file ("DON" + timestamp), and is then remembered so we can
/// tell the rp2040 to erase it rather than offload it again.
fn streamed_recording_sent(
    streamed_recording: &mut Option<StreamedRecording>,
    awaiting_done: &mut VecDeque<u64>,
) {
    match streamed_recording.take() {
        Some(StreamedRecording { timestamp, intact: true }) => {
            awaiting_done.push_back(timestamp);
            info!(
                "Streamed recording {timestamp} sent in full, waiting for the client to finish \
                it. Waiting for {awaiting_done:?}"
            );
            if awaiting_done.len() > MAX_AWAITING_DONE
                && let Some(oldest) = awaiting_done.pop_front()
            {
                warn!(
                    "Client never finished streamed recording {oldest}, it will be offloaded \
                    again from the rp2040 during normal offloading"
                );
            }
        }
        Some(StreamedRecording { timestamp, intact: false }) => {
            warn!(
                "Streamed recording {timestamp} had crc errors, it will be offloaded again \
                from the rp2040 during normal offloading"
            );
        }
        None => warn!("Finished sending a recording, but no streamed recording was being tracked"),
    }
}

/// Clears the streamed recordings waiting for the client to finish them, logging `reason`. They
/// won't be saved as streamed, so they'll be offloaded again from the rp2040.
fn clear_awaiting_done(awaiting_done: &mut VecDeque<u64>, reason: &str) {
    if !awaiting_done.is_empty() {
        warn!(
            "{reason}, {} streamed recordings the client hadn't finished will be offloaded again \
            from the rp2040 during normal offloading",
            awaiting_done.len()
        );
        awaiting_done.clear();
    }
}

/// Removes `timestamp` from the recordings waiting for the client, returning whether it was there.
/// The client answers recordings in the order they were streamed, so any sent before it that
/// haven't been answered never will be, and are removed too.
fn take_awaiting_done(awaiting_done: &mut VecDeque<u64>, timestamp: u64) -> bool {
    let Some(index) = awaiting_done.iter().position(|t| *t == timestamp) else {
        warn!("Recording {timestamp} isn't waiting for the client, waiting for {awaiting_done:?}");
        return false;
    };
    for unanswered in awaiting_done.drain(..=index).take(index) {
        warn!(
            "Client never answered for streamed recording {unanswered}, it will be offloaded \
            again from the rp2040 during normal offloading"
        );
    }
    true
}

/// Handles status messages from the medium power client. Returns false if the connection is
/// broken, or the client failed the recording currently being streamed, in which case the socket
/// should be shut down.
fn poll_medium_power_client(
    stream: &mut SocketStream,
    streamed_recording: &mut Option<StreamedRecording>,
    awaiting_done: &mut VecDeque<u64>,
) -> bool {
    let msgs = match stream.poll_client() {
        Ok(msgs) => msgs,
        Err(e) => {
            warn!("Client connection lost: {e}");
            clear_awaiting_done(awaiting_done, "Client connection lost");
            return false;
        }
    };
    let mut keep_streaming = true;
    for msg in msgs {
        info!("Got {msg:?} from client");
        match msg {
            ClientMsg::Done(timestamp) => {
                if take_awaiting_done(awaiting_done, timestamp) {
                    info!("Client finished streamed recording {timestamp}");
                    medium_power_offloads::record(timestamp);
                } else {
                    warn!(
                        "Client finished recording {timestamp}, but it wasn't waiting to be \
                        finished"
                    );
                }
            }
            ClientMsg::Fail(timestamp) => {
                if take_awaiting_done(awaiting_done, timestamp) {
                    warn!(
                        "Client failed streamed recording {timestamp}, it will be offloaded \
                        again from the rp2040 during normal offloading"
                    );
                } else if let Some(recording) =
                    streamed_recording.as_mut().filter(|r| r.timestamp == timestamp)
                {
                    warn!("Client failed the live recording being streamed {timestamp}");
                    recording.intact = false;
                    keep_streaming = false;
                } else {
                    warn!("Client failed recording {timestamp}, but it isn't one we know of");
                }
            }
            ClientMsg::Ack => {}
            ClientMsg::Unknown(bytes) => warn!("Unknown message from client {bytes:?}"),
        }
    }
    keep_streaming
}

fn restart_rp2040_if_requested(
    restart_rp2040_channel_rx: &Receiver<bool>,
    run_pin: &mut OutputPin,
) {
    // FIXME: Check if the rp2040 recording flag is set before restarting.

    // Check if we need to reset rp2040 because of a config change
    if restart_rp2040_channel_rx.try_recv().is_ok() {
        restart_rp2040(run_pin);
    }
}

fn restart_rp2040(run_pin: &mut OutputPin) {
    info!("Restarting rp2040");
    if !run_pin.is_set_high() {
        run_pin.set_high();
        sleep(Duration::from_millis(1000));
    }
    run_pin.set_low();
    sleep(Duration::from_millis(1000));
    run_pin.set_high();
}

/// Handles one frame message from the rp2040 while in medium power mode.
/// In medium power mode the rp2040 wakes up the PI while recording and attempts to send the gzipped
/// CPTV frames of the current recording to tc2-agent, starting from the first frame to the current frame or end of file if the recording is now finished.
/// Each chunk is then forwarded to the connected socket (thermal-recorder-py) as it arrives;
/// if no socket is connected yet, chunks are buffered in `file_download` and flushed once a
/// socket becomes available or the recording ends, so the consumer can still gunzip and
/// process the whole file. Also detects and recovers from edge cases: a missing GZIP header
/// on the first chunk, a duplicate/out-of-order packet, a socket dropping mid-transfer, and
/// a recording that was discarded (aborted) on the rp2040 before it could be fully sent.
#[allow(clippy::too_many_arguments)]
fn handle_medium_power_message(
    message: &Result<FrameSocketServerMessage, RecvTimeoutError>,
    sockets: &mut [(String, bool, Option<SocketStream>); 2],
    address: &String,
    gzip_header: &[u8; 10],
    file_download: &mut Option<Vec<u8>>,
    frame_i: &mut i32,
    last_packet_num: &mut u8,
    sent_end: &mut bool,
    received_end: &mut bool,
    ms_elapsed: &mut u64,
    run_pin: &mut OutputPin,
    message_handled: &mut bool,
    streamed_recording: &mut Option<StreamedRecording>,
    awaiting_done: &mut VecDeque<u64>,
    waiting_for_restart: &mut Option<Instant>,
) {
    if let Ok(FrameSocketServerMessage {
        camera_handshake_info:
            Some(CameraHandshakeInfo { firmware_version, camera_serial, is_recording, .. }),
        camera_file_transfer_in_progress: false,
        file_offload,
        frame_message: true,
    }) = message.as_ref()
    {
        *message_handled = *is_recording;
        if *is_recording {
            let file_info =
                file_offload.as_ref().expect("Data should always be there if recording");

            let frame_bytes = file_info.frame_bytes;
            let is_last_part = file_info.is_last_part;
            let frame_data = &file_info.data;
            let packet_num = file_info.packet_num;
            // The first packet of a recording is numbered 0, and has the gzip header straight
            // after the timestamp, serial, firmware and file start block.
            let is_first_packet = packet_num == 0
                && frame_data.get(FIRST_PACKET_HEADER_LENGTH..FIRST_PACKET_HEADER_LENGTH + 10)
                    == Some(gzip_header.as_slice());
            // Packets queued before the restart, or sent if it didn't work, would each look like
            // a recording missing its gzip header, so drop them until a recording starts.
            if let Some(restarted_at) = *waiting_for_restart {
                if is_first_packet {
                    info!("rp2040 started sending a new recording after being restarted");
                    *waiting_for_restart = None;
                } else if restarted_at.elapsed() < RESTART_WAIT {
                    debug!("Dropping packet {packet_num} while waiting for the rp2040 to restart");
                    return;
                } else {
                    warn!(
                        "rp2040 hasn't started a new recording {}s after being restarted",
                        RESTART_WAIT.as_secs()
                    );
                    *waiting_for_restart = None;
                }
            }
            *received_end = is_last_part;
            if *frame_i == 0 {
                if file_download.is_some() || *last_packet_num != 255 || *sent_end {
                    info!("Reset file download as have new recording");
                }
                *file_download = None;
                *last_packet_num = 255;
                *sent_end = false;
            }
            let mut was_sent = false;
            if packet_num == *last_packet_num {
                // could happen if rp2040 thinks we didnt receive the last packet
                warn!("Received the same packet twice ignoring the second one");
                return;
            }

            *last_packet_num = packet_num;
            // The first packet starts with the timestamp, serial, firmware and file start block
            if *frame_i == 0 && !is_first_packet {
                // ensure is a gzip
                error!(
                    "New file packet {packet_num} is missing the GZIP header {:?} restart rp2040",
                    &frame_data[..frame_data.len().min(FIRST_PACKET_HEADER_LENGTH + 10)]
                );
                restart_rp2040(run_pin);
                *waiting_for_restart = Some(Instant::now());
                // what do we do here???
                // force rp2040 to offload last file??
                // if we could send a message ask for the start again
                return;
            }
            if *frame_i == 0 {
                let timestamp = LittleEndian::read_u64(&frame_data[0..8]);
                info!(
                    "Started streaming recording {timestamp}, first packet crc ok {}",
                    file_info.crc_ok
                );
                if let Some(previous) = streamed_recording.as_ref() {
                    warn!(
                        "Streamed recording {} never finished sending, it will be offloaded \
                        again from the rp2040 during normal offloading",
                        previous.timestamp
                    );
                }
                *streamed_recording =
                    Some(StreamedRecording { timestamp, intact: file_info.crc_ok });
            } else if let Some(recording) = streamed_recording.as_mut() {
                if !file_info.crc_ok {
                    warn!(
                        "Streamed recording {} packet {packet_num} failed its crc check",
                        recording.timestamp
                    );
                }
                recording.intact &= file_info.crc_ok;
            } else {
                warn!("Got packet {packet_num} of a recording that isn't being tracked");
            }
            *frame_i += 1;

            let socket = sockets
                .iter_mut()
                .find(|(sock_address, _, stream)| stream.is_some() && sock_address == address);
            if let Some(sock) = socket {
                was_sent = handle_medium_power(
                    sock,
                    &true,
                    firmware_version,
                    camera_serial,
                    file_download,
                    ms_elapsed,
                    Some(frame_data),
                    is_last_part,
                    *frame_i == 1,
                    is_last_part || *frame_i % CLIENT_POLL_INTERVAL == 0,
                    streamed_recording,
                    awaiting_done,
                );
                if is_last_part && was_sent {
                    *sent_end = true;
                    *frame_i = 0;
                    streamed_recording_sent(streamed_recording, awaiting_done);
                } else if is_last_part {
                    info!("Failed to send the last packet of a streamed recording");
                }
            } else if is_last_part {
                info!("Got the last packet of a streamed recording, but no client is connected");
            }
            // store file to send later
            if !was_sent {
                if let Some(file) = file_download.as_mut() {
                    info!("Adding bytes {} to memory file", frame_bytes);
                    file.extend_from_slice(frame_data);
                } else if *frame_i > 1 {
                    error!(
                        "Lost socket connection part way through medium power offload (Something must have gone wrong)"
                    );
                    // restart_rp2040(run_pin);
                    *frame_i = 0;
                } else {
                    info!("Starting new file");
                    let mut file: Vec<u8> = Vec::with_capacity(50_000_000);
                    file.extend_from_slice(frame_data);
                    *file_download = Some(file);
                }
            }
        } else if !*sent_end && (*frame_i > 0 || file_download.is_some()) {
            // have a frame message and not recording so need to finish any open recordings, generally this is done as soon as we receive the last packet
            // but there are some edge cases (abort and socket wasnt connect yet)
            let socket = sockets
                .iter_mut()
                .find(|(sock_address, _, stream)| stream.is_some() && sock_address == address);
            if let Some(sock) = socket {
                if *received_end && file_download.is_some() {
                    // handles edge case that the whole recording was received and is in file_download before anything was sent for processing
                    // very unlikely as generally the python code is waiting prior to tc2-agent starting
                    if handle_medium_power(
                        sock,
                        &true, //we may not have received the radiometry info since we don't always handshake first, so just say its on always
                        firmware_version,
                        camera_serial,
                        file_download,
                        ms_elapsed,
                        None,
                        true,
                        true,
                        true,
                        streamed_recording,
                        awaiting_done,
                    ) {
                        *sent_end = true;
                        streamed_recording_sent(streamed_recording, awaiting_done);
                    }
                } else {
                    // send abort when recording was discarded on rp2040
                    if let Some(recording) = streamed_recording.as_ref() {
                        info!(
                            "Streamed recording {} was discarded on the rp2040, sending abort",
                            recording.timestamp
                        );
                    }
                    send_abort(sock);
                    *file_download = None;
                    *sent_end = true;
                    *streamed_recording = None;
                }
            }
            *frame_i = 0;
        } else {
            // always want to send the header
            let socket = sockets.iter_mut().find(|(sock_address, _, stream)| {
                sock_address == address && stream.as_ref().is_some_and(|s| !s.sent_header)
            });
            if let Some((_, _, option_stream)) = socket {
                let stream =
                    option_stream.as_mut().expect("Never fails, because we filtered already.");
                if !send_header(stream, true, *firmware_version, camera_serial) {
                    info!("Shutting down socket");
                    let _ = option_stream.take().expect("Never fails").shutdown().is_ok();
                }
            }
        }
    }
}

pub fn spawn_frame_socket_server_thread(
    restart_rp2040_channel_rx: Receiver<bool>,
    camera_handshake_channel_rx: Receiver<FrameSocketServerMessage>,
    serve_frames_via_wifi: bool,
    mut run_pin: OutputPin,
    recording_state: &RecordingState,
    medium_power_mode_flag: Arc<AtomicBool>,
) {
    let recording_state = recording_state.clone();
    let _ = thread::Builder::new().name("frame-socket".to_string()).spawn_with_priority(
        ThreadPriority::Max,
        move |result| {
            info!("Frame socket started");

            let mut file_download: Option<Vec<u8>> = None;
            let gzip_header: [u8; 10] =
                [0x1f, 0x8b, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff];
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

            restart_rp2040_if_requested(&restart_rp2040_channel_rx, &mut run_pin);
            let mut recv_timeout_ms = 10;
            info!("Connecting to frame sockets");
            let mut ms_elapsed = 0;
            let mut last_packet_num = 255u8;
            let mut sent_end = false;
            let mut message_handled = false;
            let mut received_end = false;
            let mut streamed_recording: Option<StreamedRecording> = None;
            // Timestamps of intact streamed recordings, oldest first, that the client hasn't yet
            // said it finished.
            let mut awaiting_done: VecDeque<u64> = VecDeque::new();
            // When the rp2040 was restarted because a recording didn't start properly, until it
            // starts sending a new one.
            let mut waiting_for_restart: Option<Instant> = None;
            let address = &get_socket_address(serve_frames_via_wifi);
            let mut medium_power_mode = medium_power_mode_flag.load(Ordering::Relaxed);
            loop {
                restart_rp2040_if_requested(&restart_rp2040_channel_rx, &mut run_pin);
                let new_medium_power_mode = medium_power_mode_flag.load(Ordering::Relaxed);
                if new_medium_power_mode != medium_power_mode {
                    info!("Switching medium power mode to {new_medium_power_mode}");
                    if !sent_end && frame_i > 0 && file_download.is_none() {
                        // A recording was part way through being streamed, so tell the
                        // consumer to discard what it has received so far. If it's still
                        // buffered in file_download nothing has been sent yet.
                        if let Some(sock) = sockets.iter_mut().find(|(sock_address, _, stream)| {
                            stream.is_some() && sock_address == address
                        }) {
                            send_abort(sock);
                        }
                    }
                    if let Some(file) = file_download.take() {
                        warn!("Discarding unsent medium power recording with {} bytes", file.len());
                    }
                    clear_awaiting_done(&mut awaiting_done, "Medium power mode changed");
                    medium_power_mode = new_medium_power_mode;
                    waiting_for_restart = None;
                    frame_i = 0;
                    last_packet_num = 255;
                    sent_end = false;
                    received_end = false;
                    message_handled = false;
                }
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
                if medium_power_mode {
                    handle_medium_power_message(
                        &message,
                        &mut sockets,
                        address,
                        &gzip_header,
                        &mut file_download,
                        &mut frame_i,
                        &mut last_packet_num,
                        &mut sent_end,
                        &mut received_end,
                        &mut ms_elapsed,
                        &mut run_pin,
                        &mut message_handled,
                        &mut streamed_recording,
                        &mut awaiting_done,
                        &mut waiting_for_restart,
                    );
                    // Keep listening for the client to finish files after the last one is sent.
                    if !awaiting_done.is_empty()
                        && let Some((_, _, option_stream)) =
                            sockets.iter_mut().find(|(sock_address, _, _)| sock_address == address)
                    {
                        match option_stream {
                            Some(stream) => {
                                if !poll_medium_power_client(
                                    stream,
                                    &mut streamed_recording,
                                    &mut awaiting_done,
                                ) {
                                    info!("Shutting down socket");
                                    let _ = option_stream
                                        .take()
                                        .expect("Never fails")
                                        .shutdown()
                                        .is_ok();
                                }
                            }
                            None => clear_awaiting_done(&mut awaiting_done, "Client disconnected"),
                        }
                    }
                }
                if !message_handled {
                    // dont send normal frame messages to medium power socket, we may want to change this and send the message type
                    let sub_sockets: Vec<&mut (String, bool, Option<SocketStream>)> = sockets
                        .iter_mut()
                        .filter(|(sock_address, _, stream)| {
                            stream.is_some() && (!medium_power_mode || sock_address != address)
                        })
                        .collect();
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
            for (_, _, stream) in sockets.iter_mut().filter(|(_, use_wifi, stream)| {
                stream.is_some() && !use_wifi && !stream.as_ref().unwrap().sent_header
            }) {
                let stream = stream.as_mut().expect("Never fails, because we filtered already.");
                send_header(stream, radiometry_enabled, firmware_version, &camera_serial);
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

/// Sends the camera header followed by a "clear" to the stream.
/// Returns false if any write or the flush fails; `sent_header` is only set on success.
fn send_header(
    stream: &mut SocketStream,
    radiometry_enabled: bool,
    firmware_version: u32,
    camera_serial: &str,
) -> bool {
    info!("Sending header");
    let model = if radiometry_enabled { "lepton3.5" } else { "lepton3" };
    let header = format!(
        "ResX: 160\n\
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
        return false;
    }
    // Clear existing
    if stream.write_all(b"clear").is_err() {
        warn!("Failed clearing buffer");
        return false;
    }
    if stream.flush().is_err() {
        return false;
    }
    stream.sent_header = true;
    true
}

fn send_abort(socket: &mut (String, bool, Option<SocketStream>)) -> bool {
    info!("Aborted recording");
    let (_, _, stream) = socket;
    let stream = stream.as_mut().expect("Never fails, because we filtered already.");

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
    camera_serial: &str,
    file_download: &mut Option<Vec<u8>>,
    ms_elapsed: &mut u64,
    frame_data: Option<&[u8]>,
    is_last_part: bool,
    first_part: bool,
    poll_client: bool,
    streamed_recording: &mut Option<StreamedRecording>,
    awaiting_done: &mut VecDeque<u64>,
) -> bool {
    let (address, use_wifi, option_stream) = socket;
    let stream = option_stream.as_mut().expect("Never fails, because we filtered already.");
    if !stream.sent_header
        && !send_header(stream, *radiometry_enabled, *firmware_version, camera_serial)
    {
        info!("Shutting down socket");
        let _ = option_stream.take().expect("Never fails").shutdown().is_ok();
        return false;
    }
    let s = Instant::now();

    if poll_client && !poll_medium_power_client(stream, streamed_recording, awaiting_done) {
        info!("Shutting down socket");
        let _ = option_stream.take().expect("Never fails").shutdown().is_ok();
        return false;
    }

    if first_part || file_download.is_some() {
        info!("Sending start");
        if stream.write_all(b"start\n\n").is_err() {
            warn!("Failed sending start rec");
        }
        let sent: bool = stream.flush().is_ok();
        if !sent {
            info!("Shutting down socket");
            let _ = option_stream.take().expect("Never fails").shutdown().is_ok();
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
                let _ = option_stream.take().expect("Never fails").shutdown().is_ok();
                return false;
            }
        }
        // only remove if sent all
        file_download.take();
        info!("Sent all of file download");
    }

    if let Some(fb) = frame_data {
        let sent = cptv_frame_dispatch::send_frame(fb, stream);
        if !sent {
            warn!(
                "Medium Power Send to {} failed",
                if *use_wifi { "tc2-frames server" } else { address }
            );
            let _ = option_stream.take().expect("Never fails").shutdown().is_ok();
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn taking_a_recording_drops_the_unanswered_ones_before_it() {
        let mut awaiting_done = VecDeque::from([1, 2, 3, 4]);
        assert!(take_awaiting_done(&mut awaiting_done, 3));
        assert_eq!(awaiting_done, [4]);
    }

    #[test]
    fn taking_an_unknown_recording_changes_nothing() {
        let mut awaiting_done = VecDeque::from([1, 2]);
        assert!(!take_awaiting_done(&mut awaiting_done, 5));
        assert_eq!(awaiting_done, [1, 2]);
    }
}
