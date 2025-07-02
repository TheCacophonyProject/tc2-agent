use crate::RecordingState;
use log::error;
use rustbus::connection::Timeout;
use rustbus::connection::dispatch_conn::{HandleEnvironment, HandleResult, Matches};
use rustbus::message_builder::MarshalledMessage;
use rustbus::{DispatchConn, DuplexConn, get_system_bus_path};
use std::thread::JoinHandle;
use std::{process, thread};
use thread_priority::{ThreadBuilderExt, ThreadPriority};

// TC2-Agent dbus managementd service
type MyHandleEnv<'a, 'b> = HandleEnvironment<RecordingState, ()>;

fn default_handler(
    _recording_state_ctx: &mut RecordingState,
    _matches: Matches,
    _msg: &MarshalledMessage,
    _env: &mut MyHandleEnv,
) -> HandleResult<()> {
    Ok(None)
}

fn managementd_handler(
    recording_state_ctx: &mut RecordingState,
    _matches: Matches,
    msg: &MarshalledMessage,
    _env: &mut MyHandleEnv,
) -> HandleResult<()> {
    let cmd = msg.dynheader.member.as_ref().unwrap();
    match cmd.as_str() {
        "testaudio" => {
            let message = if recording_state_ctx.is_taking_test_audio_recording() {
                "Already making a test recording"
            } else if recording_state_ctx.is_taking_long_audio_recording() {
                "Already making a 5 minute recording"
            } else {
                recording_state_ctx.request_test_audio_recording();
                "Asked for a test recording"
            };
            let mut resp = msg.dynheader.make_response();
            resp.body.push_param(message)?;
            Ok(Some(resp))
        }
        "longaudiorecording" => {
            let message = if recording_state_ctx.is_taking_long_audio_recording() {
                "Already making a 5 minute recording"
            } else if recording_state_ctx.is_taking_test_audio_recording() {
                "Already making a 5 test recording"
            } else {
                recording_state_ctx.request_long_audio_recording();
                "Asked for a 5 minute recording"
            };
            let mut resp = msg.dynheader.make_response();
            resp.body.push_param(message)?;
            Ok(Some(resp))
        }
        "audiostatus" => {
            let mut response = msg.dynheader.make_response();
            let status = recording_state_ctx.get_audio_status();
            response.body.push_param(if recording_state_ctx.is_in_audio_mode() { 1 } else { 0 })?;
            response.body.push_param(status as u8)?;
            Ok(Some(response))
        }
        "offloadstatus" => {
            let mut response = msg.dynheader.make_response();
            let (
                is_offloading,
                percent_complete,
                remaining_seconds,
                total_files,
                remaining_files,
                total_events,
                remaining_events,
            ) = recording_state_ctx.get_offload_status();
            response.body.push_param(if is_offloading { 1 } else { 0 })?; // offload in progress
            response.body.push_param(percent_complete)?; // percent_complete
            response.body.push_param(remaining_seconds)?; // remaining_seconds
            response.body.push_param(total_files)?; // total files
            response.body.push_param(remaining_files)?; // remaining files
            response.body.push_param(total_events)?; // total files
            response.body.push_param(remaining_events)?; // remaining files

            Ok(Some(response))
        }
        "canceloffload" => {
            let mut response = msg.dynheader.make_response();
            Ok(Some(response))
        }
        _ => Ok(None),
    }
}
#[derive(PartialEq)]
pub enum AudioStatus {
    Ready = 1,
    WaitingToTakeTestRecording = 2,
    TakingTestRecording = 3,
    Recording = 4,
    TakingLongRecording = 5,
    WaitingToTakeLongRecording = 6,
}

pub fn setup_dbus_managementd_recording_service(
    recording_state: &RecordingState,
) -> std::io::Result<JoinHandle<()>> {
    // set up dbus service for handling messages between managementd and tc2-agent about when
    // to make test audio recordings, and for getting the status of any file offloads
    let recording_state = recording_state.clone();
    let session_path = get_system_bus_path().unwrap();
    thread::Builder::new().name("dbus-managementd-service".to_string()).spawn_with_priority(
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
                .send_message(&rustbus::standard_messages::request_name(
                    "org.cacophony.TC2Agent",
                    rustbus::standard_messages::DBUS_NAME_FLAG_REPLACE_EXISTING,
                ))
                .unwrap()
                .write_all()
                .unwrap();

            let mut dispatch_conn =
                DispatchConn::new(dbus_conn, recording_state, Box::new(default_handler));
            dispatch_conn.add_handler("/org/cacophony/TC2Agent", Box::new(managementd_handler));
            dispatch_conn.run().unwrap();
        },
    )
}
