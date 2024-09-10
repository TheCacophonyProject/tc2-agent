use crate::tc2_agent_state;
use byteorder::{BigEndian, ByteOrder};
use crc::{Algorithm, Crc};
use rustbus::connection::Timeout;
use rustbus::{DuplexConn, MessageBuilder, MessageType};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::Duration;

const CRC_AUG_CCITT: Algorithm<u16> = Algorithm {
    width: 16,
    poly: 0x1021,
    init: 0x1D0F,
    refin: false,
    refout: false,
    xorout: 0x0000,
    check: 0x0000,
    residue: 0x0000,
};

pub fn dbus_read_attiny_command(conn: &mut DuplexConn, command: u8) -> Result<u8, &'static str> {
    dbus_attiny_command(conn, command, None)
}

pub fn dbus_write_attiny_command(
    conn: &mut DuplexConn,
    command: u8,
    value: u8,
) -> Result<u8, &'static str> {
    dbus_attiny_command(conn, command, Some(value))
}

pub fn dbus_attiny_command(
    conn: &mut DuplexConn,
    command: u8,
    value: Option<u8>,
) -> Result<u8, &'static str> {
    let max_attempts = 10;
    let retry_delay = std::time::Duration::from_secs(2);

    for attempt in 1..=max_attempts {
        match dbus_attiny_command_attempt(conn, command, value) {
            Ok(result) => return Ok(result),
            Err(e) => {
                if attempt == max_attempts {
                    return Err("Max attempts reached: failed to execute i2c dbus command");
                }
                eprintln!(
                    "Attempt {}/{} failed: {}. Retrying in {:?}...",
                    attempt, max_attempts, e, retry_delay
                );
                std::thread::sleep(retry_delay);
            }
        }
    }
    Err("Failed to execute dbus command after maximum retries")
}

pub fn dbus_attiny_command_attempt(
    conn: &mut DuplexConn,
    command: u8,
    value: Option<u8>,
) -> Result<u8, &'static str> {
    let is_read_command = value.is_none();
    let mut payload: Vec<u8> = vec![command];
    if let Some(value) = value {
        payload.push(value);
    }
    let crc = Crc::<u16>::new(&CRC_AUG_CCITT).checksum(&payload);
    payload.push(0);
    payload.push(0);
    let len = payload.len();
    BigEndian::write_u16(&mut payload[len - 2..], crc);
    let mut call = MessageBuilder::new()
        .call("Tx")
        .with_interface("org.cacophony.i2c")
        .on("/org/cacophony/i2c")
        .at("org.cacophony.i2c")
        .build();
    call.body.push_param(0x25u8).unwrap();
    call.body.push_param(payload).unwrap();
    if is_read_command {
        call.body.push_param(3i32).unwrap(); // Num bytes to receive including CRC validation code
    } else {
        call.body.push_param(0i32).unwrap(); // Receive nothing if this was a write.
    }
    call.body.push_param(1000i32).unwrap(); // Timeout ms
    let id = conn.send.send_message(&call).unwrap().write_all().unwrap();
    let mut attempts = 0;
    // Now wait for the reply that matches our call id
    loop {
        if let Ok(message) = conn
            .recv
            .get_next_message(Timeout::Duration(Duration::from_millis(10)))
        {
            match message.typ {
                MessageType::Reply => {
                    let reply_id = message.dynheader.response_serial.unwrap();
                    if reply_id == id {
                        // Looks like the first 4 bytes are a u32 with the length of the following bytes
                        // which can be ignored.
                        return if is_read_command {
                            let response = &message.get_buf()[4..][0..3];
                            let crc = Crc::<u16>::new(&CRC_AUG_CCITT).checksum(&response[0..1]);
                            let received_crc = BigEndian::read_u16(&response[1..=2]);
                            if received_crc != crc {
                                Err("CRC Mismatch")
                            } else {
                                Ok(response[0])
                            }
                        } else {
                            // Check that the written value was actually written correctly.
                            let set_value = dbus_attiny_command(conn, command, None);
                            match set_value {
                                Ok(new_value) => {
                                    if new_value != value.unwrap() {
                                        Err("Failed setting state on attiny")
                                    } else {
                                        Ok(0)
                                    }
                                }
                                Err(e) => Err(e),
                            }
                        };
                    }
                }
                MessageType::Error => {
                    let reply_id = message.dynheader.response_serial.unwrap();
                    if reply_id == id {
                        return Err("Dbus error");
                    }
                }
                _ => {}
            }
        }
        attempts += 1;
        if attempts == 100 {
            return Err("Timed out waiting for response from Dbus service");
        }
    }
}

pub fn exit_cleanly(conn: &mut DuplexConn) {
    let _ = dbus_write_attiny_command(conn, 0x07, 0x00);
}

pub fn process_interrupted(term: &Arc<AtomicBool>, conn: &mut DuplexConn) -> bool {
    if term.load(Ordering::Relaxed) {
        // We got terminated - before we exit, clean up the tc2-agent ready state register
        exit_cleanly(conn);
        true
    } else {
        false
    }
}

pub fn read_tc2_agent_state(conn: &mut DuplexConn) -> Result<u8, &'static str> {
    dbus_read_attiny_command(conn, 0x07)
}

pub fn read_attiny_recording_flag(conn: &mut DuplexConn) -> bool {
    read_tc2_agent_state(conn).map_or(false, |x| {
        x & tc2_agent_state::RECORDING == tc2_agent_state::RECORDING
    })
}
pub fn safe_to_restart_rp2040(conn: &mut DuplexConn) -> bool {
    !read_attiny_recording_flag(conn)
}

pub fn read_attiny_firmware_version(conn: &mut DuplexConn) -> Result<u8, &'static str> {
    dbus_read_attiny_command(conn, 0x01)
}

pub fn set_attiny_tc2_agent_test_audio_rec(conn: &mut DuplexConn) -> Result<u8, &'static str> {
    let state = read_tc2_agent_state(conn);
    if let Ok(state) = state {
        if (state & tc2_agent_state::RECORDING) == tc2_agent_state::RECORDING {
            Err("Already recording so not doing test rec")
        } else {
            let res = dbus_write_attiny_command(
                conn,
                0x07,
                state | tc2_agent_state::TEST_AUDIO_RECORDING,
            );
            if res.is_ok() {
                Ok(state | tc2_agent_state::TEST_AUDIO_RECORDING)
            } else {
                Err(res.unwrap_err())
            }
        }
    } else {
        Err("Failed reading ready state from attiny")
    }
}

pub fn set_attiny_tc2_agent_ready(conn: &mut DuplexConn) -> Result<(), &'static str> {
    let state = read_tc2_agent_state(conn);
    if let Ok(state) = state {
        dbus_write_attiny_command(conn, 0x07, state | tc2_agent_state::READY).map(|_| ())
    } else {
        Err("Failed reading ready state from attiny")
    }
}