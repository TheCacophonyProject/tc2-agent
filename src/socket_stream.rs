use byteorder::{ByteOrder, LittleEndian};
use log::info;
use std::io;
use std::io::{ErrorKind, Read, Write};
use std::net::{Shutdown, TcpStream};
use std::os::unix::net::UnixStream;
use std::time::Duration;

/// Status messages the client sends back to us. Each starts with a 3-byte command. "DON" and
/// "FAI" are followed by the recording's timestamp (u64 LE), as sent in its first packet.
#[derive(Debug, PartialEq)]
pub enum ClientMsg {
    Ack,
    // The client failed to process this recording.
    Fail(u64),
    // The client finished this recording.
    Done(u64),
    Unknown([u8; 3]),
}

const CLIENT_CMD_LEN: usize = 3;
const CLIENT_TIMESTAMP_LEN: usize = 8;

/// The 3-byte command at the start of each client message.
enum ClientCmd {
    Ack,
    Done,
    Fail,
}

impl ClientCmd {
    /// Bytes after the command.
    fn args_len(&self) -> usize {
        match self {
            ClientCmd::Ack => 0,
            ClientCmd::Done | ClientCmd::Fail => CLIENT_TIMESTAMP_LEN,
        }
    }
}

impl TryFrom<&[u8; CLIENT_CMD_LEN]> for ClientCmd {
    // The bytes, if they aren't a command we know.
    type Error = [u8; CLIENT_CMD_LEN];

    fn try_from(bytes: &[u8; CLIENT_CMD_LEN]) -> Result<Self, Self::Error> {
        match bytes {
            b"ACK" => Ok(ClientCmd::Ack),
            b"DON" => Ok(ClientCmd::Done),
            b"FAI" => Ok(ClientCmd::Fail),
            other => Err(*other),
        }
    }
}

/// Takes every complete message from the start of `rx_buf`, leaving any partial one there until
/// the rest arrives.
fn parse_client_msgs(rx_buf: &mut Vec<u8>) -> Vec<ClientMsg> {
    let mut msgs = Vec::new();
    let mut offset = 0;
    while rx_buf.len() - offset >= CLIENT_CMD_LEN {
        let cmd_bytes: &[u8; CLIENT_CMD_LEN] =
            rx_buf[offset..offset + CLIENT_CMD_LEN].try_into().unwrap();
        let cmd = match ClientCmd::try_from(cmd_bytes) {
            Ok(cmd) => cmd,
            Err(bytes) => {
                msgs.push(ClientMsg::Unknown(bytes));
                offset += CLIENT_CMD_LEN;
                continue;
            }
        };
        let end = offset + CLIENT_CMD_LEN + cmd.args_len();
        if rx_buf.len() < end {
            break;
        }
        let args = &rx_buf[offset + CLIENT_CMD_LEN..end];
        msgs.push(match cmd {
            ClientCmd::Ack => ClientMsg::Ack,
            ClientCmd::Done => ClientMsg::Done(LittleEndian::read_u64(args)),
            ClientCmd::Fail => ClientMsg::Fail(LittleEndian::read_u64(args)),
        });
        offset = end;
    }
    rx_buf.drain(..offset);
    msgs
}

pub struct SocketStream {
    unix: Option<UnixStream>,
    tcp: Option<TcpStream>,
    pub sent_header: bool,
    // Bytes received from the client that don't yet make up a full command.
    rx_buf: Vec<u8>,
}

impl SocketStream {
    pub fn from_address(address: &str, use_wifi: bool) -> io::Result<SocketStream> {
        if !use_wifi {
            UnixStream::connect(address).map(|stream| SocketStream {
                unix: Some(stream),
                tcp: None,
                sent_header: false,
                rx_buf: Vec::new(),
            })
        } else {
            TcpStream::connect(address).map(|stream| {
                stream.set_nodelay(true).unwrap();
                stream.set_write_timeout(Some(Duration::from_millis(1500))).unwrap();
                SocketStream {
                    unix: None,
                    tcp: Some(stream),
                    sent_header: false,
                    rx_buf: Vec::new(),
                }
            })
        }
    }

    pub(crate) fn flush(&mut self) -> io::Result<()> {
        match &mut self.tcp {
            Some(stream) => stream.flush(),
            None => match &mut self.unix {
                Some(stream) => stream.flush(),
                None => unreachable!("Must have stream"),
            },
        }
    }

    pub(crate) fn write_all(&mut self, bytes: &[u8]) -> io::Result<()> {
        match &mut self.tcp {
            Some(stream) => stream.write_all(bytes),
            None => match &mut self.unix {
                Some(stream) => stream.write_all(bytes),
                None => unreachable!("Must have stream"),
            },
        }
    }

    fn read_nonblocking(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        match (&mut self.tcp, &mut self.unix) {
            (Some(stream), _) => {
                stream.set_nonblocking(true)?;
                let r = stream.read(buf);
                stream.set_nonblocking(false)?;
                r
            }
            (None, Some(stream)) => {
                stream.set_nonblocking(true)?;
                let r = stream.read(buf);
                stream.set_nonblocking(false)?;
                r
            }
            _ => unreachable!("Must have stream"),
        }
    }

    /// Drain everything the client has sent without blocking, returning any complete commands.
    /// Partial commands are kept until the rest arrives.
    /// Err means the client closed the connection or it is broken.
    pub(crate) fn poll_client(&mut self) -> io::Result<Vec<ClientMsg>> {
        let mut buf = [0u8; 64];
        loop {
            match self.read_nonblocking(&mut buf) {
                Ok(0) => {
                    return Err(io::Error::new(ErrorKind::UnexpectedEof, "client closed socket"));
                }
                Ok(n) => self.rx_buf.extend_from_slice(&buf[..n]),
                Err(e) if e.kind() == ErrorKind::WouldBlock => break,
                Err(e) if e.kind() == ErrorKind::Interrupted => continue,
                Err(e) => return Err(e),
            }
        }
        let msgs = parse_client_msgs(&mut self.rx_buf);
        Ok(msgs)
    }

    pub(crate) fn shutdown(&mut self) -> io::Result<()> {
        match &mut self.tcp {
            Some(stream) => stream.shutdown(Shutdown::Both),
            None => match &mut self.unix {
                Some(stream) => stream.shutdown(Shutdown::Both),
                None => unreachable!("Must have stream"),
            },
        }
    }
}

pub fn get_socket_address(serve_frames_via_wifi: bool) -> String {
    // Find the socket address
    let address = if serve_frames_via_wifi {
        // Scan for servers on port 34254.
        use mdns_sd::{ServiceDaemon, ServiceEvent};
        // Create a daemon
        let mdns = ServiceDaemon::new().expect("Failed to create daemon");

        // Browse for a service type.
        let service_type = "_mdns-tc2-frames._udp.local.";
        let receiver = mdns.browse(service_type).expect("Failed to browse");
        let address;
        info!(
            "Trying to resolve tc2-frames service, please ensure t2c-frames app is running on the same network"
        );
        'service_finder: loop {
            while let Ok(event) = receiver.recv() {
                if let ServiceEvent::ServiceResolved(info) = event
                    && let Some(add) = info.get_addresses().iter().next()
                {
                    address = Some(add.to_string());
                    info!("Resolved a tc2-frames service at: {add:?}");
                    break 'service_finder;
                }
            }
        }
        address
    } else {
        Some("/var/run/lepton-frames".to_string())
    };
    if serve_frames_via_wifi && address.is_none() {
        panic!("t2c-frames service not found on local network");
    }
    let address = address.unwrap_or_default();
    if serve_frames_via_wifi { format!("{address}:34254") } else { address }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn with_timestamp(cmd: &[u8], timestamp: u64) -> Vec<u8> {
        let mut msg = cmd.to_vec();
        msg.extend_from_slice(&timestamp.to_le_bytes());
        msg
    }

    #[test]
    fn parses_messages_with_and_without_timestamps() {
        let mut rx_buf = b"ACK".to_vec();
        rx_buf.extend(with_timestamp(b"DON", 1758607259530000));
        rx_buf.extend(with_timestamp(b"FAI", 42));
        rx_buf.extend(b"XYZ");
        assert_eq!(
            parse_client_msgs(&mut rx_buf),
            vec![
                ClientMsg::Ack,
                ClientMsg::Done(1758607259530000),
                ClientMsg::Fail(42),
                ClientMsg::Unknown(*b"XYZ"),
            ]
        );
        assert!(rx_buf.is_empty());
    }

    #[test]
    fn keeps_partial_message_until_the_rest_arrives() {
        let done = with_timestamp(b"DON", 1758607259530000);
        let mut rx_buf = b"ACK".to_vec();
        rx_buf.extend(&done[..7]);
        assert_eq!(parse_client_msgs(&mut rx_buf), vec![ClientMsg::Ack]);
        assert_eq!(rx_buf, done[..7]);
        rx_buf.extend(&done[7..]);
        assert_eq!(parse_client_msgs(&mut rx_buf), vec![ClientMsg::Done(1758607259530000)]);
        assert!(rx_buf.is_empty());
    }
}
