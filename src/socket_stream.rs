use crate::ModeConfig;
use log::info;
use std::io;
use std::io::Write;
use std::net::{Shutdown, TcpStream};
use std::os::unix::net::UnixStream;
use std::time::Duration;

pub struct SocketStream {
    unix: Option<UnixStream>,
    tcp: Option<TcpStream>,
    pub sent_header: bool,
}

impl SocketStream {
    pub fn from_address(address: &str, use_wifi: bool) -> io::Result<SocketStream> {
        if !use_wifi {
            UnixStream::connect(address).map(|stream| SocketStream {
                unix: Some(stream),
                tcp: None,
                sent_header: false,
            })
        } else {
            TcpStream::connect(address).map(|stream| {
                stream.set_nodelay(true).unwrap();
                stream
                    .set_write_timeout(Some(Duration::from_millis(1500)))
                    .unwrap();
                SocketStream {
                    unix: None,
                    tcp: Some(stream),
                    sent_header: false,
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

pub fn get_socket_address(config: &ModeConfig) -> String {
    let address = {
        // Find the socket address
        let address = if config.use_wifi {
            // Scan for servers on port 34254.
            use mdns_sd::{ServiceDaemon, ServiceEvent};
            // Create a daemon
            let mdns = ServiceDaemon::new().expect("Failed to create daemon");

            // Browse for a service type.
            let service_type = "_mdns-tc2-frames._udp.local.";
            let receiver = mdns.browse(service_type).expect("Failed to browse");
            let address;
            info!("Trying to resolve tc2-frames service, please ensure t2c-frames app is running on the same network");
            'service_finder: loop {
                while let Ok(event) = receiver.recv() {
                    match event {
                        ServiceEvent::ServiceResolved(info) => {
                            for add in info.get_addresses().iter() {
                                address = Some(add.to_string());
                                info!("Resolved a tc2-frames service at: {:?}", add);
                                break 'service_finder;
                            }
                        }
                        _ => {}
                    }
                }
            }
            address
        } else {
            Some("/var/run/lepton-frames".to_string())
        };
        if config.use_wifi && address.is_none() {
            panic!("t2c-frames service not found on local network");
        }
        let address = if config.use_wifi {
            format!("{}:34254", address.unwrap())
        } else {
            address.unwrap()
        };
        address
    };
    address
}
