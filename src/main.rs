use std::{time::{Duration}, thread, cell::RefCell, sync::Mutex, io};
use std::io::{Write};
use std::net::{Shutdown, TcpStream};
use std::os::unix::net::UnixStream;
use std::sync::atomic::{AtomicUsize, Ordering};
use rppal::{spi::{Bus, Mode, SlaveSelect, Spi, Polarity}, gpio::{Gpio, Trigger}};
use std::sync::mpsc::channel;
use std::thread::sleep;
use std::time::Instant;
use rppal::spi::BitOrder;
use byteorder::{LittleEndian, ByteOrder, BigEndian};
use thread_priority::*;
use thread_priority::ThreadBuilderExt;
use argh::FromArgs;
use std::fs;

use log::{info, warn};
use simplelog::*;
use crc::{Crc, CRC_16_XMODEM};

const SEGMENT_LENGTH: usize = 9760;
const CHUNK_LENGTH: usize = SEGMENT_LENGTH / 4;
const FRAME_LENGTH: usize = SEGMENT_LENGTH * 4;
type Frame = [u8; FRAME_LENGTH];
pub static FRAME_BUFFER: DoubleBuffer = DoubleBuffer {
    front: Mutex::new(RefCell::new(None)),
    back: Mutex::new(RefCell::new(None)),
    swapper: AtomicUsize::new(0),
};

pub struct DoubleBuffer {
    pub front: Mutex<RefCell<Option<Frame>>>,
    pub back: Mutex<RefCell<Option<Frame>>>,
    swapper: AtomicUsize,
}

impl DoubleBuffer {
    pub fn swap(&self) {
        self.swapper.fetch_add(1, Ordering::Relaxed);
    }

    pub fn get_front(&self) -> &Mutex<RefCell<Option<Frame>>> {
        let val = self.swapper.load(Ordering::Acquire);
        if val % 2 == 0 {
            &self.front
        } else {
            &self.back
        }
    }

    pub fn get_back(&self) -> &Mutex<RefCell<Option<Frame>>> {
        let val = self.swapper.load(Ordering::Acquire);
        if val % 2 == 0 {
            &self.back
        } else {
            &self.front
        }
    }
}
pub unsafe fn u8_as_u16_slice(p: &[u8]) -> &[u16] {
    core::slice::from_raw_parts((p as *const [u8]) as *const u16, p.len() / 2)
}

struct Telemetry {
    frame_num: u32,
    msec_on: u32,
    ffc_in_progress: bool,
    msec_since_last_ffc: u32
}

fn read_telemetry(frame: &Frame) -> Telemetry {
    let mut buf = [0u8;160];
    BigEndian::write_u16_into(unsafe  { u8_as_u16_slice(&frame[0..160]) }, &mut buf);
    let frame_num = LittleEndian::read_u32(&buf[40..44]);
    let msec_on = LittleEndian::read_u32(&buf[2..6]);
    let time_at_last_ffc = LittleEndian::read_u32(&buf[60..64]);
    let msec_since_last_ffc = msec_on - time_at_last_ffc;
    let status_bits = LittleEndian::read_u32(&buf[6..10]);
    let ffc_state = (status_bits >> 4) & 0b11;
    let ffc_in_progress = ffc_state == 0b10;
    Telemetry {
        frame_num,
        msec_on,
        ffc_in_progress,
        msec_since_last_ffc
    }
}

fn send_frame(stream: &mut SocketStream) -> (Option<Telemetry>, bool) {
    let fb = {
        FRAME_BUFFER.get_front().lock().unwrap().take()
    };
    if let Some(fb) = fb {
        // Make sure there are no zero/bad pixels:
        let pixels = unsafe { u8_as_u16_slice(&fb[640..]) };
        let pos = pixels.iter().position(|&px| px == 0);
        let mut found_bad_pixels = false;
        if let Some(_pos) = pos {
            // let y = pos / 160;
            // let x = pos - (y * 160);
            // println!("Zero px found at ({}, {}) not sending", x, y);
            found_bad_pixels = true;
        }
        // Make sure each frame number is ascending correctly.
        let telemetry = read_telemetry(&fb);
        if !telemetry.ffc_in_progress && !found_bad_pixels {
            if let Err(e) = stream.write_all(&fb) {
                return (None, false);
            }
            (Some(telemetry), stream.flush().is_ok())
        } else {
            (Some(telemetry), true)
        }
    } else {
        return (None, true)
    }
}

fn save_file_to_disk(bytes: Vec<u8>) {
    thread::spawn(move || {
        let now = chrono::Local::now();
        fs::write(format!("{}.cptv", now.format("%Y-%m-%d--%H-%M-%S")), &bytes).unwrap();
    });
}

struct SocketStream {
    unix: Option<UnixStream>,
    tcp: Option<TcpStream>
}

impl SocketStream {
    fn flush(&mut self) -> io::Result<()> {
        match &mut self.tcp {
            Some(stream) => stream.flush(),
            None => match &mut self.unix {
                Some(stream) => stream.flush(),
                None => unreachable!("Must have stream")
            }
        }
    }

    fn write_all(&mut self, bytes: &[u8]) -> io::Result<()> {
        match &mut self.tcp {
            Some(stream) => stream.write_all(bytes),
            None => match &mut self.unix {
                Some(stream) => stream.write_all(bytes),
                None => unreachable!("Must have stream")
            }
        }
    }

    fn shutdown(&mut self) -> io::Result<()> {
        match &mut self.tcp {
            Some(stream) => stream.shutdown(Shutdown::Both),
            None => match &mut self.unix {
                Some(stream) => stream.shutdown(Shutdown::Both),
                None => unreachable!("Must have stream")
            }
        }
    }
}

fn get_stream(use_wifi: &bool, address: &str) -> io::Result<SocketStream> {
    if !*use_wifi {
        UnixStream::connect(address).map(|stream| {
            //stream.set_write_timeout(Some(Duration::from_millis(1500))).unwrap();
            //stream.set_nonblocking(true).unwrap();
            SocketStream { unix: Some(stream), tcp: None }
        })

    } else {
        TcpStream::connect(address).map(|stream| {
           //stream.set_nonblocking(true).unwrap();
           stream.set_nodelay(true).unwrap();
           stream.set_write_timeout(Some(Duration::from_millis(1500))).unwrap();
           SocketStream { unix: None, tcp: Some(stream) }
       })
    }
}

fn default_spi_speed() -> u32 {
    12
}

#[derive(FromArgs)]
/// Agent for `tc2-firmware` to output thermal camera frames to either a local unix domain socket,
/// or a remote viewer application via wifi.
struct ModeConfig {
    /// output frames over wifi to `tc2-frames` desktop application
    #[argh(switch)]
    use_wifi: bool,

    /// raspberry pi SPI speed in Mhz, defaults to 12
    #[argh(option, default = "default_spi_speed()")]
    spi_speed: u32
}

const CAMERA_CONNECT_INFO: u8 = 0x1;

const CAMERA_RAW_FRAME_TRANSFER: u8 = 0x2;

const CAMERA_BEGIN_FILE_TRANSFER: u8 = 0x3;

const CAMERA_RESUME_FILE_TRANSFER: u8 = 0x4;

const CAMERA_END_FILE_TRANSFER: u8 = 0x5;

const CAMERA_BEGIN_AND_END_FILE_TRANSFER: u8 = 0x6;

fn get_socket_address(config: &ModeConfig) -> String {
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
            let mut address = None;
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
        if let Some(address) = &address {
            info!("Got address {}", address);
        }
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


fn main() {
    let log_config = ConfigBuilder::default().set_time_level(LevelFilter::Off).build();
    TermLogger::init(LevelFilter::Info, log_config, TerminalMode::Mixed, ColorChoice::Auto).unwrap();
    println!("\n=========\nStarting thermal camera 2 agent, run with --help to see options.\n");
    let config: ModeConfig = argh::from_env();

    // We want real-time priority for all the work we do.
    let _ = thread::Builder::new().name("frame-acquire".to_string()).spawn_with_priority(ThreadPriority::Max, move |result| {
        assert!(result.is_ok(), "Thread must have permissions to run with realtime priority, run as root user");


        // 1MB buffer that we won't fully use at the moment.
        let mut raw_read_buffer = [0u8; 1024 * 1024];

        //let spi_speed = 30_000_000; // rPi4 can handle this in PIO mode
        let spi_speed = config.spi_speed * 1_000_000; // rPi3 can handle 12Mhz (@600Mhz), may need to back it off a little to have some slack.
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
        let mut run_pin = gpio.get(23).unwrap().into_output();
        if !run_pin.is_set_high() {
            run_pin.set_high();
            sleep(Duration::from_millis(1000));
        }
        let debug_mode = true;
        if !debug_mode {
            let date = chrono::Local::now();
            println!("Resetting rp2040 on startup, {}", date.format("%Y-%m-%d--%H:%M:%S"));
            run_pin.set_low();
            sleep(Duration::from_millis(1000));
            run_pin.set_high();
        }

        pin.clear_interrupt().expect("Unable to clear pi ping interrupt pin");
        pin.set_interrupt(Trigger::RisingEdge).expect("Unable to set pi ping interrupt");
        let (tx, rx) = channel();

        let _ = thread::Builder::new().name("frame-socket".to_string()).spawn_with_priority(ThreadPriority::Max, move |result| {
            // Spawn a thread which can output the frames, converted to rgb grayscale
            // This is printed out from within the spawned thread.
            assert!(result.is_ok(), "Thread must have permissions to run with realtime priority, run as root user");

            info!("Connecting to frame socket {}", address);
            let mut reconnects = 0;
            let mut prev_frame_num = None;
            let mut prev_time_on_msec = 0u32;

            loop {
                match get_stream(&config.use_wifi, &address) {
                    Ok(mut stream) => {
                        info!("Connected to {}, waiting for frames from rp2040", if config.use_wifi { &"tc2-frames server"} else { &"thermal-recorder unix socket"});
                        let mut sent_header = false;
                        let mut ms_elapsed = 0;
                        'send_loop: loop {
                            let rec_timeout_ms = 10;
                            if let Ok(radiometry_enabled) = rx.recv_timeout(Duration::from_millis(rec_timeout_ms)) { // Increasing to 100 seams to make hte connect more reliably. Was there a reason for it being 1?
                                if !config.use_wifi && !sent_header {
                                    // Send the header info here:
                                    let header: &[u8] =
                                        if radiometry_enabled {
                                            b"ResX: 160\nResX: 160\nResY: 120\nFrameSize: 39040\nModel: lepton3.5\nBrand: flir\nFPS: 9\nFirmware: 1.0\nCameraSerial: f00bar\n\n"
                                        } else {
                                            b"ResX: 160\nResX: 160\nResY: 120\nFrameSize: 39040\nModel: lepton3\nBrand: flir\nFPS: 9\nFirmware: 1.0\nCameraSerial: f00bar\n\n"
                                        };
                                    if let Err(_) = stream.write_all(header) {
                                        warn!("Failed sending header info");
                                    }
                                    // println!("Sent header");

                                    // Clear existing
                                    if let Err(_) = stream.write_all(b"clear") {
                                        warn!("Failed clearing buffer");
                                    }
                                    let _ = stream.flush();
                                    // println!("Clear buffer");
                                    sent_header = true;
                                }

                                if reconnects > 0 {
                                    info!("Got frame connection");
                                    reconnects = 0;
                                    prev_frame_num = None;
                                    reconnects = 0;
                                }
                                let s = Instant::now();
                                let (telemetry, sent) = send_frame(&mut stream);
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
                            } else if !debug_mode {
                                const NUM_ATTEMPTS_BEFORE_RESET: usize = 10;
                                ms_elapsed += rec_timeout_ms;
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
                    Err(e) => warn!("{} not found: {}", if config.use_wifi {&"tc2-frames server"} else {&"thermal-recorder unix socket"}, e.to_string())
                }
                // Wait 1 second between attempts to connect to the frame socket
                sleep(Duration::from_millis(1000));
            }
        });

        info!("Waiting to acquire frames from rp2040");
        let mut got_first_frame = false;
        let mut file_download: Option<Vec<u8>> = None;
        let mut transfer_count = 0;
        let mut header = [0u8; 18];
        let mut crc_buf = [0u8; 32];
        crc_buf[0] = 1;
        crc_buf[1] = 2;
        crc_buf[2] = 3;
        crc_buf[3] = 4;
        let mut part_count = 0;
        let mut start = Instant::now();
        let crc_check = Crc::<u16>::new(&CRC_16_XMODEM);
        let max_size: usize = raw_read_buffer.len();
        'transfer: loop {
            let mut got_frame = false;
            let mut radiometry_enabled = false;
            // Align our SPI reads to the start of the sequence 1, 2, 3, 4
            if let Ok(_pin_level) = pin.poll_interrupt(false, None) {
                if _pin_level.is_some() {
                    spi.read(&mut header).unwrap();
                    //info!("Got header {:?}", header);
                    let transfer_type = header[0];
                    let transfer_type_dup = header[1];

                    let mut num_bytes = LittleEndian::read_u32(&header[2..6]) as usize;
                    let num_bytes_dup = LittleEndian::read_u32(&header[6..10]) as usize;
                    num_bytes = num_bytes.min(max_size);
                    let crc_from_remote = LittleEndian::read_u16(&header[10..12]);
                    let crc_from_remote_dup = LittleEndian::read_u16(&header[12..14]);
                    let crc_from_remote_inv = LittleEndian::read_u16(&header[14..16]);
                    let crc_from_remote_inv_dup = LittleEndian::read_u16(&header[16..=17]);

                    let num_bytes_check = num_bytes == num_bytes_dup;
                    let header_crc_check = crc_from_remote == crc_from_remote_dup && crc_from_remote_inv_dup == crc_from_remote_inv && crc_from_remote_inv.reverse_bits() == crc_from_remote;
                    let transfer_type_check = transfer_type == transfer_type_dup;

                    if num_bytes != 0 {
                        //info!("Header {:?}, tt {}, crc {}, num_bytes {}", &header, transfer_type, crc_from_remote, num_bytes);
                    }

                    if !num_bytes_check || !header_crc_check || !transfer_type_check {
                        warn!("Header integrity check failed {:?}", header);
                        LittleEndian::write_u16(&mut crc_buf[4..6], 0);
                        LittleEndian::write_u16(&mut crc_buf[6..8], 0);
                        spi.write(&crc_buf).unwrap();
                        continue 'transfer;
                    }

                    if num_bytes == 0 {
                        warn!("zero-sized payload");
                        LittleEndian::write_u16(&mut crc_buf[4..6], 0);
                        LittleEndian::write_u16(&mut crc_buf[6..8], 0);
                        spi.write(&crc_buf).unwrap();
                        continue 'transfer;
                    }

                    if transfer_type < 1 || transfer_type > 6 {
                        warn!("unknown transfer type {}", transfer_type);
                        LittleEndian::write_u16(&mut crc_buf[4..6], 0);
                        LittleEndian::write_u16(&mut crc_buf[6..8], 0);
                        spi.write(&crc_buf).unwrap();
                        continue 'transfer;
                    }

                    if transfer_type > 1 {
                        transfer_count += 1;
                    }
                    if transfer_type == CAMERA_BEGIN_FILE_TRANSFER {
                        start = Instant::now();
                    }

                    let now = chrono::Local::now();
                    // We sort of have to assume we got the right number of bytes to read, right?
                    if num_bytes != 0 && transfer_type > 0 && transfer_type < 6 {
                        let mut chunk = &mut raw_read_buffer[0..num_bytes];
                        spi.read(chunk).unwrap();
                        //info!("-- [{}] SPI got transfer type {}, #{} of {} bytes", now.format("%H:%M:%S:%.3f"), transfer_type, transfer_count, num_bytes);

                        // Write back the crc we calculated.
                        let crc = crc_check.checksum(&chunk);
                        LittleEndian::write_u16(&mut crc_buf[4..6], crc);
                        LittleEndian::write_u16(&mut crc_buf[6..8], crc);

                        if let Ok(_pin_level) = pin.poll_interrupt(false, None) {
                            spi.write(&crc_buf).unwrap();
                            if crc == crc_from_remote {
                                match transfer_type {
                                    CAMERA_CONNECT_INFO => {
                                        radiometry_enabled = LittleEndian::read_u32(&chunk[0..4]) == 1;
                                        let firmware_version = LittleEndian::read_u32(&chunk[4..8]);
                                        info!("Got startup info: radiometry enabled: {}, firmware version: {}", radiometry_enabled, firmware_version);
                                        // Terminate any existing file download.
                                        let in_progress_file_transfer = file_download.take();
                                        if let Some(file) = in_progress_file_transfer {
                                            warn!("Aborting in progress file transfer with {} bytes", file.len());
                                        }
                                    }
                                    CAMERA_RAW_FRAME_TRANSFER => {
                                        // Frame
                                        let mut frame = [0u8; FRAME_LENGTH];
                                        // NOTE: We need to swizzle the pixel bytes.
                                        LittleEndian::write_u32_into(unsafe { &u8_slice_to_u32(&chunk) }, &mut frame);
                                        let back = FRAME_BUFFER.get_back().lock().unwrap();
                                        back.replace(Some(frame));
                                        if !got_first_frame {
                                            got_first_frame = true;
                                            info!("Got first frame from rp2040");
                                        }
                                        got_frame = true;
                                        FRAME_BUFFER.swap();
                                        let _ = tx.send(radiometry_enabled);
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
                                    }
                                    CAMERA_RESUME_FILE_TRANSFER => {
                                        if let Some(file) = &mut file_download {
                                            // Continue current file transfer
                                            //println!("Continue file transfer");
                                            part_count += 1;
                                            file.extend_from_slice(&chunk);
                                        } else {
                                            warn!("Trying to continue file with no open file");
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
                                            save_file_to_disk(file);
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
                                        save_file_to_disk(file);
                                    }
                                    _ => if num_bytes != 0 { warn!("Unhandled transfer type, {:#x}", transfer_type) }
                                }
                            } else {
                                warn!("Crc check failed, remote was notified and will re-transmit");
                            }
                        }

                    }
                }
            }
        }
    }).unwrap().join();
}

pub unsafe fn u8_slice_to_u32(p: &[u8]) -> &[u32] {
    core::slice::from_raw_parts((p as *const [u8]) as *const u32, p.len() / 4)
}
