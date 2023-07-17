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
    let ffc_state = ((status_bits & 0b00000000000000000000000000110000) >> 4);
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
        if let Some(pos) = pos {
            let y = pos / 160;
            let x = pos - (y * 160);
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

fn get_stream() -> io::Result<SocketStream> {
    if PROD_MODE {
        UnixStream::connect("/var/run/lepton-frames").map(|stream| {
            //stream.set_write_timeout(Some(Duration::from_millis(1500))).unwrap();
            //stream.set_nonblocking(true).unwrap();
            SocketStream { unix: Some(stream), tcp: None }
        })

    } else {
        TcpStream::connect("192.168.178.68:34254").map(|stream| {
           //stream.set_nonblocking(true).unwrap();
           stream.set_nodelay(true).unwrap();
           stream.set_write_timeout(Some(Duration::from_millis(1500))).unwrap();
           SocketStream { unix: None, tcp: Some(stream) }
       })
    }
}

// Set `true` if running on development hardware, which has a few pin differences from
// the initial dev board.
const P2: bool = true;
// Set `false` to build using live frame debug over wifi mode.
const PROD_MODE: bool = true;

fn main() {
    // We want real-time priority for all the work we do.
    let _ = thread::Builder::new().name("frame-acquire".to_string()).spawn_with_priority(ThreadPriority::Max, move |result| {
        assert!(result.is_ok(), "Thread must have permissions to run with realtime priority, run as root user");


        // 1MB buffer that we won't fully use at the moment.
        let mut raw_read_buffer = [0u8; 1024 * 1024];

        //let spi_speed = 30_000_000; // rPi4 can handle this in PIO mode
        let spi_speed = 12_000_000; // rPi3 can handle this (@600Mhz), may need to back it off a little to have some slack.
        let mut spi = Spi::new(Bus::Spi0, SlaveSelect::Ss0, spi_speed, Mode::Mode2).unwrap();
        spi.set_bits_per_word(8).unwrap();
        spi.set_bit_order(BitOrder::MsbFirst).unwrap();
        spi.set_ss_polarity(Polarity::ActiveLow).unwrap();
        let address = if PROD_MODE {
            "/var/run/lepton-frames"
        } else {
            "192.168.178.68:34254" // If in frame debug mode, use the local IP address of your dev machine.
        };
        let gpio = Gpio::new().unwrap();


        let pin_num = if P2 { 7 } else { 14 };
        // Signal pin, was 7 in production t2c hardware
        let mut pin = gpio.get(pin_num).unwrap().into_input_pulldown();

        let mut run_pin = if P2 {
            let mut run_pin = gpio.get(23).unwrap().into_output();
            if !run_pin.is_set_high() {
                run_pin.set_high();
            }
            use chrono::Local;
            let date = Local::now();
            println!("Resetting rp2040 on startup, {}", date.format("%Y-%m-%d][%H:%M:%S"));
            run_pin.set_low();
            sleep(Duration::from_millis(10));
            run_pin.set_high();
            Some(run_pin)
        } else { None };

        // TODO: Log mean time between reconnects.  Is it just that we should perhaps do an FFC?
        // TODO: Scan for anything on the local network with port 34254 open.

        pin.clear_interrupt().unwrap();
        pin.set_interrupt(Trigger::RisingEdge).unwrap();
        let (tx, rx) = channel();
        let _ = thread::Builder::new().name("frame-socket".to_string()).spawn_with_priority(ThreadPriority::Max, move |result| {
            // Spawn a thread which can output the frames, converted to rgb grayscale
            // This is printed out from within the spawned thread.
            assert!(result.is_ok(), "Thread must have permissions to run with realtime priority, run as root user");

            println!("Connecting to socket {}", address);
            let mut reconnects = 0;
            let mut prev_frame_num = None;
            let mut prev_time_on_msec = 0u32;

            loop {
                match get_stream() {
                    Ok(mut stream) => {
                        reconnects += 1;
                        if reconnects == 10 {
                            use chrono::Local;
                            let date = Local::now();
                            println!("Resetting (attempt #{reconnects}), {}", date.format("%Y-%m-%d][%H:%M:%S"));
                            reconnects = 0;
                            prev_frame_num = None;
                            if let Some(run_pin) = &mut run_pin {
                                run_pin.set_low();
                                sleep(Duration::from_millis(10));
                                run_pin.set_high();
                            }
                        } else {
                            println!("Connected to tc2 (attempt #{reconnects})");
                        }
                        let mut sent_header = false;
                        let mut ms_elapsed = 0;
                        'send_loop: loop {
                            if let Ok(radiometry_enabled) = rx.recv_timeout(Duration::from_millis(1)) {

                                if PROD_MODE && !sent_header {
                                    // Send the header info here:
                                    let header: &[u8] =
                                        if radiometry_enabled {
                                            b"ResX: 160\nResX: 160\nResY: 120\nFrameSize: 39040\nModel: lepton3.5\nBrand: flir\nFPS: 9\nFirmware: 1.0\nCameraSerial: f00bar\n\n"
                                        } else {
                                            b"ResX: 160\nResX: 160\nResY: 120\nFrameSize: 39040\nModel: lepton3\nBrand: flir\nFPS: 9\nFirmware: 1.0\nCameraSerial: f00bar\n\n"
                                        };
                                    if let Err(_) = stream.write_all(header) {
                                        println!("Failed sending header info");
                                    }
                                    // println!("Sent header");

                                    // Clear existing
                                    if let Err(_) = stream.write_all(b"clear") {
                                        println!("Failed clearing buffer");
                                    }
                                    let _ = stream.flush();
                                    // println!("Clear buffer");
                                    sent_header = true;
                                }

                                if reconnects > 0 {
                                    println!("Got frame connection");
                                    reconnects = 0;
                                    prev_frame_num = None;
                                    reconnects = 0;
                                }
                                let s = Instant::now();
                                let (telemetry, sent) = send_frame(&mut stream);
                                let e = s.elapsed().as_secs_f32();
                                if e > 0.1 {
                                    println!("socket send took {}s", e);
                                }
                                if !sent {
                                    println!("Send to frame socket failed");
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
                            } else {
                                ms_elapsed += 1;
                                if ms_elapsed > 5000 {
                                    ms_elapsed = 0;
                                    let _ = stream.shutdown().is_ok();
                                    //println!("Breaking send loop");
                                    break 'send_loop;
                                }
                            }
                        }
                    }
                    Err(e) => println!("Frame socket not found: {}", e.to_string())
                }
                // Wait 1 second between attempts to connect to the frame socket
                sleep(Duration::from_millis(1000));
            }
        });

        println!("Begin frame loop");
        'frame: loop {
            let mut got_frame = false;
            let mut radiometry_enabled = false;
            if let Ok(_pin_level) = pin.poll_interrupt(true, Some(Duration::from_millis(150))) {
                if _pin_level.is_some() {
                    // TODO: Add more protocol layer to this - maybe an error correction ack at the end
                    //  that causes a resend.
                    let mut size = [0u8; 4];
                    // First, the rp2040 sends the size of the chunk to read out, which is a fixed size u32
                    let start = Instant::now();
                    if let Ok(_) = spi.read(&mut size) {
                        // The top 4 bits are the read type.
                        // 0x1 = initial startup info
                        // 0x2 = a raw frame
                        // 0x3 = a cptv file?
                        // The bottom 28 bits are the transfer size.

                        let transfer_header = BigEndian::read_u32(&size) as usize;
                        let transfer_type = transfer_header >> 28;
                        let num_bytes = transfer_header & 0x0fff_ffff;
                        let max_size: usize = raw_read_buffer.len();//1 << 28;
                        if num_bytes >= 4 && num_bytes < max_size {
                            let chunk = &mut raw_read_buffer[0..num_bytes];
                            match spi.read(chunk) {
                                Ok(_) => {
                                    match transfer_type {
                                        0x1 => {
                                           radiometry_enabled = BigEndian::read_u32(&chunk[0..4]) == 1;
                                            let firmware_version = BigEndian::read_u32(&chunk[4..8]);
                                            println!("Got startup info: radiometry enabled: {}, firmware version: {}", radiometry_enabled, firmware_version);
                                        }
                                        0x2 => {
                                            // Frame
                                            let mut frame = [0u8; FRAME_LENGTH];
                                            // NOTE: We need to swizzle the pixel bytes.
                                            BigEndian::write_u32_into(unsafe { &u8_slice_to_u32(&chunk) }, &mut frame);
                                            let mut back = FRAME_BUFFER.get_back().lock().unwrap();
                                            back.replace(Some(frame));
                                            got_frame = true;
                                        }
                                        _ => println!("Unhandled transfer type, {:#x}", transfer_type)
                                    }
                                }
                                Err(e) => {
                                    println!("Failed reading all bytes of chunk {:?}", e);
                                }
                            }
                        } else {
                            // No bytes to read
                        }
                    } else {
                        println!("Failed reading size to pull");
                    }
                    //println!("transfer took {:?}", start.elapsed());

                    if got_frame {
                        FRAME_BUFFER.swap();
                        got_frame = false;
                        let _ = tx.send(radiometry_enabled);
                    }
                } else {
                    // println!("No ping from rp2040");
                }
            }
        }
    }).unwrap().join();
}

pub unsafe fn u8_slice_to_u32(p: &[u8]) -> &[u32] {
    core::slice::from_raw_parts((p as *const [u8]) as *const u32, p.len() / 4)
}
