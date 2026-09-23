//! Streams the RP2040's firmware logs into journald.
//!
//! The RP2040 writes defmt log frames into an RTT ring buffer in its own RAM.
//! openocd reads that buffer over the same SWD pins the pi already uses to
//! program it, and re-serves the bytes on a TCP port. This decodes them using
//! the string table inside the firmware elf and prints them to stdout, where
//! systemd captures them:
//!
//!     journalctl -u tc2-rp2040-logs -f
//!
//! Each line is prefixed with a syslog priority, so `journalctl -p warning`
//! and friends filter on the firmware's own log levels.

use defmt_decoder::{DecodeError, Frame, Table};
use object::{Object, ObjectSymbol};
use std::io::{BufRead, BufReader, ErrorKind, Read};
use std::net::{SocketAddr, TcpStream};
use std::process::{Child, Command, Stdio};
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::thread::{self, sleep};
use std::time::{Duration, Instant};
use std::{env, fs};

/// Must be the elf that is actually flashed, or the decoded strings are noise.
const DEFAULT_ELF: &str = "/etc/cacophony/rp2040-firmware.elf";
const DEFAULT_SWD_CONFIG: &str = "/etc/cacophony/raspberrypi-swd.cfg";
/// The conventional RTT port, to avoid colliding with anything else on the pi.
const DEFAULT_RTT_PORT: u16 = 19021;
const RETRY_DELAY: Duration = Duration::from_secs(10);
const CONNECT_TIMEOUT: Duration = Duration::from_secs(30);
/// How long to give openocd to find the ring buffer before starting over. It
/// has to get the SWD pins first, which it cannot while the RP2040 is being
/// programmed.
const ATTACH_TIMEOUT: Duration = Duration::from_secs(30);

/// openocd holds the SWD pins for as long as it runs, so it must not outlive us
/// - otherwise programming the RP2040 would be blocked by an orphan.
struct OpenOcd(Child);

impl Drop for OpenOcd {
    fn drop(&mut self) {
        let _ = self.0.kill();
        let _ = self.0.wait();
    }
}

fn main() {
    let elf_path = env::var("RP2040_FIRMWARE_ELF").unwrap_or_else(|_| DEFAULT_ELF.to_string());
    let swd_config =
        env::var("RP2040_SWD_CONFIG").unwrap_or_else(|_| DEFAULT_SWD_CONFIG.to_string());
    let port =
        env::var("RP2040_RTT_PORT").ok().and_then(|p| p.parse().ok()).unwrap_or(DEFAULT_RTT_PORT);

    // Never give up: the rp2040 gets reprogrammed and reset under us, and the
    // swd pins are taken away while that happens.
    loop {
        if let Err(e) = stream_logs(&elf_path, &swd_config, port) {
            eprintln!("<4>rp2040 log capture stopped: {e}");
        }
        sleep(RETRY_DELAY);
    }
}

fn stream_logs(elf_path: &str, swd_config: &str, port: u16) -> Result<(), String> {
    let elf = fs::read(elf_path).map_err(|e| format!("could not read {elf_path}: {e}"))?;
    let table = Table::parse(&elf)
        .map_err(|e| format!("could not read defmt data from {elf_path}: {e}"))?
        .ok_or_else(|| format!("{elf_path} has no defmt data"))?;
    let rtt_address = rtt_control_block_address(&elf)?;

    let (_openocd, attached, rtt_broken) = spawn_openocd(swd_config, rtt_address, port)?;

    // openocd opens the rtt server port even when it never found the ring
    // buffer - so connecting proves nothing. Without this gate the service sits
    // on a socket that will never carry data, looking healthy and logging
    // nothing. Wait for openocd to say it attached, and start over if it does
    // not: the RP2040 may be mid-reprogram, or held in reset.
    let deadline = Instant::now() + ATTACH_TIMEOUT;
    while !attached.load(Ordering::Relaxed) {
        if Instant::now() >= deadline {
            return Err(
                "openocd could not attach to the rp2040 (swd busy, or the chip is in reset)"
                    .to_string(),
            );
        }
        sleep(Duration::from_millis(200));
    }

    let mut rtt = connect(port)?;
    rtt.set_read_timeout(Some(READ_POLL_INTERVAL))
        .map_err(|e| format!("could not set a read timeout: {e}"))?;
    eprintln!("<6>streaming rp2040 logs, rtt control block at {rtt_address:#010x}");

    let mut decoder = table.new_stream_decoder();
    let mut buf = [0u8; 1024];
    loop {
        if rtt_broken.load(Ordering::Relaxed) {
            return Err("openocd lost the rtt link (the rp2040 probably reset)".to_string());
        }
        let read = match rtt.read(&mut buf) {
            Ok(read) => read,
            // Nothing to read yet. The firmware is allowed to be quiet, so go
            // back round and re-check whether the link is still good.
            Err(e) if matches!(e.kind(), ErrorKind::WouldBlock | ErrorKind::TimedOut) => continue,
            Err(e) => return Err(format!("reading from openocd failed: {e}")),
        };
        if read == 0 {
            return Err("openocd closed the rtt connection".to_string());
        }
        decoder.received(&buf[..read]);
        loop {
            match decoder.decode() {
                Ok(frame) => print_frame(&frame),
                Err(DecodeError::UnexpectedEof) => break,
                // A reset mid-frame leaves a partial frame in the buffer, which
                // is normal here - the rp2040 restarts often.
                Err(DecodeError::Malformed) if table.encoding().can_recover() => continue,
                Err(DecodeError::Malformed) => {
                    return Err(format!(
                        "malformed defmt frame; is {elf_path} the firmware that is flashed?"
                    ));
                }
            }
        }
    }
}

/// Where the RTT ring buffer lives in RAM. It moves between builds, so read it
/// out of the elf rather than scanning for it.
fn rtt_control_block_address(elf: &[u8]) -> Result<u64, String> {
    let file = object::File::parse(elf).map_err(|e| format!("could not parse elf: {e}"))?;
    file.symbols()
        .find(|symbol| symbol.name() == Ok("_SEGGER_RTT"))
        .map(|symbol| symbol.address())
        .ok_or_else(|| "no _SEGGER_RTT symbol in the firmware elf".to_string())
}

/// How long to sit on a silent RTT socket before checking whether the link is
/// still alive. The firmware can legitimately be quiet, so this only wakes us
/// up to look at `rtt_broken`; it is not an inactivity timeout.
const READ_POLL_INTERVAL: Duration = Duration::from_secs(5);

/// What openocd reports when it can no longer read the ring buffer - usually
/// after the RP2040 reset out from under it. The socket stays open and simply
/// goes silent, so without watching for this the service looks healthy forever
/// while logging nothing.
fn is_rtt_failure(line: &str) -> bool {
    line.starts_with("Error:") && line.contains("rtt:")
}

/// Returns the running openocd, plus flags for "found the control block" and
/// "the rtt link has broken".
fn spawn_openocd(
    swd_config: &str,
    rtt_address: u64,
    port: u16,
) -> Result<(OpenOcd, Arc<AtomicBool>, Arc<AtomicBool>), String> {
    let mut child = Command::new("openocd")
        .args(["-f", swd_config])
        .args(["-f", "target/rp2040.cfg"])
        .args(["-c", "init"])
        // We only want openocd to read the RTT buffer. Left polling, it keeps
        // re-examining the cores every time the firmware resets, which floods
        // the log and puts needless traffic on the SWD pins.
        .args(["-c", "poll off"])
        .args(["-c", &format!("rtt setup {rtt_address:#x} 0x30 \"SEGGER RTT\"")])
        .args(["-c", "rtt polling_interval 20"])
        .args(["-c", "rtt start"])
        .args(["-c", &format!("rtt server start {port} 0")])
        .stdout(Stdio::null())
        .stderr(Stdio::piped())
        .spawn()
        .map_err(|e| format!("could not start openocd: {e}"))?;

    let stderr = child.stderr.take().ok_or("openocd stderr was not captured")?;
    let attached = Arc::new(AtomicBool::new(false));
    let rtt_broken = Arc::new(AtomicBool::new(false));
    let flag = Arc::clone(&attached);
    let broken = Arc::clone(&rtt_broken);

    // openocd narrates every examine and poll. Those lines vastly outnumber the
    // firmware's own, and journalctl shows debug priority by default, so
    // forwarding them all buries the logs we actually came for. Keep the
    // attach confirmation and anything that went wrong, drop the narration,
    // and collapse runs of the same message.
    thread::spawn(move || {
        let mut previous = String::new();
        let mut repeats = 0u32;
        for line in BufReader::new(stderr).lines().map_while(Result::ok) {
            let line = line.trim();
            if line.contains("Control block found") {
                flag.store(true, Ordering::Relaxed);
            } else if is_rtt_failure(line) {
                broken.store(true, Ordering::Relaxed);
            } else if !line.starts_with("Error:") && !line.starts_with("Warn :") {
                continue;
            }
            if line == previous {
                repeats += 1;
                continue;
            }
            if repeats > 0 {
                eprintln!("<4>openocd: (previous message repeated {repeats} times)");
                repeats = 0;
            }
            let priority = if line.starts_with("Error:") { 4 } else { 6 };
            eprintln!("<{priority}>openocd: {line}");
            previous = line.to_string();
        }
    });

    Ok((OpenOcd(child), attached, rtt_broken))
}

fn connect(port: u16) -> Result<TcpStream, String> {
    // openocd has to find the control block and open the port first. Connect on
    // 127.0.0.1 rather than localhost: openocd binds IPv4 only.
    let address = SocketAddr::from(([127, 0, 0, 1], port));
    let deadline = Instant::now() + CONNECT_TIMEOUT;
    loop {
        match TcpStream::connect_timeout(&address, Duration::from_secs(1)) {
            Ok(stream) => return Ok(stream),
            Err(e) => {
                if Instant::now() >= deadline {
                    return Err(format!("could not reach openocd's rtt server: {e}"));
                }
                sleep(Duration::from_millis(250));
            }
        }
    }
}

fn print_frame(frame: &Frame) {
    // systemd reads a leading <N> as the syslog priority, so the firmware's own
    // levels survive into journalctl.
    let priority = match frame.level().map(|level| level.as_str()) {
        Some("error") => 3,
        Some("warn") => 4,
        Some("debug" | "trace") => 7,
        _ => 6,
    };
    println!("<{priority}>{}", frame.display_message());
}
