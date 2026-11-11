use crate::EXPECTED_RP2040_FIRMWARE_HASH;
use log::{error, info};
use std::path::Path;
use std::process::Command;
use std::{fs, io, process};

const RP2040_LOG_SERVICE: &str = "tc2-rp2040-logs.service";

/// If this file exists, keep forcing offloads until the rp2040 has no files left,
/// then erase its flash.
const OFFLOAD_ERASE_FILE: &str = "/etc/cacophony/offload_erase";

/// Stops the RP2040 log capture for as long as this is alive, so that the
/// programmer has the SWD pins to itself.
struct SuspendedRp2040Logging {
    was_active: bool,
}

impl SuspendedRp2040Logging {
    fn suspend() -> Self {
        let was_active = Command::new("systemctl")
            .args(["is-active", "--quiet", RP2040_LOG_SERVICE])
            .status()
            .is_ok_and(|status| status.success());
        if was_active {
            info!("Pausing {RP2040_LOG_SERVICE} while the RP2040 is programmed");
            // Blocking stop: openocd must be gone before the programmer starts.
            if let Err(e) = Command::new("systemctl").args(["stop", RP2040_LOG_SERVICE]).status() {
                error!("Failed to stop {RP2040_LOG_SERVICE}: {e}");
            }
        }
        Self { was_active }
    }
}

impl Drop for SuspendedRp2040Logging {
    fn drop(&mut self) {
        if self.was_active {
            let _ = Command::new("systemctl")
                .args(["start", "--no-block", RP2040_LOG_SERVICE])
                .status();
        }
    }
}

pub fn program_rp2040() -> io::Result<()> {
    let bytes: Vec<u8> = fs::read("/etc/cacophony/rp2040-firmware.elf")
        .expect("firmware file should exist at /etc/cacophony/rp2040-firmware.elf");
    let hash = sha256::digest(&bytes);
    let expected_hash = EXPECTED_RP2040_FIRMWARE_HASH.trim();
    if hash != expected_hash {
        return Err(io::Error::other(format!(
            "rp2040-firmware.elf does not match \
        expected hash. Expected: '{expected_hash}', Calculated: '{hash}'",
        )));
    }
    // tc2-rp2040-logs.service holds the SWD pins via openocd, and the programmer
    // needs them exclusively - two processes bit-banging SWD at once corrupts
    // the programming run. Stop it for the duration and start it again after.
    // This is done here rather than with a systemd Conflicts= because the unit
    // restarts on a timer, so its start job races the programming job and
    // systemd ends up cancelling one of them.
    let _logging = SuspendedRp2040Logging::suspend();

    let status = Command::new("tc2-hat-rp2040")
        .arg("--elf")
        .arg("/etc/cacophony/rp2040-firmware.elf")
        .status()?;

    if !status.success() {
        return Err(io::Error::other("Command execution failed"));
    }

    info!("Updated RP2040 firmware.");
    Ok(())
}

/// How many forced offload boots in a row may make no progress before we give up on
/// offloading and erase the flash anyway.
pub const MAX_STALLED_OFFLOAD_ERASE_ATTEMPTS: u32 = 5;

/// Decides when to give up on offloading before an offload_erase.
///
/// If the rp2040 flash is corrupt, the rp2040 can report files to offload but never manage
/// to send them (e.g. it gets reset by its watchdog while searching for the start of a file).
/// Without a limit we'd keep forcing offloads forever and never erase the flash.
#[derive(Debug, Default)]
pub struct OffloadEraseTracker {
    last_totals: Option<(u16, u16)>,
    file_received: bool,
    stalled_attempts: u32,
}

impl OffloadEraseTracker {
    pub fn new() -> Self {
        Self::default()
    }

    /// Call when a file has been received from the rp2040.
    pub fn file_received(&mut self) {
        self.file_received = true;
    }

    /// Call on each startup handshake that was told to force an offload, with the totals it
    /// reported. Returns true if offloading has stalled and the flash should be erased anyway.
    pub fn forced_offload_handshake(&mut self, num_files: u16, num_blocks: u16) -> bool {
        let totals = (num_files, num_blocks);
        let made_progress = self.file_received
            || self
                .last_totals
                .is_none_or(|(files, blocks)| num_files < files || num_blocks < blocks);
        if made_progress {
            self.stalled_attempts = 0;
        } else {
            self.stalled_attempts += 1;
        }
        self.last_totals = Some(totals);
        self.file_received = false;
        self.stalled_attempts >= MAX_STALLED_OFFLOAD_ERASE_ATTEMPTS
    }

    pub fn stalled_attempts(&self) -> u32 {
        self.stalled_attempts
    }
}

pub fn offload_erase_requested() -> bool {
    Path::new(OFFLOAD_ERASE_FILE).exists()
}

/// Erases the rp2040 2Gb SPI flash, removes the offload_erase file and exits.
/// `tc2-hat-rp2040 --erase-flash` creates /etc/cacophony/program_rp2040, so the normal
/// firmware is programmed back on when tc2-agent restarts.
pub fn erase_rp2040_flash_and_exit() -> ! {
    info!("All files offloaded, erasing rp2040 2Gb SPI flash");
    let result = {
        // Same as programming, the erase needs the SWD pins to itself.
        let _logging = SuspendedRp2040Logging::suspend();
        Command::new("tc2-hat-rp2040").arg("--erase-flash").status()
    };
    match result {
        Ok(status) if status.success() => {
            if let Err(e) = fs::remove_file(OFFLOAD_ERASE_FILE) {
                error!("Failed to remove '{OFFLOAD_ERASE_FILE}' after erasing flash: {e}");
                process::exit(1);
            }
            info!("Erased rp2040 2Gb SPI flash");
            process::exit(0);
        }
        Ok(status) => {
            error!("Failed to erase rp2040 2Gb SPI flash: {status}");
            process::exit(1);
        }
        Err(e) => {
            error!("Failed to run tc2-hat-rp2040 --erase-flash: {e}");
            process::exit(1);
        }
    }
}

pub fn check_if_rp2040_needs_programming() {
    // Check if the file indicating that the RP2040 needs to be programmed.
    // This is used to save time when setting up cameras
    // so it will program the RP2040 instead of trying to connect first.
    let program_rp2040_file = Path::new("/etc/cacophony/program_rp2040");
    if program_rp2040_file.exists() {
        println!("Program RP2040 because /etc/cacophony/program_rp2040 exists");
        match program_rp2040() {
            Ok(()) => match fs::remove_file(program_rp2040_file) {
                Ok(()) => process::exit(0),
                Err(e) => {
                    error!(
                        "Failed to remove 'program_rp2040' \
                        file after successful reprogram: {e}"
                    );
                    process::exit(1);
                }
            },
            Err(e) => {
                error!("Failed to reprogram RP2040: {e}");
                process::exit(1);
            }
        }
    }
}

#[cfg(test)]
mod offload_erase_tests {
    use super::*;

    #[test]
    fn gives_up_when_rp2040_never_sends_files() {
        let mut tracker = OffloadEraseTracker::new();
        // The first attempt is never counted as stalled.
        assert!(!tracker.forced_offload_handshake(2, 1230));
        for _ in 1..MAX_STALLED_OFFLOAD_ERASE_ATTEMPTS {
            assert!(!tracker.forced_offload_handshake(2, 1230));
        }
        assert!(tracker.forced_offload_handshake(2, 1230));
    }

    #[test]
    fn received_file_resets_stall_count() {
        let mut tracker = OffloadEraseTracker::new();
        for _ in 0..MAX_STALLED_OFFLOAD_ERASE_ATTEMPTS {
            assert!(!tracker.forced_offload_handshake(2, 1230));
        }
        tracker.file_received();
        assert!(!tracker.forced_offload_handshake(2, 1230));
        assert_eq!(tracker.stalled_attempts(), 0);
    }

    #[test]
    fn fewer_files_or_blocks_counts_as_progress() {
        let mut tracker = OffloadEraseTracker::new();
        assert!(!tracker.forced_offload_handshake(10, 500));
        assert!(!tracker.forced_offload_handshake(10, 500));
        assert_eq!(tracker.stalled_attempts(), 1);
        assert!(!tracker.forced_offload_handshake(9, 450));
        assert_eq!(tracker.stalled_attempts(), 0);
        assert!(!tracker.forced_offload_handshake(9, 440));
        assert_eq!(tracker.stalled_attempts(), 0);
    }

    #[test]
    fn more_files_is_not_progress() {
        let mut tracker = OffloadEraseTracker::new();
        assert!(!tracker.forced_offload_handshake(2, 100));
        assert!(!tracker.forced_offload_handshake(3, 120));
        assert_eq!(tracker.stalled_attempts(), 1);
    }
}
