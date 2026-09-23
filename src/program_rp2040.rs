use crate::EXPECTED_RP2040_FIRMWARE_HASH;
use log::{error, info};
use std::path::Path;
use std::process::Command;
use std::{fs, io, process};

const RP2040_LOG_SERVICE: &str = "tc2-rp2040-logs.service";

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
