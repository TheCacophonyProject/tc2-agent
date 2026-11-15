//! Recordings we've already received in full by medium power streaming. The rp2040 keeps these
//! on flash and tries to offload them again later, so when it starts sending one of them we tell
//! it to erase it instead.
//!
//! Recordings are identified by the timestamp in their CPTV header, which is when the recording
//! started. The medium power stream sends the same timestamp in its first packet.
//!
//! The frame socket server thread appends each streamed recording to the file. The camera
//! transfer thread loads the whole file into memory when an offload starts, and drops it when a
//! medium power recording starts. Offloading and medium power recording never happen at the same
//! time, so the file needs no locking.

use log::{debug, error, info, warn};
use std::fs;
use std::fs::OpenOptions;
use std::io::Write;
use std::path::{Path, PathBuf};

const MEDIUM_POWER_OFFLOADS_FILE: &str = "/etc/cacophony/medium-power-offloads";

/// Remembers that the recording that started at `timestamp` was received in full.
pub fn record(timestamp: u64) {
    record_in(Path::new(MEDIUM_POWER_OFFLOADS_FILE), timestamp);
}

fn record_in(path: &Path, timestamp: u64) {
    let result = OpenOptions::new()
        .create(true)
        .append(true)
        .open(path)
        .and_then(|mut file| writeln!(file, "{timestamp}"));
    match result {
        Ok(()) => info!("Saved streamed recording {timestamp} to '{}'", path.display()),
        Err(e) => error!("Failed to write '{}': {e}", path.display()),
    }
}

/// The streamed recordings, loaded for the current offload.
pub struct MediumPowerOffloads {
    path: PathBuf,
    timestamps: Option<Vec<u64>>,
}

impl MediumPowerOffloads {
    pub fn new() -> MediumPowerOffloads {
        MediumPowerOffloads { path: PathBuf::from(MEDIUM_POWER_OFFLOADS_FILE), timestamps: None }
    }

    fn timestamps(&mut self) -> &mut Vec<u64> {
        let path = &self.path;
        self.timestamps.get_or_insert_with(|| {
            let timestamps: Vec<u64> = match fs::read_to_string(path) {
                Ok(contents) => {
                    contents.lines().filter_map(|line| line.trim().parse().ok()).collect()
                }
                Err(e) if e.kind() == std::io::ErrorKind::NotFound => Vec::new(),
                Err(e) => {
                    warn!("Failed to read '{}': {e}", path.display());
                    Vec::new()
                }
            };
            info!("Loaded streamed recordings from '{}': {timestamps:?}", path.display());
            timestamps
        })
    }

    fn save(&mut self) {
        let contents: String =
            self.timestamps().iter().map(|timestamp| format!("{timestamp}\n")).collect();
        if let Err(e) = fs::write(&self.path, contents) {
            error!("Failed to write '{}': {e}", self.path.display());
        }
    }

    /// A medium power recording has started, so the file will change. It's loaded again for the
    /// next offload.
    pub fn medium_power_recording_started(&mut self) {
        self.timestamps = None;
    }

    /// Whether the recording that started at `timestamp` was already received in full. If so,
    /// it's forgotten, as the rp2040 will erase it.
    pub fn take(&mut self, timestamp: u64) -> bool {
        let timestamps = self.timestamps();
        let Some(index) = timestamps.iter().position(|t| *t == timestamp) else {
            debug!("Offloading recording {timestamp}, it wasn't streamed");
            return false;
        };
        info!("Offloading recording {timestamp}, it was already streamed");
        timestamps.remove(index);
        self.save();
        true
    }

    fn clear(&mut self, reason: &str) {
        let count = self.timestamps().len();
        if count > 0 {
            info!("{reason}, forgetting {count} streamed recordings");
            self.timestamps().clear();
            self.save();
        }
    }

    /// The rp2040 has no recordings left, so none of the streamed recordings can be offloaded
    /// again.
    pub fn no_recordings_left(&mut self) {
        self.clear("rp2040 has no recordings left");
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn test_offloads(name: &str) -> MediumPowerOffloads {
        let path = std::env::temp_dir()
            .join(format!("tc2-agent-medium-power-offloads-{name}-{}", std::process::id()));
        let _ = fs::remove_file(&path);
        MediumPowerOffloads { path, timestamps: None }
    }

    fn in_file(offloads: &MediumPowerOffloads) -> Vec<u64> {
        MediumPowerOffloads { path: offloads.path.clone(), timestamps: None }.timestamps().clone()
    }

    #[test]
    fn take_removes_from_the_list_and_the_file() {
        let mut offloads = test_offloads("take");
        record_in(&offloads.path, 1);
        record_in(&offloads.path, 2);
        assert!(offloads.take(1));
        assert!(!offloads.take(1));
        assert_eq!(in_file(&offloads), [2]);
        let _ = fs::remove_file(&offloads.path);
    }

    #[test]
    fn recordings_are_only_seen_after_being_reloaded() {
        let mut offloads = test_offloads("reload");
        assert!(!offloads.take(1));
        record_in(&offloads.path, 1);
        assert!(!offloads.take(1));
        offloads.medium_power_recording_started();
        assert!(offloads.take(1));
        let _ = fs::remove_file(&offloads.path);
    }
}
