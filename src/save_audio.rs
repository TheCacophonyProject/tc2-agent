use crate::device_config::DeviceConfig;
use byteorder::{ByteOrder, LittleEndian};
use chrono::{DateTime, NaiveDateTime, Utc};
use log::{error, info};
use std::io::Write;
use std::process::Command;
use std::{fs, io, thread};

fn wav_header(audio_bytes: &Vec<u8>) -> [u8; 44] {
    let mut header: [u8; 44] = [0u8; 44];
    let mut cursor = 0;
    for b in "RIFF".bytes() {
        header[cursor] = b;
        cursor += 1;
    }
    let file_size = (audio_bytes.len() - 12 + 36) as u32;
    for b in file_size.to_le_bytes() {
        header[cursor] = b;
        cursor += 1;
    }

    for b in "WAVEfmt".bytes() {
        header[cursor] = b;
        cursor += 1;
    }

    header[cursor] = 32;
    cursor += 1;
    header[cursor] = 16;
    cursor += 4;

    // PCM
    header[cursor] = 1;
    cursor += 2;

    // channels
    header[cursor] = 1;
    cursor += 2;

    let sr = LittleEndian::read_u16(&audio_bytes[10..12]) as u32;
    // SR
    for b in sr.to_le_bytes() {
        header[cursor] = b;
        cursor += 1;
    }

    let sr = sr * 2;
    for b in sr.to_le_bytes() {
        header[cursor] = b;
        cursor += 1;
    }

    header[cursor] = 16 / 2;
    cursor += 2;

    for b in 16u16.to_le_bytes() {
        header[cursor] = b;
        cursor += 1;
    }

    for b in "data".bytes() {
        header[cursor] = b;
        cursor += 1;
    }

    for b in ((audio_bytes.len() - 12) as u32).to_le_bytes() {
        header[cursor] = b;
        cursor += 1;
    }
    header
}

pub fn save_audio_file_to_disk(audio_bytes: Vec<u8>, device_config: DeviceConfig) {
    let output_dir = String::from(device_config.output_dir());
    let temp_dir = "/home/pi/audio-temp";
    thread::spawn(move || {
        let header = wav_header(&audio_bytes);
        let timestamp = LittleEndian::read_u64(&audio_bytes[2..10]);
        let recording_date_time = DateTime::from_timestamp_millis(timestamp as i64 / 1000)
            .unwrap_or(chrono::Local::now().with_timezone(&Utc));
        info!("Saving Audio file with header");
        if fs::metadata(&output_dir).is_err() {
            fs::create_dir(&output_dir)
                .expect(&format!("Failed to create output directory {}", output_dir));
        }
        if fs::metadata(temp_dir).is_err() {
            fs::create_dir(temp_dir)
                .expect(&format!("Failed to create audio temporary directory {}", temp_dir));
        }
        let temp_path: String =
            format!("{}/{}.wav", temp_dir, recording_date_time.format("%Y-%m-%d--%H-%M-%S"));
        let output_path: String =
            format!("{}/{}.aac", output_dir, recording_date_time.format("%Y-%m-%d--%H-%M-%S"));
        // If the file already exists, don't re-save it.
        if !fs::exists(&output_path) {
            match fs::write(&temp_path, &header) {
                Ok(()) => {
                    info!("Saved Audio file header {} bytes are {}", temp_path, header.len());
                }
                Err(e) => {
                    error!("Failed writing Audio file to storage at {}, reason: {}", temp_path, e);
                    return;
                }
            }

            {
                let mut f = fs::OpenOptions::new()
                    .append(true)
                    .create(false)
                    .open(&temp_path)
                    .expect("Unable to open file");
                match f.write_all(&audio_bytes[12..]) {
                    Ok(()) => {
                        info!("Saved Audio file {} bytes are {}", temp_path, audio_bytes.len());
                    }
                    Err(e) => {
                        error!(
                            "Failed writing Audio file to storage at {}, reason: {}",
                            temp_path, e
                        );
                        return;
                    }
                }
            }

            let recording_date_time =
                format!("recordingDateTime=\"{}\"", recording_date_time.to_rfc3339());
            let latitude = format!("latitude=\"{}\"", device_config.lat_lng().0);
            let longitude = format!("longitude=\"{}\"", device_config.lat_lng().1);
            let altitude =
                format!("locAltitude=\"{}\"", device_config.location_altitude().unwrap_or(0.0));
            let location_accuracy =
                format!("locAccuracy=\"{}\"", device_config.location_accuracy().unwrap_or(0.0));
            let location_timestamp =
                format!("locTimestamp=\"{}\"", device_config.location_timestamp().unwrap_or(0));
            let device_id = format!("deviceId=\"{}\"", device_config.device_id());
            // Now transcode with ffmpeg
            match Command::new("ffmpeg")
                .arg("-i")
                .arg(&temp_path)
                .arg("-metadata")
                .arg(recording_date_time)
                .arg("-metadata")
                .arg(latitude)
                .arg("-metadata")
                .arg(longitude)
                .arg("-metadata")
                .arg(device_id)
                .arg("-codec:a")
                .arg("aac")
                .arg("-b:a")
                .arg("128k")
                .arg(&output_path)
                .status()
            {
                Ok(status) => {
                    if !status.success() {
                        error!("Failed transcoding {} to AAC", temp_path);
                    } else {
                        fs::remove_file(&temp_path)
                            .expect("Should be able to remove temporary .wav file");
                    }
                }
                Err(e) => {
                    error!("Failed invoking ffmpeg to transcode {}, reason: {}", temp_path, e);
                }
            }
        } else {
            error!("File {} already exists, discarding duplicate", temp_path);
        }
    });
}
