use crate::device_config::DeviceConfig;
use byteorder::LittleEndian;
use byteorder::{ByteOrder, WriteBytesExt};
use chrono::{DateTime, Utc};
use log::{error, info};
use std::io::Cursor;
use std::io::Write;
use std::process::{Command, Stdio};
use std::{fs, thread};
use thread_priority::{ThreadBuilderExt, ThreadPriority};

fn wav_header(audio_length: usize, sample_rate: u32) -> [u8; 44] {
    let header_inner = [0u8; 44];
    let bits_per_sample = 16;
    let bytes_per_block = bits_per_sample / 8;
    let bytes_per_second: u32 = sample_rate * bytes_per_block as u32;
    let num_channels = 1;
    let format_pcm = 1;
    let file_size = (audio_length + (header_inner.len() - 8)) as u32;

    let mut cursor = Cursor::new(header_inner);

    // RIFF header (12 bytes)
    cursor.write_all(b"RIFF").unwrap();
    // Overall file size minus 8 bytes
    cursor.write_u32::<LittleEndian>(file_size).unwrap();
    cursor.write_all(b"WAVE").unwrap();

    // fmt block (24 bytes)
    cursor.write_all(b"fmt ").unwrap();
    // Size of format data after this point (minus "fmt " and 16u32, i.e. 8 bytes)
    cursor.write_u32::<LittleEndian>(16).unwrap();
    cursor.write_u16::<LittleEndian>(format_pcm).unwrap();
    cursor.write_u16::<LittleEndian>(num_channels).unwrap();
    cursor.write_u32::<LittleEndian>(sample_rate).unwrap();
    cursor.write_u32::<LittleEndian>(bytes_per_second).unwrap();
    cursor.write_u16::<LittleEndian>(bytes_per_block).unwrap();
    cursor.write_u16::<LittleEndian>(bits_per_sample).unwrap();

    // Beginning of data block / end of header (8 bytes)
    cursor.write_all(b"data").unwrap();
    cursor
        .write_u32::<LittleEndian>(audio_length as u32)
        .unwrap();
    cursor.into_inner()
}

pub fn save_audio_file_to_disk(mut audio_bytes: Vec<u8>, device_config: DeviceConfig) {
    let output_dir = String::from(device_config.output_dir());
    //let output_dir = String::from("/home/pi/temp");
    let _ = thread::Builder::new()
        .name("audio-transcode".to_string())
        .spawn_with_priority(ThreadPriority::Min, move |_| {
            // Reclaim some memory
            audio_bytes.shrink_to_fit();
            let timestamp = LittleEndian::read_u64(&audio_bytes[2..10]);
            let recording_date_time = DateTime::from_timestamp_millis(timestamp as i64 / 1000)
                .unwrap_or(chrono::Local::now().with_timezone(&Utc))
                .with_timezone(&chrono::Local);
            info!("Saving AAC file");
            if !fs::exists(&output_dir).unwrap_or(false) {
                fs::create_dir(&output_dir).expect(&format!(
                    "Failed to create AAC output directory {}",
                    output_dir
                ));
            }

            let output_path: String = format!(
                "{}/{}.aac",
                output_dir,
                recording_date_time.format("%Y-%m-%d--%H-%M-%S")
            );
            // If the file already exists, don't re-save it.
            if !fs::exists(&output_path).unwrap_or(false) {
                let recording_date_time =
                    format!("recordingDateTime={}", recording_date_time.to_rfc3339());
                let latitude = format!("latitude={}", device_config.lat_lng().0);
                let longitude = format!("longitude={}", device_config.lat_lng().1);
                let altitude = format!(
                    "locAltitude={}",
                    device_config.location_altitude().unwrap_or(0.0)
                );
                let location_accuracy = format!(
                    "locAccuracy={}",
                    device_config.location_accuracy().unwrap_or(0.0)
                );
                let location_timestamp = format!(
                    "locTimestamp={}",
                    device_config.location_timestamp().unwrap_or(0)
                );
                let device_id = format!("deviceId={}", device_config.device_id());
                let sample_rate = LittleEndian::read_u16(&audio_bytes[10..12]) as u32;
                let duration = format!(
                    "duration={}",
                    audio_bytes[12..].len() as f32 / sample_rate as f32 / 2.0
                );
                info!("Sample rate is {}", sample_rate);

                // Now transcode with ffmpeg – we create an aac stream in an m4a wrapper in order
                // to support adding metadata tags.
                let mut cmd = Command::new("ffmpeg")
                    .arg("-i")
                    .arg("pipe:0")
                    .arg("-codec:a")
                    .arg("aac")
                    .arg("-q:a")
                    .arg("1.2") // VBR, more appropriate for audio with lots of nothing
                    // Faster perceptual coder that should give faster +
                    // better results at the higher bitrates we're using.
                    .arg("-aac_coder")
                    .arg("fast")
                    .arg("-movflags")
                    .arg("faststart") // Move the metadata to the beginning of file
                    .arg("-movflags")
                    .arg("+use_metadata_tags") // Allow custom metadata tags
                    .arg("-map_metadata") // Keep existing metadata?
                    .arg("0")
                    .arg("-metadata")
                    .arg(recording_date_time)
                    .arg("-metadata")
                    .arg(duration)
                    .arg("-metadata")
                    .arg(latitude)
                    .arg("-metadata")
                    .arg(longitude)
                    .arg("-metadata")
                    .arg(device_id)
                    .arg("-metadata")
                    .arg(altitude)
                    .arg("-metadata")
                    .arg(location_timestamp)
                    .arg("-metadata")
                    .arg(location_accuracy)
                    .arg("-f")
                    .arg("mp4")
                    .arg(output_path.clone())
                    .stdin(Stdio::piped())
                    .stdout(Stdio::piped())
                    .stderr(Stdio::piped())
                    .spawn()
                    .expect("Failed to spawn ffmpeg process");

                let mut stdin = cmd.stdin.take().expect("Failed to open stdin");
                thread::spawn(move || {
                    // Write wav to stdin:
                    let audio_bytes = &audio_bytes[12..];
                    // let header = &wav_header(audio_bytes.len(), sample_rate);

                    let path: String = format!("{}/test.wav", output_dir);
                    // If the file already exists, don't re-save it.
                    let is_existing_file = match fs::metadata(&path) {
                        Ok(metadata) => metadata.len() as usize == audio_bytes.len() - 12,
                        Err(_) => false,
                    };
                    if !is_existing_file {
                        match fs::write(&path, &audio_bytes) {
                            Ok(()) => {
                                info!(
                                    "Saved Audio file header {} bytes are {}",
                                    path,
                                    audio_bytes.len()
                                );
                            }
                            Err(e) => {
                                error!(
                                    "Failed writing Audio file to storage at {}, reason: {}",
                                    path, e
                                );
                            }
                        }

                        // let mut f = fs::OpenOptions::new()
                        //     .append(true)
                        //     .create(false)
                        //     .open(&path)
                        //     .expect("Unable to open file");
                        // match f.write_all(&audio_bytes) {
                        //     Ok(()) => {
                        //         info!("Saved Audio file {} bytes are {}", path, audio_bytes.len());
                        //     }
                        //     Err(e) => {
                        //         error!(
                        //             "Failed writing Audio file to storage at {}, reason: {}",
                        //             path, e
                        //         );
                        //     }
                        // }
                    } else {
                        error!("File {} already exists, discarding duplicate", path);
                    }

                    // stdin
                    //     .write_all(&wav_header(audio_bytes.len(), sample_rate))
                    //     .unwrap();
                    // stdin.write_all(&audio_bytes).unwrap();
                });
                // match cmd.wait() {
                //     Ok(exit_status) => {
                //         if exit_status.success() {
                //             info!("Saved AAC file {}", output_path);
                //         } else {
                //             error!("Failed transcoding {} to AAC", output_path);
                //         }
                //     }
                //     Err(e) => {
                //         error!(
                //             "Failed invoking ffmpeg to transcode {}, reason: {}",
                //             output_path, e
                //         );
                //     }
                // }
            } else {
                error!("File {} already exists, discarding duplicate", output_path);
            }
        });
}
