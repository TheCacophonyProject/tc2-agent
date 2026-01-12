use crate::cptv_header::{CptvHeader, decode_cptv_header_streaming};
use chrono::{DateTime, Utc};
use log::{error, info};
use std::{fs, thread};
use thread_priority::{ThreadBuilderExt, ThreadPriority};

pub fn save_cptv_file_to_disk(mut cptv_bytes: Vec<u8>, output_dir: &str,postprocess: bool) {
    {
        cptv_bytes.shrink_to_fit();
    }
    let output_dir  = if postprocess{
        format!("{}{}",  output_dir, String::from("/postprocess"))
    }else{
        String::from(output_dir)
    };

    let _ = thread::Builder::new().name("cptv-save".to_string()).spawn_with_priority(
        ThreadPriority::Min,
        move |_| match decode_cptv_header_streaming(&cptv_bytes) {
            Ok(header) => match header {
                CptvHeader::V2(header) => {
                    info!("Saving CPTV file with header {header:?}");
                    let recording_date_time =
                        DateTime::from_timestamp_millis(header.timestamp as i64 / 1000)
                            .unwrap_or(chrono::Local::now().with_timezone(&Utc))
                            .with_timezone(&chrono::Local);
                    if !fs::exists(&output_dir).unwrap_or(false) {
                        fs::create_dir(&output_dir).unwrap_or_else(|_| {
                            panic!("Failed to create output directory {output_dir}")
                        });
                    }
                    let filename = recording_date_time.format("%Y-%m-%d--%H-%M-%S");
                    let path = if postprocess {
                        // tell postprocessing it's not expecting a metadata .txt file
                        format!("{output_dir}/{filename}-track.cptv",)
                    }else{
                        format!("{output_dir}/{filename}.cptv",)
                    };
                    //very slow and cpu intensive offered 18% saving on a 10 minute recording
                    // let decoder = MultiGzDecoder::new(&cptv_bytes[..]);
                    // let mut encoder = GzEncoder::new(decoder, Compression::default());
                    // let mut new_cptv_bytes = Vec::new();
                    // encoder.read_to_end(&mut new_cptv_bytes).unwrap();

                    // Only use the re-compressed file if it actually got smaller.
                    // if new_cptv_bytes.len() > cptv_bytes.len() {
                    //     new_cptv_bytes = cptv_bytes;
                    // }
                    // If the file already exists, don't re-save it.
                    let is_existing_file = match fs::metadata(&path) {
                        Ok(metadata) => metadata.len() as usize == cptv_bytes.len(),
                        Err(_) => false,
                    };
                    if !is_existing_file {
                        match fs::write(&path, &cptv_bytes) {
                            Ok(()) => {
                                info!("Saved CPTV file {path}");
                            }
                            Err(e) => {
                                error!(
                                    "Failed writing CPTV file to storage at {path}, reason: {e}"
                                );
                            }
                        }

                        // NOTE: For debug purposes, we may want to also save the CPTV file locally for inspection.
                        // let path = format!(
                        //     "{}/{}.cptv",
                        //     "/home/pi",
                        //     recording_date_time.format("%Y-%m-%d--%H-%M-%S")
                        // );
                        // match fs::write(&path, &cptv_bytes) {
                        //     Ok(()) => {
                        //         info!("Saved CPTV file {}", path);
                        //     }
                        //     Err(e) => {
                        //         error!(
                        //             "Failed writing CPTV file to storage at {}, reason: {}",
                        //             path, e
                        //         );
                        //     }
                        // }
                    } else {
                        error!("File {path} already exists, discarding duplicate");
                    }
                }
                _ => error!("Unsupported CPTV file format, discarding file"),
            },
            Err(e) => {
                error!("Invalid CPTV file: ({e:?}), discarding");
            }
        },
    );
}
