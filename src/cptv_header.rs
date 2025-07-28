use flate2::read::MultiGzDecoder;
use log::warn;
use nom::bytes::streaming::{tag, take};
use nom::error::{ContextError, ErrorKind};
use nom::number::streaming::{le_f32, le_u8, le_u16, le_u32, le_u64};
use std::error::Error;
use std::io::Read;

// Decode CPTV headers
#[derive(Debug, Clone)]
pub struct Cptv2Header {
    pub timestamp: u64,
    pub width: u32,
    pub height: u32,
    pub compression: u8,
    pub device_name: String,

    pub fps: u8,
    pub brand: Option<String>,
    pub model: Option<String>,
    pub device_id: Option<u32>,
    pub serial_number: Option<u32>,
    pub firmware_version: Option<String>,
    pub motion_config: Option<String>,
    pub preview_secs: Option<u8>,
    pub latitude: Option<f32>,
    pub longitude: Option<f32>,
    pub loc_timestamp: Option<u64>,
    pub altitude: Option<f32>,
    pub accuracy: Option<f32>,
    pub has_background_frame: bool,

    pub total_frame_count: Option<u16>,
    pub min_value: Option<u16>,
    pub max_value: Option<u16>,
}

impl Cptv2Header {
    pub fn new() -> Cptv2Header {
        // NOTE: Set default values for things not included in
        // older CPTVv1 files, which can otherwise be decoded as
        // v2.
        Cptv2Header {
            timestamp: 0,
            width: 0,
            height: 0,
            compression: 0,
            device_name: "".to_string(),
            fps: 9,
            brand: None,
            model: None,
            device_id: None,
            serial_number: None,
            firmware_version: None,
            motion_config: None,
            preview_secs: None,
            latitude: None,
            longitude: None,
            loc_timestamp: None,
            altitude: None,
            accuracy: None,
            has_background_frame: false,
            total_frame_count: None,
            min_value: None,
            max_value: None,
        }
    }
}
#[derive(Debug, Clone)]
#[allow(clippy::large_enum_variant)]
pub enum CptvHeader {
    #[allow(unused)]
    Uninitialised,
    V2(Cptv2Header),
}

#[repr(u8)]
#[derive(PartialEq, Debug, Copy, Clone)]
pub enum FieldType {
    // K remaining
    Header = b'H',
    Timestamp = b'T',
    Width = b'X',
    Height = b'Y',
    Compression = b'C',
    DeviceName = b'D',
    MotionConfig = b'M',
    PreviewSecs = b'P',
    Latitude = b'L',
    Longitude = b'O',
    LocTimestamp = b'S',
    Altitude = b'A',
    Accuracy = b'U',
    Model = b'E',
    Brand = b'B',
    DeviceID = b'I',
    FirmwareVersion = b'V',
    CameraSerial = b'N',
    FrameRate = b'Z',
    BackgroundFrame = b'g',

    // TODO: Other header fields I've added to V2
    MinValue = b'Q',
    MaxValue = b'K',
    NumFrames = b'J',
    FramesPerIframe = b'G',
    FrameHeader = b'F',

    BitsPerPixel = b'w',
    FrameSize = b'f',
    LastFfcTime = b'c',
    FrameTempC = b'a',
    LastFfcTempC = b'b',
    TimeOn = b't',
    Unknown = b';',
}

impl From<char> for FieldType {
    fn from(val: char) -> Self {
        use FieldType::*;
        match val {
            'H' => Header,
            'T' => Timestamp,
            'X' => Width,
            'Y' => Height,
            'C' => Compression,
            'D' => DeviceName,
            'E' => Model,
            'B' => Brand,
            'I' => DeviceID,
            'M' => MotionConfig,
            'P' => PreviewSecs,
            'L' => Latitude,
            'O' => Longitude,
            'S' => LocTimestamp,
            'A' => Altitude,
            'U' => Accuracy,
            'Q' => MinValue,
            'K' => MaxValue,
            'N' => CameraSerial,
            'V' => FirmwareVersion,
            'J' => NumFrames,
            'Z' => FrameRate,
            'G' => FramesPerIframe,
            'F' => FrameHeader,
            'g' => BackgroundFrame,
            'w' => BitsPerPixel,
            'f' => FrameSize,
            'c' => LastFfcTime,
            't' => TimeOn,
            'a' => FrameTempC,
            'b' => LastFfcTempC,
            _ => Unknown,
        }
    }
}

pub fn decode_cptv2_header(i: &[u8]) -> nom::IResult<&[u8], CptvHeader> {
    let mut meta = Cptv2Header::new();
    let (i, val) = take(1usize)(i)?;
    let (_, _) = nom::character::streaming::char('H')(val)?;
    let (i, num_header_fields) = le_u8(i)?;
    let mut outer = i;
    for _ in 0..num_header_fields {
        let (i, field_length) = le_u8(outer)?;
        let (i, field) = take(1usize)(i)?;
        let (_, field) = nom::character::streaming::char(field[0] as char)(field)?;
        let (i, val) = take(field_length)(i)?;
        outer = i;
        let field_type = FieldType::from(field);
        match field_type {
            FieldType::Timestamp => {
                meta.timestamp = le_u64(val)?.1;
            }
            FieldType::Width => {
                meta.width = le_u32(val)?.1;
            }
            FieldType::Height => {
                meta.height = le_u32(val)?.1;
            }
            FieldType::Compression => {
                meta.compression = le_u8(val)?.1;
            }
            FieldType::DeviceName => {
                meta.device_name = String::from_utf8_lossy(val).into();
            }

            // Optional fields
            FieldType::FrameRate => meta.fps = le_u8(val)?.1,
            FieldType::CameraSerial => meta.serial_number = Some(le_u32(val)?.1),
            FieldType::FirmwareVersion => {
                meta.firmware_version = Some(String::from_utf8_lossy(val).into());
            }
            FieldType::Model => {
                meta.model = Some(String::from_utf8_lossy(val).into());
            }
            FieldType::Brand => {
                meta.brand = Some(String::from_utf8_lossy(val).into());
            }
            FieldType::DeviceID => {
                meta.device_id = Some(le_u32(val)?.1);
            }
            FieldType::MotionConfig => {
                meta.motion_config = Some(String::from_utf8_lossy(val).into());
            }
            FieldType::PreviewSecs => {
                meta.preview_secs = Some(le_u8(val)?.1);
            }
            FieldType::Latitude => {
                meta.latitude = Some(le_f32(val)?.1);
            }
            FieldType::Longitude => {
                meta.longitude = Some(le_f32(val)?.1);
            }
            FieldType::LocTimestamp => {
                meta.loc_timestamp = Some(le_u64(val)?.1);
            }
            FieldType::Altitude => {
                meta.altitude = Some(le_f32(i)?.1);
            }
            FieldType::Accuracy => {
                meta.accuracy = Some(le_f32(val)?.1);
            }
            FieldType::NumFrames => {
                meta.total_frame_count = Some(le_u16(val)?.1);
            }
            FieldType::MinValue => {
                meta.min_value = Some(le_u16(val)?.1);
            }
            FieldType::MaxValue => {
                meta.max_value = Some(le_u16(val)?.1);
            }
            FieldType::BackgroundFrame => {
                let has_background_frame = le_u8(val)?.1;
                // NOTE: We expect this to always be 1 if present
                meta.has_background_frame = has_background_frame == 1;
            }
            _ => {
                warn!("Unknown header field type {field}, {field_length}");
            }
        }
    }
    Ok((outer, CptvHeader::V2(meta)))
}
pub fn decode_cptv_header(i: &[u8]) -> nom::IResult<&[u8], CptvHeader> {
    let (i, val) = take(4usize)(i)?;
    let (_, _) = tag(b"CPTV")(val)?;
    let (i, version) = le_u8(i)?;
    match version {
        1 | 2 => decode_cptv2_header(i),
        _ => Err(nom::Err::Failure(ContextError::add_context(
            i,
            "Unknown CPTV version",
            nom::error::Error::new(i, ErrorKind::Tag),
        ))),
    }
}

pub fn decode_cptv_header_streaming(cptv_bytes: &[u8]) -> Result<CptvHeader, Box<dyn Error>> {
    let mut decoder = MultiGzDecoder::new(cptv_bytes);
    let mut unzipped = Vec::new();
    let mut buffer = [0u8; 32];
    let cptv_header: CptvHeader;
    loop {
        match decode_cptv_header(&unzipped) {
            Ok((_, header)) => {
                cptv_header = header;
                break;
            }
            Err(e) => match e {
                nom::Err::Incomplete(_) => {
                    let read = decoder.read(&mut buffer)?;
                    unzipped.extend_from_slice(&buffer[0..read]);
                }
                nom::Err::Failure(e) | nom::Err::Error(e) => {
                    Err(format!("Parse error, not a valid CPTV file ({:?})", e.code))?;
                }
            },
        }
    }
    Ok(cptv_header)
}
