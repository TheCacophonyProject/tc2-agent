use core::fmt;
use std::collections::HashMap;
// Read camera config file
use crate::detection_mask::DetectionMask;
use byteorder::{LittleEndian, WriteBytesExt};
use chrono::{
    DateTime, Duration, FixedOffset, Local, NaiveDate, NaiveDateTime, NaiveTime, TimeZone, Utc,
};
use log::{error, info, warn};
use louvre::triangulate;
use notify::event::{AccessKind, AccessMode};
use notify::{Event, EventKind, RecommendedWatcher, RecursiveMode, Watcher};
use serde::de::Error;
use serde::{Deserialize, Deserializer};
use std::fmt::{Debug, Formatter};
use std::io::{Cursor, Write};
use std::ops::Add;
use std::path::Path;
use std::str::FromStr;
use std::sync::mpsc::{Receiver, Sender, TryRecvError};
use std::{fs, process};
use sun_times::sun_times;
use toml::Value;
use toml::value::Offset;

fn default_constant_recorder() -> bool {
    false
}
fn default_low_power_mode() -> bool {
    false
}

fn default_mask_regions() -> DetectionMask {
    DetectionMask::new(None)
}

fn default_min_disk_space_mb() -> u32 {
    200
}

fn default_location_timestamp() -> Option<u64> {
    None
}

fn default_location_accuracy() -> Option<f32> {
    None
}
fn default_output_dir() -> String {
    String::from("/var/spool/cptv")
}
fn default_activate_thermal_throttler() -> bool {
    false
}

fn default_recording_start_time() -> AbsRelTime {
    AbsRelTime { relative_time_seconds: Some(-(60 * 30)), absolute_time: None }
}

fn default_recording_stop_time() -> AbsRelTime {
    AbsRelTime { relative_time_seconds: Some(60 * 30), absolute_time: None }
}

#[derive(Debug)]
struct TimeUnit(char);

#[derive(Debug)]
struct NumberString(String, Option<TimeUnit>, bool);

fn sign(p1: (f64, f64), p2: (f64, f64), p3: (f64, f64)) -> f64 {
    (p1.0 - p3.0) * (p2.1 - p3.1) - (p2.0 - p3.0) * (p1.1 - p3.1)
}

fn point_in_triangle(triangle: ((f64, f64), (f64, f64), (f64, f64)), point: (f64, f64)) -> bool {
    let d1 = sign(point, triangle.0, triangle.1);
    let d2 = sign(point, triangle.1, triangle.2);
    let d3 = sign(point, triangle.2, triangle.0);

    let has_neg = (d1 < 0.) || (d2 < 0.) || (d3 < 0.);
    let has_pos = (d1 > 0.) || (d2 > 0.) || (d3 > 0.);

    !(has_neg && has_pos)
}

fn deserialize_mask_regions<'de, D>(deserializer: D) -> Result<DetectionMask, D::Error>
where
    D: Deserializer<'de>,
{
    let masks: toml::map::Map<String, Value> = Deserialize::deserialize(deserializer)?;
    let mut regions: HashMap<String, Vec<[f64; 2]>> = HashMap::new();
    for (label, mask_region) in masks {
        let mut region = Vec::new();
        match mask_region {
            Value::Array(val) => {
                for (i, item) in val.iter().enumerate() {
                    match item {
                        Value::Array(coord) => {
                            if coord.len() != 2 {
                                error!(
                                    "Region '{}'[{}]: Expected coord array of length 2, got {}",
                                    label,
                                    i,
                                    coord.len()
                                );
                            } else {
                                let mut x = 0.0;
                                let mut y;
                                for (idx, el) in coord.iter().enumerate() {
                                    let el_val = match &el {
                                        Value::Float(float_val) => Some(*float_val),
                                        Value::Integer(int_val) => Some(*int_val as f64),
                                        _ => {
                                            error!(
                                                "Region '{}'[{}].{}: Unsupported coordinate value, expected Float or Integer",
                                                label,
                                                i,
                                                if idx == 0 { 'x' } else { 'y' }
                                            );
                                            None
                                        }
                                    };
                                    if let Some(val) = el_val {
                                        if idx == 0 {
                                            x = val;
                                        } else {
                                            y = val;
                                            region.push([x, y]);
                                        }
                                    }
                                }
                            }
                        }
                        _ => error!("Region '{label}'[{i}]: Expected array of [x, y] coordinates",),
                    }
                }
            }
            _ => error!("Region '{label}': Must be an array of [[x, y], ...] coordinates"),
        }
        regions.insert(label.clone(), region);
    }
    // Now need to triangulate polygons, and then fill the mask.
    let mut triangles = Vec::new();
    let w = 160.0;
    let h = 120.0;
    for (_label, polygon) in regions {
        let mut polygon = polygon.concat();
        let (polygon, triangulated_indices) = triangulate(&mut polygon, 2);
        for corners in triangulated_indices.chunks_exact(6) {
            // Map each triangle into the frame space, then do 'point-in triangle checks for each pixel of the frame.
            triangles.push((
                (polygon[corners[0]] * w, polygon[corners[1]] * h),
                (polygon[corners[2]] * w, polygon[corners[3]] * h),
                (polygon[corners[4]] * w, polygon[corners[5]] * h),
            ));
        }
    }
    let mut mask = DetectionMask::new(None);
    for y in 0..120 {
        for x in 0..160 {
            for triangle in &triangles {
                if point_in_triangle(*triangle, (x as f64, y as f64)) {
                    mask.set_pos(x, y);
                }
            }
        }
    }
    Ok(mask)
}

fn from_time_abs_or_rel_str<'de, D>(deserializer: D) -> Result<AbsRelTime, D::Error>
where
    D: Deserializer<'de>,
{
    let s: String = Deserialize::deserialize(deserializer)?;

    // NOTE: This is probably not that robust on all possible input strings – but we should solve this
    //  with better validation/UI elsewhere where users are inputting time offsets
    let mut tokens: Vec<NumberString> = Vec::new();
    for char in s.chars() {
        match char {
            '-' | '+' | '0' | '1' | '2' | '3' | '4' | '5' | '6' | '7' | '8' | '9' => {
                if let Some(NumberString(n, _, _)) = tokens.last_mut() {
                    n.push(char);
                } else {
                    tokens.push(NumberString(String::from(char), None, true));
                }
            }
            's' | 'h' | 'm' | 'z' => {
                if let Some(NumberString(_, o, _)) = tokens.last_mut() {
                    *o = Some(TimeUnit(char));
                } else {
                    // Parse error
                    return Err(Error::custom(format!(
                        "Unexpected token in time string '{s}': unit specifier before integer"
                    )));
                }
                tokens.push(NumberString(String::from(""), None, true));
            }
            ':' => {
                let count = tokens.len();
                if let Some(NumberString(_, o, is_relative)) = tokens.last_mut() {
                    if count == 1 {
                        *o = Some(TimeUnit('h'));
                    } else if count == 2 {
                        *o = Some(TimeUnit('m'));
                    } else if count == 3 {
                        *o = Some(TimeUnit('s'));
                    };
                    *is_relative = false;
                } else {
                    // Parse error
                    return Err(Error::custom(format!(
                        "Unexpected token in time string '{s}': ':' before hour specifier"
                    )));
                }
                tokens.push(NumberString(String::from(""), None, false));
            }
            _ => {
                return Err(Error::custom(format!(
                    "Unexpected token in time string '{s}': '{char}'"
                )));
            }
        }
    }
    let mut relative_time_seconds = None;
    let mut absolute_time = None;
    for token in &tokens {
        if token.2 {
            if relative_time_seconds.is_none() {
                relative_time_seconds = Some(0);
            }
        } else if absolute_time.is_none() {
            absolute_time = Some(HourMin { hour: 0, min: 0 });
        }

        if let Some(ref mut seconds) = relative_time_seconds {
            if let Ok(mut num) = token.0.parse::<i32>() {
                if let Some(unit) = &token.1 {
                    let mul = match unit.0 {
                        's' => 1,
                        'm' => 60,
                        'h' => 60 * 60,
                        _ => 1,
                    };
                    num *= mul;
                } else {
                    num *= 60; // Default unit is minutes if none specified
                }
                if *seconds < 0 && num > 0 {
                    *seconds += -num;
                } else {
                    *seconds += num;
                }
            }
        } else if let Some(ref mut hour_min) = absolute_time {
            if let Ok(num) = token.0.parse::<i32>() {
                if let Some(unit) = &token.1 {
                    match unit.0 {
                        'm' => hour_min.min = num as u8,
                        'h' => hour_min.hour = num as u8,
                        _ => {}
                    };
                } else {
                    hour_min.min = num as u8
                }
            }
        }
    }
    if absolute_time.is_none() && relative_time_seconds.is_none() {
        Err(Error::custom(format!("Failed to parse window time: {s}")))
    } else {
        Ok(AbsRelTime { absolute_time, relative_time_seconds })
    }
}

fn timestamp_to_u64<'de, D>(deserializer: D) -> Result<Option<u64>, D::Error>
where
    D: Deserializer<'de>,
{
    let date_time: toml::value::Datetime = Deserialize::deserialize(deserializer)?;
    let date = date_time.date.expect("Should have date");
    let time = date_time.time.expect("should have time");
    let offset = date_time.offset.expect("should have offset");
    let offset_minutes = match offset {
        Offset::Z => 0,
        Offset::Custom { minutes } => minutes,
    } as i32;
    let fixed_offset = if offset_minutes < 0 {
        FixedOffset::east_opt(offset_minutes * 60)
    } else {
        FixedOffset::west_opt(offset_minutes * 60)
    };
    if let Some(fixed_offset) = fixed_offset {
        let naive_utc = NaiveDateTime::new(
            NaiveDate::from_ymd_opt(date.year as i32, date.month as u32, date.day as u32).unwrap(),
            NaiveTime::from_hms_nano_opt(
                time.hour as u32,
                time.minute as u32,
                time.second as u32,
                time.nanosecond,
            )
            .unwrap(),
        )
        .add(fixed_offset);
        let local = DateTime::<Utc>::from_naive_utc_and_offset(naive_utc, Utc);
        Ok(Some(local.with_timezone(&Utc).timestamp_micros() as u64))
    } else {
        Ok(None)
    }
}

fn location_accuracy_to_f32<'de, D>(deserializer: D) -> Result<Option<f32>, D::Error>
where
    D: Deserializer<'de>,
{
    let location_accuracy: f32 = Deserialize::deserialize(deserializer)?;
    if location_accuracy == 0.0 { Ok(None) } else { Ok(Some(location_accuracy)) }
}

#[derive(Deserialize, Debug, PartialEq, Clone)]
struct LocationSettings {
    latitude: Option<f32>,
    longitude: Option<f32>,
    altitude: Option<f32>,

    #[serde(deserialize_with = "timestamp_to_u64", default = "default_location_timestamp")]
    timestamp: Option<u64>,
    #[serde(deserialize_with = "location_accuracy_to_f32", default = "default_location_accuracy")]
    accuracy: Option<f32>,
}

#[derive(Debug, PartialEq, Clone, Copy)]
struct HourMin {
    hour: u8,
    min: u8,
}

fn timezone_offset_seconds() -> i32 {
    // IMPORTANT: This relies on the system timezone being set correctly to the same locale as the
    // devices' GPS coordinates to work out correct absolute start/end recording window times.
    let now = Local::now();
    let local_tz = now.timezone();
    local_tz.offset_from_utc_datetime(&now.naive_utc()).local_minus_utc()
}
#[derive(PartialEq, Clone)]
pub struct AbsRelTime {
    absolute_time: Option<HourMin>,
    relative_time_seconds: Option<i32>,
}

impl Debug for AbsRelTime {
    fn fmt(&self, f: &mut Formatter<'_>) -> fmt::Result {
        let absolute_time = self.absolute_time;
        let relative_time = self.relative_time_seconds;
        if let Some(time) = absolute_time {
            return f
                .debug_struct("AbsoluteTime")
                .field("hour", &time.hour)
                .field("min", &time.min)
                .finish();
        }
        if let Some(time) = relative_time {
            return f.debug_struct("RelativeOffset").field("secs", &time).finish();
        }
        Err(fmt::Error)
    }
}

impl AbsRelTime {
    pub fn time_offset(&self) -> (bool, i32) {
        // Absolute or relative time in seconds in the day
        if let Some(abs_time) = &self.absolute_time {
            // NOTE: We need to convert this to UTC offsets, since that's what our timestamp is.
            let seconds_past_midnight =
                (abs_time.hour as i32 * 60 * 60) + (abs_time.min as i32 * 60);
            //info!("Seconds past midnight local {}", seconds_past_midnight);
            let tz_offset = timezone_offset_seconds();
            // info!(
            //     "TZ offset {}, seconds past UTC midnight {}",
            //     tz_offset,
            //     (seconds_past_midnight - tz_offset) % 86_400
            // );
            (true, (seconds_past_midnight - tz_offset) % 86_400)
        } else {
            (false, self.relative_time_seconds.unwrap())
        }
    }
}

#[derive(Deserialize, Debug, PartialEq, Clone)]
struct TimeWindow {
    #[serde(
        rename = "start-recording",
        deserialize_with = "from_time_abs_or_rel_str",
        default = "default_recording_start_time"
    )]
    start_recording: AbsRelTime,
    #[serde(
        rename = "stop-recording",
        deserialize_with = "from_time_abs_or_rel_str",
        default = "default_recording_stop_time"
    )]
    stop_recording: AbsRelTime,
}

impl Default for TimeWindow {
    fn default() -> Self {
        TimeWindow {
            start_recording: default_recording_start_time(),
            stop_recording: default_recording_stop_time(),
        }
    }
}

#[repr(u8)]
#[derive(Deserialize, Debug, PartialEq, Clone)]
pub enum AudioMode {
    Disabled = 0,
    AudioOnly = 1,
    AudioOrThermal = 2,
    AudioAndThermal = 3,
}

impl From<AudioMode> for u8 {
    fn from(audio_mode: AudioMode) -> u8 {
        audio_mode as u8
    }
}

impl FromStr for AudioMode {
    type Err = ();

    fn from_str(input: &str) -> Result<AudioMode, Self::Err> {
        match input {
            "Disabled" => Ok(AudioMode::Disabled),
            "AudioOnly" => Ok(AudioMode::AudioOnly),
            "AudioOrThermal" => Ok(AudioMode::AudioOrThermal),
            "AudioAndThermal" => Ok(AudioMode::AudioAndThermal),
            _ => Err(()),
        }
    }
}

impl TryFrom<u8> for AudioMode {
    type Error = ();

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        use AudioMode::*;

        match value {
            0 => Ok(AudioOnly),
            1 => Ok(AudioOrThermal),
            2 => Ok(AudioAndThermal),
            _ => Err(()),
        }
    }
}

#[derive(Deserialize, Debug, PartialEq, Clone)]
pub struct AudioSettings {
    #[serde(
        rename = "audio-mode",
        default = "default_audio_mode",
        deserialize_with = "deserialize_audio_mode"
    )]
    pub audio_mode: AudioMode,
    #[serde(rename = "random-seed", default = "default_audio_seed")]
    pub audio_seed: u32,
}

impl Default for AudioSettings {
    fn default() -> Self {
        AudioSettings { audio_mode: default_audio_mode(), audio_seed: 0 }
    }
}

fn default_audio_seed() -> u32 {
    0
}

fn default_audio_mode() -> AudioMode {
    AudioMode::Disabled
}

fn deserialize_audio_mode<'de, D>(deserializer: D) -> Result<AudioMode, D::Error>
where
    D: Deserializer<'de>,
{
    let audio_mode_raw: String = Deserialize::deserialize(deserializer)?;
    if let Ok(mode) = AudioMode::from_str(&audio_mode_raw) {
        Ok(mode)
    } else {
        Err(Error::custom(format!("Failed to parse audio mode: {audio_mode_raw}")))
    }
}

#[derive(Deserialize, Debug, PartialEq, Clone)]
struct DeviceRegistration {
    id: Option<u32>,
    group: Option<String>,
    name: Option<String>,
    server: Option<String>,
}

#[derive(Deserialize, Debug, PartialEq, Clone)]
struct ThermalRecordingSettings {
    #[serde(rename = "output-dir", default = "default_output_dir")]
    output_dir: String,
    #[serde(rename = "constant-recorder", default = "default_constant_recorder")]
    constant_recorder: bool,
    #[serde(rename = "use-low-power-mode", default = "default_low_power_mode")]
    use_low_power_mode: bool,
    #[serde(rename = "min-disk-space-mb", default = "default_min_disk_space_mb")]
    min_disk_space_mb: u32,
    #[serde(
        rename = "mask-regions",
        default = "default_mask_regions",
        deserialize_with = "deserialize_mask_regions"
    )]
    mask_regions: DetectionMask,
}

impl Default for ThermalRecordingSettings {
    fn default() -> Self {
        ThermalRecordingSettings {
            output_dir: default_output_dir(),
            constant_recorder: default_constant_recorder(),
            min_disk_space_mb: default_min_disk_space_mb(),
            use_low_power_mode: default_low_power_mode(),
            mask_regions: default_mask_regions(),
        }
    }
}

#[derive(Deserialize, Debug, PartialEq, Clone)]
struct ThermalThrottlerSettings {
    #[serde(default = "default_activate_thermal_throttler")]
    activate: bool,
}

#[derive(Deserialize, Debug, PartialEq, Clone)]
pub struct DeviceConfig {
    #[serde(rename = "audio-recording", default)]
    audio_info: AudioSettings,
    #[serde(rename = "windows", default)]
    recording_window: TimeWindow,
    #[serde(rename = "device")]
    device_info: Option<DeviceRegistration>,
    #[serde(rename = "thermal-recorder", default)]
    recording_settings: ThermalRecordingSettings,
    location: Option<LocationSettings>,
}

impl DeviceConfig {
    pub fn has_location(&self) -> bool {
        if let Some(location_settings) = &self.location {
            location_settings.longitude.is_some() && location_settings.latitude.is_some()
        } else {
            false
        }
    }
    pub fn is_registered(&self) -> bool {
        if let Some(device) = &self.device_info {
            device.id.is_some() && device.name.is_some() && device.group.is_some()
        } else {
            false
        }
    }

    // Only call these once we know the device is registered
    pub fn device_id(&self) -> u32 {
        self.device_info.as_ref().unwrap().id.unwrap()
    }

    pub fn device_name(&self) -> &[u8] {
        self.device_info.as_ref().unwrap().name.as_ref().unwrap().as_bytes()
    }

    pub fn lat_lng(&self) -> (f32, f32) {
        (
            self.location.as_ref().unwrap().latitude.unwrap(),
            self.location.as_ref().unwrap().longitude.unwrap(),
        )
    }
    pub fn location_timestamp(&self) -> Option<u64> {
        self.location.as_ref().unwrap().timestamp
    }
    pub fn location_altitude(&self) -> Option<f32> {
        self.location.as_ref().unwrap().altitude
    }
    pub fn location_accuracy(&self) -> Option<f32> {
        self.location.as_ref().unwrap().accuracy
    }
    pub fn recording_window(&self) -> (AbsRelTime, AbsRelTime) {
        (
            self.recording_window.start_recording.clone(),
            self.recording_window.stop_recording.clone(),
        )
    }

    pub fn mask_piece(&self, index: usize) -> &[u8] {
        let piece_length = 100;
        let offset = index * piece_length;
        &self.recording_settings.mask_regions.inner()[offset..2400.min(offset + piece_length)]
    }

    #[allow(unused)]
    pub fn mask(&self) -> &DetectionMask {
        &self.recording_settings.mask_regions
    }

    pub fn output_dir(&self) -> &str {
        &self.recording_settings.output_dir
    }

    pub fn is_continuous_recorder(&self) -> bool {
        self.recording_settings.constant_recorder
            || (self.recording_window.start_recording.absolute_time.is_some()
                && self.recording_window.stop_recording.absolute_time.is_some()
                && self.recording_window.start_recording == self.recording_window.stop_recording)
    }
    pub fn use_low_power_mode(&self) -> bool {
        self.recording_settings.use_low_power_mode
    }

    pub fn use_high_power_mode(&self) -> bool {
        !self.recording_settings.use_low_power_mode
    }

    pub fn load_from_fs() -> Result<DeviceConfig, &'static str> {
        let config_toml =
            fs::read("/etc/cacophony/config.toml").map_err(|_| "Error reading file from disk")?;
        let config_toml_str =
            String::from_utf8(config_toml).map_err(|_| "Error parsing string from utf8")?;
        let device_config: Result<DeviceConfig, toml::de::Error> = toml::from_str(&config_toml_str);
        match device_config {
            Ok(device_config) => {
                // TODO: Make sure device has sane windows etc.
                if !device_config.has_location() {
                    error!(
                        "No location set for this device. To enter recording mode, a location must be set."
                    );
                    // TODO: Event log error?
                    process::exit(1);
                }
                if !device_config.is_registered() {
                    error!(
                        "This device is not yet registered.  \
                    To enter recording mode the device must be named assigned to a project."
                    );
                    // TODO: Event log error?
                    process::exit(1);
                }
                info!("Got config {:?}", device_config);
                if device_config.audio_info.audio_mode != AudioMode::AudioOnly {
                    let inside_recording_window =
                        device_config.time_is_in_recording_window(&Utc::now().naive_utc());
                    info!("Inside recording window: {inside_recording_window}");
                    if !inside_recording_window {
                        device_config.print_next_recording_window(&Utc::now().naive_utc());
                    }
                }
                Ok(device_config)
            }
            Err(msg) => {
                error!("Toml parse error: {msg:?}");
                Err("Error deserializing TOML config")
            }
        }
    }

    pub fn next_recording_window(&self, now_utc: &NaiveDateTime) -> (NaiveDateTime, NaiveDateTime) {
        let (is_absolute_start, mut start_offset) =
            self.recording_window.start_recording.time_offset();
        let (is_absolute_end, mut end_offset) = self.recording_window.stop_recording.time_offset();
        if is_absolute_end && end_offset < 0 {
            end_offset += 86_400;
        }
        if is_absolute_start && start_offset < 0 {
            start_offset += 86_400;
        }
        let (window_start, window_end) = if !is_absolute_start || !is_absolute_end {
            let location =
                self.location.as_ref().expect("Relative recording windows require a location");
            let (lat, lng) = (
                location.latitude.expect("Relative recording windows require a valid latitude"),
                location.longitude.expect("Relative recording windows require a valid longitude"),
            );
            let altitude = location.altitude;
            let yesterday_utc = *now_utc - Duration::days(1);
            let (_, yesterday_sunset) = sun_times(
                yesterday_utc.date(),
                lat as f64,
                lng as f64,
                altitude.unwrap_or(0.0) as f64,
            )
            .unwrap();
            let yesterday_sunset =
                yesterday_sunset.naive_utc() + Duration::seconds(start_offset as i64);
            let (today_sunrise, today_sunset) =
                sun_times(now_utc.date(), lat as f64, lng as f64, altitude.unwrap_or(0.0) as f64)
                    .unwrap();
            let today_sunrise = today_sunrise.naive_utc() + Duration::seconds(end_offset as i64);
            let today_sunset = today_sunset.naive_utc() + Duration::seconds(start_offset as i64);
            let tomorrow_utc = *now_utc + Duration::days(1);
            let (tomorrow_sunrise, tomorrow_sunset) = sun_times(
                tomorrow_utc.date(),
                lat as f64,
                lng as f64,
                altitude.unwrap_or(0.0) as f64,
            )
            .unwrap();
            let tomorrow_sunrise =
                tomorrow_sunrise.naive_utc() + Duration::seconds(end_offset as i64);
            let tomorrow_sunset =
                tomorrow_sunset.naive_utc() + Duration::seconds(start_offset as i64);

            if *now_utc > today_sunset && *now_utc > tomorrow_sunrise {
                let two_days_from_now_utc = *now_utc + Duration::days(2);
                let (two_days_sunrise, _) = sun_times(
                    two_days_from_now_utc.date(),
                    lat as f64,
                    lng as f64,
                    altitude.unwrap_or(0.0) as f64,
                )
                .unwrap();
                let two_days_sunrise =
                    two_days_sunrise.naive_utc() + Duration::seconds(end_offset as i64);
                (Some(tomorrow_sunset), Some(two_days_sunrise))
            } else if (*now_utc > today_sunset && *now_utc < tomorrow_sunrise)
                || (*now_utc < today_sunset && *now_utc > today_sunrise)
            {
                (Some(today_sunset), Some(tomorrow_sunrise))
            } else if *now_utc < tomorrow_sunset && *now_utc < today_sunrise {
                (Some(yesterday_sunset), Some(today_sunrise))
            } else {
                panic!("Unable to calculate relative time window");
            }
        } else {
            (None, None)
        };

        let mut start_time = if !is_absolute_start {
            window_start.unwrap()
        } else {
            NaiveDateTime::new(
                now_utc.date(),
                NaiveTime::from_num_seconds_from_midnight_opt(start_offset as u32, 0).unwrap(),
            )
        };
        let mut end_time = if !is_absolute_end {
            window_end.unwrap()
        } else {
            NaiveDateTime::new(
                now_utc.date(),
                NaiveTime::from_num_seconds_from_midnight_opt(end_offset as u32, 0).unwrap(),
            )
        };

        if is_absolute_start || is_absolute_end {
            let start_minus_one_day = start_time - Duration::days(1);
            let mut start_plus_one_day = start_time + Duration::days(1);
            let mut end_minus_one_day = end_time - Duration::days(1);
            let end_plus_one_day = end_time + Duration::days(1);

            if start_minus_one_day > end_minus_one_day {
                end_minus_one_day += Duration::days(1);
            }
            if start_plus_one_day > end_plus_one_day {
                start_plus_one_day = start_time;
            }
            if end_minus_one_day > *now_utc {
                if is_absolute_start {
                    start_time = start_minus_one_day;
                }
                if is_absolute_end {
                    end_time = end_minus_one_day;
                }
            }
            if end_time < start_time && is_absolute_end {
                end_time = end_plus_one_day;
            }
            if *now_utc > end_time {
                if is_absolute_start {
                    start_time = start_plus_one_day;
                }
                if is_absolute_end {
                    end_time = end_plus_one_day;
                }
            }
        }
        (start_time, end_time)
    }

    #[allow(unused)]
    pub fn next_recording_window_start(&self, now_utc: &NaiveDateTime) -> NaiveDateTime {
        self.next_recording_window(now_utc).0
    }

    pub fn print_next_recording_window(&self, date_time_utc: &NaiveDateTime) {
        let (start_time, end_time) = self.next_recording_window(date_time_utc);
        let starts_in = start_time - *date_time_utc;
        let starts_in_hours = starts_in.num_hours();
        let starts_in_mins = starts_in.num_minutes() - (starts_in_hours * 60);
        let ends_in = end_time - *date_time_utc;
        let ends_in_hours = ends_in.num_hours();
        let ends_in_mins = ends_in.num_minutes() - (ends_in_hours * 60);
        let window = end_time - start_time;
        let window_hours = window.num_hours();
        let window_mins = window.num_minutes() - (window_hours * 60);
        info!(
            "Next recording window will start in {starts_in_hours}h{starts_in_mins}m \
            and end in {ends_in_hours}h{ends_in_mins}m, \
            window duration {window_hours}h{window_mins}m",
        );
    }
    pub fn time_is_in_recording_window(&self, date_time_utc: &NaiveDateTime) -> bool {
        if self.is_continuous_recorder() {
            return true;
        }
        let (start_time, end_time) = self.next_recording_window(date_time_utc);
        *date_time_utc >= start_time && *date_time_utc <= end_time
    }

    pub fn is_audio_device(&self) -> bool {
        self.audio_info.audio_mode != AudioMode::Disabled
    }

    pub fn is_thermal_device(&self) -> bool {
        self.audio_info.audio_mode != AudioMode::AudioOnly
    }

    pub fn write_to_slice(
        &self,
        output: &mut [u8],
        prefer_not_to_offload_files_now: bool,
        force_offload_files_now: bool,
    ) {
        let mut buf = Cursor::new(output);
        let device_id = self.device_id();
        buf.write_u32::<LittleEndian>(device_id).unwrap();
        let audio_mode: u8 = self.audio_info.audio_mode.clone().into();
        buf.write_u8(audio_mode).unwrap();
        let (latitude, longitude) = self.lat_lng();
        buf.write_f32::<LittleEndian>(latitude).unwrap();
        buf.write_f32::<LittleEndian>(longitude).unwrap();
        let (has_loc_timestamp, timestamp) = if let Some(timestamp) = self.location_timestamp() {
            (1u8, timestamp)
        } else {
            (0u8, 0)
        };
        buf.write_u8(has_loc_timestamp).unwrap();
        buf.write_u64::<LittleEndian>(timestamp).unwrap();
        let (has_loc_altitude, altitude) = if let Some(altitude) = self.location_altitude() {
            (1u8, altitude)
        } else {
            (0u8, 0.0)
        };
        buf.write_u8(has_loc_altitude).unwrap();
        buf.write_f32::<LittleEndian>(altitude).unwrap();
        let (has_loc_accuracy, accuracy) = if let Some(accuracy) = self.location_accuracy() {
            (1u8, accuracy)
        } else {
            (0u8, 0.0)
        };
        buf.write_u8(has_loc_accuracy).unwrap();
        buf.write_f32::<LittleEndian>(accuracy).unwrap();
        let (abs_rel_start, abs_rel_end) = self.recording_window();
        let (start_is_abs, start_seconds_offset) = abs_rel_start.time_offset();
        let (end_is_abs, end_seconds_offset) = abs_rel_end.time_offset();
        buf.write_u8(if start_is_abs { 1 } else { 0 }).unwrap();
        buf.write_i32::<LittleEndian>(start_seconds_offset).unwrap();
        buf.write_u8(if end_is_abs { 1 } else { 0 }).unwrap();
        buf.write_i32::<LittleEndian>(end_seconds_offset).unwrap();
        buf.write_u8(if self.is_continuous_recorder() { 1 } else { 0 }).unwrap();
        buf.write_u8(if self.use_low_power_mode() { 1 } else { 0 }).unwrap();
        let device_name = self.device_name();
        let device_name_length = device_name.len().min(63);
        buf.write_u8(device_name_length as u8).unwrap();
        buf.write_all(&device_name[0..device_name_length]).unwrap();
        buf.write_u32::<LittleEndian>(self.audio_info.audio_seed).unwrap();
        buf.write_u8(if prefer_not_to_offload_files_now { 5 } else { 0 }).unwrap();
        buf.write_u8(if force_offload_files_now { 1 } else { 0 }).unwrap();
    }
}

pub fn watch_local_config_file_changes(
    mut current_config: DeviceConfig,
    config_tx: &Sender<DeviceConfig>,
) -> RecommendedWatcher {
    let config_tx = config_tx.clone();
    let mut watcher = notify::recommended_watcher(move |res| match res {
        Ok(Event { kind, .. }) => {
            if let EventKind::Access(AccessKind::Close(AccessMode::Write)) = kind {
                // File got written to
                // Send event to
                match DeviceConfig::load_from_fs() {
                    Ok(config) => {
                        // Send to rp2040 to write via a channel somehow.
                        // Maybe we have to restart the rp2040 here so that it re-handshakes
                        // and picks up the new info
                        if config != current_config {
                            current_config = config;
                            warn!("Config updated");
                            let _ = config_tx.send(current_config.clone());
                        }
                    }
                    Err(msg) => {
                        error!("Load config error: {msg}");
                        process::exit(1);
                    }
                }
            }
        }
        Err(e) => error!("file watch error for /etc/cacophony/config.toml: {e:?}"),
    })
    .map_err(|e| {
        error!("{}", e);
        process::exit(1);
    })
    .unwrap();
    // Add a path to be watched. All files and directories at that path and
    // below will be monitored for changes.
    watcher
        .watch(Path::new("/etc/cacophony/config.toml"), RecursiveMode::NonRecursive)
        .map_err(|e| {
            error!("File watcher setup error: {e}");
            process::exit(1);
        })
        .unwrap();
    watcher
}

// FIXME: Use RecordingModeState here?
pub fn check_for_device_config_changes(
    device_config_change_channel_rx: &Receiver<DeviceConfig>,
    device_config: &mut DeviceConfig,
    is_audio_device: &mut bool,
    is_thermal_device: &mut bool,
    rp2040_needs_reset: &mut bool,
) {
    // Check once per frame to see if the config file may have been changed
    let updated_config = device_config_change_channel_rx.try_recv();
    match updated_config {
        Ok(config) => {
            // NOTE: Defer this till a time we know the rp2040 isn't writing to flash memory.
            if *device_config != config {
                info!("Config updated, should update rp2040");
                *device_config = config;
                *is_audio_device = device_config.is_audio_device();
                *is_thermal_device = device_config.is_thermal_device();
            }
            *rp2040_needs_reset = true;
        }
        Err(kind) => match kind {
            TryRecvError::Empty => {}
            TryRecvError::Disconnected => {
                warn!("Disconnected from config file watcher channel");
            }
        },
    }
}
