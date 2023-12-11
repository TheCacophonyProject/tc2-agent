// Read camera config file
use byteorder::{LittleEndian, WriteBytesExt};
use chrono::{DateTime, FixedOffset, Local, NaiveDate, NaiveDateTime, NaiveTime, TimeZone, Utc};
use log::{error, info};
use serde::{Deserialize, Deserializer};
use std::fs;
use std::io::{Cursor, Write};
use std::ops::Add;
use toml::value::Offset;

fn default_constant_recorder() -> bool {
    false
}
fn default_low_power_mode() -> bool {
    false
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
    AbsRelTime {
        relative_time_seconds: Some(-(60 * 30)),
        absolute_time: None,
    }
}

fn default_recording_stop_time() -> AbsRelTime {
    AbsRelTime {
        relative_time_seconds: Some(60 * 30),
        absolute_time: None,
    }
}

#[derive(Debug)]
struct TimeUnit(char);

#[derive(Debug)]
struct NumberString(String, Option<TimeUnit>, bool);

fn from_time_abs_or_rel_str<'de, D>(deserializer: D) -> Result<AbsRelTime, D::Error>
where
    D: Deserializer<'de>,
{
    let s: String = Deserialize::deserialize(deserializer)?;

    info!("Deserialising time from config {}", s);
    // NOTE: This is probably not that robust on all possible input strings – but we should solve this
    //  with better validation/UI elsewhere where users are inputting time offsets
    let mut tokens: Vec<NumberString> = Vec::new();
    for char in s.chars() {
        match char {
            '-' | '+' | '0' | '1' | '2' | '3' | '4' | '5' | '6' | '7' | '8' | '9' => {
                if let Some(NumberString(ref mut n, _, _)) = tokens.last_mut() {
                    n.push(char);
                } else {
                    tokens.push(NumberString(String::from(char), None, true));
                }
            }
            's' | 'h' | 'm' | 'z' => {
                if let Some(NumberString(_, ref mut o, _)) = tokens.last_mut() {
                    *o = Some(TimeUnit(char));
                } else {
                    // Parse error
                }
                tokens.push(NumberString(String::from(""), None, true));
            }
            ':' => {
                let count = tokens.len();
                if let Some(NumberString(_, ref mut o, ref mut is_relative)) = tokens.last_mut() {
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
                }
                tokens.push(NumberString(String::from(""), None, false));
            }
            _ => {} // Skip unknown, maybe log a parse error?
        }
    }
    let mut relative_time_seconds = None;
    let mut absolute_time = None;
    for token in &tokens {
        if token.2 {
            if relative_time_seconds.is_none() {
                relative_time_seconds = Some(0);
            }
        } else {
            if absolute_time.is_none() {
                absolute_time = Some(HourMin { hour: 0, min: 0 });
            }
        }
        if let Some(ref mut seconds) = relative_time_seconds {
            if let Ok(mut num) = i32::from_str_radix(&token.0, 10) {
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
            if let Ok(num) = i32::from_str_radix(&token.0, 10) {
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

    Ok(AbsRelTime {
        absolute_time,
        relative_time_seconds,
    })
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
    if location_accuracy == 0.0 {
        Ok(None)
    } else {
        Ok(Some(location_accuracy))
    }
}

#[derive(Deserialize, Debug, PartialEq, Clone)]
struct LocationSettings {
    latitude: Option<f32>,
    longitude: Option<f32>,
    altitude: Option<f32>,

    #[serde(
        deserialize_with = "timestamp_to_u64",
        default = "default_location_timestamp"
    )]
    timestamp: Option<u64>,
    #[serde(
        deserialize_with = "location_accuracy_to_f32",
        default = "default_location_accuracy"
    )]
    accuracy: Option<f32>,
}

#[derive(Debug, PartialEq, Clone)]
struct HourMin {
    hour: u8,
    min: u8,
}

fn timezone_offset_seconds() -> i32 {
    // IMPORTANT: This relies on the system timezone being set correctly to the same locale as the
    // devices' GPS coordinates to work out correct absolute start/end recording window times.
    let now = Local::now();
    let local_tz = now.timezone();
    local_tz
        .offset_from_utc_datetime(&now.naive_utc())
        .local_minus_utc()
}
#[derive(Debug, PartialEq, Clone)]
pub struct AbsRelTime {
    absolute_time: Option<HourMin>,
    relative_time_seconds: Option<i32>,
}

impl AbsRelTime {
    pub fn time_offset(&self) -> (bool, i32) {
        // Absolute or relative time in seconds in the day
        if let Some(abs_time) = &self.absolute_time {
            // NOTE: We need to convert this to UTC offsets, since that's what our timestamp is.
            let seconds_past_midnight =
                (abs_time.hour as i32 * 60 * 60) + (abs_time.min as i32 * 60);
            info!("Seconds past midnight local {}", seconds_past_midnight);
            let tz_offset = timezone_offset_seconds();
            info!(
                "TZ offset {}, seconds past UTC midnight {}",
                tz_offset,
                (seconds_past_midnight - tz_offset) % 86_400
            );
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
}

impl Default for ThermalRecordingSettings {
    fn default() -> Self {
        ThermalRecordingSettings {
            output_dir: default_output_dir(),
            constant_recorder: default_constant_recorder(),
            min_disk_space_mb: default_min_disk_space_mb(),
            use_low_power_mode: default_low_power_mode(),
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
        self.device_info
            .as_ref()
            .unwrap()
            .name
            .as_ref()
            .unwrap()
            .as_bytes()
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

    pub fn output_dir(&self) -> &str {
        &self.recording_settings.output_dir
    }

    pub fn is_continuous_recorder(&self) -> bool {
        self.recording_settings.constant_recorder
    }
    pub fn use_low_power_mode(&self) -> bool {
        self.recording_settings.use_low_power_mode
    }

    pub fn load_from_fs() -> Result<DeviceConfig, &'static str> {
        let config_toml =
            fs::read("/etc/cacophony/config.toml").map_err(|_| "Error reading file from disk")?;
        let config_toml_str =
            String::from_utf8(config_toml).map_err(|_| "Error parsing string from utf8")?;
        let device_config: DeviceConfig =
            toml::from_str(&config_toml_str).map_err(|_| "Error deserializing toml config")?;

        // TODO: Make sure device has sane windows etc.
        if !device_config.has_location() {
            error!(
                "No location set for this device. To enter recording mode, a location must be set."
            );
            // TODO: Event log error?
            std::process::exit(1);
        }
        if !device_config.is_registered() {
            error!("This device is not yet registered.  To enter recording mode the device must be named assigned to a project.");
            // TODO: Event log error?
            std::process::exit(1);
        }
        info!("Got config {:?}", device_config);
        Ok(device_config)
    }

    pub fn write_to_slice(&self, output: &mut [u8]) {
        let mut buf = Cursor::new(output);
        let device_id = self.device_id();
        buf.write_u32::<LittleEndian>(device_id).unwrap();

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
        buf.write_u8(if self.is_continuous_recorder() { 1 } else { 0 })
            .unwrap();
        buf.write_u8(if self.use_low_power_mode() { 1 } else { 0 })
            .unwrap();

        let device_name = self.device_name();
        let device_name_length = device_name.len().min(63);
        buf.write_u8(device_name_length as u8).unwrap();
        buf.write(&device_name[0..device_name_length]).unwrap();
    }
}

#[cfg(test)]
mod tests {
    use crate::device_config::DeviceConfig;

    #[test]
    fn load_config() {
        let config: DeviceConfig = toml::from_str(
            r#"
[device]
id = 1
group = "test-group"
name = "test-name"
server = "test-url"

[thermal-recorder]
use-sunrise-sunset = false
max-secs = 300
min-disk-space-mb = 200
min-secs = 5
output-dir = "/var/spool/cptv"
preview-secs = 1

[location]
accuracy = 0.0
altitude = 103.0
latitude = -46.60101
longitude = 172.71303
timestamp = 2023-11-02T08:24:21+13:00
updated = 2023-11-02T08:24:21+13:00

[thermal-throttler]
activate = true

[windows]
start-recording = "30m"
stop-recording = "07:50"
"#,
        )
        .unwrap();
        println!("Config {:#?}", config)
    }
}
