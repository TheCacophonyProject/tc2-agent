use rustbus::{DuplexConn, MessageBuilder};

#[repr(u8)]
#[derive(Debug, Clone, Copy)]

pub enum WakeReason {
    Unknown = 0,
    ThermalOffload = 1,
    ThermalOffloadAfter24Hours = 2,
    ThermalHighPower = 3,
    AudioThermalEnded = 4,
    AudioShouldOffload = 5,
    AudioTooFull = 6,
    ThermalTooFull = 7,
    EventsTooFull = 8,
    OffloadTestRecording = 9,
    OffloadOnUserDemand = 10,
}
impl std::fmt::Display for WakeReason {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        write!(f, "{self:?}")
    }
}

impl TryFrom<u8> for WakeReason {
    type Error = ();

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        use WakeReason::*;

        match value {
            0 => Ok(Unknown),
            1 => Ok(ThermalOffload),
            2 => Ok(ThermalOffloadAfter24Hours),
            3 => Ok(ThermalHighPower),
            4 => Ok(AudioThermalEnded),
            5 => Ok(AudioShouldOffload),
            6 => Ok(AudioTooFull),
            7 => Ok(ThermalTooFull),
            8 => Ok(EventsTooFull),
            9 => Ok(OffloadTestRecording),
            10 => Ok(OffloadOnUserDemand),
            _ => Err(()),
        }
    }
}

impl From<WakeReason> for u64 {
    fn from(value: WakeReason) -> Self {
        value as u64
    }
}

impl From<WakeReason> for u8 {
    fn from(value: WakeReason) -> Self {
        value as u8
    }
}

#[derive(Debug)]
pub enum LoggerEventKind {
    Rp2040Sleep,
    OffloadedRecording,
    SavedNewConfig,
    StartedSendingFramesToRpi,
    StartedRecording,
    EndedRecording,
    ToldRpiToSleep,
    GotRpiPoweredDown,
    GotRpiPoweredOn,
    ToldRpiToWake(WakeReason),
    LostSync,
    SetAlarm(i64), // Also has a time that the alarm is set for as additional data?  Events can be bigger
    GotPowerOnTimeout,
    WouldDiscardAsFalsePositive,
    StartedGettingFrames,
    FlashStorageNearlyFull,
    Rp2040WokenByAlarm,
    RtcCommError,
    AttinyCommError,
    Rp2040MissedAudioAlarm(i64),
    AudioRecordingFailed,
    ErasePartialOrCorruptRecording,
    StartedAudioRecording,
    ThermalMode,
    AudioMode,
    RecordingNotFinished,
    FileOffloadFailed,
    LogOffloadFailed,
    OffloadedLogs,
    CorruptFile,
    LostFrames(u64),
    FileOffloadInterruptedByUser,
}

impl From<LoggerEventKind> for u16 {
    fn from(val: LoggerEventKind) -> Self {
        use LoggerEventKind::*;
        match val {
            Rp2040Sleep => 1,
            OffloadedRecording => 2,
            SavedNewConfig => 3,
            StartedSendingFramesToRpi => 4,
            StartedRecording => 5,
            EndedRecording => 6,
            ToldRpiToSleep => 7,
            GotRpiPoweredDown => 8,
            GotRpiPoweredOn => 9,
            ToldRpiToWake(_) => 10,
            LostSync => 11,
            SetAlarm(_) => 12,
            GotPowerOnTimeout => 13,
            WouldDiscardAsFalsePositive => 14,
            StartedGettingFrames => 15,
            FlashStorageNearlyFull => 16,
            Rp2040WokenByAlarm => 17,
            RtcCommError => 18,
            AttinyCommError => 19,
            Rp2040MissedAudioAlarm(_) => 20,
            AudioRecordingFailed => 21,
            ErasePartialOrCorruptRecording => 22,
            StartedAudioRecording => 23,
            ThermalMode => 24,
            AudioMode => 25,
            RecordingNotFinished => 26,
            FileOffloadFailed => 27,
            OffloadedLogs => 28,
            LogOffloadFailed => 29,
            CorruptFile => 30,
            LostFrames(_) => 31,
            FileOffloadInterruptedByUser => 32,
        }
    }
}

impl TryFrom<u16> for LoggerEventKind {
    type Error = ();

    fn try_from(value: u16) -> Result<Self, Self::Error> {
        use LoggerEventKind::*;
        match value {
            1 => Ok(Rp2040Sleep),
            2 => Ok(OffloadedRecording),
            3 => Ok(SavedNewConfig),
            4 => Ok(StartedSendingFramesToRpi),
            5 => Ok(StartedRecording),
            6 => Ok(EndedRecording),
            7 => Ok(ToldRpiToSleep),
            8 => Ok(GotRpiPoweredDown),
            9 => Ok(GotRpiPoweredOn),
            10 => Ok(ToldRpiToWake(WakeReason::Unknown)),
            11 => Ok(LostSync),
            12 => Ok(SetAlarm(0)),
            13 => Ok(GotPowerOnTimeout),
            14 => Ok(WouldDiscardAsFalsePositive),
            15 => Ok(StartedGettingFrames),
            16 => Ok(FlashStorageNearlyFull),
            17 => Ok(Rp2040WokenByAlarm),
            18 => Ok(RtcCommError),
            19 => Ok(AttinyCommError),
            20 => Ok(Rp2040MissedAudioAlarm(0)),
            21 => Ok(AudioRecordingFailed),
            22 => Ok(ErasePartialOrCorruptRecording),
            23 => Ok(StartedAudioRecording),
            24 => Ok(ThermalMode),
            25 => Ok(AudioMode),
            26 => Ok(RecordingNotFinished),
            27 => Ok(FileOffloadFailed),
            28 => Ok(OffloadedLogs),
            29 => Ok(LogOffloadFailed),
            30 => Ok(CorruptFile),
            31 => Ok(LostFrames(0)),
            _ => Err(()),
        }
    }
}
pub struct LoggerEvent {
    timestamp: i64,
    event: LoggerEventKind,
}

impl LoggerEvent {
    pub fn new(event: LoggerEventKind, timestamp: i64) -> LoggerEvent {
        LoggerEvent { event, timestamp }
    }

    pub fn log(&self, conn: &mut DuplexConn, json_payload: Option<String>) {
        let mut call = MessageBuilder::new()
            .call("Add")
            .with_interface("org.cacophony.Events")
            .on("/org/cacophony/Events")
            .at("org.cacophony.Events")
            .build();
        // If the type is SavedNewConfig, maybe make the payload the config?
        if let LoggerEventKind::SetAlarm(alarm) = self.event {
            call.body.push_param(format!(r#"{{ "alarm-time": {} }}"#, alarm * 1000)).unwrap(); // Microseconds to nanoseconds
            call.body.push_param("SetAlarm").unwrap();
        } else if let LoggerEventKind::Rp2040MissedAudioAlarm(alarm) = self.event {
            call.body.push_param(format!(r#"{{ "alarm-time": {} }}"#, alarm * 1000)).unwrap(); // Microseconds to nanoseconds
            call.body.push_param("Rp2040MissedAudioAlarm").unwrap();
        } else if let LoggerEventKind::ToldRpiToWake(reason) = self.event {
            call.body.push_param(format!(r#"{{ "wakeup-reason": "{reason}" }}"#)).unwrap();
            call.body.push_param("ToldRpiToWake").unwrap();
        } else if let LoggerEventKind::LostFrames(lost_frames) = self.event {
            call.body.push_param(format!(r#"{{ "lost-frames": "{lost_frames}" }}"#)).unwrap();
            call.body.push_param("LostFrames").unwrap();
        } else {
            call.body.push_param(json_payload.unwrap_or(String::from("{}"))).unwrap();
            call.body.push_param(format!("{:?}", self.event)).unwrap();
        }

        call.body.push_param(self.timestamp * 1000).unwrap(); // Microseconds to nanoseconds
        conn.send.send_message(&call).unwrap().write_all().unwrap();
    }
}
