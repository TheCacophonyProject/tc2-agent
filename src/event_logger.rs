use rustbus::{DuplexConn, MessageBuilder};

#[derive(Debug)]
pub enum LoggerEventKind {
    Rp2040Sleep,
    OffloadedRecording(u64),
    SavedNewConfig,
    StartedSendingFramesToRpi,
    StartedRecording(u64),
    EndedRecording(u64),
    ToldRpiToSleep,
    GotRpiPoweredDown,
    GotRpiPoweredOn,
    ToldRpiToWake(u64),
    LostSync,
    SetAlarm(u64), // Also has a time that the alarm is set for as additional data?  Events can be bigger
    GotPowerOnTimeout,
    WouldDiscardAsFalsePositive,
    StartedGettingFrames,
    FlashStorageNearlyFull,
    Rp2040WokenByAlarm,
    RtcCommError,
    AttinyCommError,
    Rp2040MissedAudioAlarm(u64),
    AudioRecordingFailed,
    RTCTime(u64),
    StartedAudioRecording(u64),
    ThermalMode,
    AudioMode,
    RecordingNotFinished,
    FileOffloadFailed,
    LogOffloadFailed,

    OffloadedLogs,
}

impl Into<u16> for LoggerEventKind {
    fn into(self) -> u16 {
        use LoggerEventKind::*;
        match self {
            Rp2040Sleep => 1,
            OffloadedRecording(_) => 2,
            SavedNewConfig => 3,
            StartedSendingFramesToRpi => 4,
            StartedRecording(_) => 5,
            EndedRecording(_) => 6,
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
            RTCTime(_) => 22,
            StartedAudioRecording(_) => 23,
            ThermalMode => 24,
            AudioMode => 25,
            RecordingNotFinished => 26,
            FileOffloadFailed => 27,
            OffloadedLogs => 28,
            LogOffloadFailed => 29,
        }
    }
}

impl TryFrom<u16> for LoggerEventKind {
    type Error = ();

    fn try_from(value: u16) -> Result<Self, Self::Error> {
        use LoggerEventKind::*;
        match value {
            1 => Ok(Rp2040Sleep),
            2 => Ok(OffloadedRecording(0)),
            3 => Ok(SavedNewConfig),
            4 => Ok(StartedSendingFramesToRpi),
            5 => Ok(StartedRecording(0)),
            6 => Ok(EndedRecording(0)),
            7 => Ok(ToldRpiToSleep),
            8 => Ok(GotRpiPoweredDown),
            9 => Ok(GotRpiPoweredOn),
            10 => Ok(ToldRpiToWake(0)),
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
            22 => Ok(RTCTime(0)),
            23 => Ok(StartedAudioRecording(0)),
            24 => Ok(ThermalMode),
            25 => Ok(AudioMode),
            26 => Ok(RecordingNotFinished),
            27 => Ok(FileOffloadFailed),
            28 => Ok(OffloadedLogs),
            29 => Ok(LogOffloadFailed),
            _ => Err(()),
        }
    }
}

pub struct LoggerEvent {
    timestamp: u64,
    event: LoggerEventKind,
}

impl LoggerEvent {
    pub fn new(event: LoggerEventKind, timestamp: u64) -> LoggerEvent {
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
            call.body
                .push_param(format!(r#"{{ "alarm-time": {} }}"#, alarm * 1000))
                .unwrap(); // Microseconds to nanoseconds
            call.body.push_param("SetAlarm").unwrap();
        } else if let LoggerEventKind::Rp2040MissedAudioAlarm(alarm) = self.event {
            call.body
                .push_param(format!(r#"{{ "alarm-time": {} }}"#, alarm * 1000))
                .unwrap(); // Microseconds to nanoseconds
            call.body.push_param("Rp2040MissedAudioAlarm").unwrap();
        } else if let LoggerEventKind::RTCTime(alarm) = self.event {
            call.body
                .push_param(format!(r#"{{ "alarm-time": {} }}"#, alarm * 1000))
                .unwrap(); // Microseconds to nanoseconds
            call.body.push_param("RTCTime").unwrap();
        } else if let LoggerEventKind::ToldRpiToWake(reason) = self.event {
            call.body
                .push_param(format!(r#"{{ "wakeup-reason": {} }}"#, reason))
                .unwrap(); // Microseconds to nanoseconds
            call.body.push_param("ToldRpiToWake").unwrap();
        } else if let LoggerEventKind::OffloadedRecording(reason) = self.event {
            call.body
                .push_param(format!(
                    r#"{{ "block": {},"page":{} }}"#,
                    (reason >> 32) as isize,
                    (reason & 0x0FFFFFFFF) as isize
                ))
                .unwrap(); // Microseconds to nanoseconds
            call.body.push_param("OffloadedRecording").unwrap();
        } else if let LoggerEventKind::StartedRecording(reason) = self.event {
            call.body
                .push_param(format!(
                    r#"{{ "block": {},"page":{} }}"#,
                    (reason >> 32) as isize,
                    (reason & 0x0FFFFFFFF) as isize
                ))
                .unwrap(); // Microseconds to nanoseconds
            call.body.push_param("StartedRecording").unwrap();
        } else if let LoggerEventKind::EndedRecording(reason) = self.event {
            call.body
                .push_param(format!(
                    r#"{{ "block": {},"page":{} }}"#,
                    (reason >> 32) as isize,
                    (reason & 0x0FFFFFFFF) as isize
                ))
                .unwrap(); // Microseconds to nanoseconds
            call.body.push_param("EndedRecording").unwrap();
        } else if let LoggerEventKind::StartedAudioRecording(reason) = self.event {
            call.body
                .push_param(format!(
                    r#"{{ "block": {},"page":{} }}"#,
                    (reason >> 32) as isize,
                    (reason & 0x0FFFFFFFF) as isize
                ))
                .unwrap(); // Microseconds to nanoseconds
            call.body.push_param("StartedAudioRecording").unwrap();
        } else {
            call.body
                .push_param(json_payload.unwrap_or(String::from("{}")))
                .unwrap();
            call.body.push_param(format!("{:?}", self.event)).unwrap();
        }

        call.body.push_param(self.timestamp * 1000).unwrap(); // Microseconds to nanoseconds
        conn.send.send_message(&call).unwrap().write_all().unwrap();
    }
}
