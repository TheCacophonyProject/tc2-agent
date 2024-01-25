use crate::event_logger::LoggerEventKind::{
    EndedRecording, GotPowerOnTimeout, GotRpiPoweredDown, GotRpiPoweredOn, LostSync,
    OffloadedRecording, Rp2040Sleep, SavedNewConfig, SetAlarm, StartedRecording,
    StartedSendingFramesToRpi, ToldRpiToSleep, ToldRpiToWake, WouldDiscardAsFalsePositive,
};
use rustbus::{DuplexConn, MessageBuilder};

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
    ToldRpiToWake,
    LostSync,
    SetAlarm(u64), // Also has a time that the alarm is set for as additional data?  Events can be bigger
    GotPowerOnTimeout,
    WouldDiscardAsFalsePositive,
}

impl Into<u16> for LoggerEventKind {
    fn into(self) -> u16 {
        use LoggerEventKind::*;
        match self {
            Rp2040Sleep => 1,
            OffloadedRecording => 2,
            SavedNewConfig => 3,
            StartedSendingFramesToRpi => 4,
            StartedRecording => 5,
            EndedRecording => 6,
            ToldRpiToSleep => 7,
            GotRpiPoweredDown => 8,
            GotRpiPoweredOn => 9,
            ToldRpiToWake => 10,
            LostSync => 11,
            SetAlarm(_) => 12,
            GotPowerOnTimeout => 13,
            WouldDiscardAsFalsePositive => 14,
        }
    }
}

impl TryFrom<u16> for LoggerEventKind {
    type Error = ();

    fn try_from(value: u16) -> Result<Self, Self::Error> {
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
            10 => Ok(ToldRpiToWake),
            11 => Ok(LostSync),
            12 => Ok(SetAlarm(0)),
            13 => Ok(GotPowerOnTimeout),
            14 => Ok(WouldDiscardAsFalsePositive),
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
        if let SetAlarm(alarm) = self.event {
            call.body
                .push_param(format!(r#"{{ "alarm-time": {} }}"#, alarm * 1000))
                .unwrap(); // Microseconds to nanoseconds
        } else {
            call.body
                .push_param(json_payload.unwrap_or(String::from("{}")))
                .unwrap();
        }
        call.body.push_param(format!("{:?}", self.event)).unwrap();
        call.body.push_param(self.timestamp * 1000).unwrap(); // Microseconds to nanoseconds
        conn.send.send_message(&call).unwrap().write_all().unwrap();
    }
}
