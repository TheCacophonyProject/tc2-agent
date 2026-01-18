use crate::cptv_frame_dispatch::Frame;
use crate::utils::u8_slice_as_u16_slice;
use byteorder::{BigEndian, ByteOrder, LittleEndian};

#[allow(unused)]
pub struct Telemetry {
    pub frame_num: u32,
    pub msec_on: u32,
    pub ffc_in_progress: bool,
    pub msec_since_last_ffc: u32,
}

pub fn read_telemetry(frame: &Frame) -> Telemetry {
    let mut buf = [0u8; 160];
    BigEndian::write_u16_into(u8_slice_as_u16_slice(&frame[0..160]), &mut buf);
    let frame_num = LittleEndian::read_u32(&buf[40..44]);
    let msec_on = LittleEndian::read_u32(&buf[2..6]);
    let time_at_last_ffc = LittleEndian::read_u32(&buf[60..64]);
    let msec_since_last_ffc = msec_on - time_at_last_ffc;
    let status_bits = LittleEndian::read_u32(&buf[6..10]);
    let ffc_state = (status_bits >> 4) & 0b11;
    let ffc_in_progress = ffc_state == 0b10;
    Telemetry { frame_num, msec_on, ffc_in_progress, msec_since_last_ffc }
}
