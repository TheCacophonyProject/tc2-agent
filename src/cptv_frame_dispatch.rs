use crate::FRAME_LENGTH;
use crate::double_buffer::DoubleBuffer;
use crate::socket_stream::SocketStream;

pub type Frame = [u8; FRAME_LENGTH];
pub static FRAME_BUFFER: DoubleBuffer = DoubleBuffer::new();
pub fn get_frame(is_recording: bool) -> Option<[u8; 39040]> {
    let fb = { FRAME_BUFFER.get_front().lock().unwrap().take() };
    if let Some(mut fb) = fb {
        if is_recording {
            // Write recording flag into unused telemetry.
            fb[639] = 1;
        } else {
            fb[638] = 0;
        }
        return Some(fb);
    }
    None
}
pub fn send_frame(fb: [u8; 39040], stream: &mut SocketStream) -> bool {
    if stream.write_all(&fb).is_err() {
        return false;
    }
    stream.flush().is_ok()
}
