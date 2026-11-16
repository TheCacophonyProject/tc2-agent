use log::warn;
use std::time::{Duration, Instant};

/// How long to wait for a startup handshake after requesting an rp2040 reset before
/// allowing the request to be sent again. A normal reset + handshake takes a few seconds,
/// but the frame socket thread (which actually toggles the RUN pin) can be blocked in a
/// socket send for several seconds before it services the request.
pub const RP2040_RESET_TIMEOUT: Duration = Duration::from_secs(30);

/// Tracks reset requests made by the camera transfer loop.
///
/// A request is pending from the moment it's sent until the rp2040's next startup handshake.
/// Any handshake completes the request: the rp2040 has booted since we asked, and the
/// handshake response carries the current device config, so whatever the reset was for
/// has been done.
///
/// While a request is pending, further resets are not requested; config changes that arrive
/// meanwhile update device_config and are sent when the handshake arrives
#[derive(Debug)]
pub struct Rp2040ResetRequest {
    requested_at: Option<Instant>,
    timeout: Duration,
}

impl Rp2040ResetRequest {
    pub fn new(timeout: Duration) -> Self {
        Rp2040ResetRequest { requested_at: None, timeout }
    }

    /// Call when a reset is wanted. Returns true if a restart request should be sent to
    /// the frame socket thread now, false if one is already pending.
    pub fn should_send_request(&mut self, now: Instant) -> bool {
        match self.requested_at {
            Some(requested_at) if now.saturating_duration_since(requested_at) < self.timeout => {
                false
            }
            Some(requested_at) => {
                warn!(
                    "No rp2040 startup handshake {:?} after requesting a reset, requesting again",
                    now.saturating_duration_since(requested_at)
                );
                self.requested_at = Some(now);
                true
            }
            None => {
                self.requested_at = Some(now);
                true
            }
        }
    }

    /// Call when a startup handshake is received from the rp2040.
    pub fn handshake_received(&mut self) {
        self.requested_at = None;
    }

    #[cfg(test)]
    pub fn is_pending(&self) -> bool {
        self.requested_at.is_some()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn tracker() -> Rp2040ResetRequest {
        Rp2040ResetRequest::new(RP2040_RESET_TIMEOUT)
    }

    #[test]
    fn first_request_is_sent() {
        let mut reset = tracker();
        assert!(!reset.is_pending());
        assert!(reset.should_send_request(Instant::now()));
        assert!(reset.is_pending());
    }

    #[test]
    fn repeated_requests_while_pending_are_not_sent() {
        // e.g. every frame before the reboot sets rp2040_needs_reset again.
        let mut reset = tracker();
        let start = Instant::now();
        assert!(reset.should_send_request(start));
        for ms in [1, 100, 1_000, 10_000] {
            assert!(!reset.should_send_request(start + Duration::from_millis(ms)));
        }
        assert!(reset.is_pending());
    }

    #[test]
    fn handshake_completes_request_and_allows_another() {
        let mut reset = tracker();
        let start = Instant::now();
        assert!(reset.should_send_request(start));
        reset.handshake_received();
        assert!(!reset.is_pending());
        assert!(reset.should_send_request(start + Duration::from_secs(1)));
    }

    #[test]
    fn handshake_before_restart_is_serviced_does_not_get_stuck() {
        // The handshake can arrive before the frame socket thread has toggled the RUN pin
        // (e.g. the rp2040 rebooted on its own, or the restart and handshake both happened
        // while the transfer loop was blocked in poll_interrupt). Previously a late ack could
        // then mark a reset as in progress with no further handshake to clear it, and all
        // later config changes were ignored. Now the handshake alone completes the request.
        let mut reset = tracker();
        let start = Instant::now();
        assert!(reset.should_send_request(start));
        reset.handshake_received();
        // Nothing arriving afterwards can leave us pending.
        assert!(!reset.is_pending());
        // A later config change can request a reset straight away.
        assert!(reset.should_send_request(start + Duration::from_secs(2)));
    }

    #[test]
    fn stray_handshake_without_request_is_harmless() {
        let mut reset = tracker();
        reset.handshake_received();
        assert!(!reset.is_pending());
        assert!(reset.should_send_request(Instant::now()));
    }

    #[test]
    fn request_is_resent_if_no_handshake_before_timeout() {
        let mut reset = Rp2040ResetRequest::new(Duration::from_secs(30));
        let start = Instant::now();
        assert!(reset.should_send_request(start));
        assert!(!reset.should_send_request(start + Duration::from_secs(29)));
        assert!(reset.should_send_request(start + Duration::from_secs(30)));
        // The retry restarts the timeout.
        assert!(!reset.should_send_request(start + Duration::from_secs(31)));
        assert!(reset.should_send_request(start + Duration::from_secs(60)));
    }
}
