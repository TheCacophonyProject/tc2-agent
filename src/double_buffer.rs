use crate::cptv_frame_dispatch::Frame;
use std::cell::RefCell;
use std::sync::Mutex;
use std::sync::atomic::{AtomicUsize, Ordering};

pub struct DoubleBuffer {
    pub front: Mutex<RefCell<Option<Frame>>>,
    pub back: Mutex<RefCell<Option<Frame>>>,
    swapper: AtomicUsize,
}

impl DoubleBuffer {
    pub const fn new() -> DoubleBuffer {
        DoubleBuffer {
            front: Mutex::new(RefCell::new(None)),
            back: Mutex::new(RefCell::new(None)),
            swapper: AtomicUsize::new(0),
        }
    }

    pub fn swap(&self) {
        self.swapper.fetch_add(1, Ordering::Relaxed);
    }

    pub fn get_front(&self) -> &Mutex<RefCell<Option<Frame>>> {
        let val = self.swapper.load(Ordering::Acquire);
        if val.is_multiple_of(2) {
            &self.front
        } else {
            &self.back
        }
    }

    pub fn get_back(&self) -> &Mutex<RefCell<Option<Frame>>> {
        let val = self.swapper.load(Ordering::Acquire);
        if val.is_multiple_of(2) {
            &self.back
        } else {
            &self.front
        }
    }
}
