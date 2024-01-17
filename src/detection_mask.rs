#[derive(Debug, PartialEq, Clone)]
pub struct DetectionMask {
    inner: [u8; 2400],
}

impl DetectionMask {
    pub fn new(mask: Option<[u8; 2400]>) -> DetectionMask {
        DetectionMask {
            inner: mask.unwrap_or([0u8; 2400]),
        }
    }
    pub fn is_masked_at_pos(&self, x: usize, y: usize) -> bool {
        let index = (y * 160) + x;
        self.inner[index >> 3] & (1 << (index % 8)) != 0
    }

    pub fn set_index(&mut self, index: usize) {
        self.inner[index >> 3] |= 1 << (index % 8);
    }

    pub fn set_pos(&mut self, x: usize, y: usize) {
        let i = (y * 160) + x;
        self.inner[i >> 3] |= 1 << (i % 8);
    }

    #[inline(always)]
    pub fn is_masked_at_index(&self, index: usize) -> bool {
        let group = self.inner[index >> 3];
        group != 0 && group & (1 << (index % 8)) != 0
    }

    pub fn inner(&self) -> &[u8; 2400] {
        &self.inner
    }
}
