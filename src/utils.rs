pub fn u8_slice_as_u16_slice(p: &[u8]) -> &[u16] {
    assert_eq!(
        (p.len() / 2) * 2,
        p.len(),
        "slice must be evenly divisible by 2"
    );
    unsafe { core::slice::from_raw_parts((p as *const [u8]) as *const u16, p.len() / 2) }
}
