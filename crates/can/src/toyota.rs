//! Toyota message integrity as the TSS2 DBC uses it (from opendbc's
//! `toyota_checksum`): the last byte is a checksum over the address, the
//! length and the other bytes; many messages also carry a rolling counter.

/// Checksum over `data` (its last byte excluded) for message `addr`.
pub fn checksum(addr: u32, data: &[u8]) -> u8 {
    let mut s: u32 = (addr & 0xFF) + ((addr >> 8) & 0xFF) + data.len() as u32;
    for &b in &data[..data.len().saturating_sub(1)] {
        s += b as u32;
    }
    (s & 0xFF) as u8
}

/// Writes the checksum into the last byte.
pub fn seal(addr: u32, data: &mut [u8]) {
    if let Some(last) = data.len().checked_sub(1) {
        data[last] = checksum(addr, data);
    }
}

/// Checks a received frame's checksum.
pub fn verify(addr: u32, data: &[u8]) -> bool {
    !data.is_empty() && data[data.len() - 1] == checksum(addr, data)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn checksum_matches_the_reference_formula() {
        // addr 0x191 (STEERING_LTA), 8 bytes: (0x91 + 0x01 + 8 + sum of the first 7) & 0xFF.
        let mut d = [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x00];
        seal(0x191, &mut d);
        assert_eq!(d[7], ((0x91 + 0x01 + 8 + 28) & 0xFF) as u8);
        assert!(verify(0x191, &d));
        d[3] ^= 0x10;
        assert!(!verify(0x191, &d), "a corrupted byte must fail");
    }
}
