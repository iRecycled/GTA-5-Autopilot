//! Fault injection on a bus: drop, corrupt or delay frames, by message, with
//! a probability and a time window. Deterministic: each bus draws from its
//! own seeded generator, so a run with faults replays exactly.

/// What happens to a matching frame.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum FaultKind {
    /// Never transmitted: the sender's frame is lost (with p = 1, the sender
    /// is silent).
    Drop,
    /// Damaged on the wire: it takes its bus time, fails the CAN CRC, and no
    /// receiver gets it.
    Corrupt,
    /// Transmitted this much later (s).
    Delay(f64),
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Fault {
    /// The message id, or `None` for every frame on the bus.
    pub id: Option<u32>,
    pub kind: FaultKind,
    /// Probability per frame, 0..=1.
    pub p: f64,
    /// Active for bus times in `[from, until)` (s).
    pub from: f64,
    pub until: f64,
}

impl Fault {
    /// A fault on every matching frame for all time.
    pub fn always(id: Option<u32>, kind: FaultKind) -> Self {
        Self { id, kind, p: 1.0, from: f64::NEG_INFINITY, until: f64::INFINITY }
    }

    pub fn matches(&self, id: u32, t: f64) -> bool {
        self.id.is_none_or(|i| i == id) && t >= self.from && t < self.until
    }
}

/// splitmix64: small, fast and good enough for fault draws.
#[derive(Clone, Debug)]
pub struct Rng(u64);

impl Rng {
    pub fn seeded(name: &str) -> Self {
        // FNV-1a of the bus name.
        let mut h: u64 = 0xcbf2_9ce4_8422_2325;
        for b in name.bytes() {
            h = (h ^ b as u64).wrapping_mul(0x0100_0000_01b3);
        }
        Self(h)
    }

    /// Uniform in [0, 1).
    pub fn next_f64(&mut self) -> f64 {
        self.0 = self.0.wrapping_add(0x9e37_79b9_7f4a_7c15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xbf58_476d_1ce4_e5b9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94d0_49bb_1331_11eb);
        z ^= z >> 31;
        (z >> 11) as f64 / (1u64 << 53) as f64
    }
}
