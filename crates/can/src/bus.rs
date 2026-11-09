//! A timing model of one CAN bus.
//!
//! Each frame's length comes from its real bitstream: the fields, the CRC
//! (CRC-15 for classic CAN, CRC-17/21 for FD) and the stuff bits (a bit of the
//! opposite level after five equal bits). Classic CAN runs at the nominal bit
//! rate throughout; CAN FD with bit-rate switching sends its data phase at the
//! data bit rate. When several frames are waiting, the lowest id wins
//! arbitration. Bus load is busy time over elapsed time.
//!
//! Delivery (D23): `Reactive` delivers every frame queued in a step at that
//! step's time (no added latency, still accounting load); `Realistic` delivers
//! each frame when its transmission ends.

use std::collections::VecDeque;

use crate::fault::{Fault, FaultKind, Rng};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Delivery {
    Reactive,
    Realistic,
}

#[derive(Clone, Debug, PartialEq)]
pub struct BusConfig {
    pub name: String,
    /// Nominal (arbitration) bit rate (bit/s).
    pub nominal_bps: u32,
    /// CAN FD data-phase bit rate; `None` = classic CAN.
    pub data_bps: Option<u32>,
    pub delivery: Delivery,
}

#[derive(Clone, Debug, PartialEq)]
pub struct Frame {
    pub id: u32,
    pub data: Vec<u8>,
    /// Time it was queued (s).
    pub queued: f64,
    /// Time it was delivered (s), set by the bus.
    pub delivered: f64,
    /// Damaged on the wire: it fails the CAN CRC and every receiver discards it.
    pub crc_error: bool,
}

/// A frame's length in bits: (at the nominal rate, at the data rate).
pub fn frame_bits(id: u32, data: &[u8], fd: bool) -> (u32, u32) {
    let ext = id > 0x7FF;
    if !fd {
        classic_bits(id, ext, data)
    } else {
        fd_bits(id, ext, data)
    }
}

fn push_bits(v: &mut Vec<bool>, value: u64, n: u32) {
    for i in (0..n).rev() {
        v.push((value >> i) & 1 == 1);
    }
}

/// Stuff bits needed in `bits` (after five equal bits, one opposite bit).
fn stuff_count(bits: &[bool]) -> u32 {
    let (mut count, mut run, mut last) = (0u32, 0u32, None);
    for &b in bits {
        if Some(b) == last {
            run += 1;
        } else {
            run = 1;
            last = Some(b);
        }
        if run == 5 {
            count += 1;
            // The stuff bit is the opposite level and starts a new run.
            last = Some(!b);
            run = 1;
        }
    }
    count
}

fn crc(bits: &[bool], poly: u32, width: u32, init: u32) -> u32 {
    let top = 1u32 << (width - 1);
    let m = (1u32 << width) - 1;
    let mut c = init;
    for &b in bits {
        let fb = ((c & top) != 0) ^ b;
        c = (c << 1) & m;
        if fb {
            c ^= poly;
        }
    }
    c
}

fn classic_bits(id: u32, ext: bool, data: &[u8]) -> (u32, u32) {
    let mut v = vec![false]; // SOF (dominant)
    if ext {
        push_bits(&mut v, (id >> 18) as u64, 11);
        v.push(true); // SRR
        v.push(true); // IDE
        push_bits(&mut v, (id & 0x3FFFF) as u64, 18);
        v.push(false); // RTR
        v.push(false); // r1
        v.push(false); // r0
    } else {
        push_bits(&mut v, id as u64, 11);
        v.push(false); // RTR
        v.push(false); // IDE
        v.push(false); // r0
    }
    push_bits(&mut v, data.len().min(8) as u64, 4);
    for &b in &data[..data.len().min(8)] {
        push_bits(&mut v, b as u64, 8);
    }
    let c = crc(&v, 0x4599, 15, 0);
    push_bits(&mut v, c as u64, 15);
    let stuffed = v.len() as u32 + stuff_count(&v);
    // CRC delimiter, ACK slot and delimiter, EOF (7), interframe space (3).
    (stuffed + 1 + 2 + 7 + 3, 0)
}

/// CAN FD payload sizes.
pub fn fd_dlc_len(len: usize) -> usize {
    [0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64].into_iter().find(|&l| l >= len).unwrap_or(64)
}

fn fd_bits(id: u32, ext: bool, data: &[u8]) -> (u32, u32) {
    let len = fd_dlc_len(data.len());
    // Arbitration phase (nominal rate): SOF, id, RRS, IDE, FDF, res, BRS.
    let mut arb = vec![false];
    if ext {
        push_bits(&mut arb, (id >> 18) as u64, 11);
        arb.push(true);
        arb.push(true);
        push_bits(&mut arb, (id & 0x3FFFF) as u64, 18);
    } else {
        push_bits(&mut arb, id as u64, 11);
    }
    arb.extend_from_slice(&[false, false, true, false, true]); // RRS, IDE(0 for std), FDF, res, BRS
    // Data phase (data rate): ESI, DLC, payload; dynamic stuffing applies here.
    let mut d = vec![false];
    let dlc = [0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64].iter().position(|&l| l == len).unwrap_or(15);
    push_bits(&mut d, dlc as u64, 4);
    for i in 0..len {
        push_bits(&mut d, data.get(i).copied().unwrap_or(0) as u64, 8);
    }
    let dyn_stuff = stuff_count(&[arb.clone(), d.clone()].concat());
    // Stuff count (4 bits incl. parity) and CRC (17 or 21) with fixed stuff
    // bits every 4 bits, then the CRC delimiter (still at the data rate).
    let crc_len: u32 = if len <= 16 { 17 } else { 21 };
    let fixed_stuff = (4 + crc_len).div_ceil(4);
    let data_bits = d.len() as u32 + 4 + crc_len + fixed_stuff + 1;
    // Nominal: arbitration (+ its dynamic stuffing share), ACK (2), EOF (7), IFS (3).
    let nominal = arb.len() as u32 + dyn_stuff + 2 + 7 + 3;
    (nominal, data_bits)
}

pub struct Bus {
    pub cfg: BusConfig,
    queue: VecDeque<Frame>,
    now: f64,
    busy_until: f64,
    busy_time: f64,
    since: f64,
    /// Time the bus has been busy since it started (s). Never reset, so a
    /// viewer can compute load over any window.
    pub busy_total: f64,
    pub delivered_total: u64,
    faults: Vec<Fault>,
    rng: Rng,
    /// Frames held back by a delay fault: (release time, frame).
    delayed: Vec<(f64, Frame)>,
    /// Frames the faults dropped, corrupted and delayed so far.
    pub faulted: [u64; 3],
}

impl Bus {
    pub fn new(cfg: BusConfig) -> Self {
        let rng = Rng::seeded(&cfg.name);
        Self {
            cfg,
            queue: VecDeque::new(),
            now: 0.0,
            busy_until: 0.0,
            busy_time: 0.0,
            since: 0.0,
            busy_total: 0.0,
            delivered_total: 0,
            faults: Vec::new(),
            rng,
            delayed: Vec::new(),
            faulted: [0; 3],
        }
    }

    /// Adds a fault; every fault that matches a frame applies, in order.
    pub fn inject(&mut self, f: Fault) {
        self.faults.push(f);
    }

    pub fn clear_faults(&mut self) {
        self.faults.clear();
    }

    pub fn faults(&self) -> &[Fault] {
        &self.faults
    }

    /// Transmission time of a frame on this bus (s).
    pub fn frame_time(&self, id: u32, data: &[u8]) -> f64 {
        let (n, d) = frame_bits(id, data, self.cfg.data_bps.is_some());
        n as f64 / self.cfg.nominal_bps as f64 + self.cfg.data_bps.map(|r| d as f64 / r as f64).unwrap_or(0.0)
    }

    pub fn send(&mut self, id: u32, mut data: Vec<u8>) {
        let mut delay = 0.0;
        let mut crc_error = false;
        for f in &self.faults {
            if !f.matches(id, self.now) || self.rng.next_f64() >= f.p {
                continue;
            }
            match f.kind {
                FaultKind::Drop => {
                    self.faulted[0] += 1;
                    return;
                }
                FaultKind::Corrupt => {
                    // One bit flipped on the wire.
                    if let Some(b) = data.first_mut() {
                        *b ^= 0x01;
                    }
                    crc_error = true;
                    self.faulted[1] += 1;
                }
                FaultKind::Delay(d) => {
                    delay += d;
                    self.faulted[2] += 1;
                }
            }
        }
        let frame = Frame { id, data, queued: self.now, delivered: f64::NAN, crc_error };
        if delay > 0.0 {
            self.delayed.push((self.now + delay, frame));
        } else {
            self.queue.push_back(frame);
        }
    }

    /// Advances to time `t` (s); returns the frames delivered, in bus order.
    pub fn step(&mut self, t: f64) -> Vec<Frame> {
        // Delayed frames whose time has come join the queue.
        if !self.delayed.is_empty() {
            let mut i = 0;
            while i < self.delayed.len() {
                if self.delayed[i].0 <= t {
                    let (at, mut f) = self.delayed.remove(i);
                    f.queued = at;
                    self.queue.push_back(f);
                } else {
                    i += 1;
                }
            }
        }
        let mut out = Vec::new();
        // Arbitration: whenever the bus is free, the lowest waiting id goes.
        let mut clock = self.busy_until.max(self.now);
        while !self.queue.is_empty() {
            let start = clock.max(self.queue.iter().map(|f| f.queued).fold(f64::INFINITY, f64::min));
            if self.cfg.delivery == Delivery::Realistic && start > t {
                break;
            }
            let (i, _) = self
                .queue
                .iter()
                .enumerate()
                .filter(|(_, f)| f.queued <= start)
                .min_by_key(|(_, f)| f.id)
                .expect("a frame is waiting");
            let mut f = self.queue.remove(i).expect("index valid");
            let dur = self.frame_time(f.id, &f.data);
            let end = start + dur;
            if self.cfg.delivery == Delivery::Realistic && end > t {
                // Still on the wire at `t`: put it back and wait.
                self.queue.push_front(f);
                break;
            }
            self.busy_time += dur;
            self.busy_total += dur;
            clock = end;
            f.delivered = match self.cfg.delivery {
                Delivery::Reactive => t,
                Delivery::Realistic => end,
            };
            // A frame that fails its CRC took its bus time, but nobody gets it.
            // (Real controllers send an error frame and retry; not modelled.)
            if !f.crc_error {
                self.delivered_total += 1;
                out.push(f);
            }
        }
        self.busy_until = clock;
        self.now = t;
        out
    }

    /// Bus load since the last call, 0..=1 (can exceed 1 when overloaded in
    /// reactive mode), and resets the window.
    pub fn take_load(&mut self) -> f64 {
        let span = (self.now - self.since).max(1e-9);
        let l = self.busy_time / span;
        self.busy_time = 0.0;
        self.since = self.now;
        l
    }

    /// Bus time (s): the last `step`'s `t`.
    pub fn now(&self) -> f64 {
        self.now
    }

    pub fn backlog(&self) -> usize {
        self.queue.len() + self.delayed.len()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn classic(delivery: Delivery) -> Bus {
        Bus::new(BusConfig { name: "chassis".into(), nominal_bps: 500_000, data_bps: None, delivery })
    }

    #[test]
    fn classic_frame_lengths() {
        // An 8-byte standard frame: 111 bits before stuffing (incl. IFS 3), plus
        // at most 24 stuff bits.
        let (n, d) = frame_bits(0x123, &[0x55; 8], false);
        assert_eq!(d, 0);
        assert!((111..=135).contains(&n), "{n} bits");
        // All-zero data stuffs a lot more than alternating data.
        let (zeros, _) = frame_bits(0x000, &[0; 8], false);
        let (alt, _) = frame_bits(0x2AA, &[0x55; 8], false);
        assert!(zeros > alt, "zeros {zeros}, alternating {alt}");
        // At 500 kbit/s an 8-byte frame takes about 0.22–0.27 ms.
        let t = classic(Delivery::Realistic).frame_time(0x123, &[0x55; 8]);
        assert!((0.000_22..0.000_27).contains(&t), "{t} s");
    }

    #[test]
    fn crc15_known_vector() {
        // CRC-15/CAN of the bits 0b1 (a single recessive bit) is the polynomial itself.
        assert_eq!(crc(&[true], 0x4599, 15, 0), 0x4599);
        assert_eq!(crc(&[false; 20], 0x4599, 15, 0), 0);
    }

    #[test]
    fn fd_data_phase_is_faster() {
        let fd = Bus::new(BusConfig { name: "adas".into(), nominal_bps: 500_000, data_bps: Some(2_000_000), delivery: Delivery::Realistic });
        let t64 = fd.frame_time(0x200, &[0xA5; 64]);
        // 64 bytes at 2 Mbit/s ≈ 0.26 ms of data phase plus ≈ 0.06 ms of arbitration.
        assert!((0.000_25..0.000_45).contains(&t64), "{t64} s");
        assert_eq!(fd_dlc_len(9), 12);
        assert_eq!(fd_dlc_len(33), 48);
    }

    #[test]
    fn lowest_id_wins_and_realistic_delivery_takes_time() {
        let mut b = classic(Delivery::Realistic);
        b.send(0x300, vec![1; 8]);
        b.send(0x100, vec![2; 8]);
        b.send(0x200, vec![3; 8]);
        let out = b.step(0.01);
        let ids: Vec<u32> = out.iter().map(|f| f.id).collect();
        assert_eq!(ids, vec![0x100, 0x200, 0x300]);
        assert!(out[0].delivered > 0.0 && out[2].delivered > out[1].delivered);
        // Load: three frames of ~0.25 ms in 10 ms ≈ 7–8 %.
        let load = b.take_load();
        assert!((0.06..0.09).contains(&load), "load {load}");
    }

    #[test]
    fn realistic_holds_a_frame_still_on_the_wire() {
        let mut b = classic(Delivery::Realistic);
        b.send(0x100, vec![0; 8]);
        assert!(b.step(0.000_1).is_empty(), "delivered before its last bit");
        assert_eq!(b.step(0.001).len(), 1);
    }

    #[test]
    fn faults_drop_corrupt_and_delay() {
        let mut b = classic(Delivery::Reactive);
        b.inject(Fault::always(Some(0x100), FaultKind::Drop));
        b.inject(Fault::always(Some(0x200), FaultKind::Corrupt));
        b.inject(Fault::always(Some(0x300), FaultKind::Delay(0.05)));
        b.send(0x100, vec![0; 8]);
        b.send(0x200, vec![0; 8]);
        b.send(0x300, vec![0; 8]);
        b.send(0x400, vec![0; 8]);
        let out = b.step(0.01);
        let ids: Vec<u32> = out.iter().map(|f| f.id).collect();
        assert_eq!(ids, vec![0x400], "dropped, corrupted (CRC) and delayed frames missing");
        assert!(b.take_load() > 0.0);
        assert!(b.step(0.04).is_empty());
        let late = b.step(0.06);
        assert_eq!(late.len(), 1);
        assert_eq!(late[0].id, 0x300);
        assert_eq!(b.faulted, [1, 1, 1]);
    }

    #[test]
    fn faults_respect_probability_window_and_are_deterministic() {
        let run = || {
            let mut b = classic(Delivery::Reactive);
            b.inject(Fault { id: None, kind: FaultKind::Drop, p: 0.3, from: 1.0, until: 2.0 });
            let mut got = Vec::new();
            for i in 0..300 {
                let t = i as f64 * 0.01;
                b.send(0x100, vec![0; 8]);
                got.push(!b.step(t).is_empty());
            }
            got
        };
        let a = run();
        assert_eq!(a, run(), "same draws every run");
        // Nothing lost outside the window; about 30 % inside it.
        assert!(a[..100].iter().all(|&x| x) && a[201..].iter().all(|&x| x));
        let lost = a[101..201].iter().filter(|&&x| !x).count();
        assert!((15..=45).contains(&lost), "{lost} of 100 lost");
    }

    #[test]
    fn reactive_delivers_in_the_same_step() {
        let mut b = classic(Delivery::Reactive);
        b.send(0x100, vec![0; 8]);
        b.send(0x050, vec![0; 8]);
        let out = b.step(0.0);
        assert_eq!(out.len(), 2);
        assert_eq!(out[0].id, 0x050);
        assert!(out.iter().all(|f| f.delivered == 0.0));
    }
}
