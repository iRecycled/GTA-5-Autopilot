//! A DBC parser covering what vehicle DBCs use: messages (`BO_`) and their
//! signals (`SG_`), Intel (`@1`) and Motorola (`@0`) byte order, signedness,
//! factor and offset. Multiplexed signals are parsed and flagged; comments,
//! value tables and attributes are skipped.

use std::collections::HashMap;

#[derive(Clone, Debug, PartialEq)]
pub struct Signal {
    pub name: String,
    /// DBC start bit: for Intel the LSB, for Motorola the MSB (DBC numbering).
    pub start: u32,
    pub len: u32,
    /// `true` = Intel (little endian, `@1`); `false` = Motorola (`@0`).
    pub little_endian: bool,
    pub signed: bool,
    pub factor: f64,
    pub offset: f64,
    pub min: f64,
    pub max: f64,
    pub unit: String,
    /// A multiplexed signal (`m<n>`) or the multiplexor (`M`).
    pub multiplexed: bool,
}

#[derive(Clone, Debug, PartialEq)]
pub struct Message {
    pub id: u32,
    pub name: String,
    pub len: usize,
    pub sender: String,
    pub signals: Vec<Signal>,
}

#[derive(Clone, Debug, Default)]
pub struct Dbc {
    pub messages: Vec<Message>,
    by_name: HashMap<String, usize>,
    by_id: HashMap<u32, usize>,
}

impl Dbc {
    pub fn parse(text: &str) -> Result<Self, String> {
        let mut messages: Vec<Message> = Vec::new();
        for (n, raw) in text.lines().enumerate() {
            let line = raw.trim();
            if let Some(rest) = line.strip_prefix("BO_ ") {
                // BO_ <id> <name>: <len> <sender>
                let (head, tail) = rest.split_once(':').ok_or(format!("line {}: bad BO_", n + 1))?;
                let mut h = head.split_whitespace();
                let id: u32 = h.next().and_then(|v| v.parse().ok()).ok_or(format!("line {}: bad id", n + 1))?;
                let name = h.next().ok_or(format!("line {}: no name", n + 1))?.to_string();
                let mut t = tail.split_whitespace();
                let len: usize = t.next().and_then(|v| v.parse().ok()).ok_or(format!("line {}: bad length", n + 1))?;
                let sender = t.next().unwrap_or("").to_string();
                // Extended ids carry bit 31 in DBC files.
                messages.push(Message { id: id & 0x1FFF_FFFF, name, len, sender, signals: Vec::new() });
            } else if let Some(rest) = line.strip_prefix("SG_ ") {
                let msg = messages.last_mut().ok_or(format!("line {}: SG_ before any BO_", n + 1))?;
                msg.signals.push(parse_signal(rest).map_err(|e| format!("line {}: {e}", n + 1))?);
            }
        }
        let by_name = messages.iter().enumerate().map(|(i, m)| (m.name.clone(), i)).collect();
        let by_id = messages.iter().enumerate().map(|(i, m)| (m.id, i)).collect();
        Ok(Self { messages, by_name, by_id })
    }

    pub fn message(&self, name: &str) -> Option<&Message> {
        self.by_name.get(name).map(|&i| &self.messages[i])
    }

    pub fn message_by_id(&self, id: u32) -> Option<&Message> {
        self.by_id.get(&id).map(|&i| &self.messages[i])
    }
}

fn parse_signal(rest: &str) -> Result<Signal, String> {
    // <name> [M|m<n>] : <start>|<len>@<order><sign> (<factor>,<offset>) [<min>|<max>] "<unit>" <receivers>
    let (head, body) = rest.split_once(':').ok_or("no ':'")?;
    let mut h = head.split_whitespace();
    let name = h.next().ok_or("no signal name")?.to_string();
    let multiplexed = h.next().is_some();
    let body = body.trim();
    let (layout, after) = body.split_once(' ').ok_or("no layout")?;
    let (pos, order) = layout.split_once('@').ok_or("no '@'")?;
    let (start, len) = pos.split_once('|').ok_or("no '|'")?;
    let start: u32 = start.parse().map_err(|_| "bad start")?;
    let len: u32 = len.parse().map_err(|_| "bad length")?;
    let mut oc = order.chars();
    let little_endian = match oc.next() {
        Some('1') => true,
        Some('0') => false,
        _ => return Err("bad byte order".into()),
    };
    let signed = match oc.next() {
        Some('-') => true,
        Some('+') => false,
        _ => return Err("bad sign".into()),
    };
    let after = after.trim();
    let fo = after.strip_prefix('(').and_then(|s| s.split_once(')')).ok_or("no (factor,offset)")?;
    let (factor, offset) = fo.0.split_once(',').ok_or("bad (factor,offset)")?;
    let rest2 = fo.1.trim();
    let mm = rest2.strip_prefix('[').and_then(|s| s.split_once(']')).ok_or("no [min|max]")?;
    let (min, max) = mm.0.split_once('|').ok_or("bad [min|max]")?;
    let unit = mm.1.trim().strip_prefix('"').and_then(|s| s.split_once('"')).map(|u| u.0.to_string()).unwrap_or_default();
    let num = |s: &str| s.trim().parse::<f64>().map_err(|_| format!("bad number {s}"));
    Ok(Signal {
        name,
        start,
        len,
        little_endian,
        signed,
        factor: num(factor)?,
        offset: num(offset)?,
        min: num(min)?,
        max: num(max)?,
        unit,
        multiplexed,
    })
}

impl Signal {
    /// Bit positions (0 = byte 0 bit 0) from the most to the least significant bit.
    fn bits(&self) -> Vec<u32> {
        let mut out = Vec::with_capacity(self.len as usize);
        if self.little_endian {
            for i in (0..self.len).rev() {
                out.push(self.start + i);
            }
        } else {
            // Motorola: start is the MSB; walk down within a byte, then to the
            // next byte's bit 7 (DBC "sawtooth" numbering).
            let mut b = self.start;
            for _ in 0..self.len {
                out.push(b);
                b = if b.is_multiple_of(8) { b + 15 } else { b - 1 };
            }
        }
        out
    }

    /// The raw (unscaled) value.
    pub fn get_raw(&self, data: &[u8]) -> u64 {
        let mut v = 0u64;
        for b in self.bits() {
            let byte = (b / 8) as usize;
            let bit = (data.get(byte).copied().unwrap_or(0) >> (b % 8)) & 1;
            v = (v << 1) | bit as u64;
        }
        v
    }

    pub fn set_raw(&self, data: &mut [u8], raw: u64) {
        let bits = self.bits();
        let n = bits.len();
        for (i, b) in bits.into_iter().enumerate() {
            let bit = (raw >> (n - 1 - i)) & 1;
            let byte = (b / 8) as usize;
            if byte < data.len() {
                let mask = 1u8 << (b % 8);
                data[byte] = if bit == 1 { data[byte] | mask } else { data[byte] & !mask };
            }
        }
    }

    /// The physical value.
    pub fn decode(&self, data: &[u8]) -> f64 {
        raw_to_phys(self.get_raw(data), self.len, self.signed, self.factor, self.offset)
    }

    /// Writes a physical value (rounded to the nearest step, saturated to the
    /// raw range).
    pub fn encode(&self, data: &mut [u8], value: f64) {
        self.set_raw(data, phys_to_raw(value, self.len, self.signed, self.factor, self.offset));
    }

    /// Bit runs within one byte, for code generation: (byte, lowest bit in
    /// the byte, width, the run's lowest bit in the raw value), skipping bytes
    /// at or past `msg_len`.
    pub fn runs(&self, msg_len: usize) -> Vec<(usize, u32, u32, u32)> {
        let bits = self.bits();
        let n = bits.len() as u32;
        let mut out = Vec::new();
        let mut i = 0;
        while i < bits.len() {
            let mut j = i;
            while j + 1 < bits.len() && bits[j + 1] + 1 == bits[j] && bits[j + 1] / 8 == bits[i] / 8 {
                j += 1;
            }
            let byte = (bits[i] / 8) as usize;
            if byte < msg_len {
                out.push((byte, bits[j] % 8, (j - i + 1) as u32, n - 1 - j as u32));
            }
            i = j + 1;
        }
        out
    }
}

/// A raw value to a physical one (sign-extends signed signals).
pub fn raw_to_phys(raw: u64, len: u32, signed: bool, factor: f64, offset: f64) -> f64 {
    let v = if signed && len < 64 && raw >> (len - 1) & 1 == 1 { raw as i64 - (1i64 << len) } else { raw as i64 };
    v as f64 * factor + offset
}

/// A physical value to a raw one: rounded to the nearest step, saturated to
/// the raw range.
pub fn phys_to_raw(value: f64, len: u32, signed: bool, factor: f64, offset: f64) -> u64 {
    let steps = ((value - offset) / factor).round();
    if signed {
        let lo = -(1i64 << (len - 1)) as f64;
        let hi = ((1i64 << (len - 1)) - 1) as f64;
        let v = steps.clamp(lo, hi) as i64;
        (v as u64) & mask(len)
    } else {
        steps.clamp(0.0, mask(len) as f64) as u64
    }
}

fn mask(len: u32) -> u64 {
    if len >= 64 { u64::MAX } else { (1u64 << len) - 1 }
}

impl Message {
    pub fn signal(&self, name: &str) -> Option<&Signal> {
        self.signals.iter().find(|s| s.name == name)
    }

    /// Encodes `values` (by signal name) into a fresh payload; other signals are 0.
    pub fn encode(&self, values: &[(&str, f64)]) -> Result<Vec<u8>, String> {
        let mut data = vec![0u8; self.len];
        for (name, v) in values {
            self.signal(name).ok_or(format!("{} has no signal {name}", self.name))?.encode(&mut data, *v);
        }
        Ok(data)
    }

    pub fn decode(&self, data: &[u8], name: &str) -> Option<f64> {
        self.signal(name).map(|s| s.decode(data))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn tss2() -> Dbc {
        let text = std::fs::read_to_string(concat!(env!("CARGO_MANIFEST_DIR"), "/../../dbc/toyota_nodsu_pt_generated.dbc")).unwrap();
        Dbc::parse(&text).unwrap()
    }

    #[test]
    fn parses_the_tss2_chassis_dbc() {
        let d = tss2();
        assert_eq!(d.messages.len(), 60);
        let lta = d.message("STEERING_LTA").expect("STEERING_LTA (D28)");
        assert_eq!((lta.id, lta.len), (401, 8));
        let cmd = lta.signal("STEER_ANGLE_CMD").unwrap();
        assert_eq!((cmd.start, cmd.len, cmd.little_endian, cmd.signed), (15, 16, false, true));
        assert!((cmd.factor - 0.0573).abs() < 1e-9);
        for m in ["WHEEL_SPEEDS", "STEER_ANGLE_SENSOR", "KINEMATICS", "SPEED", "BRAKE_MODULE", "EPS_STATUS", "ACC_CONTROL"] {
            assert!(d.message(m).is_some(), "{m} missing");
        }
        assert_eq!(d.message_by_id(401).unwrap().name, "STEERING_LTA");
    }

    #[test]
    fn motorola_bit_layout() {
        let d = tss2();
        let lta = d.message("STEERING_LTA").unwrap();
        // COUNTER: start 6, 6 bits, Motorola: bits 6..1 of byte 0.
        let mut data = vec![0u8; 8];
        lta.signal("COUNTER").unwrap().set_raw(&mut data, 0x3F);
        assert_eq!(data[0], 0x7E);
        // STEER_ANGLE_CMD: start 15, 16 bits: byte 1 is the high byte, byte 2 the low.
        let mut data = vec![0u8; 8];
        lta.signal("STEER_ANGLE_CMD").unwrap().set_raw(&mut data, 0xABCD);
        assert_eq!((data[1], data[2]), (0xAB, 0xCD));
    }

    #[test]
    fn round_trips_scaled_signed_values() {
        let d = tss2();
        let lta = d.message("STEERING_LTA").unwrap();
        for deg in [-540.0, -12.3456, 0.0, 0.0573, 45.0, 539.9] {
            let data = lta.encode(&[("STEER_ANGLE_CMD", deg), ("STEER_REQUEST", 1.0)]).unwrap();
            let back = lta.decode(&data, "STEER_ANGLE_CMD").unwrap();
            assert!((back - deg).abs() <= 0.0573 / 2.0 + 1e-9, "{deg} -> {back}");
            assert_eq!(lta.decode(&data, "STEER_REQUEST"), Some(1.0));
        }
        // Saturates instead of wrapping.
        let data = lta.encode(&[("STEER_ANGLE_CMD", 5000.0)]).unwrap();
        assert!(lta.decode(&data, "STEER_ANGLE_CMD").unwrap() > 1800.0);
    }

    #[test]
    fn our_adas_dbc_round_trips_an_object() {
        let text = std::fs::read_to_string(concat!(env!("CARGO_MANIFEST_DIR"), "/../../dbc/adas_canfd.dbc")).unwrap();
        let d = Dbc::parse(&text).unwrap();
        let obj = d.message("RADAR_OBJECT").unwrap();
        assert_eq!(obj.len, 16);
        let data = obj
            .encode(&[("SLOT", 3.0), ("TRACK_ID", 40001.0), ("DIST_X", 87.34), ("DIST_Y", -1.62), ("VEL_X", 22.25), ("VEL_Y", -0.4), ("STATIONARY", 0.0)])
            .unwrap();
        let get = |n: &str| obj.decode(&data, n).unwrap();
        assert_eq!(get("SLOT"), 3.0);
        assert_eq!(get("TRACK_ID"), 40001.0);
        assert!((get("DIST_X") - 87.34).abs() < 0.011);
        assert!((get("DIST_Y") + 1.62).abs() < 0.011);
        assert!((get("VEL_X") - 22.25).abs() < 0.026);
        assert!((get("VEL_Y") + 0.4).abs() < 0.026);
        // No two signals overlap.
        let mut used = [false; 128];
        for s in &obj.signals {
            for b in s.start..s.start + s.len {
                assert!(!used[b as usize], "bit {b} used twice ({})", s.name);
                used[b as usize] = true;
            }
        }
    }

    #[test]
    fn intel_signals() {
        let d = Dbc::parse(
            "BO_ 100 TEST: 8 X\n SG_ A : 0|12@1+ (0.5,-10) [0|1] \"m/s\" X\n SG_ B : 12|4@1- (1,0) [0|1] \"\" X\n",
        )
        .unwrap();
        let m = d.message("TEST").unwrap();
        let data = m.encode(&[("A", 100.0), ("B", -3.0)]).unwrap();
        assert_eq!(m.decode(&data, "A"), Some(100.0));
        assert_eq!(m.decode(&data, "B"), Some(-3.0));
        // A = (100 + 10) / 0.5 = 220 = 0x0DC in bits 0..11; B = -3 = 0xD in bits 12..15.
        assert_eq!((data[0], data[1]), (0xDC, 0xD0));
        assert_eq!(m.signal("A").unwrap().unit, "m/s");
    }
}
