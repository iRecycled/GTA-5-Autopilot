//! The vehicle's CAN messages, generated at build time from its DBC files
//! (`build.rs`, `can::codegen`).
//!
//! - [`chassis`]: the Toyota TSS2 chassis bus (`dbc/toyota_nodsu_pt_generated.dbc`);
//! - [`adas`]: our ADAS CAN FD bus (`dbc/adas_canfd.dbc`).

#[allow(clippy::all, unused_mut, unused_variables, unused_parens, dead_code)]
pub mod chassis {
    include!(concat!(env!("OUT_DIR"), "/chassis.rs"));
}

#[allow(clippy::all, unused_mut, unused_variables, unused_parens, dead_code)]
pub mod adas {
    include!(concat!(env!("OUT_DIR"), "/adas.rs"));
}

#[cfg(test)]
mod tests {
    use super::*;
    use can::{CanMessage, Dbc};

    /// A small deterministic generator (xorshift64*).
    struct Rng(u64);
    impl Rng {
        fn next(&mut self) -> u64 {
            self.0 ^= self.0 >> 12;
            self.0 ^= self.0 << 25;
            self.0 ^= self.0 >> 27;
            self.0.wrapping_mul(0x2545_F491_4F6C_DD1D)
        }
        fn unit(&mut self) -> f64 {
            (self.next() >> 11) as f64 / (1u64 << 53) as f64
        }
    }

    type Dec = fn(u32, &[u8]) -> Option<Vec<(&'static str, f64)>>;
    type Enc = fn(u32, &[(&str, f64)]) -> Option<Vec<u8>>;

    /// Generated and run-time packing agree on every message, both ways:
    /// random payloads decode to the same values, and random values (in and
    /// beyond range) encode to the same bytes.
    fn agree(text: &str, dec: Dec, enc: Enc, table: &[(u32, &str, usize)]) {
        let dbc = Dbc::parse(text).unwrap();
        assert_eq!(table.len(), dbc.messages.len());
        let mut rng = Rng(0x9E37_79B9_7F4A_7C15);
        for m in &dbc.messages {
            for _ in 0..200 {
                let data: Vec<u8> = (0..m.len).map(|_| rng.next() as u8).collect();
                let got = dec(m.id, &data).unwrap();
                for (name, v) in &got {
                    let want = m.decode(&data, name).unwrap();
                    assert_eq!(v.to_bits(), want.to_bits(), "{}.{name}: generated {v}, run time {want}", m.name);
                }
                let values: Vec<(&str, f64)> = m
                    .signals
                    .iter()
                    .map(|s| {
                        let span = (s.max - s.min).abs().max(s.factor.abs());
                        let v = s.min - 0.1 * span + 1.2 * span * rng.unit();
                        // One-bit flags: send exactly 0 or 1.
                        let v = if s.len == 1 && !s.signed && s.factor == 1.0 && s.offset == 0.0 { (v > 0.5) as u8 as f64 } else { v };
                        (s.name.as_str(), v)
                    })
                    .collect();
                let a = enc(m.id, &values).unwrap();
                let b = m.encode(&values).unwrap();
                assert_eq!(a, b, "{}: encode differs for {values:?}", m.name);
            }
            // Default is the all-zero payload.
            assert!(enc(m.id, &[]).unwrap().iter().all(|&x| x == 0), "{} default", m.name);
        }
    }

    #[test]
    fn generated_chassis_messages_match_the_dbc() {
        agree(include_str!("../../../dbc/toyota_nodsu_pt_generated.dbc"), chassis::decode_signals, chassis::encode_signals, chassis::MESSAGES);
    }

    #[test]
    fn generated_adas_messages_match_the_dbc() {
        agree(include_str!("../../../dbc/adas_canfd.dbc"), adas::decode_signals, adas::encode_signals, adas::MESSAGES);
    }

    #[test]
    fn typed_access() {
        let lta = chassis::SteeringLta { steer_request: true, steer_angle_cmd: 12.3, ..Default::default() };
        let d = lta.encode();
        assert_eq!(d.len(), chassis::SteeringLta::LEN);
        let back = chassis::SteeringLta::decode(&d);
        assert!(back.steer_request);
        assert!((back.steer_angle_cmd - 12.3).abs() <= 0.0573 / 2.0);
        assert_eq!(chassis::id("STEERING_LTA"), Some(401));
        assert_eq!(chassis::name(401), Some("STEERING_LTA"));
        assert!(chassis::SteeringLta::has_signal("CHECKSUM") && !chassis::SteeringLta::has_signal("NOPE"));
        assert_eq!(adas::RadarObject::LEN, 16);
    }
}
