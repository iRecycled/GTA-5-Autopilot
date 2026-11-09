//! GTA V build-specific facts, and nothing else (D2, D5).
//!
//! This is the only place that knows vehicle-struct offsets or input quirks of
//! a particular game build. When a game update moves the offsets, re-run the
//! S4/S5 steering sweep and wheel analysis (`spikes/s45-vehicle`, F9/F11) and
//! add a row here; nothing else changes.

/// Vehicle-struct offsets (bytes from the address `getScriptHandleBaseAddress`
/// returns for a vehicle handle).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Offsets {
    /// Smoothed steering input, f32 in ±0.948, positive left. Written every
    /// frame to steer (S4); the game turns it into the angle the same frame.
    pub steer_input: usize,
    /// Road-wheel steering angle, f32 rad, positive left. Read only: the game
    /// recomputes it every frame.
    pub steer_angle: usize,
    /// Pointer to the wheel-pointer array, followed by the wheel count (u32).
    pub wheel_array: usize,
    pub wheel_count: usize,
    /// In each wheel struct: spin speed, f32 rad/s, negative rolling forward.
    pub wheel_omega: usize,
}

/// One supported game build.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Edition {
    /// Script Hook V's `eGameVersion`.
    pub game_version: i32,
    pub name: &'static str,
    pub offsets: Offsets,
    /// The offsets were measured on this build (not assumed from another).
    pub verified: bool,
}

/// Injected control values have a 25% deadzone: an injected value v gives the
/// vehicle (v − 0.25) / 0.75 (S4). A demand d in (0, 1] is sent as this.
pub const PEDAL_DEADZONE: f32 = 0.25;

pub fn pedal_inject(demand: f32) -> f32 {
    PEDAL_DEADZONE + (1.0 - PEDAL_DEADZONE) * demand.clamp(0.0, 1.0)
}

/// Full-lock smoothed steering input (S4: ±0.948 on all four test vehicles).
pub const STEER_INPUT_MAX: f32 = 0.948;

const OFFSETS_1158_13: Offsets = Offsets { steer_input: 0x9D4, steer_angle: 0x9DC, wheel_array: 0xC30, wheel_count: 0xC38, wheel_omega: 0x170 };

const KNOWN: &[Edition] = &[Edition { game_version: 1012, name: "Enhanced 1.0.1158.13", offsets: OFFSETS_1158_13, verified: true }];

/// The edition for a game version. Unknown versions fall back to the newest
/// known offsets, marked unverified, so the adapter can warn on the HUD.
pub fn for_game_version(v: i32) -> Edition {
    KNOWN.iter().copied().find(|e| e.game_version == v).unwrap_or(Edition {
        game_version: v,
        name: "unknown build (offsets unverified)",
        offsets: KNOWN[KNOWN.len() - 1].offsets,
        verified: false,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn known_and_unknown() {
        let e = for_game_version(1012);
        assert!(e.verified);
        assert_eq!(e.offsets.steer_input, 0x9D4);
        let u = for_game_version(9999);
        assert!(!u.verified);
        assert_eq!(u.offsets, e.offsets);
    }

    #[test]
    fn deadzone_mapping() {
        assert_eq!(pedal_inject(0.0), 0.25);
        assert_eq!(pedal_inject(1.0), 1.0);
        assert!((pedal_inject(0.5) - 0.625).abs() < 1e-6);
        assert_eq!(pedal_inject(2.0), 1.0);
    }
}
