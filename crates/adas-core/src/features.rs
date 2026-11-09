//! Features the core can run. Phase 0 has two: a scripted speed profile (the
//! S3 end-to-end test) and the test driver (drive-by-wire through the whole
//! loop). ACC, FCW and AEB arrive in Phase 1.

use std::collections::BTreeMap;

use adapter_api::{ActorList, VehicleSample};
use control::SpeedController;
use features::{Acc, AccConfig};
use fusion::Lead;

/// A stick input for the test driver: pedals 0..=1, steering -1..=1 (left +).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Stick {
    pub throttle: f32,
    pub brake: f32,
    pub steer: f32,
}

#[derive(Clone, Debug, PartialEq)]
pub enum Feature {
    /// (seconds, target m/s) segments, then stop and hold.
    SpeedProfile(Vec<(f32, f32)>),
    /// The driver's stick drives the car through the core (drive-by-wire).
    TestDriver(TestDriverConfig),
    /// Adaptive cruise (F1).
    Acc(AccConfig),
}

impl Feature {
    /// The S3 profile: 8, 15, 5, 12 m/s, then stop and hold.
    pub fn speed_profile() -> Self {
        Feature::SpeedProfile(vec![(10.0, 8.0), (10.0, 15.0), (10.0, 5.0), (9.0, 12.0), (9.0, 0.0)])
    }

    pub fn name(&self) -> &'static str {
        match self {
            Feature::SpeedProfile(_) => "speed-profile",
            Feature::TestDriver(_) => "test-driver",
            Feature::Acc(_) => "acc",
        }
    }

    /// The feature and its parameters as recording metadata.
    pub fn to_meta(&self) -> BTreeMap<String, String> {
        let mut m = BTreeMap::new();
        m.insert("feature".into(), self.name().into());
        match self {
            Feature::SpeedProfile(segs) => {
                let s: Vec<String> = segs.iter().map(|(t, v)| format!("{t}:{v}")).collect();
                m.insert("segments".into(), s.join(","));
            }
            Feature::TestDriver(c) => {
                m.insert("max_steer".into(), c.max_steer.to_string());
                m.insert("steer_fade_speed".into(), c.steer_fade_speed.to_string());
                m.insert("steer_rate".into(), c.steer_rate.to_string());
            }
            Feature::Acc(_) => {}
        }
        m
    }

    /// Rebuilds a feature from recording metadata.
    pub fn from_meta(m: &BTreeMap<String, String>) -> Result<Self, String> {
        let f = |k: &str| -> Result<f32, String> {
            m.get(k).ok_or(format!("missing {k}"))?.parse().map_err(|e| format!("{k}: {e}"))
        };
        match m.get("feature").map(String::as_str) {
            Some("speed-profile") => {
                let segs = m.get("segments").ok_or("missing segments")?;
                let mut out = Vec::new();
                for part in segs.split(',').filter(|p| !p.is_empty()) {
                    let (t, v) = part.split_once(':').ok_or("bad segment")?;
                    out.push((t.parse().map_err(|_| "bad segment time")?, v.parse().map_err(|_| "bad segment speed")?));
                }
                Ok(Feature::SpeedProfile(out))
            }
            Some("test-driver") => Ok(Feature::TestDriver(TestDriverConfig {
                max_steer: f("max_steer")?,
                steer_fade_speed: f("steer_fade_speed")?,
                steer_rate: f("steer_rate")?,
            })),
            Some("acc") => Ok(Feature::Acc(AccConfig::default())),
            other => Err(format!("unknown feature {other:?}")),
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TestDriverConfig {
    /// Road-wheel angle at full stick, at standstill (rad).
    pub max_steer: f32,
    /// Above this speed the steering range shrinks (m/s), as speed-sensitive
    /// steering does on real cars.
    pub steer_fade_speed: f32,
    /// Steering rate limit (rad/s).
    pub steer_rate: f32,
}

impl Default for TestDriverConfig {
    fn default() -> Self {
        Self { max_steer: 0.5, steer_fade_speed: 15.0, steer_rate: 1.0 }
    }
}

/// Per-engagement feature state.
#[derive(Clone, Copy, Debug, Default)]
pub struct FeatureState {
    steer: f32,
}

pub struct Ctx<'a> {
    pub sample: &'a VehicleSample,
    pub actors: &'a ActorList,
    pub stick: Option<Stick>,
    pub dt: f32,
    /// Seconds since engaging.
    pub t: f32,
    /// The lead ACC would follow, if any.
    pub follow: Option<Lead>,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct Out {
    pub throttle: f32,
    pub brake: f32,
    pub hold: bool,
    /// Some = steer to this road-wheel angle (rad).
    pub steer: Option<f32>,
    pub diag: [f32; 4],
    /// ACC is following a lead rather than cruising at the set speed.
    pub following: bool,
}

pub fn run(feature: &Feature, st: &mut FeatureState, speed: &mut SpeedController, acc: &Acc, c: Ctx) -> Out {
    match feature {
        Feature::SpeedProfile(segments) => {
            let target = target_at(segments, c.t);
            let o = speed.step(target, c.sample.speed, c.dt);
            Out { throttle: o.throttle, brake: o.brake, hold: o.hold, steer: Some(0.0), diag: [target, o.reference, o.ref_accel, o.demand], following: false }
        }
        Feature::TestDriver(cfg) => {
            let stick = c.stick.unwrap_or_default();
            let fade = 1.0 / (1.0 + (c.sample.speed / cfg.steer_fade_speed).powi(2));
            let want = stick.steer.clamp(-1.0, 1.0) * cfg.max_steer * fade;
            let step = cfg.steer_rate * c.dt;
            st.steer += (want - st.steer).clamp(-step, step);
            // Auto-hold: stopped with no throttle.
            let hold = c.sample.speed < 0.1 && stick.throttle < 0.02;
            Out {
                throttle: stick.throttle.clamp(0.0, 1.0),
                brake: if hold { 0.0 } else { stick.brake.clamp(0.0, 1.0) },
                hold,
                steer: Some(st.steer),
                diag: [stick.throttle, stick.brake, stick.steer, st.steer],
                following: false,
            }
        }
        Feature::Acc(_) => {
            let r = acc.request(c.sample.speed, c.follow);
            let o = speed.step(r.target, c.sample.speed, c.dt);
            // Longitudinal only: no steering request (the driver steers).
            Out {
                throttle: o.throttle,
                brake: o.brake,
                hold: o.hold,
                steer: None,
                diag: [r.target, o.reference, c.follow.map(|l| l.gap).unwrap_or(f32::NAN), r.desired_gap],
                following: r.following,
            }
        }
    }
}

fn target_at(segments: &[(f32, f32)], t: f32) -> f32 {
    let mut acc = 0.0;
    for &(len, tgt) in segments {
        if t < acc + len {
            return tgt;
        }
        acc += len;
    }
    0.0
}
