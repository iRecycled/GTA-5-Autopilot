//! Forward collision warning (F2) and staged emergency braking (F3).
//!
//! Time to collision (TTC) = gap ÷ closing speed. Stages, as in the PRD:
//! warn at about 2.5 s, partial braking at about 1.6 s, full braking at about
//! 0.9 s. Once AEB has brought the car to a stop it holds it there until the
//! driver presses the accelerator. AEB runs whether or not ACC is engaged.
//!
//! A stage is entered when TTC falls below its threshold and left only when
//! TTC rises a margin above it (hysteresis), or the threat is gone; braking
//! stages also last at least a minimum time, so the brake doesn't flicker.

use fusion::Lead;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AebConfig {
    pub warn_ttc: f32,
    pub partial_ttc: f32,
    pub full_ttc: f32,
    /// Leave a stage when TTC is this much above its threshold (s).
    pub hysteresis: f32,
    /// Brake demand in the partial stage (0..=1).
    pub partial_brake: f32,
    /// Ignore threats below this ego speed (m/s): parking manoeuvres.
    pub min_speed: f32,
    /// Ignore closing speeds below this (m/s).
    pub min_closing: f32,
    /// Braking stages last at least this long (s).
    pub min_brake_time: f32,
    /// Driver throttle above this cancels AEB (the driver's intent wins).
    pub driver_cancel_throttle: f32,
}

impl Default for AebConfig {
    fn default() -> Self {
        Self {
            warn_ttc: 2.5,
            partial_ttc: 1.6,
            full_ttc: 0.9,
            hysteresis: 0.4,
            partial_brake: 0.5,
            min_speed: 2.0,
            min_closing: 1.0,
            min_brake_time: 0.5,
            driver_cancel_throttle: 0.8,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum AebStage {
    Off,
    Warn,
    Partial,
    Full,
    /// Stopped by AEB; holding until the driver accelerates.
    Hold,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Aeb {
    pub cfg: AebConfig,
    pub stage: AebStage,
    in_stage: f32,
    /// Activations of a braking stage so far (for the false-activation count).
    pub activations: u32,
}

/// What AEB asks for this step.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AebRequest {
    pub stage: AebStage,
    /// Brake demand (0..=1); 0 when not braking.
    pub brake: f32,
    /// Hold the car at a standstill.
    pub hold: bool,
    /// Warn the driver (FCW chime and flash).
    pub warn: bool,
    /// TTC to the threat (s), infinite without one.
    pub ttc: f32,
}

pub fn ttc(lead: &Lead, ego_speed: f32) -> f32 {
    let closing = ego_speed - lead.v;
    if closing <= 0.0 || lead.gap <= 0.0 { if lead.gap <= 0.0 && closing > 0.0 { 0.0 } else { f32::INFINITY } } else { lead.gap / closing }
}

impl Aeb {
    pub fn new(cfg: AebConfig) -> Self {
        Self { cfg, stage: AebStage::Off, in_stage: 0.0, activations: 0 }
    }

    pub fn step(&mut self, dt: f32, ego_speed: f32, threat: Option<Lead>, driver_throttle: f32) -> AebRequest {
        let c = self.cfg;
        let t = threat.map(|l| ttc(&l, ego_speed)).unwrap_or(f32::INFINITY);
        let closing = threat.map(|l| ego_speed - l.v).unwrap_or(0.0);
        let relevant = ego_speed >= c.min_speed && closing >= c.min_closing;
        self.in_stage += dt.max(0.0);

        let want = if !relevant {
            AebStage::Off
        } else if t < c.full_ttc {
            AebStage::Full
        } else if t < c.partial_ttc {
            AebStage::Partial
        } else if t < c.warn_ttc {
            AebStage::Warn
        } else {
            AebStage::Off
        };
        let threshold = |s: AebStage| match s {
            AebStage::Warn => c.warn_ttc,
            AebStage::Partial => c.partial_ttc,
            AebStage::Full => c.full_ttc,
            _ => f32::INFINITY,
        };

        let next = match self.stage {
            AebStage::Hold => {
                if driver_throttle > 0.05 { AebStage::Off } else { AebStage::Hold }
            }
            _ if driver_throttle > c.driver_cancel_throttle => AebStage::Off,
            s if want > s => want,
            s @ (AebStage::Partial | AebStage::Full) if ego_speed < 0.3 => {
                let _ = s;
                AebStage::Hold
            }
            s @ (AebStage::Partial | AebStage::Full) if self.in_stage < c.min_brake_time => s,
            s if want < s && (t > threshold(s) + c.hysteresis || !relevant) => want,
            s => s,
        };
        if next != self.stage {
            if next >= AebStage::Partial && next <= AebStage::Full && self.stage < AebStage::Partial {
                self.activations += 1;
            }
            self.stage = next;
            self.in_stage = 0.0;
        }
        let brake = match self.stage {
            AebStage::Partial => c.partial_brake,
            AebStage::Full => 1.0,
            _ => 0.0,
        };
        AebRequest { stage: self.stage, brake, hold: self.stage == AebStage::Hold, warn: self.stage >= AebStage::Warn && self.stage != AebStage::Hold, ttc: t }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn lead(gap: f32, v: f32) -> Lead {
        Lead { id: 1, gap, v, a: 0.0, offset: 0.0, stationary: v < 0.5 }
    }

    #[test]
    fn stages_with_falling_ttc() {
        let mut a = Aeb::new(AebConfig::default());
        let v = 14.0; // 50 km/h toward a stopped car
        assert_eq!(a.step(0.01, v, Some(lead(50.0, 0.0)), 0.0).stage, AebStage::Off); // 3.6 s
        assert_eq!(a.step(0.01, v, Some(lead(30.0, 0.0)), 0.0).stage, AebStage::Warn); // 2.1 s
        let r = a.step(0.01, v, Some(lead(20.0, 0.0)), 0.0); // 1.4 s
        assert_eq!((r.stage, r.brake), (AebStage::Partial, 0.5));
        let r = a.step(0.01, v, Some(lead(10.0, 0.0)), 0.0); // 0.7 s
        assert_eq!((r.stage, r.brake), (AebStage::Full, 1.0));
        assert_eq!(a.activations, 1);
    }

    #[test]
    fn holds_after_stopping_until_the_driver_accelerates() {
        let mut a = Aeb::new(AebConfig::default());
        a.step(0.01, 10.0, Some(lead(5.0, 0.0)), 0.0);
        assert_eq!(a.stage, AebStage::Full);
        for _ in 0..100 {
            a.step(0.01, 0.0, Some(lead(1.0, 0.0)), 0.0);
        }
        let r = a.step(0.01, 0.0, Some(lead(1.0, 0.0)), 0.0);
        assert!(r.hold && r.brake == 0.0);
        assert_eq!(a.step(0.01, 0.0, Some(lead(1.0, 0.0)), 0.3).stage, AebStage::Off);
    }

    #[test]
    fn no_activation_when_following_or_slow() {
        let mut a = Aeb::new(AebConfig::default());
        // Following at the same speed: no closing, no threat.
        for _ in 0..1000 {
            assert_eq!(a.step(0.01, 25.0, Some(lead(30.0, 25.0)), 0.0).stage, AebStage::Off);
        }
        // Creeping in a car park: below the minimum speed.
        assert_eq!(a.step(0.01, 1.5, Some(lead(0.5, 0.0)), 0.0).stage, AebStage::Off);
        assert_eq!(a.activations, 0);
    }

    #[test]
    fn hysteresis_and_minimum_brake_time() {
        let mut a = Aeb::new(AebConfig::default());
        a.step(0.01, 14.0, Some(lead(20.0, 0.0)), 0.0); // partial
        // The threat vanishes at once: braking still lasts the minimum time.
        let r = a.step(0.1, 14.0, None, 0.0);
        assert_eq!(r.stage, AebStage::Partial);
        let r = a.step(0.5, 14.0, None, 0.0);
        assert_eq!(r.stage, AebStage::Off);
    }

    #[test]
    fn hard_throttle_cancels() {
        let mut a = Aeb::new(AebConfig::default());
        a.step(0.01, 14.0, Some(lead(10.0, 0.0)), 0.0);
        assert_eq!(a.step(0.01, 14.0, Some(lead(9.0, 0.0)), 0.9).stage, AebStage::Off);
    }
}
