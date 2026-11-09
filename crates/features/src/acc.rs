//! Adaptive cruise control (F1): hold the set speed; behind a lead, keep a
//! time gap of 1.0–2.5 s; follow it down to a stop and back up (stop-and-go).
//!
//! ACC is an outer loop on the speed controller: it turns the gap into a target
//! speed, and the speed controller's comfort limits (accel, braking, jerk) make
//! the motion smooth. At steady state the ego car matches the lead's speed at
//! the desired gap:
//!
//!   target = min(set speed, lead speed + k · (gap − desired gap))
//!   desired gap = standstill gap + time gap × ego speed

use fusion::Lead;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AccConfig {
    /// Selectable time gaps (s), shortest first.
    pub time_gaps: [f32; 4],
    /// Gap kept at a standstill (m).
    pub standstill_gap: f32,
    /// Gap error → speed correction (1/s).
    pub gap_gain: f32,
    /// Lead acceleration feed-forward horizon (s).
    pub lead_accel_horizon: f32,
    /// Lowest and highest set speeds (m/s) and the button step (m/s).
    pub min_set: f32,
    pub max_set: f32,
    pub step: f32,
    /// Braking the ACC may ask for (m/s², positive). Production ACC stays at or
    /// below about 3.5 m/s²; anything harder is AEB's job.
    pub max_decel: f32,
}

impl Default for AccConfig {
    fn default() -> Self {
        Self {
            time_gaps: [1.0, 1.5, 2.0, 2.5],
            standstill_gap: 4.0,
            gap_gain: 0.3,
            lead_accel_horizon: 0.5,
            min_set: 30.0 / 3.6,
            max_set: 160.0 / 3.6,
            step: 5.0 / 3.6,
            max_decel: 3.5,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Acc {
    pub cfg: AccConfig,
    /// Set speed (m/s).
    pub set_speed: f32,
    /// Index into `cfg.time_gaps`.
    pub gap_index: usize,
}

/// What the ACC asks for this step.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AccRequest {
    /// Target speed for the speed controller (m/s).
    pub target: f32,
    /// The gap it is trying to keep (m); NaN without a lead.
    pub desired_gap: f32,
    /// The lead it is following, if any.
    pub following: bool,
}

impl Acc {
    pub fn new(cfg: AccConfig) -> Self {
        Self { cfg, set_speed: cfg.min_set, gap_index: 2 }
    }

    pub fn time_gap(&self) -> f32 {
        self.cfg.time_gaps[self.gap_index.min(self.cfg.time_gaps.len() - 1)]
    }

    /// On engaging: the set speed becomes the current speed, rounded up to the
    /// next button step and at least the minimum.
    pub fn engage(&mut self, speed: f32) {
        let steps = (speed / self.cfg.step).ceil();
        self.set_speed = (steps * self.cfg.step).clamp(self.cfg.min_set, self.cfg.max_set);
    }

    pub fn speed_up(&mut self) {
        self.set_speed = (self.set_speed + self.cfg.step).min(self.cfg.max_set);
    }

    pub fn speed_down(&mut self) {
        self.set_speed = (self.set_speed - self.cfg.step).max(self.cfg.min_set);
    }

    /// Cycles the time gap: longest → shortest wraps around.
    pub fn cycle_gap(&mut self) {
        self.gap_index = (self.gap_index + 1) % self.cfg.time_gaps.len();
    }

    pub fn request(&self, ego_speed: f32, lead: Option<Lead>) -> AccRequest {
        match lead {
            None => AccRequest { target: self.set_speed, desired_gap: f32::NAN, following: false },
            Some(l) => {
                let desired = self.cfg.standstill_gap + self.time_gap() * ego_speed.max(0.0);
                let lead_v = (l.v + l.a.min(0.0) * self.cfg.lead_accel_horizon).max(0.0);
                let gap_term = self.cfg.gap_gain * (l.gap - desired);
                let target = (lead_v + gap_term).clamp(0.0, self.set_speed);
                AccRequest { target, desired_gap: desired, following: target < self.set_speed - 0.05 }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn lead(gap: f32, v: f32) -> Lead {
        Lead { id: 1, gap, v, a: 0.0, offset: 0.0, stationary: v < 0.5 }
    }

    #[test]
    fn set_speed_buttons() {
        let mut a = Acc::new(AccConfig::default());
        a.engage(15.0); // 54 km/h → 55 km/h
        assert!((a.set_speed * 3.6 - 55.0).abs() < 0.01);
        a.engage(2.0); // below the minimum → 30 km/h
        assert!((a.set_speed * 3.6 - 30.0).abs() < 0.01);
        a.speed_up();
        assert!((a.set_speed * 3.6 - 35.0).abs() < 0.01);
        a.speed_down();
        a.speed_down();
        assert!((a.set_speed * 3.6 - 30.0).abs() < 0.01, "went below the minimum");
    }

    #[test]
    fn follows_at_the_time_gap() {
        let mut a = Acc::new(AccConfig::default());
        a.engage(25.0);
        // At 20 m/s with a 2.0 s gap the desired gap is 44 m. Exactly there,
        // the target is the lead's speed.
        let r = a.request(20.0, Some(lead(44.0, 20.0)));
        assert!((r.target - 20.0).abs() < 1e-4 && r.following);
        // Too close: slower than the lead. Too far: faster, capped at set speed.
        assert!(a.request(20.0, Some(lead(30.0, 20.0))).target < 20.0);
        assert!(a.request(20.0, Some(lead(200.0, 20.0))).target <= a.set_speed + 1e-6);
        // No lead: the set speed.
        assert_eq!(a.request(20.0, None).target, a.set_speed);
    }

    #[test]
    fn stops_behind_a_stopped_lead() {
        let a = Acc::new(AccConfig::default());
        // At standstill, 4 m behind a stopped car: target 0.
        let r = a.request(0.0, Some(lead(4.0, 0.0)));
        assert_eq!(r.target, 0.0);
        // When it pulls away, the target rises again (stop-and-go).
        assert!(a.request(0.0, Some(lead(8.0, 3.0))).target > 0.0);
    }

    #[test]
    fn gap_cycles() {
        let mut a = Acc::new(AccConfig::default());
        assert_eq!(a.time_gap(), 2.0);
        a.cycle_gap();
        assert_eq!(a.time_gap(), 2.5);
        a.cycle_gap();
        assert_eq!(a.time_gap(), 1.0);
    }
}
