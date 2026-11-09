//! The ADAS core: one deterministic `step()` per host frame (D23).
//!
//! `Core::step` is a pure function of its inputs and the core's own state: the
//! vehicle sample, the actor list and (for the test driver) a stick input. Every
//! input is recorded, so a replay of a drive reproduces the same commands (D20).
//! Nothing here knows which host game is running.

pub mod features;
pub mod layout;

use adapter_api::{ActorList, Command, VehicleSample, command_flags, sample_flags};
use control::{Config, SpeedController};

pub use features::{Feature, Stick};

/// The core's inputs for one step.
#[derive(Clone, Copy, Debug)]
pub struct Inputs<'a> {
    pub sample: &'a VehicleSample,
    pub actors: &'a ActorList,
    /// The test driver's stick, when the test-driver feature is running.
    pub stick: Option<Stick>,
}

/// Engagement state, as a production ADAS shows it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum State {
    /// Not engaged.
    Off,
    /// Engaged and in control.
    Active,
    /// Engaged, but the driver is pressing the accelerator (temporary override).
    Override,
}

/// Why the core changed state this step, for logs and the HUD.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Event {
    None,
    Engaged,
    /// The driver braked or steered.
    CancelledByDriver,
    /// The engage request was withdrawn.
    Disengaged,
}

pub struct Core {
    feature: Feature,
    speed: SpeedController,
    state: State,
    prev_request: bool,
    last_game_us: u64,
    /// Seconds since engaging (game time).
    t_engaged: f32,
    feature_state: features::FeatureState,
}

impl Core {
    /// `initial_request`: the engage-request flag as first seen, so a request
    /// that was already set before this core started doesn't engage it.
    pub fn new(feature: Feature, initial_request: bool) -> Self {
        Self {
            feature,
            speed: SpeedController::new(Config::default()),
            state: State::Off,
            prev_request: initial_request,
            last_game_us: 0,
            t_engaged: 0.0,
            feature_state: features::FeatureState::default(),
        }
    }

    pub fn state(&self) -> State {
        self.state
    }

    pub fn feature(&self) -> &Feature {
        &self.feature
    }

    /// One step. Returns the command payload (the caller fills the header and
    /// `answers_seq`) and what changed.
    pub fn step(&mut self, inp: Inputs) -> (Command, Event) {
        let s = inp.sample;
        let request = s.flags & sample_flags::ENGAGE_REQUEST != 0;
        let cancel = s.flags & sample_flags::DRIVER_CANCEL != 0;
        let override_ = s.flags & sample_flags::DRIVER_OVERRIDE != 0;
        let paused = s.flags & sample_flags::PAUSED != 0;

        let dt = if self.last_game_us == 0 { 0.0 } else { s.header.game_time_us.saturating_sub(self.last_game_us) as f32 / 1e6 };
        self.last_game_us = s.header.game_time_us;

        let mut event = Event::None;
        if self.state == State::Off {
            if request && !self.prev_request && !cancel {
                self.state = State::Active;
                self.speed.reset(s.speed);
                self.t_engaged = 0.0;
                self.feature_state = features::FeatureState::default();
                event = Event::Engaged;
            }
        } else if cancel {
            self.state = State::Off;
            event = Event::CancelledByDriver;
        } else if !request {
            self.state = State::Off;
            event = Event::Disengaged;
        } else {
            self.state = if override_ { State::Override } else { State::Active };
        }
        self.prev_request = request;

        let mut cmd = Command::default();
        if self.state == State::Off {
            return (cmd, event);
        }
        cmd.flags = command_flags::ENGAGED;
        if paused || dt <= 0.0 {
            return (cmd, event);
        }
        self.t_engaged += dt;

        let out = features::run(
            &self.feature,
            &mut self.feature_state,
            &mut self.speed,
            features::Ctx { sample: s, actors: inp.actors, stick: inp.stick, dt, t: self.t_engaged },
        );
        if self.state == State::Override {
            // The driver's foot wins. Restart the reference from the current
            // speed, so letting go eases back to the target instead of braking.
            self.speed.reset(s.speed);
        }
        cmd.throttle = out.throttle;
        cmd.brake = out.brake;
        if out.hold {
            cmd.flags |= command_flags::HOLD;
        }
        if let Some(a) = out.steer {
            cmd.flags |= command_flags::STEER;
            cmd.steer_angle = a;
        }
        cmd.diag = out.diag;
        (cmd, event)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use adapter_api::Header;

    fn sample(seq: u32, flags: u32, speed: f32) -> VehicleSample {
        VehicleSample {
            header: Header { seq, frame: seq as u64, game_time_us: seq as u64 * 16_667, ..Default::default() },
            flags,
            speed,
            ..Default::default()
        }
    }

    fn step(core: &mut Core, seq: u32, flags: u32, speed: f32) -> (Command, Event) {
        let s = sample(seq, flags, speed);
        let a = ActorList::default();
        core.step(Inputs { sample: &s, actors: &a, stick: None })
    }

    #[test]
    fn never_engages_on_a_stale_request() {
        let mut core = Core::new(Feature::speed_profile(), true);
        for i in 1..10 {
            let (c, _) = step(&mut core, i, sample_flags::ENGAGE_REQUEST, 0.0);
            assert_eq!(c.flags, 0, "engaged on a request that predates the core");
        }
        step(&mut core, 10, 0, 0.0);
        let (c, e) = step(&mut core, 11, sample_flags::ENGAGE_REQUEST, 0.0);
        assert_eq!(e, Event::Engaged);
        assert!(c.flags & command_flags::ENGAGED != 0);
    }

    #[test]
    fn driver_cancel_disengages_and_needs_a_new_request() {
        let mut core = Core::new(Feature::speed_profile(), false);
        step(&mut core, 1, sample_flags::ENGAGE_REQUEST, 0.0);
        let (_, e) = step(&mut core, 2, sample_flags::ENGAGE_REQUEST | sample_flags::DRIVER_CANCEL, 0.0);
        assert_eq!(e, Event::CancelledByDriver);
        let (c, _) = step(&mut core, 3, sample_flags::ENGAGE_REQUEST, 0.0);
        assert_eq!(c.flags, 0, "re-engaged without a fresh request");
    }

    #[test]
    fn override_keeps_engagement_and_releases_pedals_to_driver() {
        let mut core = Core::new(Feature::speed_profile(), false);
        step(&mut core, 1, sample_flags::ENGAGE_REQUEST, 0.0);
        let (c, _) = step(&mut core, 2, sample_flags::ENGAGE_REQUEST | sample_flags::DRIVER_OVERRIDE, 3.0);
        assert!(c.flags & command_flags::ENGAGED != 0);
        assert_eq!(core.state(), State::Override);
        step(&mut core, 3, sample_flags::ENGAGE_REQUEST, 3.0);
        assert_eq!(core.state(), State::Active);
    }

    #[test]
    fn paused_means_no_actuation() {
        let mut core = Core::new(Feature::speed_profile(), false);
        step(&mut core, 1, sample_flags::ENGAGE_REQUEST, 0.0);
        let (c, _) = step(&mut core, 2, sample_flags::ENGAGE_REQUEST | sample_flags::PAUSED, 0.0);
        assert_eq!((c.throttle, c.brake), (0.0, 0.0));
    }

    #[test]
    fn deterministic_for_identical_inputs() {
        let run = || {
            let mut core = Core::new(Feature::speed_profile(), false);
            (1..600).map(|i| step(&mut core, i, sample_flags::ENGAGE_REQUEST, (i as f32 * 0.01).min(8.0)).0).collect::<Vec<_>>()
        };
        assert_eq!(run(), run());
    }
}
