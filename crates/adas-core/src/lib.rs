//! The ADAS core: one deterministic `step()` per host frame (D23).
//!
//! `Core::step` is a pure function of its inputs and the core's own state: the
//! vehicle sample, the actor list, the adapter info and (for the test driver) a
//! stick input. Every input is recorded, so a replay of a drive reproduces the
//! same commands (D20). Nothing here knows which host game is running.
//!
//! Every step, through the virtual vehicle's buses (`vbus`): the chassis ECUs
//! publish speed, yaw rate and pedals on chassis CAN and the front radar ECU
//! its object list on ADAS CAN FD; the ADAS ECU reads both back, so the
//! features see what a real ADAS ECU sees. Then perception (tracker → lead
//! selection) and AEB run whether or not anything is engaged; the engaged
//! feature (ACC, the speed profile or the test driver) runs on top; AEB wins.

pub mod features;
pub mod layout;
pub mod vbus;

use adapter_api::{ActorList, AdapterInfo, Command, Hud, SensorLayout, VehicleSample, buttons, command_flags, hud_flags, sample_flags};
use control::{Config, SpeedController};
use features::FeatureState;
use fusion::{Lead, LeadKind, Tracker, TrackerConfig, select_lead};
use sensors::{FrontRadar, RadarConfig};

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
    Off,
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
    /// Emergency braking took over; ACC is cancelled.
    CancelledByAeb,
    /// An impact was detected (as an airbag controller's crash signal would
    /// report it): everything is cancelled.
    Impact,
}

/// Deceleration that counts as an impact (m/s², about 1.5 g). Hard braking on
/// good tyres stays below about 1.1 g.
const IMPACT_DECEL: f32 = 15.0;

/// Half a lane for in-path decisions (m).
const HALF_LANE: f32 = 1.8;

pub struct Core {
    feature: Feature,
    speed: SpeedController,
    state: State,
    prev_request: bool,
    last_game_us: u64,
    t_engaged: f32,
    feature_state: FeatureState,
    info: Option<AdapterInfo>,
    layout: SensorLayout,
    radar: FrontRadar,
    tracker: Tracker,
    since_scan: f32,
    acc: ::features::Acc,
    aeb: ::features::Aeb,
    hud: Hud,
    /// Longitudinal acceleration estimated from speed (m/s², low-passed).
    a_est: f32,
    prev_speed: Option<f32>,
    /// The AEB deceleration loop's integral (brake demand).
    aeb_i: f32,
    /// The host's brake gain, learned on line: deceleration per unit of brake
    /// demand (m/s²). Hosts differ a lot (sim 8, GTA about 80 at light
    /// braking); this is the D19 vehicle profile, learned rather than tuned.
    brake_gain: f32,
    prev_brake: f32,
    prev_speed_for_impact: Option<f32>,
    vehicle: vbus::Vehicle,
}

/// Starting guess for the brake gain (m/s² per unit of brake demand); learned
/// from then on. The deceleration loop's integral covers the rest.
const BRAKE_GAIN_INIT: f32 = 8.0;
/// Integral gain of the AEB deceleration loop (demand per m/s²·s).
const AEB_KI: f32 = 0.35;

impl Core {
    /// `initial_request`: the engage-request flag as first seen, so a request
    /// that was already set before this core started doesn't engage it.
    pub fn new(feature: Feature, initial_request: bool) -> Self {
        // ACC may brake harder than the cruise default (up to 3.5 m/s²).
        let speed_cfg = match feature {
            Feature::Acc(_) => Config { decel_max: 3.5, ..Config::default() },
            _ => Config::default(),
        };
        let acc_cfg = match &feature {
            Feature::Acc(c) => *c,
            _ => Default::default(),
        };
        Self {
            feature,
            speed: SpeedController::new(speed_cfg),
            state: State::Off,
            prev_request: initial_request,
            last_game_us: 0,
            t_engaged: 0.0,
            feature_state: FeatureState::default(),
            info: None,
            layout: SensorLayout::default(),
            radar: FrontRadar::new(RadarConfig::default(), layout::FRONT_RADAR),
            tracker: Tracker::new(TrackerConfig::default()),
            since_scan: 0.0,
            acc: ::features::Acc::new(acc_cfg),
            aeb: ::features::Aeb::new(Default::default()),
            hud: Hud::default(),
            a_est: 0.0,
            prev_speed: None,
            aeb_i: 0.0,
            brake_gain: BRAKE_GAIN_INIT,
            prev_brake: 0.0,
            prev_speed_for_impact: None,
            vehicle: vbus::Vehicle::new(can::Delivery::Reactive),
        }
    }

    pub fn state(&self) -> State {
        self.state
    }

    pub fn feature(&self) -> &Feature {
        &self.feature
    }

    /// The adapter's identity and ego geometry. Rebuilds the sensor layout.
    pub fn set_info(&mut self, info: &AdapterInfo) {
        self.info = Some(*info);
        self.layout = layout::default_layout(info.box_min, info.box_max);
    }

    /// The standing line-of-sight queries for the adapter.
    pub fn layout(&self) -> &SensorLayout {
        &self.layout
    }

    /// What the HUD should show after the last step.
    pub fn hud(&self) -> &Hud {
        &self.hud
    }

    pub fn tracker(&self) -> &Tracker {
        &self.tracker
    }

    pub fn aeb_activations(&self) -> u32 {
        self.aeb.activations
    }

    /// One step. Returns the command payload (the caller fills the header and
    /// `answers_seq`) and what changed.
    pub fn step(&mut self, inp: Inputs) -> (Command, Event) {
        let host = inp.sample;
        let request = host.flags & sample_flags::ENGAGE_REQUEST != 0;
        let cancel = host.flags & sample_flags::DRIVER_CANCEL != 0;
        let override_ = host.flags & sample_flags::DRIVER_OVERRIDE != 0;
        let paused = host.flags & sample_flags::PAUSED != 0;

        let dt = if self.last_game_us == 0 { 0.0 } else { host.header.game_time_us.saturating_sub(self.last_game_us) as f32 / 1e6 };
        self.last_game_us = host.header.game_time_us;

        // --- The buses and perception (always) ---
        // From here on the core works on the car as the ADAS ECU sees it on
        // the bus, not on the host's exact values.
        let (follow, threat, bus_speed, bus_yaw) = self.perceive(host, inp.actors, dt, host.flags & sample_flags::PAUSED != 0);
        let mut view = *host;
        view.speed = bus_speed;
        view.vx = bus_speed;
        view.yaw_rate = bus_yaw;
        let s = &view;

        // Acceleration from speed (the deceleration loop's feedback), and the
        // brake gain from what last frame's brake demand achieved.
        if dt > 0.0 && !paused {
            if let Some(p) = self.prev_speed {
                let a = (s.speed - p) / dt;
                self.a_est += (a - self.a_est) * (dt / 0.08).min(1.0);
            }
            self.prev_speed = Some(s.speed);
            if self.prev_brake >= 0.03 && s.speed > 2.0 && self.a_est < -0.5 {
                let g = (-self.a_est / self.prev_brake).clamp(3.0, 150.0);
                self.brake_gain += (g - self.brake_gain) * (dt / 0.5).min(1.0);
            }
        }

        // An impact cancels everything, as a crash signal does on a real car.
        let impact = dt > 0.0 && !paused && self.prev_speed_for_impact.is_some_and(|p| (p - s.speed) / dt > IMPACT_DECEL);
        self.prev_speed_for_impact = (!paused).then_some(s.speed);


        // --- AEB (always) ---
        let aeb = if paused || dt <= 0.0 || self.info.is_none() {
            None
        } else {
            Some(self.aeb.step(dt, s.speed, threat, s.driver_throttle, s.driver_brake))
        };
        let aeb_braking = aeb.is_some_and(|a| a.decel > 0.0 || a.hold);

        // --- Engagement ---
        let mut event = Event::None;
        if impact && self.state != State::Off {
            self.state = State::Off;
            event = Event::Impact;
        } else if self.state == State::Off {
            if request && !self.prev_request && !cancel && !aeb_braking {
                self.state = State::Active;
                self.speed.reset(s.speed);
                self.t_engaged = 0.0;
                self.feature_state = FeatureState::default();
                if matches!(self.feature, Feature::Acc(_)) {
                    self.acc.engage(s.speed);
                }
                event = Event::Engaged;
            }
        } else if cancel {
            self.state = State::Off;
            event = Event::CancelledByDriver;
        } else if !request {
            self.state = State::Off;
            event = Event::Disengaged;
        } else if aeb_braking && matches!(self.feature, Feature::Acc(_)) {
            self.state = State::Off;
            event = Event::CancelledByAeb;
        } else {
            self.state = if override_ { State::Override } else { State::Active };
        }
        self.prev_request = request;

        // Buttons work while ACC is engaged.
        if self.state != State::Off && matches!(self.feature, Feature::Acc(_)) {
            if s.buttons & buttons::SPEED_UP != 0 {
                self.acc.speed_up();
            }
            if s.buttons & buttons::SPEED_DOWN != 0 {
                self.acc.speed_down();
            }
            if s.buttons & buttons::GAP != 0 {
                self.acc.cycle_gap();
            }
        }

        // The speed controller limits jerk through the host's learned brake gain.
        self.speed.set_gains(f32::NAN, self.brake_gain);

        // --- The engaged feature ---
        let mut cmd = Command::default();
        let mut following = false;
        if self.state != State::Off {
            cmd.flags = command_flags::ENGAGED;
            if !paused && dt > 0.0 {
                self.t_engaged += dt;
                let out = features::run(
                    &self.feature,
                    &mut self.feature_state,
                    &mut self.speed,
                    &self.acc,
                    features::Ctx { sample: s, actors: inp.actors, stick: inp.stick, dt, t: self.t_engaged, follow },
                );
                if self.state == State::Override {
                    // The driver's foot wins. Restart the reference from the
                    // current speed, so letting go eases back to the target.
                    self.speed.reset(s.speed);
                }
                following = out.following;
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
            }
        }

        // --- Arbitration: AEB wins ---
        if let Some(a) = aeb {
            if a.warn {
                cmd.flags |= command_flags::FCW;
            }
            if a.decel > 0.0 {
                // Track the requested deceleration on the host's brakes.
                let brake = if a.decel.is_finite() {
                    self.aeb_i = (self.aeb_i + AEB_KI * (a.decel + self.a_est) * dt).clamp(-0.6, 0.6);
                    (a.decel / self.brake_gain + self.aeb_i).clamp(0.02, 1.0)
                } else {
                    1.0
                };
                cmd.flags |= command_flags::AEB;
                cmd.throttle = 0.0;
                cmd.brake = cmd.brake.max(brake);
                cmd.flags &= !command_flags::HOLD;
            } else {
                self.aeb_i = 0.0;
            }
            if a.hold {
                cmd.flags |= command_flags::AEB | command_flags::HOLD;
                cmd.throttle = 0.0;
                cmd.brake = 0.0;
            }
        }

        // Only a commanded brake that the host will apply counts for learning.
        self.prev_brake = if cmd.flags & (command_flags::ENGAGED | command_flags::AEB) != 0 && !paused { cmd.brake } else { 0.0 };

        // --- HUD ---
        let acc_on = self.state != State::Off && matches!(self.feature, Feature::Acc(_));
        let mut f = 0;
        if acc_on {
            f |= hud_flags::ACC;
        }
        if acc_on && following {
            f |= hud_flags::FOLLOWING;
        }
        if follow.is_some() {
            f |= hud_flags::LEAD;
        }
        if cmd.flags & command_flags::FCW != 0 {
            f |= hud_flags::FCW;
        }
        if cmd.flags & command_flags::AEB != 0 {
            f |= hud_flags::AEB;
        }
        if cmd.flags & command_flags::HOLD != 0 {
            f |= hud_flags::HOLD;
        }
        if self.state == State::Override {
            f |= hud_flags::OVERRIDE;
        }
        self.hud = Hud {
            header: Default::default(),
            flags: f,
            set_speed: self.acc.set_speed,
            time_gap: self.acc.time_gap(),
            gap_index: self.acc.gap_index as u32,
            lead_gap: follow.map(|l| l.gap).unwrap_or(f32::NAN),
            lead_speed: follow.map(|l| l.v).unwrap_or(f32::NAN),
            ttc: aeb.map(|a| a.ttc).unwrap_or(f32::INFINITY),
            _reserved: 0,
        };
        (cmd, event)
    }

    /// The buses, then tracker → leads. Returns the leads and the ego speed
    /// and yaw rate as read from chassis CAN. Without adapter info (no ego
    /// geometry, no sensor layout yet) there is no radar.
    fn perceive(&mut self, s: &VehicleSample, actors: &ActorList, dt: f32, paused: bool) -> (Option<Lead>, Option<Lead>, f32, f32) {
        let have_bus = self.vehicle.ego.frames > 0;
        let held = |v: &vbus::Vehicle| if have_bus { (v.ego.speed, v.ego.yaw_rate) } else { (s.speed, s.yaw_rate) };
        if paused || dt <= 0.0 {
            let (v, w) = held(&self.vehicle);
            return (None, None, v, w);
        }
        // The front radar ECU's scan (its sensor model sees the actors).
        let scan = match (self.info.is_some(), self.layout.as_slice().get(layout::FRONT_RADAR)) {
            (true, Some(mount)) => self.radar.step(dt, mount, actors, s.header.frame),
            _ => None,
        };
        let t = s.header.game_time_us as f64 / 1e6;
        let arrived = self.vehicle.step(t, s.header.frame, s, scan.as_ref());
        let (v, w) = (self.vehicle.ego.speed, self.vehicle.ego.yaw_rate);
        let Some(info) = self.info else { return (None, None, v, w) };
        self.tracker.predict(dt, v, w);
        self.since_scan += dt;
        if let Some(sc) = arrived {
            self.tracker.update(&sc, self.since_scan);
            self.since_scan = 0.0;
        }
        let front = info.box_max[0];
        let tracks = self.tracker.tracks();
        (
            select_lead(tracks, v, w, front, HALF_LANE, LeadKind::Follow),
            select_lead(tracks, v, w, front, HALF_LANE, LeadKind::Collision),
            v,
            w,
        )
    }

    /// The virtual vehicle (bus statistics, the frames of the last step).
    pub fn vehicle(&self) -> &vbus::Vehicle {
        &self.vehicle
    }

    pub fn vehicle_mut(&mut self) -> &mut vbus::Vehicle {
        &mut self.vehicle
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
            vx: speed,
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
    fn override_keeps_engagement() {
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

    #[test]
    fn an_impact_cancels_engagement() {
        let mut core = Core::new(Feature::Acc(Default::default()), false);
        step(&mut core, 1, sample_flags::ENGAGE_REQUEST, 13.5);
        step(&mut core, 2, sample_flags::ENGAGE_REQUEST, 13.5);
        // 13.5 → 5.3 m/s in one frame: about 490 m/s², an impact.
        let (c, e) = step(&mut core, 3, sample_flags::ENGAGE_REQUEST, 5.3);
        assert_eq!(e, Event::Impact);
        assert_eq!((c.flags, c.throttle), (0, 0.0));
        // Hard braking (8 m/s²) is not an impact.
        let mut core = Core::new(Feature::Acc(Default::default()), false);
        step(&mut core, 1, sample_flags::ENGAGE_REQUEST, 20.0);
        let (_, e) = step(&mut core, 2, sample_flags::ENGAGE_REQUEST, 20.0 - 8.0 * 0.016_667);
        assert_ne!(e, Event::Impact);
    }

    #[test]
    fn acc_buttons_change_the_set_speed() {
        let mut core = Core::new(Feature::Acc(Default::default()), false);
        step(&mut core, 1, sample_flags::ENGAGE_REQUEST, 20.0);
        let set0 = core.hud().set_speed;
        let mut s = sample(2, sample_flags::ENGAGE_REQUEST, 20.0);
        s.buttons = buttons::SPEED_UP;
        core.step(Inputs { sample: &s, actors: &ActorList::default(), stick: None });
        assert!((core.hud().set_speed - set0 - 5.0 / 3.6).abs() < 1e-4);
    }
}
