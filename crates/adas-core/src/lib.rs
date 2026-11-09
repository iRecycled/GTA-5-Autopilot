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
//! The ADAS ECU sends its requests back on chassis CAN (`ACC_CONTROL`,
//! `PRE_COLLISION_2`, `STEERING_LTA`), and the actuator ECUs (`actuators`)
//! turn them into the pedals, hold and steering the host applies.

pub mod actuators;
pub mod degrade;
pub mod driver;
pub mod features;
pub mod layout;
pub mod msgs;
pub mod telem;
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
    /// The driver braked.
    CancelledByDriver,
    /// The engage request was withdrawn.
    Disengaged,
    /// Emergency braking took over; ACC is cancelled.
    CancelledByAeb,
    /// An impact was detected (as an airbag controller's crash signal would
    /// report it): everything is cancelled.
    Impact,
    /// The feature can't run (D21): engaging was refused, or, while engaged,
    /// a takeover request starts (D11).
    Unavailable(degrade::Reason),
    /// The driver answered a takeover request (accelerator or steering).
    TakenOver,
    /// A takeover request went unanswered: the car makes a controlled stop
    /// with its hazard lights on.
    ControlledStop,
}

/// Takeover escalation (D11): a visual request at once, a chime from
/// `TAKEOVER_CHIME_AFTER`, a controlled stop at `TAKEOVER_DECEL` from
/// `TAKEOVER_STOP_AFTER` (s since the request).
pub const TAKEOVER_CHIME_AFTER: f32 = 1.5;
pub const TAKEOVER_STOP_AFTER: f32 = 4.0;
pub const TAKEOVER_DECEL: f32 = 2.0;
/// The controlled stop's deceleration builds up over this long (s).
pub const TAKEOVER_DECEL_RAMP: f32 = 0.8;

/// An impact, as crash sensing judges one: more than `IMPACT_DV` of speed
/// lost within `IMPACT_WINDOW_US` (a mean of over 15 m/s², about 1.5 g).
/// Hard braking on good tyres stays below about 1.1 g, so it loses about
/// 1.1 m/s in that time. A one-frame glitch in the host's speed (a game
/// stalling while it loads a model) loses far less, so it doesn't count.
const IMPACT_DV: f32 = 1.5;
const IMPACT_WINDOW_US: u64 = 100_000;

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
    powertrain: actuators::Powertrain,
    eps: actuators::Eps,
    /// The host's brake gain, learned on line: deceleration per unit of brake
    /// demand (m/s²). Hosts differ a lot (sim 8, GTA about 80 at light
    /// braking); this is the D19 vehicle profile, learned rather than tuned.
    brake_gain: f32,
    prev_brake: f32,
    /// Recent (game time µs, speed), for impact detection.
    impact_window: std::collections::VecDeque<(u64, f32)>,
    vehicle: vbus::Vehicle,
    /// Frames of the last chassis frame and the last radar scan (D21 health).
    chassis_last: Option<u64>,
    /// Valid chassis frames counted at the last step (health counts only
    /// frames that pass their checksums).
    chassis_seen: u64,
    radar_last: Option<u64>,
    scan_arrived: bool,
    avail: degrade::Availability,
    /// Track ids of the follow lead and the collision threat, last step (for views).
    lead_ids: (Option<u32>, Option<u32>),
    driver: driver::DriverMonitor,
    /// A takeover request in progress: (seconds since it started, why).
    takeover: Option<(f32, degrade::Reason)>,
}

/// Starting guess for the brake gain (m/s² per unit of brake demand); learned
/// from then on. The brake ECU's deceleration loop covers the rest.
const BRAKE_GAIN_INIT: f32 = 8.0;
/// The brake gain is learned only while the demand asks for less than this
/// (m/s²): well inside the tyres' grip.
const GAIN_LEARN_MAX_DECEL: f32 = 6.0;

/// One step of on-line brake-gain learning from what last frame's brake
/// demand achieved. Only in the brakes' linear range: when the demand asks
/// for more than the tyres can give, grip sets the deceleration and the gain
/// would be learned far too low.
fn learn_brake_gain(gain: f32, prev_brake: f32, speed: f32, a_est: f32, dt: f32) -> f32 {
    let linear = prev_brake * gain < GAIN_LEARN_MAX_DECEL;
    if prev_brake >= 0.03 && linear && speed > 2.0 && a_est < -0.5 {
        let g = (-a_est / prev_brake).clamp(3.0, 150.0);
        gain + (g - gain) * (dt / 0.5).min(1.0)
    } else {
        gain
    }
}

/// Acceleration per unit of throttle demand (m/s²), the calibration's guess.
const THROTTLE_GAIN: f32 = 4.0;

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
            powertrain: actuators::Powertrain::new(),
            eps: actuators::Eps::new(),
            brake_gain: BRAKE_GAIN_INIT,
            prev_brake: 0.0,
            impact_window: std::collections::VecDeque::new(),
            vehicle: vbus::Vehicle::new(can::Delivery::Reactive),
            chassis_last: None,
            chassis_seen: 0,
            radar_last: None,
            scan_arrived: false,
            avail: degrade::evaluate(None, degrade::Health::default()),
            lead_ids: (None, None),
            driver: driver::DriverMonitor::new(),
            takeover: None,
        }
    }

    /// The takeover request in progress, if any: (seconds since it started, why).
    pub fn takeover(&self) -> Option<(f32, degrade::Reason)> {
        self.takeover
    }

    /// Track ids of the follow lead and the collision threat after the last step.
    pub fn lead_ids(&self) -> (Option<u32>, Option<u32>) {
        self.lead_ids
    }

    /// The longitudinal acceleration estimate (m/s²).
    pub fn accel_estimate(&self) -> f32 {
        self.a_est
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
        // Give the radar its first scans' worth of grace from here.
        self.radar_last = None;
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

    /// The learned brake gain (m/s² per unit of brake demand).
    pub fn brake_gain(&self) -> f32 {
        self.brake_gain
    }

    /// Starts from a known brake gain (a vehicle profile, D19).
    pub fn set_brake_gain(&mut self, g: f32) {
        if g.is_finite() && (3.0..=150.0).contains(&g) {
            self.brake_gain = g;
        }
    }

    /// What may run, as of the last step (D21).
    pub fn availability(&self) -> &degrade::Availability {
        &self.avail
    }

    pub fn aeb_activations(&self) -> u32 {
        self.aeb.activations
    }

    /// One step. Returns the command payload (the caller fills the header and
    /// `answers_seq`) and what changed.
    pub fn step(&mut self, inp: Inputs) -> (Command, Event) {
        let host = inp.sample;
        let request = host.flags & sample_flags::ENGAGE_REQUEST != 0;
        let paused = host.flags & sample_flags::PAUSED != 0;

        let dt = if self.last_game_us == 0 { 0.0 } else { host.header.game_time_us.saturating_sub(self.last_game_us) as f32 / 1e6 };
        self.last_game_us = host.header.game_time_us;

        // The driver (D11): brake cancels, gas overrides, steering pauses lateral.
        let drv = self.driver.step(host, if paused { 0.0 } else { dt });
        let (cancel, override_) = (drv.brake, drv.gas);

        // --- The buses and perception (always) ---
        // From here on the core works on the car as the ADAS ECU sees it on
        // the bus, not on the host's exact values.
        let (follow, threat, bus_speed, bus_yaw) = self.perceive(host, inp.actors, dt, host.flags & sample_flags::PAUSED != 0);
        self.lead_ids = (follow.map(|l| l.id), threat.map(|l| l.id));
        let mut view = *host;
        view.speed = bus_speed;
        view.vx = bus_speed;
        view.yaw_rate = bus_yaw;
        let s = &view;

        // --- What may run (D21) ---
        let frame = host.header.frame;
        if !paused {
            if self.vehicle.ego.frames > self.chassis_seen || self.chassis_last.is_none() {
                self.chassis_last = Some(frame);
            }
            self.chassis_seen = self.vehicle.ego.frames;
            if self.scan_arrived || (self.radar_last.is_none() && self.info.is_some()) {
                self.radar_last = Some(frame);
            }
        }
        let age = |last: Option<u64>| last.map_or(u64::MAX, |l| frame.saturating_sub(l));
        self.avail = degrade::evaluate(self.info.as_ref(), degrade::Health { chassis_age: age(self.chassis_last), radar_age: age(self.radar_last) });
        self.radar.ignore_los = self.avail.los_degraded;
        let feature_ok = match self.feature {
            Feature::Acc(_) => {
                let too_slow = !self.avail.stop_and_go && s.speed < degrade::ACC_MIN_SPEED;
                self.avail.acc.and(if too_slow { Err(degrade::Reason::NoHold) } else { Ok(()) })
            }
            Feature::SpeedProfile(_) => self.avail.drive,
            Feature::TestDriver(_) => self.avail.drive.and(self.avail.steer),
        };

        // Acceleration from speed (the deceleration loop's feedback), and the
        // brake gain from what last frame's brake demand achieved.
        if dt > 0.0 && !paused {
            if let Some(p) = self.prev_speed {
                let a = (s.speed - p) / dt;
                self.a_est += (a - self.a_est) * (dt / 0.08).min(1.0);
            }
            self.prev_speed = Some(s.speed);
            self.brake_gain = learn_brake_gain(self.brake_gain, self.prev_brake, s.speed, self.a_est, dt);
        }

        // An impact cancels everything, as a crash signal does on a real car.
        let impact = if paused {
            self.impact_window.clear();
            false
        } else {
            let now = host.header.game_time_us;
            while self.impact_window.front().is_some_and(|&(t, _)| now.saturating_sub(t) > IMPACT_WINDOW_US) {
                self.impact_window.pop_front();
            }
            let peak = self.impact_window.iter().map(|&(_, v)| v).fold(s.speed, f32::max);
            self.impact_window.push_back((now, s.speed));
            let hit = dt > 0.0 && peak - s.speed > IMPACT_DV;
            if hit {
                self.impact_window.clear();
            }
            hit
        };


        // --- AEB (always) ---
        let aeb = if paused || dt <= 0.0 || self.info.is_none() || self.avail.aeb.is_err() {
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
        } else if self.state != State::Off
            && let Some((t, why)) = self.takeover
        {
            // A takeover request runs until the driver answers it (D11).
            if cancel {
                self.state = State::Off;
                event = Event::CancelledByDriver;
            } else if !request {
                self.state = State::Off;
                event = Event::Disengaged;
            } else if drv.gas || drv.steering {
                self.state = State::Off;
                event = Event::TakenOver;
            } else if aeb_braking {
                self.state = State::Off;
                event = Event::CancelledByAeb;
            } else {
                let t2 = if paused { t } else { t + dt };
                if t < TAKEOVER_STOP_AFTER && t2 >= TAKEOVER_STOP_AFTER {
                    event = Event::ControlledStop;
                }
                self.takeover = Some((t2, why));
                self.state = State::Active;
            }
        } else if self.state != State::Off && let Err(r) = feature_ok {
            // The feature can't continue: ask the driver to take over.
            self.takeover = Some((0.0, r));
            self.state = State::Active;
            event = Event::Unavailable(r);
        } else if self.state == State::Off {
            if request && !self.prev_request && !cancel && !aeb_braking && let Err(r) = feature_ok {
                event = Event::Unavailable(r);
            } else if request && !self.prev_request && !cancel && !aeb_braking {
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
        if self.state == State::Off {
            self.takeover = None;
        }
        let takeover_t = self.takeover.map(|(t, _)| t);
        let stopping = takeover_t.is_some_and(|t| t >= TAKEOVER_STOP_AFTER);

        // Buttons work while ACC is engaged (not during a takeover request).
        if self.state != State::Off && takeover_t.is_none() && matches!(self.feature, Feature::Acc(_)) {
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
            // During a takeover request the feature doesn't run: the car
            // coasts, then makes a controlled stop (below).
            if !paused && dt > 0.0 && takeover_t.is_none() {
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
        // The driver steering pauses lateral control (D11).
        if drv.steering {
            cmd.flags &= !command_flags::STEER;
        }

        // --- Arbitration: AEB wins (in the brake ECU) ---
        let gains = actuators::Gains { throttle: THROTTLE_GAIN, brake: self.brake_gain };
        let mut req = actuators::AdasRequest::default();
        if cmd.flags & command_flags::ENGAGED != 0 {
            req.acc_active = true;
            req.accel = gains.to_accel(cmd.throttle, cmd.brake);
            req.hold = cmd.flags & command_flags::HOLD != 0;
            req.steer_request = cmd.flags & command_flags::STEER != 0;
            req.steer_angle = cmd.steer_angle;
        }
        if let Some(t) = takeover_t.filter(|_| stopping) {
            // The controlled stop: a steady deceleration, ramped in to limit
            // jerk (about 2.5 m/s³), then the standstill hold, hazards on.
            let ramp = ((t - TAKEOVER_STOP_AFTER) / TAKEOVER_DECEL_RAMP).clamp(0.0, 1.0);
            req.accel = -TAKEOVER_DECEL * ramp;
            req.hold = s.speed < 0.3;
        }
        if let Some(a) = aeb {
            if a.warn {
                cmd.flags |= command_flags::FCW;
            }
            if a.decel > 0.0 {
                cmd.flags |= command_flags::AEB;
                req.aeb_decel = a.decel;
            }
            if a.hold {
                cmd.flags |= command_flags::AEB;
                req.aeb_hold = true;
            }
        }

        // --- Actuation over chassis CAN ---
        cmd.throttle = 0.0;
        cmd.brake = 0.0;
        cmd.steer_angle = 0.0;
        cmd.flags &= !(command_flags::HOLD | command_flags::STEER);
        if !paused && dt > 0.0 {
            let t = host.header.game_time_us as f64 / 1e6;
            let (accel_rx, steer_rx) = self.vehicle.actuate(t, &req);
            let (throttle, brake, hold) = self.powertrain.step(&accel_rx, gains, self.a_est, dt);
            cmd.throttle = throttle;
            cmd.brake = brake;
            if hold {
                cmd.flags |= command_flags::HOLD;
            }
            if let Some(a) = self.eps.step(&steer_rx, self.vehicle.ego.steer_angle, dt) {
                cmd.flags |= command_flags::STEER;
                cmd.steer_angle = a;
            }
        }

        if stopping {
            cmd.flags |= command_flags::HAZARDS;
        }

        // Only a commanded brake that the host will apply counts for learning.
        self.prev_brake = if cmd.flags & (command_flags::ENGAGED | command_flags::AEB) != 0 && !paused { cmd.brake } else { 0.0 };

        // --- HUD ---
        let acc_on = self.state != State::Off && takeover_t.is_none() && matches!(self.feature, Feature::Acc(_));
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
        let mut message = match feature_ok {
            Err(r) => {
                f |= hud_flags::UNAVAILABLE;
                r.code()
            }
            Ok(()) => 0,
        };
        if let Some((t, why)) = self.takeover {
            // The request stands until the driver answers, even if the cause clears.
            f |= hud_flags::TAKEOVER;
            if t >= TAKEOVER_CHIME_AFTER {
                f |= hud_flags::TAKEOVER_CHIME;
            }
            if t >= TAKEOVER_STOP_AFTER {
                f |= hud_flags::CONTROLLED_STOP;
            }
            message = why.code();
        }
        if self.avail.aeb.is_err() {
            f |= hud_flags::AEB_OFF;
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
            message,
        };
        (cmd, event)
    }

    /// The buses, then tracker → leads. Returns the leads and the ego speed
    /// and yaw rate as read from chassis CAN. Without adapter info (no ego
    /// geometry, no sensor layout yet) there is no radar.
    fn perceive(&mut self, s: &VehicleSample, actors: &ActorList, dt: f32, paused: bool) -> (Option<Lead>, Option<Lead>, f32, f32) {
        let have_bus = self.vehicle.ego.frames > 0;
        let held = |v: &vbus::Vehicle| if have_bus { (v.ego.speed, v.ego.yaw_rate) } else { (s.speed, s.yaw_rate) };
        self.scan_arrived = false;
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
        self.scan_arrived = arrived.is_some();
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

    fn with_info(f: Feature, initial: bool) -> Core {
        let mut c = Core::new(f, initial);
        c.set_info(&full_info());
        c
    }

    fn full_info() -> AdapterInfo {
        use adapter_api::caps;
        AdapterInfo { capabilities: caps::STEER | caps::PEDALS | caps::HOLD | caps::ACTORS | caps::LINE_OF_SIGHT, box_max: [2.3, 0.9, 1.5], ..Default::default() }
    }

    fn step(core: &mut Core, seq: u32, flags: u32, speed: f32) -> (Command, Event) {
        let s = sample(seq, flags, speed);
        let a = ActorList::default();
        core.step(Inputs { sample: &s, actors: &a, stick: None })
    }

    #[test]
    fn never_engages_on_a_stale_request() {
        let mut core = with_info(Feature::speed_profile(), true);
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
        let mut core = with_info(Feature::speed_profile(), false);
        step(&mut core, 1, sample_flags::ENGAGE_REQUEST, 0.0);
        let (_, e) = step(&mut core, 2, sample_flags::ENGAGE_REQUEST | sample_flags::DRIVER_CANCEL, 0.0);
        assert_eq!(e, Event::CancelledByDriver);
        let (c, _) = step(&mut core, 3, sample_flags::ENGAGE_REQUEST, 0.0);
        assert_eq!(c.flags, 0, "re-engaged without a fresh request");
    }

    #[test]
    fn override_keeps_engagement() {
        let mut core = with_info(Feature::speed_profile(), false);
        step(&mut core, 1, sample_flags::ENGAGE_REQUEST, 0.0);
        let (c, _) = step(&mut core, 2, sample_flags::ENGAGE_REQUEST | sample_flags::DRIVER_OVERRIDE, 3.0);
        assert!(c.flags & command_flags::ENGAGED != 0);
        assert_eq!(core.state(), State::Override);
        step(&mut core, 3, sample_flags::ENGAGE_REQUEST, 3.0);
        assert_eq!(core.state(), State::Active);
    }

    #[test]
    fn paused_means_no_actuation() {
        let mut core = with_info(Feature::speed_profile(), false);
        step(&mut core, 1, sample_flags::ENGAGE_REQUEST, 0.0);
        let (c, _) = step(&mut core, 2, sample_flags::ENGAGE_REQUEST | sample_flags::PAUSED, 0.0);
        assert_eq!((c.throttle, c.brake), (0.0, 0.0));
    }

    #[test]
    fn deterministic_for_identical_inputs() {
        let run = || {
            let mut core = with_info(Feature::speed_profile(), false);
            (1..600).map(|i| step(&mut core, i, sample_flags::ENGAGE_REQUEST, (i as f32 * 0.01).min(8.0)).0).collect::<Vec<_>>()
        };
        assert_eq!(run(), run());
    }

    #[test]
    fn the_brake_gain_is_learned_only_inside_grip() {
        // Brakes give 40 m/s² per unit: 0.1 of brake measures 4 m/s².
        let mut g = 8.0;
        for _ in 0..200 {
            g = learn_brake_gain(g, 0.1, 15.0, -4.0, 1.0 / 72.0);
        }
        assert!((g - 40.0).abs() < 1.0, "learned {g}");
        // Full braking measures the tyres' 9 m/s², not the brakes: no change.
        for _ in 0..200 {
            g = learn_brake_gain(g, 1.0, 15.0, -9.0, 1.0 / 72.0);
        }
        assert!((g - 40.0).abs() < 1.0, "full braking taught {g}");
    }

    #[test]
    fn an_impact_cancels_engagement() {
        let mut core = Core::new(Feature::Acc(Default::default()), false);
        core.set_info(&full_info());
        step(&mut core, 1, sample_flags::ENGAGE_REQUEST, 13.5);
        step(&mut core, 2, sample_flags::ENGAGE_REQUEST, 13.5);
        // 13.5 → 5.3 m/s in one frame: an impact.
        let (c, e) = step(&mut core, 3, sample_flags::ENGAGE_REQUEST, 5.3);
        assert_eq!(e, Event::Impact);
        assert_eq!((c.flags, c.throttle), (0, 0.0));
        // Hard braking (8 m/s²) is not an impact.
        let mut core = Core::new(Feature::Acc(Default::default()), false);
        core.set_info(&full_info());
        step(&mut core, 1, sample_flags::ENGAGE_REQUEST, 20.0);
        for i in 2..40 {
            let (_, e) = step(&mut core, i, sample_flags::ENGAGE_REQUEST, 20.0 - 11.0 * 0.016_667 * (i - 1) as f32);
            assert_ne!(e, Event::Impact, "hard braking at step {i}");
        }
        // A one-frame glitch, as when GTA stalls loading a model: 0.24 m/s
        // in one frame (about 15 m/s²), then back to normal.
        let mut core = Core::new(Feature::Acc(Default::default()), false);
        core.set_info(&full_info());
        for (i, v) in [(1, 13.88), (2, 13.88), (3, 13.64), (4, 13.6), (5, 13.6)] {
            let (_, e) = step(&mut core, i, sample_flags::ENGAGE_REQUEST, v);
            assert_ne!(e, Event::Impact, "glitch at step {i}");
        }
    }

    /// ACC at 20 m/s, then the radar's bus goes silent for good.
    fn acc_with_a_dead_radar() -> (Core, u32) {
        let mut core = Core::new(Feature::Acc(Default::default()), false);
        core.set_info(&full_info());
        let mut seq = 1;
        for _ in 0..60 {
            step(&mut core, seq, sample_flags::ENGAGE_REQUEST, 20.0);
            seq += 1;
        }
        assert_eq!(core.state(), State::Active);
        core.vehicle_mut().inject("adas", None, can::FaultKind::Drop, 1.0, f64::NEG_INFINITY, f64::INFINITY).unwrap();
        (core, seq)
    }

    #[test]
    fn an_unanswered_takeover_request_escalates_to_a_controlled_stop() {
        let (mut core, mut seq) = acc_with_a_dead_radar();
        let mut events = Vec::new();
        let (mut asked, mut chimed, mut stopped) = (None, None, None);
        let mut speed = 20.0f32;
        let mut last = Command::default();
        for _ in 0..60 * 20 {
            let (c, e) = step(&mut core, seq, sample_flags::ENGAGE_REQUEST, speed);
            let t = seq as f32 / 60.0;
            if e != Event::None {
                events.push(e);
            }
            let f = core.hud().flags;
            if f & hud_flags::TAKEOVER != 0 && asked.is_none() {
                asked = Some(t);
            }
            if f & hud_flags::TAKEOVER_CHIME != 0 && chimed.is_none() {
                chimed = Some(t);
            }
            if f & hud_flags::CONTROLLED_STOP != 0 && stopped.is_none() {
                stopped = Some(t);
            }
            // The car obeys the brake (8 m/s² per unit, the initial gain).
            speed = (speed - 8.0 * c.brake / 60.0).max(0.0);
            last = c;
            seq += 1;
        }
        let (asked, chimed, stopped) = (asked.expect("takeover requested"), chimed.expect("chime"), stopped.expect("controlled stop"));
        assert!(matches!(events[0], Event::Unavailable(degrade::Reason::RadarSilent)), "{events:?}");
        assert!(events.contains(&Event::ControlledStop), "{events:?}");
        assert!((chimed - asked - TAKEOVER_CHIME_AFTER).abs() < 0.05 && (stopped - asked - TAKEOVER_STOP_AFTER).abs() < 0.05);
        // Stopped, held, hazards on, still engaged (the system has the car).
        assert!(speed < 0.3, "still at {speed} m/s");
        assert!(last.flags & command_flags::HAZARDS != 0 && last.flags & command_flags::HOLD != 0 && last.flags & command_flags::ENGAGED != 0);
        assert_eq!(core.hud().message, degrade::Reason::RadarSilent.code());
    }

    #[test]
    fn the_driver_answers_a_takeover_request_with_the_accelerator() {
        let (mut core, mut seq) = acc_with_a_dead_radar();
        for _ in 0..60 * 2 {
            step(&mut core, seq, sample_flags::ENGAGE_REQUEST, 20.0);
            seq += 1;
        }
        assert!(core.takeover().is_some());
        let mut s = sample(seq, sample_flags::ENGAGE_REQUEST, 20.0);
        s.driver_throttle = 0.3;
        let (c, e) = core.step(Inputs { sample: &s, actors: &ActorList::default(), stick: None });
        assert_eq!(e, Event::TakenOver);
        assert_eq!(c.flags & command_flags::HAZARDS, 0);
        assert_eq!(core.state(), State::Off);
        assert_eq!(core.hud().flags & hud_flags::TAKEOVER, 0);
    }

    #[test]
    fn steering_pauses_lateral_but_does_not_cancel() {
        let mut core = with_info(Feature::TestDriver(Default::default()), false);
        let stick = Some(Stick { throttle: 0.0, brake: 0.0, steer: 0.5 });
        let mut steered = |seq: u32, driver_steer: f32| {
            let mut s = sample(seq, sample_flags::ENGAGE_REQUEST, 5.0);
            s.driver_steer = driver_steer;
            let (c, e) = core.step(Inputs { sample: &s, actors: &ActorList::default(), stick });
            (c.flags & command_flags::STEER != 0, e)
        };
        let mut seq = 1;
        for _ in 0..30 {
            steered(seq, 0.0);
            seq += 1;
        }
        assert!(steered(seq, 0.0).0, "the test driver steers");
        // The driver holds a steering key for half a second.
        let mut flags = Vec::new();
        for _ in 0..30 {
            seq += 1;
            let (st, e) = steered(seq, 1.0);
            assert_ne!(e, Event::CancelledByDriver);
            flags.push(st);
        }
        assert!(flags[0], "a key tap doesn't count yet");
        assert!(!*flags.last().unwrap(), "held: lateral control paused");
        assert_ne!(core.state(), State::Off);
    }

    #[test]
    fn an_unavailable_feature_refuses_to_engage() {
        // ACC with no adapter info: refused, with the reason.
        let mut core = Core::new(Feature::Acc(Default::default()), false);
        let (c, e) = step(&mut core, 1, sample_flags::ENGAGE_REQUEST, 20.0);
        assert_eq!(e, Event::Unavailable(degrade::Reason::NoAdapterInfo));
        assert_eq!(c.flags, 0);
        assert_ne!(core.hud().flags & hud_flags::UNAVAILABLE, 0);
        // A host without pedals.
        let mut core = Core::new(Feature::Acc(Default::default()), false);
        core.set_info(&AdapterInfo { capabilities: adapter_api::caps::ACTORS | adapter_api::caps::STEER, ..Default::default() });
        let (_, e) = step(&mut core, 1, sample_flags::ENGAGE_REQUEST, 20.0);
        assert_eq!(e, Event::Unavailable(degrade::Reason::NoPedals));
    }

    #[test]
    fn acc_buttons_change_the_set_speed() {
        let mut core = Core::new(Feature::Acc(Default::default()), false);
        core.set_info(&full_info());
        step(&mut core, 1, sample_flags::ENGAGE_REQUEST, 20.0);
        let set0 = core.hud().set_speed;
        let mut s = sample(2, sample_flags::ENGAGE_REQUEST, 20.0);
        s.buttons = buttons::SPEED_UP;
        core.step(Inputs { sample: &s, actors: &ActorList::default(), stick: None });
        assert!((core.hud().set_speed - set0 - 5.0 / 3.6).abs() < 1e-4);
    }
}
