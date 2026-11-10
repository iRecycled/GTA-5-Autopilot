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

use adapter_api::{
    ActorList, AdapterInfo, Command, FanLayout, Hud, LaneMap, RangeReport, SensorLayout, VehicleSample, buttons, caps, command_flags, fan_flags,
    hud_flags, sample_flags, surround_flags,
};
use control::{Config, SpeedController};
use features::FeatureState;
use fusion::{Lead, LeadKind, SurroundConfig, SurroundTracker, Tracker, TrackerConfig, select_lead, select_lead_in_lane, select_pedestrian};
use sensors::{Camera, CameraConfig, FrontRadar, LaneFit, Radar, RadarConfig, RadarObject, RadarScan, UltrasonicConfig, Ultrasonics, camera};

pub use features::{Feature, Stick};

/// The core's inputs for one step.
#[derive(Clone, Copy, Debug)]
pub struct Inputs<'a> {
    pub sample: &'a VehicleSample,
    pub actors: &'a ActorList,
    /// The host's answers to the range fans, while they are active.
    pub ranges: Option<&'a RangeReport>,
    /// The lane markings around the car (`caps::MAP`).
    pub lanes: Option<&'a LaneMap>,
    /// The test driver's stick, when the test-driver feature is running.
    pub stick: Option<Stick>,
}

/// The parking sensors run below this speed, and stop above the second
/// (m/s), as on real cars.
pub const FANS_ON_SPEED: f32 = 18.0 / 3.6;
pub const FANS_OFF_SPEED: f32 = 20.0 / 3.6;
/// After the parking sensors start, missing data isn't a fault for this
/// long (frames): the host needs a few frames to answer.
const PARK_GRACE_FRAMES: u64 = 36;

/// Which surround features are switched on (the driver's settings; all on
/// by default).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Toggles {
    pub bsd: bool,
    pub rcta: bool,
    pub park: bool,
    pub lka: bool,
    /// Junction AEB: braking for traffic crossing the path ahead.
    pub junction: bool,
    /// The driver monitor (D11, optional): hands on the wheel while the
    /// autopilot steers.
    pub driver_monitor: bool,
}

impl Default for Toggles {
    fn default() -> Self {
        Self { bsd: true, rcta: true, park: true, lka: true, junction: true, driver_monitor: false }
    }
}

/// The driver monitor's timings (openpilot's hands-on-wheel policy): a
/// reminder after this long without steering input, and a takeover request
/// (D11's escalation) after the second (s).
pub const HANDS_REMIND_AFTER: f32 = 15.0;
pub const HANDS_TAKEOVER_AFTER: f32 = 25.0;

/// What the surround features did in the last step (for the HUD and views).
#[derive(Clone, Copy, Debug, Default)]
pub struct Surround {
    pub bsd: ::features::BsdOut,
    pub rcta: ::features::RctaOut,
    pub park: ::features::ParkOut,
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
    /// The brake gain loaded from a vehicle profile, if any, and the core's
    /// brake demand smoothed (the learning waits for it to hold steady).
    brake_profile: Option<f32>,
    brake_smoothed: f32,
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
    /// The AEB threat this step is crossing traffic (junction AEB).
    aeb_crossing: bool,
    driver: driver::DriverMonitor,
    /// A takeover request in progress: (seconds since it started, why).
    takeover: Option<(f32, degrade::Reason)>,
    // --- Surround (Phase 2) ---
    corners: [Radar; 4],
    surround: SurroundTracker,
    ultrasonics: Ultrasonics,
    fans: FanLayout,
    /// Frames since the parking sensors started (`None` = off).
    fans_on: Option<u64>,
    bsd: ::features::Bsd,
    rcta: ::features::Rcta,
    park: ::features::ParkAssist,
    /// Frames of the last scan from each corner radar, and of the last park cycle.
    corner_last: [Option<u64>; 4],
    park_last: Option<u64>,
    park_fault: bool,
    surround_out: Surround,
    toggles: Toggles,
    // --- Camera and lanes (Phase 3) ---
    camera: Camera,
    camera_last: Option<u64>,
    /// The camera's lane lines, when a frame arrived this step.
    cam_lines: Option<[LaneFit; 4]>,
    lanes: ::features::Lanes,
    lane_out: ::features::LaneOut,
    /// Pedestrians, tracked from the camera's objects.
    peds: Tracker,
    since_cam: f32,
    /// The lateral command was LKA's nudge this step.
    lka_on: bool,
    /// LKA nudges applied so far.
    lka_count: u32,
    /// The EPS applied a lane feature's request last step.
    eps_steering: bool,
    /// Seconds since the driver's hands were last felt on the wheel.
    hands_off: f32,
}

/// Starting guess for the brake gain (m/s² per unit of brake demand); learned
/// from then on. The brake ECU's deceleration loop covers the rest.
const BRAKE_GAIN_INIT: f32 = 8.0;
/// The brake gain is learned only while the demand asks for less than this
/// (m/s²): well inside the tyres' grip.
const GAIN_LEARN_MAX_DECEL: f32 = 6.0;

/// The brake gain's learning time constant (s), and how far a session may
/// move it from a saved profile (a factor either way).
const GAIN_LEARN_TAU: f32 = 2.0;
const GAIN_PROFILE_BAND: f32 = 1.3;

/// What the brake-gain learning needs to know about the moment.
#[derive(Clone, Copy, Debug)]
struct BrakeSample {
    /// Last frame's brake demand from the core, and that demand smoothed.
    demand: f32,
    smoothed: f32,
    speed: f32,
    a_est: f32,
    /// The driver is on a pedal: the deceleration isn't the core's alone.
    driver_pedals: bool,
}

/// One step of on-line brake-gain learning from what last frame's brake
/// demand achieved. Only when the deceleration is the core's demand's doing:
/// in the brakes' linear range (when the demand asks for more than the tyres
/// can give, grip sets the deceleration and the gain would be learned far
/// too low), with the demand steady (the brakes lag it), and with the
/// driver off the pedals (a manual GTA drive taught 85 and 110 with the
/// driver braking over a small AEB demand). Within `GAIN_PROFILE_BAND` of
/// the saved profile, when there is one.
fn learn_brake_gain(gain: f32, b: BrakeSample, profile: Option<f32>, dt: f32) -> f32 {
    let linear = b.demand * gain < GAIN_LEARN_MAX_DECEL;
    let steady = (b.demand - b.smoothed).abs() <= 0.15 * b.demand;
    if b.demand < 0.03 || !linear || !steady || b.driver_pedals || b.speed <= 2.0 || b.a_est >= -0.5 {
        return gain;
    }
    let (lo, hi) = match profile {
        Some(p) => (p / GAIN_PROFILE_BAND, p * GAIN_PROFILE_BAND),
        None => (3.0, 150.0),
    };
    let g = (-b.a_est / b.demand).clamp(lo, hi);
    (gain + (g - gain) * (dt / GAIN_LEARN_TAU).min(1.0)).clamp(lo, hi)
}

/// Acceleration per unit of throttle demand (m/s²), the calibration's guess.
const THROTTLE_GAIN: f32 = 4.0;

impl Core {
    /// `initial_request`: the engage-request flag as first seen, so a request
    /// that was already set before this core started doesn't engage it.
    pub fn new(feature: Feature, initial_request: bool) -> Self {
        // ACC may brake harder than the cruise default (up to 3.5 m/s²).
        let speed_cfg = if feature.longitudinal() { Config { decel_max: 3.5, ..Config::default() } } else { Config::default() };
        let acc_cfg = feature.acc().unwrap_or_default();
        // A production EPS for the features; the development test driver
        // and speed profile keep Phase 0's wide limits.
        let eps_limits = if feature.longitudinal() { actuators::EpsLimits::Tss2 } else { actuators::EpsLimits::Wide };
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
            eps: actuators::Eps::with_limits(eps_limits),
            brake_gain: BRAKE_GAIN_INIT,
            brake_profile: None,
            brake_smoothed: 0.0,
            prev_brake: 0.0,
            impact_window: std::collections::VecDeque::new(),
            vehicle: vbus::Vehicle::new(can::Delivery::Reactive),
            chassis_last: None,
            chassis_seen: 0,
            radar_last: None,
            scan_arrived: false,
            avail: degrade::evaluate(None, degrade::Health::default()),
            lead_ids: (None, None),
            aeb_crossing: false,
            driver: driver::DriverMonitor::new(),
            takeover: None,
            corners: core::array::from_fn(|k| Radar::new(RadarConfig::corner(), layout::CORNERS[k])),
            surround: SurroundTracker::new(SurroundConfig::default()),
            ultrasonics: Ultrasonics::new(UltrasonicConfig::default()),
            fans: FanLayout::default(),
            fans_on: None,
            bsd: Default::default(),
            rcta: Default::default(),
            park: Default::default(),
            corner_last: [None; 4],
            park_last: None,
            park_fault: false,
            surround_out: Surround::default(),
            toggles: Toggles::default(),
            camera: Camera::new(CameraConfig::default(), layout::FRONT_CAMERA),
            camera_last: None,
            cam_lines: None,
            lanes: Default::default(),
            lane_out: Default::default(),
            peds: Tracker::new(TrackerConfig { confirm_hits: 2, max_misses: 4, pos_noise_m: 0.2, pos_noise_frac: 0.04, vel_noise: 0.4, ..Default::default() }),
            since_cam: 0.0,
            lka_on: false,
            lka_count: 0,
            eps_steering: false,
            hands_off: 0.0,
        }
    }

    /// What the lane features saw and did in the last step.
    pub fn lane_out(&self) -> &::features::LaneOut {
        &self.lane_out
    }

    /// Pedestrian tracks (camera).
    pub fn pedestrians(&self) -> &[fusion::Track] {
        self.peds.tracks()
    }

    /// The AEB threat in the last step was crossing traffic (junction AEB).
    pub fn aeb_crossing(&self) -> bool {
        self.aeb_crossing
    }

    /// LKA nudges applied so far.
    /// Lane centering's learned steering response at speed `v` (m/s).
    pub fn lane_steer_gain(&self, v: f32) -> f32 {
        self.lanes.response.at(v)
    }

    pub fn lka_nudges(&self) -> u32 {
        self.lka_count
    }

    /// The wheelbase: the host's, else 60 % of the box length (m).
    fn wheelbase(&self) -> f32 {
        match self.info {
            Some(i) if i.wheelbase.is_finite() && i.wheelbase > 1.0 => i.wheelbase,
            Some(i) => (0.6 * (i.box_max[0] - i.box_min[0])).clamp(2.0, 4.0),
            None => 2.7,
        }
    }

    /// Switches surround features on or off.
    pub fn set_toggles(&mut self, t: Toggles) {
        self.toggles = t;
    }

    pub fn toggles(&self) -> Toggles {
        self.toggles
    }

    /// The surround tracks (corner radars) after the last step.
    pub fn surround_tracks(&self) -> &[fusion::SurroundTrack] {
        self.surround.tracks()
    }

    /// What BSD, RCTA and park assist did in the last step.
    pub fn surround(&self) -> &Surround {
        &self.surround_out
    }

    /// The standing range fans for the adapter (`fan_flags::ACTIVE` while
    /// the parking sensors run).
    pub fn fans(&self) -> &FanLayout {
        &self.fans
    }

    /// RCTA and park assist brake activations so far.
    pub fn surround_activations(&self) -> (u32, u32) {
        (self.rcta.activations, self.park.activations)
    }

    /// BSD icon episodes so far (left, right).
    pub fn bsd_episodes(&self) -> [u32; 2] {
        self.bsd.episodes
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
        // Give the radars their first scans' worth of grace from here.
        self.radar_last = None;
        self.corner_last = [None; 4];
        self.camera_last = None;
        self.layout = layout::default_layout(info.box_min, info.box_max);
        let flags = self.fans.flags;
        self.fans = if info.capabilities & caps::RANGE_FANS != 0 { layout::default_fans(info.box_min, info.box_max) } else { FanLayout::default() };
        self.fans.flags = flags;
    }

    /// The adapter's identity and ego geometry, once received.
    pub fn info(&self) -> Option<&AdapterInfo> {
        self.info.as_ref()
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

    /// Starts from a known brake gain (a vehicle profile, D19); the session
    /// then learns within a band around it.
    pub fn set_brake_gain(&mut self, g: f32) {
        if g.is_finite() && (3.0..=150.0).contains(&g) {
            self.brake_gain = g;
            self.brake_profile = Some(g);
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

        // The parking sensors run at parking speeds (as the ADAS ECU knows
        // the speed: from the bus), if the host answers range fans.
        if !paused && dt > 0.0 {
            let v = self.vehicle.ego.speed;
            let on = self.fans.count > 0 && if self.fans_on.is_some() { v <= FANS_OFF_SPEED } else { v < FANS_ON_SPEED };
            self.fans_on = if on { Some(self.fans_on.map_or(0, |n| n + 1)) } else { None };
            self.fans.flags = if on { fan_flags::ACTIVE } else { 0 };
        }

        // --- The buses and perception (always) ---
        // From here on the core works on the car as the ADAS ECU sees it on
        // the bus, not on the host's exact values.
        let (follow, threat, bus_speed, bus_yaw, park_cycle) = self.perceive(host, inp.actors, inp.ranges, inp.lanes, dt, paused);
        let reverse = self.vehicle.ego.reverse;
        // Pedestrians the camera sees in the path (Phase 3): AEB takes the
        // nearer of the two threats.
        let ped = match self.info {
            Some(i) if !reverse => select_pedestrian(self.peds.tracks(), bus_speed, i.box_max[0], 0.5 * (i.box_max[1] - i.box_min[1])),
            _ => None,
        };
        // Going backwards, nothing ahead is a collision threat.
        let threat = match (threat.filter(|_| !reverse), ped) {
            (Some(a), Some(b)) => Some(if b.gap < a.gap { b } else { a }),
            (a, b) => a.or(b),
        };
        // Traffic crossing the path ahead (junction AEB), from the front
        // corner radars: AEB takes it if it comes sooner than the threat ahead.
        let k = if bus_speed > 1.0 { bus_yaw / bus_speed } else { 0.0 };
        let cross = if self.toggles.junction && !reverse && self.info.is_some() {
            ::features::select_crossing(&::features::JunctionConfig::default(), &self.ego(), k, self.surround.tracks()).map(|(l, _)| l)
        } else {
            None
        };
        let ttc = |l: &Option<fusion::Lead>| l.map_or(f32::INFINITY, |l| ::features::aeb::ttc(&l, bus_speed));
        self.aeb_crossing = cross.is_some() && ttc(&cross) < ttc(&threat);
        let threat = if self.aeb_crossing { cross } else { threat };
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
            for last in &mut self.corner_last {
                if last.is_none() && self.info.is_some() {
                    *last = Some(frame);
                }
            }
            if park_cycle.is_some() || self.fans_on == Some(0) {
                self.park_last = Some(frame);
            }
            if let Some(c) = &park_cycle {
                self.park_fault = c.iter().any(|d| d.is_nan());
            }
            if self.cam_lines.is_some() || (self.camera_last.is_none() && self.info.is_some()) {
                self.camera_last = Some(frame);
            }
        }
        let age = |last: Option<u64>| last.map_or(u64::MAX, |l| frame.saturating_sub(l));
        self.avail = degrade::evaluate(
            self.info.as_ref(),
            degrade::Health {
                chassis_age: age(self.chassis_last),
                radar_age: age(self.radar_last),
                rear_corner_age: [age(self.corner_last[2]), age(self.corner_last[3])],
                park_running: self.fans_on.is_some_and(|n| n > PARK_GRACE_FRAMES),
                park_age: age(self.park_last),
                park_fault: self.park_fault,
                camera_age: age(self.camera_last),
            },
        );
        self.radar.ignore_los = self.avail.los_degraded;
        for r in &mut self.corners {
            r.ignore_los = self.avail.los_degraded;
        }
        self.camera.ignore_los = self.avail.los_degraded;

        // --- Lanes (camera): lane centering, LKA, curve speed ---
        if !paused && dt > 0.0 {
            let ego = self.ego();
            // The steering response, from the bus: wheel angle, yaw rate, speed.
            let e = self.vehicle.ego;
            if !e.reverse && self.eps_steering {
                self.lanes.learn(dt, e.steer_angle, e.yaw_rate, e.speed, self.wheelbase());
            }
            // Lane centering steers while engaged, takeover requests included.
            let centering = self.state != State::Off && matches!(self.feature, Feature::Autopilot(_));
            self.lane_out = if self.avail.lanes.is_ok() {
                self.lanes.step(dt, self.cam_lines.as_ref(), &ego, self.wheelbase(), centering, drv.steering)
            } else {
                ::features::LaneOut { curve_speed: f32::INFINITY, ..Default::default() }
            };
        }
        // The driver monitor: a light touch on the wheel counts as hands on
        // (a production car feels the torque; we see the input).
        let steering_engaged = self.state != State::Off && matches!(self.feature, Feature::Autopilot(_));
        if !self.toggles.driver_monitor || !steering_engaged || host.driver_steer.abs() > 0.02 {
            self.hands_off = 0.0;
        } else if !paused {
            self.hands_off += dt;
        }
        let attentive = if self.hands_off >= HANDS_TAKEOVER_AFTER { Err(degrade::Reason::NoAttention) } else { Ok(()) };
        let lanes_ok = self.avail.lanes.and(attentive).and(match self.lane_out.issue {
            Some(::features::LaneIssue::NoLanes) => Err(degrade::Reason::NoLanes),
            Some(::features::LaneIssue::LaneEnds) => Err(degrade::Reason::LaneEnds),
            Some(::features::LaneIssue::NotInLane) => Err(degrade::Reason::NotInLane),
            None => Ok(()),
        });
        let feature_ok = match self.feature {
            Feature::Acc(_) | Feature::Autopilot(_) => {
                let too_slow = !self.avail.stop_and_go && s.speed < degrade::ACC_MIN_SPEED;
                let acc = self
                    .avail
                    .acc
                    .and(if too_slow { Err(degrade::Reason::NoHold) } else { Ok(()) })
                    .and(if reverse { Err(degrade::Reason::Reversing) } else { Ok(()) });
                if matches!(self.feature, Feature::Autopilot(_)) { acc.and(lanes_ok) } else { acc }
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
            self.brake_smoothed += (self.prev_brake - self.brake_smoothed) * (dt / 0.3).min(1.0);
            let b = BrakeSample {
                demand: self.prev_brake,
                smoothed: self.brake_smoothed,
                speed: s.speed,
                a_est: self.a_est,
                driver_pedals: s.driver_brake > 0.02 || s.driver_throttle > 0.02,
            };
            self.brake_gain = learn_brake_gain(self.brake_gain, b, self.brake_profile, dt);
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
        // --- BSD, RCTA, park assist (always) ---
        self.surround_out = self.run_surround(dt, paused, park_cycle.as_ref());
        let sur = self.surround_out;
        let auto_decel = if self.avail.surround_brake { sur.rcta.decel.max(sur.park.decel) } else { 0.0 };
        let auto_hold = self.avail.surround_brake && (sur.rcta.hold || sur.park.hold);
        let aeb_braking = aeb.is_some_and(|a| a.decel > 0.0 || a.hold) || auto_decel > 0.0 || auto_hold;

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
                if self.feature.longitudinal() {
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
        } else if aeb_braking && self.feature.longitudinal() {
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
        if self.state != State::Off && takeover_t.is_none() && self.feature.longitudinal() {
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
                    features::Ctx {
                        sample: s,
                        actors: inp.actors,
                        stick: inp.stick,
                        dt,
                        t: self.t_engaged,
                        follow,
                        curve_speed: if self.avail.lanes.is_ok() { self.lane_out.curve_speed } else { f32::INFINITY },
                        lane_angle: self.lane_out.angle,
                    },
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
        // During a takeover request the autopilot keeps steering while it
        // still sees the lane, until the driver answers (as openpilot's soft
        // disable keeps control): letting go hands-off would be worse.
        if takeover_t.is_some()
            && !paused
            && matches!(self.feature, Feature::Autopilot(_))
            && let Some(a) = self.lane_out.angle
        {
            cmd.flags |= command_flags::STEER;
            cmd.steer_angle = a;
        }
        // The driver steering pauses lateral control (D11).
        if drv.steering {
            cmd.flags &= !command_flags::STEER;
        }
        // LKA's nudge, when nothing else steers.
        let nudge = self.lane_out.nudge.filter(|_| {
            cmd.flags & command_flags::STEER == 0 && !drv.steering && !paused && self.toggles.lka && self.avail.lanes.is_ok() && !reverse
        });

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
        if let Some(a) = nudge {
            req.steer_request = true;
            req.steer_angle = a;
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
        // Rear cross-traffic and park assist brake through the same request.
        if auto_decel > 0.0 {
            cmd.flags |= command_flags::AEB;
            req.aeb_decel = req.aeb_decel.max(auto_decel);
        }
        if auto_hold && req.aeb_decel <= 0.0 {
            cmd.flags |= command_flags::AEB;
            req.aeb_hold = true;
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
            if let Some(a) = self.eps.step(&steer_rx, self.vehicle.ego.steer_angle, dt, self.vehicle.ego.speed) {
                cmd.flags |= command_flags::STEER;
                cmd.steer_angle = a;
                if nudge.is_some() && cmd.flags & command_flags::ENGAGED == 0 {
                    cmd.flags |= command_flags::LKA;
                }
            }
        }
        let lka_now = nudge.is_some() && cmd.flags & command_flags::STEER != 0;
        if lka_now && !self.lka_on {
            self.lka_count += 1;
        }
        self.lka_on = lka_now;
        self.eps_steering = cmd.flags & command_flags::STEER != 0 && (lka_now || matches!(self.feature, Feature::Autopilot(_)));

        if stopping {
            cmd.flags |= command_flags::HAZARDS;
        }

        // Only a commanded brake that the host will apply counts for learning.
        self.prev_brake = if cmd.flags & (command_flags::ENGAGED | command_flags::AEB) != 0 && !paused { cmd.brake } else { 0.0 };

        // --- HUD ---
        let acc_on = self.state != State::Off && takeover_t.is_none() && self.feature.longitudinal();
        let mut f = 0;
        if acc_on && matches!(self.feature, Feature::Autopilot(_)) && cmd.flags & command_flags::STEER != 0 {
            f |= hud_flags::LANE_CENTERING;
        }
        if acc_on && matches!(self.feature, Feature::Autopilot(_)) && drv.steering {
            f |= hud_flags::STEER_OVERRIDE;
        }
        if self.lane_out.left_seen {
            f |= hud_flags::LANE_LEFT;
        }
        if self.lane_out.right_seen {
            f |= hud_flags::LANE_RIGHT;
        }
        if self.lka_on {
            f |= hud_flags::LKA;
        }
        if self.hands_off >= HANDS_REMIND_AFTER {
            f |= hud_flags::HANDS_ON;
        }
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
            surround: self.surround_flags(),
            park_tone: if sur.park.active { sur.park.tone } else { 0.0 },
            park: if sur.park.active { sur.park.dist } else { [f32::NAN; 12] },
            lanes: {
                let line = |f: Option<sensors::LaneFit>| f.map_or([0.0; 5], |f| [f.c[0], f.c[1], f.c[2], f.c[3], f.range]);
                let p = self.lane_out.path;
                [line(p.and_then(|p| p.left)), line(p.and_then(|p| p.right))]
            },
            heading: heading_arc(self.vehicle.ego.speed, self.vehicle.ego.yaw_rate, self.vehicle.ego.reverse),
            plan: match self.lane_out.path {
                Some(p) if f & hud_flags::LANE_CENTERING != 0 => [p.c[0], p.c[1], p.c[2], p.c[3], p.range],
                _ => [0.0; 5],
            },
            _reserved: 0.0,
        };
        (cmd, event)
    }

    /// The ego car as the surround features see it (for views).
    pub fn surround_ego(&self) -> ::features::Ego {
        self.ego()
    }

    /// The ego car as the surround features see it: from the bus, and its box.
    fn ego(&self) -> ::features::Ego {
        let e = &self.vehicle.ego;
        let (lo, hi) = self.info.map_or(([-2.35, -0.95, 0.0], [2.35, 0.95, 1.5]), |i| (i.box_min, i.box_max));
        ::features::Ego {
            speed: e.speed,
            reverse: e.reverse,
            turn_left: e.turn == vbus::Turn::Left,
            turn_right: e.turn == vbus::Turn::Right,
            front: hi[0],
            rear: lo[0],
            left: hi[1],
            right: lo[1],
        }
    }

    fn run_surround(&mut self, dt: f32, paused: bool, park_cycle: Option<&[f32; 12]>) -> Surround {
        if paused || dt <= 0.0 || self.info.is_none() {
            return Surround { park: ::features::ParkOut::default(), ..self.surround_out };
        }
        let ego = self.ego();
        let brake = if self.vehicle.ego.brake_pressed { 1.0 } else { 0.0 };
        let tracks = self.surround.tracks();
        let on = self.toggles;
        let bsd = if on.bsd && self.avail.bsd.is_ok() { self.bsd.step(dt, &ego, tracks) } else { Default::default() };
        let rcta = if on.rcta && self.avail.rcta.is_ok() { self.rcta.step(dt, &ego, tracks, brake) } else { Default::default() };
        let park = if on.park && self.avail.park.is_ok() && self.fans_on.is_some() { self.park.step(dt, &ego, park_cycle, brake) } else { Default::default() };
        Surround { bsd, rcta, park }
    }

    fn surround_flags(&self) -> u32 {
        use surround_flags as sf;
        let s = &self.surround_out;
        let mut f = 0;
        let mut set = |on: bool, bit: u32| {
            if on {
                f |= bit;
            }
        };
        set(s.bsd.active, sf::BSD_ACTIVE);
        set(s.bsd.left, sf::BSD_LEFT);
        set(s.bsd.right, sf::BSD_RIGHT);
        set(s.bsd.warn_left, sf::BSD_WARN_LEFT);
        set(s.bsd.warn_right, sf::BSD_WARN_RIGHT);
        set(s.rcta.active, sf::RCTA_ACTIVE);
        set(s.rcta.left, sf::RCTA_LEFT);
        set(s.rcta.right, sf::RCTA_RIGHT);
        set(s.rcta.decel > 0.0 || s.rcta.hold, sf::RCTA_BRAKE);
        set(s.park.active, sf::PARK_ACTIVE);
        set(s.park.decel > 0.0 || s.park.hold, sf::PARK_BRAKE);
        set(self.avail.bsd.is_err(), sf::BSD_OFF);
        set(self.avail.rcta.is_err(), sf::RCTA_OFF);
        set(self.avail.park.is_err(), sf::PARK_OFF);
        f
    }

    /// The buses, then tracker → leads. Returns the leads and the ego speed
    /// and yaw rate as read from chassis CAN. Without adapter info (no ego
    /// geometry, no sensor layout yet) there is no radar.
    /// Also returns the park ECU's cycle, if one arrived.
    fn perceive(
        &mut self,
        s: &VehicleSample,
        actors: &ActorList,
        ranges: Option<&RangeReport>,
        lanes: Option<&LaneMap>,
        dt: f32,
        paused: bool,
    ) -> (Option<Lead>, Option<Lead>, f32, f32, Option<[f32; 12]>) {
        let have_bus = self.vehicle.ego.frames > 0;
        let held = |v: &vbus::Vehicle| if have_bus { (v.ego.speed, v.ego.yaw_rate) } else { (s.speed, s.yaw_rate) };
        self.scan_arrived = false;
        self.cam_lines = None;
        if paused || dt <= 0.0 {
            let (v, w) = held(&self.vehicle);
            return (None, None, v, w, None);
        }
        let frame = s.header.frame;
        let mounts = self.layout.as_slice();
        let have_info = self.info.is_some();
        // The radar ECUs' scans (their sensor models see the actors).
        let scan = match (have_info, mounts.get(layout::FRONT_RADAR)) {
            (true, Some(mount)) => self.radar.step(dt, mount, actors, frame),
            _ => None,
        };
        let mut corner: [Option<RadarScan>; 4] = Default::default();
        for (k, c) in corner.iter_mut().enumerate() {
            if let (true, Some(m)) = (have_info, mounts.get(layout::CORNERS[k])) {
                *c = self.corners[k].step(dt, m, actors, frame);
            }
        }
        // The park ECU, while the parking sensors run.
        let park = if have_info && self.fans_on.is_some() { self.ultrasonics.step(dt, &self.fans, ranges, frame) } else { None };
        // The camera ECU (its lanes only where the host gives markings).
        let map = lanes.filter(|_| self.info.is_some_and(|i| i.capabilities & caps::MAP != 0));
        let cam = match (have_info, mounts.get(layout::FRONT_CAMERA)) {
            (true, Some(m)) => self.camera.step(dt, m, actors, map, frame),
            _ => None,
        };
        let t = s.header.game_time_us as f64 / 1e6;
        let out = vbus::SensorOut { front: scan.as_ref(), corners: [corner[0].as_ref(), corner[1].as_ref(), corner[2].as_ref(), corner[3].as_ref()], park: park.as_ref(), camera: cam.as_ref() };
        let arrived = self.vehicle.step_all(t, frame, s, &out);
        self.scan_arrived = arrived.front.is_some();
        let (v, w) = (self.vehicle.ego.speed, self.vehicle.ego.yaw_rate);
        // Signed: the frame moves backwards when reversing.
        let vx = self.vehicle.ego.vx();
        let Some(info) = self.info else { return (None, None, v, w, None) };
        self.tracker.predict(dt, vx, w);
        self.since_scan += dt;
        if let Some(sc) = arrived.front {
            self.tracker.update(&sc, self.since_scan);
            self.since_scan = 0.0;
        }
        self.surround.predict(dt, vx, w);
        // Pedestrians from the camera.
        self.peds.predict(dt, vx, w);
        self.since_cam += dt;
        if let Some(c) = &arrived.camera {
            self.cam_lines = Some(c.lanes);
            // Radar–camera fusion: the vehicles the camera sees confirm the
            // radar's tracks there (ACC then stops for a stopped car).
            for o in c.objects.iter().filter(|o| matches!(o.class, camera::class::CAR | camera::class::TRUCK)) {
                self.tracker.confirm_vehicle(o.x, o.y);
            }
            let objects = c
                .objects
                .iter()
                .filter(|o| o.class == camera::class::PEDESTRIAN)
                .map(|o| RadarObject { id: o.id, x: o.x, y: o.y, vx: o.vx, vy: o.vy, width: o.width, half_length: 0.25, stationary: false, clutter: false })
                .collect();
            self.peds.update(&RadarScan { scan: c.frame, objects }, self.since_cam);
            self.since_cam = 0.0;
        }
        for (k, sc) in arrived.corners.iter().enumerate() {
            if let Some(sc) = sc {
                self.corner_last[k] = Some(frame);
                self.surround.update(k as u8, sc);
            }
        }
        let front = info.box_max[0];
        let tracks = self.tracker.tracks();
        // In the lane the camera sees, when it sees one (radar–camera
        // fusion, Phase 3); else along the path the yaw rate predicts.
        let (follow, collision) = match self.lane_out.path {
            Some(p) => {
                let half = (0.5 * p.width).clamp(1.5, 3.0);
                (select_lead_in_lane(tracks, v, &p.c, front, half, LeadKind::Follow), select_lead_in_lane(tracks, v, &p.c, front, half, LeadKind::Collision))
            }
            None => (select_lead(tracks, v, w, front, HALF_LANE, LeadKind::Follow), select_lead(tracks, v, w, front, HALF_LANE, LeadKind::Collision)),
        };
        (
            follow,
            collision,
            v,
            w,
            arrived.park.map(|p| p.dist),
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


/// How far ahead the heading arc is drawn (s of travel).
pub const HEADING_HORIZON: f32 = 3.0;

/// The arc the car traces at constant speed and yaw rate (the HUD's
/// `heading`): its curvature along the direction of travel and how far
/// along it to draw, at most half a circle. Reversing, the length is
/// negative: the arc runs behind the car.
pub fn heading_arc(speed: f32, yaw_rate: f32, reverse: bool) -> [f32; 2] {
    if !(speed > 1.0) || !yaw_rate.is_finite() {
        return [0.0; 2];
    }
    let k = yaw_rate / speed * if reverse { -1.0 } else { 1.0 };
    let mut len = (speed * HEADING_HORIZON).clamp(5.0, 120.0);
    if k.abs() > 1e-6 {
        len = len.min(std::f32::consts::PI / k.abs());
    }
    [k, if reverse { -len } else { len }]
}

#[cfg(test)]
mod tests {
    #[test]
    fn heading_arc_follows_the_turn() {
        // 20 m/s at 0.2 rad/s: a 100 m radius to the left, drawn 60 m along.
        let a = super::heading_arc(20.0, 0.2, false);
        assert!((a[0] - 0.01).abs() < 1e-6 && (a[1] - 60.0).abs() < 1e-3, "{a:?}");
        // A tight turn (4 m/s on a 2 m radius): half a circle, not 12 m.
        assert!((super::heading_arc(4.0, 2.0, false)[1] - 2.0 * std::f32::consts::PI).abs() < 1e-3);
        // Stopped: nothing to draw. Reversing: behind the car, and turning
        // the wheel left swings the tail the same way as the arc's sign says.
        assert_eq!(super::heading_arc(0.5, 0.1, false)[1], 0.0);
        let r = super::heading_arc(3.0, 0.3, true);
        assert!(r[1] < 0.0 && r[0] < 0.0, "{r:?}");
    }

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
        core.step(Inputs { sample: &s, actors: &a, ranges: None, lanes: None, stick: None })
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

    fn brake(demand: f32, a_est: f32) -> BrakeSample {
        BrakeSample { demand, smoothed: demand, speed: 15.0, a_est, driver_pedals: false }
    }

    #[test]
    fn the_brake_gain_is_learned_only_inside_grip() {
        // Brakes give 40 m/s² per unit: 0.1 of brake measures 4 m/s².
        let mut g = 8.0;
        for _ in 0..1000 {
            g = learn_brake_gain(g, brake(0.1, -4.0), None, 1.0 / 72.0);
        }
        assert!((g - 40.0).abs() < 1.0, "learned {g}");
        // Full braking measures the tyres' 9 m/s², not the brakes: no change.
        for _ in 0..200 {
            g = learn_brake_gain(g, brake(1.0, -9.0), None, 1.0 / 72.0);
        }
        assert!((g - 40.0).abs() < 1.0, "full braking taught {g}");
    }

    #[test]
    fn the_brake_gain_is_not_taught_by_the_driver_or_a_ramp() {
        // The driver brakes hard over a small demand: 0.05 "gives" 8 m/s².
        let mut g = 43.0;
        for _ in 0..500 {
            g = learn_brake_gain(g, BrakeSample { driver_pedals: true, ..brake(0.05, -8.0) }, None, 1.0 / 72.0);
        }
        assert_eq!(g, 43.0, "the driver's braking taught {g}");
        // The demand ramping up (the brakes lag it): not steady, no change.
        for _ in 0..500 {
            g = learn_brake_gain(g, BrakeSample { smoothed: 0.05, ..brake(0.1, -1.0) }, None, 1.0 / 72.0);
        }
        assert_eq!(g, 43.0, "a ramp taught {g}");
    }

    #[test]
    fn a_session_stays_near_the_saved_profile() {
        // Something the guards miss says 110: with a profile of 43, the
        // session goes no further than 43 × 1.3.
        let mut g = 43.0;
        for _ in 0..5000 {
            g = learn_brake_gain(g, brake(0.05, -5.5), Some(43.0), 1.0 / 72.0);
        }
        assert!((g - 43.0 * GAIN_PROFILE_BAND).abs() < 0.01, "{g}");
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
        let (c, e) = core.step(Inputs { sample: &s, actors: &ActorList::default(), ranges: None, lanes: None, stick: None });
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
            let (c, e) = core.step(Inputs { sample: &s, actors: &ActorList::default(), ranges: None, lanes: None, stick });
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
        core.step(Inputs { sample: &s, actors: &ActorList::default(), ranges: None, lanes: None, stick: None });
        assert!((core.hud().set_speed - set0 - 5.0 / 3.6).abs() < 1e-4);
    }
}
