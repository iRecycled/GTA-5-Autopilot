//! The virtual vehicle's buses (D9, D17, D23).
//!
//! Chassis ECUs publish what the car measures on chassis CAN with the Toyota
//! TSS2 messages, at those cars' cycle times. The front radar ECU publishes its
//! object list on ADAS CAN FD. The ADAS ECU reads both buses back: the
//! features see the vehicle the way a real ADAS ECU does, quantised to the
//! signals' resolution, at their rates, with the buses' delays.
//!
//! The messages are generated from the DBC files at build time (`msgs`) and
//! the topology is compiled in, so a replay rebuilds the same bus traffic.

use crate::actuators::{AccelRequest, AdasRequest, FULL_DECEL_SENT, SteerRequest};
use crate::msgs::{adas, chassis};
use adapter_api::{VehicleSample, sample_flags};
use can::{Bus, CanMessage, Delivery, Frame, Topology, toyota};
use sensors::{CamObject, CameraFrame, LaneFit, RadarObject, RadarScan, ULTRASONIC_COUNT, UltrasonicScan};

/// The camera's lane line message ids: left, right, left2, right2.
pub const CAM_LANE_IDS: [u32; 4] = [adas::CamLaneLeft::ID, adas::CamLaneRight::ID, adas::CamLaneLeft2::ID, adas::CamLaneRight2::ID];

/// The four corner radars' (status, object) message ids, in layout order:
/// front left, front right, rear left, rear right.
pub const CORNER_IDS: [(u32, u32); 4] = [
    (adas::CradarFlStatus::ID, adas::CradarFlObject::ID),
    (adas::CradarFrStatus::ID, adas::CradarFrObject::ID),
    (adas::CradarRlStatus::ID, adas::CradarRlObject::ID),
    (adas::CradarRrStatus::ID, adas::CradarRrObject::ID),
];

/// Park sensor distances on the bus: no echo, and no data (a sensor fault).
const PARK_NO_ECHO: f64 = 10.23;
const PARK_NO_DATA: f64 = 10.22;

/// One bumper's park sensor frame: its cycle counter and six distances (m).
type ParkHalf = (u8, [f32; 6]);

/// The turn signal as the ADAS ECU reads it from chassis CAN.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Turn {
    #[default]
    None,
    Left,
    Right,
}

/// What the sensor ECUs have to send this step.
#[derive(Clone, Copy, Debug, Default)]
pub struct SensorOut<'a> {
    pub front: Option<&'a RadarScan>,
    /// Corner radars, in [`CORNER_IDS`] order.
    pub corners: [Option<&'a RadarScan>; 4],
    pub park: Option<&'a UltrasonicScan>,
    pub camera: Option<&'a CameraFrame>,
}

/// What reached the ADAS ECU this step.
#[derive(Clone, Debug, Default)]
pub struct Arrived {
    pub front: Option<RadarScan>,
    pub corners: [Option<RadarScan>; 4],
    pub park: Option<UltrasonicScan>,
    pub camera: Option<CameraFrame>,
}

const TOPOLOGY: &str = include_str!("../../../config/topology.toml");

/// Steering-wheel angle ÷ road-wheel angle (TSS2 cars are about 14.8).
pub const STEERING_RATIO: f32 = 14.8;

/// What the ADAS ECU knows about its own car, from chassis CAN.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct EgoBus {
    /// m/s, from WHEEL_SPEEDS (mean of four).
    pub speed: f32,
    /// rad/s, positive left, from KINEMATICS.
    pub yaw_rate: f32,
    /// Road-wheel angle (rad), from STEER_ANGLE_SENSOR.
    pub steer_angle: f32,
    pub brake_pressed: bool,
    pub gas_pressed: bool,
    /// Reverse selected, from GEAR_PACKET.
    pub reverse: bool,
    /// The turn signal, from BLINKERS_STATE.
    pub turn: Turn,
    /// Frames received so far (0 = nothing yet).
    pub frames: u64,
}

impl EgoBus {
    /// Speed along the car's x axis (m/s): negative in reverse.
    pub fn vx(&self) -> f32 {
        if self.reverse { -self.speed } else { self.speed }
    }
}

/// Per-bus statistics.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct BusStats {
    pub chassis_load: f64,
    pub adas_load: f64,
    pub chassis_frames: u64,
    pub adas_frames: u64,
    /// Frames lost to bad checksums or counters.
    pub errors: u64,
    /// Frames lost to CAN CRC errors on the wire (both buses).
    pub crc_errors: u64,
    /// Radar path: frames from a scan's creation to its arrival at the ADAS
    /// ECU (the last scan's value).
    pub radar_latency_frames: u32,
    /// The same for the corner radars (the worst of the last scans).
    pub corner_latency_frames: u32,
}

struct Periodic {
    id: u32,
    period: f64,
    next: f64,
    counter: u8,
}

struct Pending {
    scan: u32,
    frame: u64,
    expected: usize,
    objects: Vec<RadarObject>,
}

pub struct Vehicle {
    pub chassis: Bus,
    pub adas: Bus,
    periodic: Vec<Periodic>,
    adas_counter: u8,
    pub ego: EgoBus,
    pending: Option<Pending>,
    /// Per corner radar: its rolling counter, the scan being received, and
    /// the scans sent (scan counter, frame) for latency.
    corner_counter: [u8; 4],
    corner_pending: [Option<Pending>; 4],
    corner_sent: [Vec<(u32, u64)>; 4],
    park_counter: u8,
    /// The park ECU's frames received so far this cycle (front, rear).
    park_rx: (Option<ParkHalf>, Option<ParkHalf>),
    cam_counter: u8,
    /// The camera frame being received, the objects it announced, the lines so far.
    cam_rx: Option<(CameraFrame, usize, u8)>,
    /// Scans whose creation frame is known, by scan counter (for latency).
    sent_scans: Vec<(u32, u64)>,
    pub stats: BusStats,
    /// Every frame delivered in the last step: (bus, frame).
    pub last_frames: Vec<(&'static str, Frame)>,
    lta_counter: u8,
    /// The yaw-rate sensor's last (time, speed), for its longitudinal
    /// acceleration, and that acceleration (m/s²).
    imu_prev: Option<(f64, f32)>,
    accel_x: f32,
    /// What the actuator ECUs last received, and when (s).
    pub accel_rx: AccelRequest,
    pub steer_rx: SteerRequest,
    accel_rx_t: f64,
    pcs_rx_t: f64,
    steer_rx_t: f64,
}

/// An actuator ECU drops a request it hasn't heard again for this long (s):
/// a silent ADAS ECU releases the car.
pub const ACTUATOR_TIMEOUT: f64 = 0.1;

/// Encodes a message, sealing Toyota's checksum if it has a `CHECKSUM` signal.
fn sealed<M: CanMessage>(m: &M) -> Vec<u8> {
    let mut d = m.encode();
    if M::has_signal("CHECKSUM") {
        toyota::seal(M::ID, &mut d);
    }
    d
}

/// A received chassis frame carries a `CHECKSUM` signal.
fn has_checksum(id: u32) -> bool {
    chassis::signals(id).is_some_and(|s| s.contains(&"CHECKSUM"))
}

impl Vehicle {
    pub fn new(delivery: Delivery) -> Self {
        let topo = Topology::parse(TOPOLOGY).expect("config/topology.toml");
        let p = |id: u32, hz: f64| Periodic { id, period: 1.0 / hz, next: 0.0, counter: 0 };
        Self {
            chassis: Bus::new(topo.bus_config("chassis", delivery).expect("chassis bus")),
            adas: Bus::new(topo.bus_config("adas", delivery).expect("adas bus")),
            periodic: vec![
                p(chassis::Kinematics::ID, 80.0),
                p(chassis::SteerAngleSensor::ID, 80.0),
                p(chassis::WheelSpeeds::ID, 80.0),
                p(chassis::Speed::ID, 50.0),
                p(chassis::BrakeModule::ID, 40.0),
                p(chassis::PcmCruise::ID, 33.0),
                p(chassis::GearPacket::ID, 20.0),
                p(chassis::BlinkersState::ID, 10.0),
            ],
            adas_counter: 0,
            ego: EgoBus::default(),
            pending: None,
            corner_counter: [0; 4],
            corner_pending: Default::default(),
            corner_sent: Default::default(),
            park_counter: 0,
            park_rx: (None, None),
            cam_counter: 0,
            cam_rx: None,
            sent_scans: Vec::new(),
            stats: BusStats::default(),
            last_frames: Vec::new(),
            lta_counter: 0,
            imu_prev: None,
            accel_x: 0.0,
            accel_rx: AccelRequest::default(),
            steer_rx: SteerRequest::default(),
            accel_rx_t: f64::NEG_INFINITY,
            pcs_rx_t: f64::NEG_INFINITY,
            steer_rx_t: f64::NEG_INFINITY,
        }
    }

    /// The chassis ECUs: encode the messages that are due at time `t` from
    /// what the car measures (sample-and-hold between host frames).
    fn publish_chassis(&mut self, t: f64, s: &VehicleSample) {
        let kph = (s.speed * 3.6) as f64;
        // The yaw-rate sensor's accelerations: longitudinal from the change
        // in speed since the last host frame, lateral v·ω (ISO 8855, left +).
        if let Some((t0, v0)) = self.imu_prev
            && t > t0
        {
            self.accel_x = ((s.speed - v0) as f64 / (t - t0)) as f32;
        }
        self.imu_prev = Some((t, s.speed));
        let (ax, ay) = (self.accel_x as f64, (s.speed * s.yaw_rate) as f64);
        let mut out: Vec<(u32, Vec<u8>)> = Vec::new();
        for p in &mut self.periodic {
            // Start-up and long pauses: resync instead of sending the backlog.
            if p.next < t - 2.0 * p.period {
                p.next = t;
            }
            while p.next <= t {
                // Messages with a CHECKSUM signal carry Toyota's checksum.
                let data = match p.id {
                    chassis::WheelSpeeds::ID => sealed(&chassis::WheelSpeeds {
                        wheel_speed_fr: kph,
                        wheel_speed_fl: kph,
                        wheel_speed_rr: kph,
                        wheel_speed_rl: kph,
                        ..Default::default()
                    }),
                    chassis::Kinematics::ID => sealed(&chassis::Kinematics { yaw_rate: (s.yaw_rate as f64).to_degrees(), accel_x: ax, accel_y: ay }),
                    chassis::SteerAngleSensor::ID => {
                        let a = if s.steer_angle.is_finite() { (s.steer_angle * STEERING_RATIO) as f64 } else { 0.0 };
                        sealed(&chassis::SteerAngleSensor { steer_angle: a.to_degrees(), ..Default::default() })
                    }
                    chassis::Speed::ID => sealed(&chassis::Speed { speed: kph, ..Default::default() }),
                    chassis::BrakeModule::ID => sealed(&chassis::BrakeModule { brake_pressed: s.driver_brake > 0.1, ..Default::default() }),
                    chassis::PcmCruise::ID => sealed(&chassis::PcmCruise { gas_released: s.driver_throttle.partial_cmp(&0.05) != Some(std::cmp::Ordering::Greater), ..Default::default() }),
                    // GEAR: 0 "D", 16 "R" (TSS2's values).
                    chassis::GearPacket::ID => sealed(&chassis::GearPacket {
                        gear: if s.flags & sample_flags::REVERSE != 0 { 16.0 } else { 0.0 },
                        drive_engaged: s.flags & sample_flags::REVERSE == 0,
                        ..Default::default()
                    }),
                    // TURN_SIGNALS: 1 "left", 2 "right", 3 "none".
                    chassis::BlinkersState::ID => {
                        let (l, r) = (s.flags & sample_flags::TURN_LEFT != 0, s.flags & sample_flags::TURN_RIGHT != 0);
                        sealed(&chassis::BlinkersState { turn_signals: if l { 1.0 } else if r { 2.0 } else { 3.0 }, ..Default::default() })
                    }
                    _ => unreachable!(),
                };
                p.counter = p.counter.wrapping_add(1);
                out.push((p.id, data));
                p.next += p.period;
            }
        }
        for (id, d) in out {
            self.chassis.send(id, d);
        }
    }

    /// The front radar ECU: one scan as a status frame and one frame per object.
    fn publish_radar(&mut self, scan: &RadarScan, frame: u64) {
        let objects = &scan.objects[..scan.objects.len().min(32)];
        let mut next_counter = || {
            let c = self.adas_counter;
            self.adas_counter = c.wrapping_add(1);
            c as f64
        };
        let status = adas::RadarStatus {
            scan_counter: (scan.scan & 0xFFFF) as f64,
            num_objects: objects.len() as f64,
            counter: next_counter(),
            ..Default::default()
        };
        let mut frames = vec![(adas::RadarStatus::ID, sealed(&status))];
        for (slot, o) in objects.iter().enumerate() {
            let obj = adas::RadarObject {
                slot: slot as f64,
                track_id: (o.id & 0xFFFF) as f64,
                dist_x: o.x as f64,
                dist_y: o.y as f64,
                vel_x: o.vx as f64,
                vel_y: o.vy as f64,
                width: o.width as f64,
                half_length: o.half_length as f64,
                stationary: o.stationary,
                exist_prob: 0.99,
                counter: next_counter(),
                ..Default::default()
            };
            frames.push((adas::RadarObject::ID, sealed(&obj)));
        }
        for (id, d) in frames {
            self.adas.send(id, d);
        }
        self.sent_scans.push((scan.scan & 0xFFFF, frame));
        if self.sent_scans.len() > 16 {
            self.sent_scans.remove(0);
        }
    }

    /// A corner radar ECU: a status frame and one frame per object, on its own ids.
    fn publish_corner(&mut self, k: usize, scan: &RadarScan, frame: u64) {
        let (status_id, object_id) = CORNER_IDS[k];
        let objects = &scan.objects[..scan.objects.len().min(32)];
        let mut next_counter = || {
            let c = self.corner_counter[k];
            self.corner_counter[k] = c.wrapping_add(1);
            c as f64
        };
        let status = [("SCAN_COUNTER", (scan.scan & 0xFFFF) as f64), ("NUM_OBJECTS", objects.len() as f64), ("COUNTER", next_counter())];
        let mut frames = vec![(status_id, adas::encode_signals(status_id, &status).expect("corner status"))];
        for (slot, o) in objects.iter().enumerate() {
            let v = [
                ("SLOT", slot as f64),
                ("TRACK_ID", (o.id & 0xFFFF) as f64),
                ("DIST_X", o.x as f64),
                ("DIST_Y", o.y as f64),
                ("VEL_X", o.vx as f64),
                ("VEL_Y", o.vy as f64),
                ("WIDTH", o.width as f64),
                ("HALF_LENGTH", o.half_length as f64),
                ("STATIONARY", o.stationary as u8 as f64),
                ("EXIST_PROB", 0.95),
                ("COUNTER", next_counter()),
            ];
            frames.push((object_id, adas::encode_signals(object_id, &v).expect("corner object")));
        }
        for (id, mut d) in frames {
            toyota::seal(id, &mut d);
            self.adas.send(id, d);
        }
        let sent = &mut self.corner_sent[k];
        sent.push((scan.scan & 0xFFFF, frame));
        if sent.len() > 16 {
            sent.remove(0);
        }
    }

    /// The park ECU: one frame per bumper per cycle.
    fn publish_park(&mut self, scan: &UltrasonicScan) {
        for (id, half) in [(adas::ParkSensorsFront::ID, &scan.dist[..6]), (adas::ParkSensorsRear::ID, &scan.dist[6..])] {
            let enc = |d: f32| if d.is_nan() { PARK_NO_DATA } else if d.is_infinite() { PARK_NO_ECHO } else { (d as f64).min(PARK_NO_DATA - 0.01) };
            let names = ["DIST_1", "DIST_2", "DIST_3", "DIST_4", "DIST_5", "DIST_6"];
            let mut v: Vec<(&str, f64)> = names.iter().zip(half).map(|(n, d)| (*n, enc(*d))).collect();
            v.push(("CYCLE", (scan.cycle & 0xFF) as f64));
            v.push(("ACTIVE", 1.0));
            v.push(("COUNTER", self.park_counter as f64));
            self.park_counter = self.park_counter.wrapping_add(1);
            let mut d = adas::encode_signals(id, &v).expect("park sensors");
            toyota::seal(id, &mut d);
            self.adas.send(id, d);
        }
    }

    /// One host frame at time `t` (s): chassis ECUs publish, the front radar
    /// ECU publishes a scan if it has one, both buses run, and the ADAS ECU
    /// reads what arrived. Returns a complete front radar scan if one arrived.
    pub fn step(&mut self, t: f64, frame: u64, s: &VehicleSample, scan: Option<&RadarScan>) -> Option<RadarScan> {
        self.step_all(t, frame, s, &SensorOut { front: scan, ..Default::default() }).front
    }

    /// As [`Vehicle::step`], with every sensor ECU: the front and corner
    /// radars and the park ECU.
    pub fn step_all(&mut self, t: f64, frame: u64, s: &VehicleSample, out: &SensorOut) -> Arrived {
        self.publish_chassis(t, s);
        if let Some(sc) = out.front {
            self.publish_radar(sc, frame);
        }
        for (k, sc) in out.corners.iter().enumerate() {
            if let Some(sc) = sc {
                self.publish_corner(k, sc, frame);
            }
        }
        if let Some(p) = out.park {
            self.publish_park(p);
        }
        if let Some(c) = out.camera {
            self.publish_camera(c);
        }
        self.last_frames.clear();
        let mut arrived = Arrived::default();
        for f in self.chassis.step(t) {
            self.stats.chassis_frames += 1;
            self.read_chassis(&f);
            self.last_frames.push(("chassis", f));
        }
        for f in self.adas.step(t) {
            self.stats.adas_frames += 1;
            if let Some(k) = CORNER_IDS.iter().position(|&(st, ob)| f.id == st || f.id == ob) {
                if let Some(sc) = self.read_corner(k, &f, frame) {
                    arrived.corners[k] = Some(sc);
                }
            } else if (adas::CamStatus::ID..=adas::CamObject::ID).contains(&f.id) {
                if let Some(c) = self.read_camera(&f) {
                    arrived.camera = Some(c);
                }
            } else if f.id == adas::ParkSensorsFront::ID || f.id == adas::ParkSensorsRear::ID {
                if let Some(p) = self.read_park(&f) {
                    arrived.park = Some(p);
                }
            } else if let Some(sc) = self.read_adas(&f, frame) {
                arrived.front = Some(sc);
            }
            self.last_frames.push(("adas", f));
        }
        arrived
    }

    fn read_corner(&mut self, k: usize, f: &Frame, frame: u64) -> Option<RadarScan> {
        if !toyota::verify(f.id, &f.data) {
            self.stats.errors += 1;
            return None;
        }
        let sig = adas::decode_signals(f.id, &f.data)?;
        let get = |n: &str| sig.iter().find(|(s, _)| *s == n).map_or(0.0, |(_, v)| *v);
        if f.id == CORNER_IDS[k].0 {
            let expected = get("NUM_OBJECTS") as usize;
            self.corner_pending[k] = Some(Pending { scan: get("SCAN_COUNTER") as u32, frame, expected, objects: Vec::with_capacity(expected) });
        } else if let Some(p) = self.corner_pending[k].as_mut() {
            p.objects.push(RadarObject {
                id: get("TRACK_ID") as u32,
                x: get("DIST_X") as f32,
                y: get("DIST_Y") as f32,
                vx: get("VEL_X") as f32,
                vy: get("VEL_Y") as f32,
                width: get("WIDTH") as f32,
                half_length: get("HALF_LENGTH") as f32,
                stationary: get("STATIONARY") > 0.5,
                clutter: false,
            });
        }
        if self.corner_pending[k].as_ref().is_some_and(|p| p.objects.len() >= p.expected) {
            let p = self.corner_pending[k].take().expect("pending corner scan");
            if let Some(&(_, sent)) = self.corner_sent[k].iter().find(|(s, _)| *s == p.scan) {
                let lat = frame.saturating_sub(sent) as u32;
                self.stats.corner_latency_frames = if k == 0 { lat } else { self.stats.corner_latency_frames.max(lat) };
            }
            return Some(RadarScan { scan: p.scan, objects: p.objects });
        }
        None
    }

    /// The camera ECU: a status frame, its four lane lines, one frame per object.
    fn publish_camera(&mut self, c: &CameraFrame) {
        let objects = &c.objects[..c.objects.len().min(32)];
        let mut counter = || {
            let n = self.cam_counter;
            self.cam_counter = n.wrapping_add(1);
            n as f64
        };
        let limit = if c.speed_limit > 0.0 { (c.speed_limit * 3.6).round() as f64 } else { 0.0 };
        let status = [("FRAME_COUNTER", (c.frame & 0xFFFF) as f64), ("NUM_OBJECTS", objects.len() as f64), ("SPEED_LIMIT", limit.min(255.0)), ("COUNTER", counter())];
        let mut frames = vec![(adas::CamStatus::ID, adas::encode_signals(adas::CamStatus::ID, &status).expect("camera status"))];
        for (k, l) in c.lanes.iter().enumerate() {
            let v = [
                ("C0", l.c[0] as f64),
                ("C1", l.c[1] as f64),
                ("C2", l.c[2] as f64),
                ("C3", l.c[3] as f64),
                ("VIEW_RANGE", l.range.min(255.0) as f64),
                ("QUALITY", l.quality as f64),
                ("LINE_TYPE", l.kind as f64),
                ("VALID", l.valid as u8 as f64),
                ("COUNTER", counter()),
            ];
            frames.push((CAM_LANE_IDS[k], adas::encode_signals(CAM_LANE_IDS[k], &v).expect("camera lane")));
        }
        for (slot, o) in objects.iter().enumerate() {
            let v = [
                ("SLOT", slot as f64),
                ("TRACK_ID", (o.id & 0xFFFF) as f64),
                ("CLASS", o.class as f64),
                ("DIST_X", o.x as f64),
                ("DIST_Y", o.y as f64),
                ("VEL_X", o.vx as f64),
                ("VEL_Y", o.vy as f64),
                ("WIDTH", o.width as f64),
                ("CONFIDENCE", o.confidence as f64),
                ("COUNTER", counter()),
            ];
            frames.push((adas::CamObject::ID, adas::encode_signals(adas::CamObject::ID, &v).expect("camera object")));
        }
        for (id, mut d) in frames {
            toyota::seal(id, &mut d);
            self.adas.send(id, d);
        }
    }

    /// A camera frame is complete once its status, four lines and all its
    /// objects have arrived.
    fn read_camera(&mut self, f: &Frame) -> Option<CameraFrame> {
        if !toyota::verify(f.id, &f.data) {
            self.stats.errors += 1;
            return None;
        }
        let sig = adas::decode_signals(f.id, &f.data)?;
        let get = |n: &str| sig.iter().find(|(s, _)| *s == n).map_or(0.0, |(_, v)| *v);
        if f.id == adas::CamStatus::ID {
            let frame = CameraFrame { frame: get("FRAME_COUNTER") as u32, speed_limit: get("SPEED_LIMIT") as f32 / 3.6, ..Default::default() };
            self.cam_rx = Some((frame, get("NUM_OBJECTS") as usize, 0));
        } else if let Some((c, _, lanes)) = self.cam_rx.as_mut() {
            if let Some(k) = CAM_LANE_IDS.iter().position(|&id| id == f.id) {
                c.lanes[k] = LaneFit {
                    valid: get("VALID") > 0.5,
                    c: [get("C0") as f32, get("C1") as f32, get("C2") as f32, get("C3") as f32],
                    range: get("VIEW_RANGE") as f32,
                    kind: get("LINE_TYPE") as u8,
                    quality: get("QUALITY") as u8,
                };
                *lanes += 1;
            } else if f.id == adas::CamObject::ID {
                c.objects.push(CamObject {
                    id: get("TRACK_ID") as u32,
                    class: get("CLASS") as u8,
                    x: get("DIST_X") as f32,
                    y: get("DIST_Y") as f32,
                    vx: get("VEL_X") as f32,
                    vy: get("VEL_Y") as f32,
                    width: get("WIDTH") as f32,
                    confidence: get("CONFIDENCE") as f32,
                });
            }
        }
        if self.cam_rx.as_ref().is_some_and(|(c, n, lanes)| *lanes >= 4 && c.objects.len() >= *n) {
            return self.cam_rx.take().map(|(c, _, _)| c);
        }
        None
    }

    /// Both bumpers' frames of the same cycle make one scan.
    fn read_park(&mut self, f: &Frame) -> Option<UltrasonicScan> {
        if !toyota::verify(f.id, &f.data) {
            self.stats.errors += 1;
            return None;
        }
        let sig = adas::decode_signals(f.id, &f.data)?;
        let get = |n: &str| sig.iter().find(|(s, _)| *s == n).map_or(0.0, |(_, v)| *v);
        let mut d = [0f32; 6];
        for (i, x) in d.iter_mut().enumerate() {
            let v = get(["DIST_1", "DIST_2", "DIST_3", "DIST_4", "DIST_5", "DIST_6"][i]);
            *x = if v >= PARK_NO_ECHO - 0.005 {
                f32::INFINITY
            } else if v >= PARK_NO_DATA - 0.005 {
                f32::NAN
            } else {
                v as f32
            };
        }
        let cycle = get("CYCLE") as u8;
        if f.id == adas::ParkSensorsFront::ID {
            self.park_rx.0 = Some((cycle, d));
        } else {
            self.park_rx.1 = Some((cycle, d));
        }
        match self.park_rx {
            (Some((a, front)), Some((b, rear))) if a == b => {
                self.park_rx = (None, None);
                let mut dist = [f32::NAN; ULTRASONIC_COUNT];
                dist[..6].copy_from_slice(&front);
                dist[6..].copy_from_slice(&rear);
                Some(UltrasonicScan { cycle: cycle as u32, dist })
            }
            _ => None,
        }
    }

    /// The ADAS ECU's requests at time `t`: `ACC_CONTROL`, `PRE_COLLISION_2`
    /// and `STEERING_LTA` on chassis CAN, once per host frame. The bus runs
    /// again and the actuator ECUs read what arrived (in reactive delivery,
    /// this step's requests; in realistic delivery, after their transmission).
    /// Returns what the actuator ECUs act on now.
    pub fn actuate(&mut self, t: f64, r: &AdasRequest) -> (AccelRequest, SteerRequest) {
        let acc = chassis::AccControl {
            accel_cmd: r.accel as f64,
            acc_type: r.acc_active as u8 as f64,
            permit_braking: true,
            release_standstill: !r.hold,
            ..Default::default()
        };
        let decel = if r.aeb_decel.is_finite() { r.aeb_decel } else { FULL_DECEL_SENT };
        let braking = r.aeb_decel > 0.0;
        let pcs = chassis::PreCollision2 {
            // Toyota sends the deceleration as a negative acceleration.
            dss1gdrv: if braking { -decel as f64 } else { 0.0 },
            ibtrgr: braking,
            pbatrgr: r.aeb_hold as u8 as f64,
            ..Default::default()
        };
        let sw_deg = if r.steer_request && r.steer_angle.is_finite() { (r.steer_angle * STEERING_RATIO).to_degrees() as f64 } else { 0.0 };
        let lta = chassis::SteeringLta {
            steer_request: r.steer_request,
            steer_request_2: r.steer_request,
            lka_active: r.steer_request,
            setme_x1: true,
            setme_x3: 3.0,
            percentage: 100.0,
            steer_angle_cmd: sw_deg,
            counter: (self.lta_counter & 0x3F) as f64,
            ..Default::default()
        };
        self.lta_counter = self.lta_counter.wrapping_add(1);
        self.chassis.send(chassis::AccControl::ID, sealed(&acc));
        self.chassis.send(chassis::PreCollision2::ID, sealed(&pcs));
        self.chassis.send(chassis::SteeringLta::ID, sealed(&lta));
        for f in self.chassis.step(t) {
            self.stats.chassis_frames += 1;
            self.read_actuation(&f, t);
            self.last_frames.push(("chassis", f));
        }
        // Requests not heard again lapse, each message on its own: losing
        // ACC's requests must not cancel AEB's.
        if t - self.accel_rx_t > ACTUATOR_TIMEOUT {
            (self.accel_rx.acc_active, self.accel_rx.accel, self.accel_rx.hold) = (false, 0.0, false);
        }
        if t - self.pcs_rx_t > ACTUATOR_TIMEOUT {
            (self.accel_rx.aeb_decel, self.accel_rx.aeb_hold) = (0.0, false);
        }
        if t - self.steer_rx_t > ACTUATOR_TIMEOUT {
            self.steer_rx = SteerRequest::default();
        }
        (self.accel_rx, self.steer_rx)
    }

    /// The powertrain/brake ECU and the EPS read their requests.
    fn read_actuation(&mut self, f: &Frame, t: f64) {
        let (acc, pcs, lta) = (chassis::AccControl::ID, chassis::PreCollision2::ID, chassis::SteeringLta::ID);
        if f.id != acc && f.id != pcs && f.id != lta {
            // Chassis traffic that was still queued: the ADAS ECU reads it too.
            self.read_chassis(f);
            return;
        }
        if !toyota::verify(f.id, &f.data) {
            self.stats.errors += 1;
            return;
        }
        if f.id == acc {
            let m = chassis::AccControl::decode(&f.data);
            self.accel_rx.acc_active = m.acc_type as u8 == 1;
            self.accel_rx.accel = m.accel_cmd as f32;
            self.accel_rx.hold = !m.release_standstill;
            self.accel_rx_t = t;
        } else if f.id == pcs {
            let m = chassis::PreCollision2::decode(&f.data);
            self.accel_rx.aeb_decel = if m.ibtrgr { (-m.dss1gdrv).max(0.0) as f32 } else { 0.0 };
            self.accel_rx.aeb_hold = m.pbatrgr as u8 == 1;
            self.pcs_rx_t = t;
        } else {
            let m = chassis::SteeringLta::decode(&f.data);
            self.steer_rx = SteerRequest { request: m.steer_request, sw_deg: m.steer_angle_cmd as f32 };
            self.steer_rx_t = t;
        }
    }

    fn read_chassis(&mut self, f: &Frame) {
        if chassis::name(f.id).is_none() {
            return;
        }
        if has_checksum(f.id) && !toyota::verify(f.id, &f.data) {
            self.stats.errors += 1;
            return;
        }
        match f.id {
            chassis::WheelSpeeds::ID => {
                let m = chassis::WheelSpeeds::decode(&f.data);
                let mean = (m.wheel_speed_fr + m.wheel_speed_fl + m.wheel_speed_rr + m.wheel_speed_rl) / 4.0;
                self.ego.speed = (mean / 3.6).max(0.0) as f32;
            }
            chassis::Kinematics::ID => self.ego.yaw_rate = (chassis::Kinematics::decode(&f.data).yaw_rate as f32).to_radians(),
            chassis::SteerAngleSensor::ID => {
                self.ego.steer_angle = (chassis::SteerAngleSensor::decode(&f.data).steer_angle as f32).to_radians() / STEERING_RATIO
            }
            chassis::BrakeModule::ID => self.ego.brake_pressed = chassis::BrakeModule::decode(&f.data).brake_pressed,
            chassis::PcmCruise::ID => self.ego.gas_pressed = !chassis::PcmCruise::decode(&f.data).gas_released,
            chassis::GearPacket::ID => self.ego.reverse = chassis::GearPacket::decode(&f.data).gear as u8 == 16,
            chassis::BlinkersState::ID => {
                self.ego.turn = match chassis::BlinkersState::decode(&f.data).turn_signals as u8 {
                    1 => Turn::Left,
                    2 => Turn::Right,
                    _ => Turn::None,
                }
            }
            _ => {}
        }
        self.ego.frames += 1;
    }

    fn read_adas(&mut self, f: &Frame, frame: u64) -> Option<RadarScan> {
        adas::name(f.id)?;
        if !toyota::verify(f.id, &f.data) {
            self.stats.errors += 1;
            return None;
        }
        match f.id {
            adas::RadarStatus::ID => {
                let m = adas::RadarStatus::decode(&f.data);
                let expected = m.num_objects as usize;
                self.pending = Some(Pending { scan: m.scan_counter as u32, frame, expected, objects: Vec::with_capacity(expected) });
            }
            adas::RadarObject::ID => {
                if let Some(p) = self.pending.as_mut() {
                    let m = adas::RadarObject::decode(&f.data);
                    p.objects.push(RadarObject {
                        id: m.track_id as u32,
                        x: m.dist_x as f32,
                        y: m.dist_y as f32,
                        vx: m.vel_x as f32,
                        vy: m.vel_y as f32,
                        width: m.width as f32,
                        half_length: m.half_length as f32,
                        stationary: m.stationary,
                        clutter: false,
                    });
                }
            }
            _ => {}
        }
        // A scan is complete when all its objects have arrived.
        let done = self.pending.as_ref().is_some_and(|p| p.objects.len() >= p.expected);
        if done {
            let p = self.pending.take().expect("pending scan");
            if let Some(&(_, sent)) = self.sent_scans.iter().find(|(s, _)| *s == p.scan) {
                self.stats.radar_latency_frames = (frame.saturating_sub(sent)) as u32;
            }
            let _ = p.frame;
            return Some(RadarScan { scan: p.scan, objects: p.objects });
        }
        None
    }

    /// Injects a fault on `bus` ("chassis" or "adas"), on the message named
    /// `message` (`None` = every frame on the bus).
    pub fn inject(&mut self, bus: &str, message: Option<&str>, kind: can::FaultKind, p: f64, from: f64, until: f64) -> Result<(), String> {
        let (b, lookup): (&mut Bus, fn(&str) -> Option<u32>) = match bus {
            "chassis" => (&mut self.chassis, chassis::id),
            "adas" => (&mut self.adas, adas::id),
            _ => return Err(format!("no bus {bus:?} (chassis, adas)")),
        };
        let id = match message {
            Some(m) => Some(lookup(m).ok_or(format!("{bus} has no message {m}"))?),
            None => None,
        };
        b.inject(can::Fault { id, kind, p, from, until });
        Ok(())
    }

    /// Injects a fault on bus 0 (chassis) or 1 (adas), by message id.
    pub fn inject_fault(&mut self, bus: u8, f: can::Fault) -> Result<(), String> {
        match bus {
            0 => self.chassis.inject(f),
            1 => self.adas.inject(f),
            _ => return Err(format!("no bus {bus}")),
        }
        Ok(())
    }

    /// Clears the faults on bus 0 or 1, or on both (`None`).
    pub fn clear_faults(&mut self, bus: Option<u8>) {
        if bus.is_none_or(|b| b == 0) {
            self.chassis.clear_faults();
        }
        if bus.is_none_or(|b| b == 1) {
            self.adas.clear_faults();
        }
    }

    /// Bus loads since the last call (resets the windows).
    pub fn take_loads(&mut self) -> (f64, f64) {
        self.stats.crc_errors = self.chassis.faulted[1] + self.adas.faulted[1];
        self.stats.chassis_load = self.chassis.take_load();
        self.stats.adas_load = self.adas.take_load();
        (self.stats.chassis_load, self.stats.adas_load)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn chassis_values_survive_the_bus_quantised() {
        let mut v = Vehicle::new(Delivery::Reactive);
        let s = VehicleSample { speed: 23.456, yaw_rate: 0.1234, steer_angle: 0.05, driver_brake: 0.5, ..Default::default() };
        for i in 0..10 {
            v.step(i as f64 / 72.0, i, &s, None);
        }
        assert!((v.ego.speed - 23.456).abs() < 0.01 / 3.6 + 1e-4, "speed {}", v.ego.speed);
        assert!((v.ego.yaw_rate - 0.1234).abs() < 0.244f32.to_radians(), "yaw {}", v.ego.yaw_rate);
        assert!((v.ego.steer_angle - 0.05).abs() < 1.5f32.to_radians() / STEERING_RATIO, "steer {}", v.ego.steer_angle);
        assert!(v.ego.brake_pressed && !v.ego.gas_pressed);
        assert_eq!(v.stats.errors, 0);
    }

    #[test]
    fn kinematics_carries_real_accelerations() {
        let mut v = Vehicle::new(Delivery::Reactive);
        // Speeding up at 2 m/s² in a left turn (v·ω = 20 × 0.1 = 2 m/s²).
        let mut last = None;
        for i in 0..20 {
            let t = i as f64 / 72.0;
            let s = VehicleSample { speed: 20.0 + 2.0 * t as f32, yaw_rate: 0.1, ..Default::default() };
            v.step(t, i, &s, None);
            if let Some((_, f)) = v.last_frames.iter().rev().find(|(_, f)| f.id == chassis::Kinematics::ID) {
                last = Some(chassis::Kinematics::decode(&f.data));
            }
        }
        let k = last.expect("KINEMATICS sent");
        assert!((k.accel_x - 2.0).abs() < 0.1, "accel_x {}", k.accel_x);
        assert!((k.accel_y - 2.07).abs() < 0.1, "accel_y {}", k.accel_y);
    }

    #[test]
    fn cycle_times_set_the_frame_rates() {
        let mut v = Vehicle::new(Delivery::Reactive);
        let s = VehicleSample::default();
        // One second at 72 fps.
        for i in 0..72 {
            v.step(i as f64 / 72.0, i, &s, None);
        }
        // 3 × 80 + 50 + 40 + 33 + 20 + 10 ≈ 393 frames in the first second (±1 per message).
        assert!((385..=400).contains(&v.stats.chassis_frames), "{} chassis frames", v.stats.chassis_frames);
        let (chassis, _) = v.take_loads();
        // About 360 × 0.24 ms ≈ 8–9 % of a 500 kbit/s bus.
        assert!((0.06..0.12).contains(&chassis), "chassis load {chassis}");
    }

    #[test]
    fn corrupted_frames_are_counted_and_ignored() {
        let mut v = Vehicle::new(Delivery::Reactive);
        v.inject("chassis", Some("STEER_ANGLE_SENSOR"), can::FaultKind::Corrupt, 1.0, f64::NEG_INFINITY, f64::INFINITY).unwrap();
        let s = VehicleSample { speed: 10.0, steer_angle: 0.05, ..Default::default() };
        for i in 0..10 {
            v.step(i as f64 / 72.0, i, &s, None);
        }
        assert!(v.chassis.faulted[1] > 0, "CRC failures injected");
        assert_eq!(v.ego.steer_angle, 0.0, "a corrupted angle is never used");
        assert!((v.ego.speed - 10.0).abs() < 0.01, "other messages unaffected");
        assert!(v.inject("body", None, can::FaultKind::Drop, 1.0, 0.0, 1.0).is_err());
        assert!(v.inject("adas", Some("NOPE"), can::FaultKind::Drop, 1.0, 0.0, 1.0).is_err());
    }

    #[test]
    fn a_late_start_does_not_flood_the_bus() {
        let mut v = Vehicle::new(Delivery::Reactive);
        // The host's clock is already at 775 s when the core starts.
        v.step(775.0, 1, &VehicleSample::default(), None);
        // One frame per periodic message (eight), not 775 s of backlog.
        assert!(v.stats.chassis_frames <= 8, "{} frames in the first step", v.stats.chassis_frames);
    }

    #[test]
    fn a_camera_frame_crosses_the_adas_bus() {
        let mut v = Vehicle::new(Delivery::Reactive);
        let mut c = CameraFrame { frame: 41, speed_limit: 100.0 / 3.6, ..Default::default() };
        c.lanes[0] = LaneFit { valid: true, c: [1.83, 0.012, 0.00125, 2.0e-6], range: 78.0, kind: 1, quality: 3 };
        c.lanes[1] = LaneFit { valid: true, c: [-1.77, 0.012, 0.0013, 0.0], range: 80.0, kind: 3, quality: 3 };
        c.objects.push(CamObject { id: 9, class: 3, x: 22.5, y: -1.1, vx: 0.0, vy: 1.4, width: 0.5, confidence: 0.9 });
        let got = v.step_all(0.0, 0, &VehicleSample::default(), &SensorOut { camera: Some(&c), ..Default::default() }).camera.expect("a frame");
        assert_eq!((got.frame, got.objects.len()), (41, 1));
        let (l, r) = (got.lanes[0], got.lanes[1]);
        assert!(l.valid && r.valid && !got.lanes[2].valid);
        assert!((l.c[0] - 1.83).abs() < 0.002 && (l.c[2] - 0.00125).abs() < 1e-6 && (l.c[3] - 2.0e-6).abs() < 2e-8, "{l:?}");
        assert_eq!((l.range, r.kind), (78.0, 3));
        assert!((got.speed_limit * 3.6 - 100.0).abs() < 0.01);
        let o = got.objects[0];
        assert!(o.class == 3 && (o.x - 22.5).abs() < 0.05 && (o.vy - 1.4).abs() < 0.05);
    }

    #[test]
    fn gear_and_turn_signal_cross_chassis_can() {
        let mut v = Vehicle::new(Delivery::Reactive);
        let s = VehicleSample { speed: 1.0, flags: sample_flags::REVERSE | sample_flags::TURN_RIGHT, ..Default::default() };
        for i in 0..10 {
            v.step(i as f64 / 72.0, i, &s, None);
        }
        assert!(v.ego.reverse && v.ego.turn == Turn::Right && v.ego.vx() == -1.0);
        let s = VehicleSample { flags: sample_flags::TURN_LEFT, ..s };
        for i in 10..30 {
            v.step(i as f64 / 72.0, i, &s, None);
        }
        assert!(!v.ego.reverse && v.ego.turn == Turn::Left);
    }

    #[test]
    fn corner_scans_and_park_distances_cross_the_adas_bus() {
        let mut v = Vehicle::new(Delivery::Realistic);
        let obj = |id, x: f32, y: f32| RadarObject { id, x, y, vx: 22.0, vy: -0.5, width: 1.9, half_length: 2.25, stationary: false, clutter: false };
        let rl = RadarScan { scan: 3, objects: vec![obj(11, -6.5, 2.7), obj(12, -30.0, 3.1)] };
        let rr = RadarScan { scan: 9, objects: vec![obj(13, -12.0, -3.4)] };
        let mut dist = [f32::INFINITY; ULTRASONIC_COUNT];
        dist[2] = 0.87;
        dist[8] = f32::NAN;
        let park = UltrasonicScan { cycle: 300, dist };
        let s = VehicleSample::default();
        let out = SensorOut { corners: [None, None, Some(&rl), Some(&rr)], park: Some(&park), ..Default::default() };
        let mut got = v.step_all(0.0, 0, &s, &out);
        let more = v.step_all(1.0 / 72.0, 1, &s, &SensorOut::default());
        for k in 0..4 {
            got.corners[k] = got.corners[k].take().or(more.corners[k].clone());
        }
        let park_rx = got.park.or(more.park).expect("park cycle arrived");
        let back_rl = got.corners[2].as_ref().expect("rear left scan");
        assert_eq!((back_rl.scan, back_rl.objects.len()), (3, 2));
        assert!((back_rl.objects[0].x + 6.5).abs() < 0.011 && (back_rl.objects[1].y - 3.1).abs() < 0.011, "signed distances");
        assert_eq!(got.corners[3].as_ref().unwrap().objects[0].id, 13);
        assert!(got.corners[0].is_none() && got.corners[1].is_none());
        assert!((park_rx.dist[2] - 0.87).abs() < 0.006 && park_rx.dist[0].is_infinite() && park_rx.dist[8].is_nan());
        assert_eq!(v.stats.errors, 0);
    }

    #[test]
    fn a_radar_scan_crosses_the_adas_bus_intact() {
        let mut v = Vehicle::new(Delivery::Realistic);
        let scan = RadarScan {
            scan: 7,
            objects: (0..5)
                .map(|i| RadarObject { id: 100 + i, x: 20.0 + i as f32, y: -1.0, vx: 15.0, vy: 0.0, width: 1.9, half_length: 2.25, stationary: false, clutter: false })
                .collect(),
        };
        let s = VehicleSample::default();
        let mut got = v.step(0.0, 0, &s, Some(&scan));
        if got.is_none() {
            got = v.step(1.0 / 72.0, 1, &s, None);
        }
        let got = got.expect("scan arrived");
        assert_eq!(got.scan, 7);
        assert_eq!(got.objects.len(), 5);
        assert!((got.objects[3].x - 23.0).abs() < 0.011 && got.objects[3].id == 103);
        assert!(v.stats.radar_latency_frames <= 1, "latency {} frames", v.stats.radar_latency_frames);
    }
}
