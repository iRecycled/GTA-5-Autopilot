//! The virtual vehicle's buses (D9, D17, D23).
//!
//! Chassis ECUs publish what the car measures on chassis CAN with the Toyota
//! TSS2 messages, at those cars' cycle times. The front radar ECU publishes its
//! object list on ADAS CAN FD. The ADAS ECU reads both buses back: the
//! features see the vehicle the way a real ADAS ECU does, quantised to the
//! signals' resolution, at their rates, with the buses' delays.
//!
//! The DBC files and the topology are compiled in, so a replay rebuilds the
//! same bus traffic.

use adapter_api::VehicleSample;
use can::{Bus, Dbc, Delivery, Frame, Message, Topology, toyota};
use sensors::{RadarObject, RadarScan};

const CHASSIS_DBC: &str = include_str!("../../../dbc/toyota_nodsu_pt_generated.dbc");
const ADAS_DBC: &str = include_str!("../../../dbc/adas_canfd.dbc");
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
    /// Frames received so far (0 = nothing yet).
    pub frames: u64,
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
    /// Radar path: frames from a scan's creation to its arrival at the ADAS
    /// ECU (the last scan's value).
    pub radar_latency_frames: u32,
}

struct Periodic {
    msg: &'static str,
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
    chassis_dbc: Dbc,
    adas_dbc: Dbc,
    pub chassis: Bus,
    pub adas: Bus,
    periodic: Vec<Periodic>,
    adas_counter: u8,
    pub ego: EgoBus,
    pending: Option<Pending>,
    /// Scans whose creation frame is known, by scan counter (for latency).
    sent_scans: Vec<(u32, u64)>,
    pub stats: BusStats,
    /// Every frame delivered in the last step: (bus, frame).
    pub last_frames: Vec<(&'static str, Frame)>,
}

fn msg<'a>(dbc: &'a Dbc, name: &str) -> &'a Message {
    dbc.message(name).unwrap_or_else(|| panic!("DBC lacks {name}"))
}

impl Vehicle {
    pub fn new(delivery: Delivery) -> Self {
        let topo = Topology::parse(TOPOLOGY).expect("config/topology.toml");
        let chassis_dbc = Dbc::parse(CHASSIS_DBC).expect("chassis DBC");
        let adas_dbc = Dbc::parse(ADAS_DBC).expect("ADAS DBC");
        let p = |msg: &'static str, hz: f64| Periodic { msg, period: 1.0 / hz, next: 0.0, counter: 0 };
        Self {
            chassis: Bus::new(topo.bus_config("chassis", delivery).expect("chassis bus")),
            adas: Bus::new(topo.bus_config("adas", delivery).expect("adas bus")),
            chassis_dbc,
            adas_dbc,
            periodic: vec![
                p("KINEMATICS", 80.0),
                p("STEER_ANGLE_SENSOR", 80.0),
                p("WHEEL_SPEEDS", 80.0),
                p("SPEED", 50.0),
                p("BRAKE_MODULE", 40.0),
                p("PCM_CRUISE", 33.0),
            ],
            adas_counter: 0,
            ego: EgoBus::default(),
            pending: None,
            sent_scans: Vec::new(),
            stats: BusStats::default(),
            last_frames: Vec::new(),
        }
    }

    /// The chassis ECUs: encode the messages that are due at time `t` from
    /// what the car measures (sample-and-hold between host frames).
    fn publish_chassis(&mut self, t: f64, s: &VehicleSample) {
        let kph = (s.speed * 3.6) as f64;
        let mut out: Vec<(u32, Vec<u8>)> = Vec::new();
        for p in &mut self.periodic {
            while p.next <= t {
                let m = msg(&self.chassis_dbc, p.msg);
                let mut data = match p.msg {
                    "WHEEL_SPEEDS" => m
                        .encode(&[("WHEEL_SPEED_FR", kph), ("WHEEL_SPEED_FL", kph), ("WHEEL_SPEED_RR", kph), ("WHEEL_SPEED_RL", kph)])
                        .expect("WHEEL_SPEEDS"),
                    "KINEMATICS" => m.encode(&[("YAW_RATE", (s.yaw_rate as f64).to_degrees())]).expect("KINEMATICS"),
                    "STEER_ANGLE_SENSOR" => {
                        let a = if s.steer_angle.is_finite() { (s.steer_angle * STEERING_RATIO) as f64 } else { 0.0 };
                        m.encode(&[("STEER_ANGLE", a.to_degrees())]).expect("STEER_ANGLE_SENSOR")
                    }
                    "SPEED" => m.encode(&[("SPEED", kph)]).expect("SPEED"),
                    "BRAKE_MODULE" => m.encode(&[("BRAKE_PRESSED", if s.driver_brake > 0.1 { 1.0 } else { 0.0 })]).expect("BRAKE_MODULE"),
                    "PCM_CRUISE" => m.encode(&[("GAS_RELEASED", if s.driver_throttle > 0.05 { 0.0 } else { 1.0 })]).expect("PCM_CRUISE"),
                    _ => unreachable!(),
                };
                // Messages with a CHECKSUM signal carry Toyota's checksum.
                if m.signal("CHECKSUM").is_some() {
                    toyota::seal(m.id, &mut data);
                }
                p.counter = p.counter.wrapping_add(1);
                out.push((m.id, data));
                p.next += p.period;
            }
        }
        for (id, d) in out {
            self.chassis.send(id, d);
        }
    }

    /// The front radar ECU: one scan as a status frame and one frame per object.
    fn publish_radar(&mut self, scan: &RadarScan, frame: u64) {
        let st = msg(&self.adas_dbc, "RADAR_STATUS");
        let ob = msg(&self.adas_dbc, "RADAR_OBJECT");
        let objects = &scan.objects[..scan.objects.len().min(32)];
        let seal = |m: &Message, mut data: Vec<u8>, ctr: &mut u8| {
            m.signal("COUNTER").expect("COUNTER").encode(&mut data, *ctr as f64);
            *ctr = ctr.wrapping_add(1);
            toyota::seal(m.id, &mut data);
            data
        };
        let status = st.encode(&[("SCAN_COUNTER", (scan.scan & 0xFFFF) as f64), ("NUM_OBJECTS", objects.len() as f64)]).expect("RADAR_STATUS");
        let status = seal(st, status, &mut self.adas_counter);
        self.adas.send(st.id, status);
        for (slot, o) in objects.iter().enumerate() {
            let d = ob
                .encode(&[
                    ("SLOT", slot as f64),
                    ("TRACK_ID", (o.id & 0xFFFF) as f64),
                    ("DIST_X", o.x as f64),
                    ("DIST_Y", o.y as f64),
                    ("VEL_X", o.vx as f64),
                    ("VEL_Y", o.vy as f64),
                    ("WIDTH", o.width as f64),
                    ("HALF_LENGTH", o.half_length as f64),
                    ("STATIONARY", o.stationary as u8 as f64),
                    ("EXIST_PROB", 0.99),
                ])
                .expect("RADAR_OBJECT");
            let d = seal(ob, d, &mut self.adas_counter);
            self.adas.send(ob.id, d);
        }
        self.sent_scans.push((scan.scan & 0xFFFF, frame));
        if self.sent_scans.len() > 16 {
            self.sent_scans.remove(0);
        }
    }

    /// One host frame at time `t` (s): chassis ECUs publish, the radar ECU
    /// publishes a scan if it has one, both buses run, and the ADAS ECU reads
    /// what arrived. Returns a complete radar scan if one arrived.
    pub fn step(&mut self, t: f64, frame: u64, s: &VehicleSample, scan: Option<&RadarScan>) -> Option<RadarScan> {
        self.publish_chassis(t, s);
        if let Some(sc) = scan {
            self.publish_radar(sc, frame);
        }
        self.last_frames.clear();
        let mut arrived = None;
        for f in self.chassis.step(t) {
            self.stats.chassis_frames += 1;
            self.read_chassis(&f);
            self.last_frames.push(("chassis", f));
        }
        for f in self.adas.step(t) {
            self.stats.adas_frames += 1;
            if let Some(sc) = self.read_adas(&f, frame) {
                arrived = Some(sc);
            }
            self.last_frames.push(("adas", f));
        }
        arrived
    }

    fn read_chassis(&mut self, f: &Frame) {
        let Some(m) = self.chassis_dbc.message_by_id(f.id) else { return };
        if m.signal("CHECKSUM").is_some() && !toyota::verify(f.id, &f.data) {
            self.stats.errors += 1;
            return;
        }
        let g = |n: &str| m.decode(&f.data, n).unwrap_or(0.0);
        match m.name.as_str() {
            "WHEEL_SPEEDS" => {
                let mean = (g("WHEEL_SPEED_FR") + g("WHEEL_SPEED_FL") + g("WHEEL_SPEED_RR") + g("WHEEL_SPEED_RL")) / 4.0;
                self.ego.speed = (mean / 3.6).max(0.0) as f32;
            }
            "KINEMATICS" => self.ego.yaw_rate = (g("YAW_RATE") as f32).to_radians(),
            "STEER_ANGLE_SENSOR" => self.ego.steer_angle = (g("STEER_ANGLE") as f32).to_radians() / STEERING_RATIO,
            "BRAKE_MODULE" => self.ego.brake_pressed = g("BRAKE_PRESSED") > 0.5,
            "PCM_CRUISE" => self.ego.gas_pressed = g("GAS_RELEASED") < 0.5,
            _ => {}
        }
        self.ego.frames += 1;
    }

    fn read_adas(&mut self, f: &Frame, frame: u64) -> Option<RadarScan> {
        let m = self.adas_dbc.message_by_id(f.id)?;
        if !toyota::verify(f.id, &f.data) {
            self.stats.errors += 1;
            return None;
        }
        let g = |n: &str| m.decode(&f.data, n).unwrap_or(0.0);
        match m.name.as_str() {
            "RADAR_STATUS" => {
                let scan = g("SCAN_COUNTER") as u32;
                let expected = g("NUM_OBJECTS") as usize;
                self.pending = Some(Pending { scan, frame, expected, objects: Vec::with_capacity(expected) });
            }
            "RADAR_OBJECT" => {
                if let Some(p) = self.pending.as_mut() {
                    p.objects.push(RadarObject {
                        id: g("TRACK_ID") as u32,
                        x: g("DIST_X") as f32,
                        y: g("DIST_Y") as f32,
                        vx: g("VEL_X") as f32,
                        vy: g("VEL_Y") as f32,
                        width: g("WIDTH") as f32,
                        half_length: g("HALF_LENGTH") as f32,
                        stationary: g("STATIONARY") > 0.5,
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

    /// Bus loads since the last call (resets the windows).
    pub fn take_loads(&mut self) -> (f64, f64) {
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
    fn cycle_times_set_the_frame_rates() {
        let mut v = Vehicle::new(Delivery::Reactive);
        let s = VehicleSample::default();
        // One second at 72 fps.
        for i in 0..72 {
            v.step(i as f64 / 72.0, i, &s, None);
        }
        // 3 × 80 + 50 + 40 + 33 ≈ 363 frames in the first second (±1 per message).
        assert!((355..=370).contains(&v.stats.chassis_frames), "{} chassis frames", v.stats.chassis_frames);
        let (chassis, _) = v.take_loads();
        // About 360 × 0.24 ms ≈ 8–9 % of a 500 kbit/s bus.
        assert!((0.06..0.12).contains(&chassis), "chassis load {chassis}");
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
