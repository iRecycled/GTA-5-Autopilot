//! The core's live telemetry (Phase 1 views).
//!
//! The core sends one [`Step`] per host frame, the CAN frames of that frame
//! ([`Packet::Frames`]), events ([`Packet::Event`]) and, every second, a
//! [`Packet::Hello`] naming the run. Viewers (the engineering panel, the Rerun
//! bridge) listen on their own UDP port on localhost. Sending never blocks
//! and never fails the core: a viewer that isn't running costs a `send_to`.
//!
//! The panel sends [`Control`] messages back (fault injection) to
//! [`CONTROL_PORT`]. The core applies them between steps and records them,
//! so a drive with injected faults still replays exactly.
//!
//! The same packets, each prefixed with its length (u32, little endian), make
//! a telemetry file (`replay --telemetry FILE`), which `adas-viz --file`
//! reads offline.
//!
//! Encoding: little endian, fixed field order, [`MAGIC`] first. Bus loads go
//! out as cumulative busy time, so a viewer computes load over its own window
//! and a lost packet loses nothing.

use std::io::{self, Read, Write};
use std::net::{SocketAddr, UdpSocket};

pub const MAGIC: [u8; 4] = *b"ADT1";
/// The engineering panel listens here.
pub const PANEL_PORT: u16 = 47300;
/// The core listens for [`Control`] messages here.
pub const CONTROL_PORT: u16 = 47301;
/// The Rerun views (`adas-viz`) listen here.
pub const RERUN_PORT: u16 = 47302;
/// Frames per [`Packet::Frames`] packet (keeps datagrams well under 64 KiB).
pub const FRAMES_PER_PACKET: usize = 400;

/// Bus indices in [`Step::buses`] and [`CanFrame::bus`].
pub const CHASSIS: u8 = 0;
pub const ADAS: u8 = 1;
pub const BUS_NAMES: [&str; 2] = ["chassis", "adas"];

#[derive(Clone, Debug, Default, PartialEq)]
pub struct BusSnap {
    /// Bus time (s).
    pub now: f64,
    /// Total time the bus has been busy since the core started (s).
    pub busy_total: f64,
    pub nominal_bps: u32,
    /// Frames delivered since start.
    pub delivered: u64,
    /// Frames the faults dropped, corrupted (CAN CRC errors) and delayed.
    pub faulted: [u64; 3],
    /// Frames waiting to be sent.
    pub backlog: u32,
    pub faults: Vec<FaultInfo>,
}

/// An active fault, as the panel shows it.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct FaultInfo {
    /// Message id; `u32::MAX` = every frame.
    pub id: u32,
    /// 0 drop, 1 corrupt, 2 delay.
    pub kind: u8,
    pub delay: f64,
    pub p: f64,
    pub from: f64,
    pub until: f64,
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Track {
    pub id: u32,
    pub x: f32,
    pub y: f32,
    pub vx: f32,
    pub vy: f32,
    pub half_length: f32,
    pub width: f32,
    /// Bit 0 confirmed, 1 follow lead, 2 collision threat.
    pub flags: u8,
}

pub mod track_flags {
    pub const CONFIRMED: u8 = 1;
    pub const LEAD: u8 = 2;
    pub const THREAT: u8 = 4;
}

/// Ground truth from the host, for the bird's-eye view only.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Actor {
    pub id: u32,
    pub kind: u8,
    pub x: f32,
    pub y: f32,
    pub yaw: f32,
    pub length: f32,
    pub width: f32,
    pub vx: f32,
}

/// One core step.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Step {
    pub frame: u64,
    /// Game time (s).
    pub t: f64,
    /// Sample written by the adapter â†’ command written by the core (ns);
    /// 0 in a replay.
    pub latency_ns: u32,
    /// The core's own step time (ns).
    pub step_ns: u32,
    // The host's values.
    pub speed: f32,
    pub yaw_rate: f32,
    pub steer_angle: f32,
    pub driver_throttle: f32,
    pub driver_brake: f32,
    pub driver_steer: f32,
    pub sample_flags: u32,
    // As the ADAS ECU reads them from chassis CAN.
    pub bus_speed: f32,
    pub bus_yaw_rate: f32,
    pub bus_steer_angle: f32,
    /// Acceleration estimate (m/sÂ²).
    pub accel: f32,
    // The command.
    pub cmd_flags: u32,
    pub throttle: f32,
    pub brake: f32,
    pub steer: f32,
    // State and HUD.
    /// 0 off, 1 active, 2 override.
    pub state: u8,
    pub hud_flags: u32,
    pub hud_message: u32,
    pub set_speed: f32,
    pub time_gap: f32,
    pub lead_gap: f32,
    pub lead_speed: f32,
    pub ttc: f32,
    pub brake_gain: f32,
    pub aeb_activations: u32,
    // Buses.
    pub buses: [BusSnap; 2],
    /// Frames the ADAS ECU rejected (checksum).
    pub checksum_errors: u64,
    pub radar_latency_frames: u32,
    pub tracks: Vec<Track>,
    pub actors: Vec<Actor>,
    /// The ego car's front edge from its reference point (m), for gaps.
    pub ego_front: f32,
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct CanFrame {
    pub bus: u8,
    pub id: u32,
    /// Delivery time (bus time, s).
    pub t: f64,
    pub data: Vec<u8>,
}

/// Fault injection from the panel.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Control {
    /// Inject a fault on `bus` for `duration` s (infinite = until cleared).
    /// `id`: `None` = every frame on the bus.
    Inject { bus: u8, id: Option<u32>, kind: u8, delay: f64, p: f64, duration: f64 },
    /// Clear the faults on `bus` (`None` = both buses).
    Clear { bus: Option<u8> },
}

#[derive(Clone, Debug, PartialEq)]
pub enum Packet {
    Hello { contract: u16, feature: String, host: String, vehicle: String, recording: String },
    Step(Box<Step>),
    Frames { frame: u64, frames: Vec<CanFrame> },
    Event { frame: u64, text: String },
    Control(Control),
}

// --- Encoding ---

struct W(Vec<u8>);

impl W {
    fn u8(&mut self, v: u8) {
        self.0.push(v);
    }
    fn u16(&mut self, v: u16) {
        self.0.extend_from_slice(&v.to_le_bytes());
    }
    fn u32(&mut self, v: u32) {
        self.0.extend_from_slice(&v.to_le_bytes());
    }
    fn u64(&mut self, v: u64) {
        self.0.extend_from_slice(&v.to_le_bytes());
    }
    fn f32(&mut self, v: f32) {
        self.0.extend_from_slice(&v.to_le_bytes());
    }
    fn f64(&mut self, v: f64) {
        self.0.extend_from_slice(&v.to_le_bytes());
    }
    fn str(&mut self, s: &str) {
        let b = &s.as_bytes()[..s.len().min(u16::MAX as usize)];
        self.u16(b.len() as u16);
        self.0.extend_from_slice(b);
    }
    fn bytes(&mut self, b: &[u8]) {
        let b = &b[..b.len().min(255)];
        self.u8(b.len() as u8);
        self.0.extend_from_slice(b);
    }
}

struct R<'a> {
    b: &'a [u8],
    i: usize,
}

impl<'a> R<'a> {
    fn take(&mut self, n: usize) -> Option<&'a [u8]> {
        let s = self.b.get(self.i..self.i + n)?;
        self.i += n;
        Some(s)
    }
    fn u8(&mut self) -> Option<u8> {
        Some(self.take(1)?[0])
    }
    fn u16(&mut self) -> Option<u16> {
        Some(u16::from_le_bytes(self.take(2)?.try_into().ok()?))
    }
    fn u32(&mut self) -> Option<u32> {
        Some(u32::from_le_bytes(self.take(4)?.try_into().ok()?))
    }
    fn u64(&mut self) -> Option<u64> {
        Some(u64::from_le_bytes(self.take(8)?.try_into().ok()?))
    }
    fn f32(&mut self) -> Option<f32> {
        Some(f32::from_le_bytes(self.take(4)?.try_into().ok()?))
    }
    fn f64(&mut self) -> Option<f64> {
        Some(f64::from_le_bytes(self.take(8)?.try_into().ok()?))
    }
    fn str(&mut self) -> Option<String> {
        let n = self.u16()? as usize;
        Some(String::from_utf8_lossy(self.take(n)?).into_owned())
    }
    fn bytes(&mut self) -> Option<Vec<u8>> {
        let n = self.u8()? as usize;
        Some(self.take(n)?.to_vec())
    }
}

const K_HELLO: u8 = 1;
const K_STEP: u8 = 2;
const K_FRAMES: u8 = 3;
const K_EVENT: u8 = 4;
const K_CONTROL: u8 = 5;

impl Packet {
    pub fn encode(&self) -> Vec<u8> {
        let mut w = W(Vec::with_capacity(256));
        w.0.extend_from_slice(&MAGIC);
        match self {
            Packet::Hello { contract, feature, host, vehicle, recording } => {
                w.u8(K_HELLO);
                w.u16(*contract);
                w.str(feature);
                w.str(host);
                w.str(vehicle);
                w.str(recording);
            }
            Packet::Step(s) => {
                w.u8(K_STEP);
                encode_step(&mut w, s);
            }
            Packet::Frames { frame, frames } => {
                w.u8(K_FRAMES);
                w.u64(*frame);
                w.u16(frames.len().min(u16::MAX as usize) as u16);
                for f in frames.iter().take(u16::MAX as usize) {
                    w.u8(f.bus);
                    w.u32(f.id);
                    w.f64(f.t);
                    w.bytes(&f.data);
                }
            }
            Packet::Event { frame, text } => {
                w.u8(K_EVENT);
                w.u64(*frame);
                w.str(text);
            }
            Packet::Control(c) => {
                w.u8(K_CONTROL);
                match *c {
                    Control::Inject { bus, id, kind, delay, p, duration } => {
                        w.u8(1);
                        w.u8(bus);
                        w.u32(id.unwrap_or(u32::MAX));
                        w.u8(kind);
                        w.f64(delay);
                        w.f64(p);
                        w.f64(duration);
                    }
                    Control::Clear { bus } => {
                        w.u8(2);
                        w.u8(bus.unwrap_or(u8::MAX));
                    }
                }
            }
        }
        w.0
    }

    pub fn decode(b: &[u8]) -> Option<Packet> {
        let mut r = R { b, i: 0 };
        if r.take(4)? != MAGIC {
            return None;
        }
        Some(match r.u8()? {
            K_HELLO => Packet::Hello { contract: r.u16()?, feature: r.str()?, host: r.str()?, vehicle: r.str()?, recording: r.str()? },
            K_STEP => Packet::Step(Box::new(decode_step(&mut r)?)),
            K_FRAMES => {
                let frame = r.u64()?;
                let n = r.u16()? as usize;
                let mut frames = Vec::with_capacity(n);
                for _ in 0..n {
                    frames.push(CanFrame { bus: r.u8()?, id: r.u32()?, t: r.f64()?, data: r.bytes()? });
                }
                Packet::Frames { frame, frames }
            }
            K_EVENT => Packet::Event { frame: r.u64()?, text: r.str()? },
            K_CONTROL => Packet::Control(match r.u8()? {
                1 => {
                    let bus = r.u8()?;
                    let id = r.u32()?;
                    Control::Inject { bus, id: (id != u32::MAX).then_some(id), kind: r.u8()?, delay: r.f64()?, p: r.f64()?, duration: r.f64()? }
                }
                2 => {
                    let bus = r.u8()?;
                    Control::Clear { bus: (bus != u8::MAX).then_some(bus) }
                }
                _ => return None,
            }),
            _ => return None,
        })
    }
}

fn encode_step(w: &mut W, s: &Step) {
    w.u64(s.frame);
    w.f64(s.t);
    w.u32(s.latency_ns);
    w.u32(s.step_ns);
    for v in [s.speed, s.yaw_rate, s.steer_angle, s.driver_throttle, s.driver_brake, s.driver_steer] {
        w.f32(v);
    }
    w.u32(s.sample_flags);
    for v in [s.bus_speed, s.bus_yaw_rate, s.bus_steer_angle, s.accel] {
        w.f32(v);
    }
    w.u32(s.cmd_flags);
    for v in [s.throttle, s.brake, s.steer] {
        w.f32(v);
    }
    w.u8(s.state);
    w.u32(s.hud_flags);
    w.u32(s.hud_message);
    for v in [s.set_speed, s.time_gap, s.lead_gap, s.lead_speed, s.ttc, s.brake_gain] {
        w.f32(v);
    }
    w.u32(s.aeb_activations);
    for b in &s.buses {
        w.f64(b.now);
        w.f64(b.busy_total);
        w.u32(b.nominal_bps);
        w.u64(b.delivered);
        for v in b.faulted {
            w.u64(v);
        }
        w.u32(b.backlog);
        w.u8(b.faults.len().min(255) as u8);
        for f in b.faults.iter().take(255) {
            w.u32(f.id);
            w.u8(f.kind);
            w.f64(f.delay);
            w.f64(f.p);
            w.f64(f.from);
            w.f64(f.until);
        }
    }
    w.u64(s.checksum_errors);
    w.u32(s.radar_latency_frames);
    w.u16(s.tracks.len().min(u16::MAX as usize) as u16);
    for t in s.tracks.iter().take(u16::MAX as usize) {
        w.u32(t.id);
        for v in [t.x, t.y, t.vx, t.vy, t.half_length, t.width] {
            w.f32(v);
        }
        w.u8(t.flags);
    }
    w.u16(s.actors.len().min(u16::MAX as usize) as u16);
    for a in s.actors.iter().take(u16::MAX as usize) {
        w.u32(a.id);
        w.u8(a.kind);
        for v in [a.x, a.y, a.yaw, a.length, a.width, a.vx] {
            w.f32(v);
        }
    }
    w.f32(s.ego_front);
}

fn decode_step(r: &mut R) -> Option<Step> {
    let mut s = Step { frame: r.u64()?, t: r.f64()?, latency_ns: r.u32()?, step_ns: r.u32()?, ..Default::default() };
    for v in [&mut s.speed, &mut s.yaw_rate, &mut s.steer_angle, &mut s.driver_throttle, &mut s.driver_brake, &mut s.driver_steer] {
        *v = r.f32()?;
    }
    s.sample_flags = r.u32()?;
    for v in [&mut s.bus_speed, &mut s.bus_yaw_rate, &mut s.bus_steer_angle, &mut s.accel] {
        *v = r.f32()?;
    }
    s.cmd_flags = r.u32()?;
    for v in [&mut s.throttle, &mut s.brake, &mut s.steer] {
        *v = r.f32()?;
    }
    s.state = r.u8()?;
    s.hud_flags = r.u32()?;
    s.hud_message = r.u32()?;
    for v in [&mut s.set_speed, &mut s.time_gap, &mut s.lead_gap, &mut s.lead_speed, &mut s.ttc, &mut s.brake_gain] {
        *v = r.f32()?;
    }
    s.aeb_activations = r.u32()?;
    for b in &mut s.buses {
        b.now = r.f64()?;
        b.busy_total = r.f64()?;
        b.nominal_bps = r.u32()?;
        b.delivered = r.u64()?;
        for v in &mut b.faulted {
            *v = r.u64()?;
        }
        b.backlog = r.u32()?;
        let n = r.u8()? as usize;
        for _ in 0..n {
            b.faults.push(FaultInfo { id: r.u32()?, kind: r.u8()?, delay: r.f64()?, p: r.f64()?, from: r.f64()?, until: r.f64()? });
        }
    }
    s.checksum_errors = r.u64()?;
    s.radar_latency_frames = r.u32()?;
    let n = r.u16()? as usize;
    for _ in 0..n {
        let mut t = Track { id: r.u32()?, ..Default::default() };
        for v in [&mut t.x, &mut t.y, &mut t.vx, &mut t.vy, &mut t.half_length, &mut t.width] {
            *v = r.f32()?;
        }
        t.flags = r.u8()?;
        s.tracks.push(t);
    }
    let n = r.u16()? as usize;
    for _ in 0..n {
        let mut a = Actor { id: r.u32()?, kind: r.u8()?, ..Default::default() };
        for v in [&mut a.x, &mut a.y, &mut a.yaw, &mut a.length, &mut a.width, &mut a.vx] {
            *v = r.f32()?;
        }
        s.actors.push(a);
    }
    s.ego_front = r.f32()?;
    Some(s)
}

/// Splits a step's frames into packets of at most [`FRAMES_PER_PACKET`].
pub fn frame_packets(frame: u64, frames: Vec<CanFrame>) -> Vec<Packet> {
    if frames.is_empty() {
        return Vec::new();
    }
    frames.chunks(FRAMES_PER_PACKET).map(|c| Packet::Frames { frame, frames: c.to_vec() }).collect()
}

// --- Transport ---

/// Sends packets to every viewer port; never blocks, never fails the caller.
pub struct Sender {
    sock: Option<UdpSocket>,
    to: Vec<SocketAddr>,
    file: Option<io::BufWriter<std::fs::File>>,
    pub sent: u64,
    pub send_errors: u64,
}

impl Sender {
    /// UDP to `127.0.0.1:<port>` for each port.
    pub fn udp(ports: &[u16]) -> Self {
        let sock = UdpSocket::bind("127.0.0.1:0").ok();
        if let Some(s) = &sock {
            let _ = s.set_nonblocking(true);
        }
        let to = ports.iter().map(|p| SocketAddr::from(([127, 0, 0, 1], *p))).collect();
        Self { sock, to, file: None, sent: 0, send_errors: 0 }
    }

    /// A telemetry file: each packet prefixed with its length.
    pub fn file(path: &std::path::Path) -> io::Result<Self> {
        Ok(Self { sock: None, to: Vec::new(), file: Some(io::BufWriter::new(std::fs::File::create(path)?)), sent: 0, send_errors: 0 })
    }

    pub fn send(&mut self, p: &Packet) {
        let b = p.encode();
        if let Some(s) = &self.sock {
            for a in &self.to {
                // A viewer that isn't listening is normal: ignore errors.
                match s.send_to(&b, a) {
                    Ok(_) => self.sent += 1,
                    Err(_) => self.send_errors += 1,
                }
            }
        }
        if let Some(f) = self.file.as_mut() {
            let ok = f.write_all(&(b.len() as u32).to_le_bytes()).and_then(|_| f.write_all(&b));
            if ok.is_err() {
                self.send_errors += 1;
                self.file = None;
            } else {
                self.sent += 1;
            }
        }
    }

    pub fn flush(&mut self) {
        if let Some(f) = self.file.as_mut() {
            let _ = f.flush();
        }
    }
}

/// Receives packets on a localhost UDP port.
pub struct Receiver {
    sock: UdpSocket,
    buf: Vec<u8>,
}

impl Receiver {
    pub fn bind(port: u16) -> io::Result<Self> {
        let sock = UdpSocket::bind(("127.0.0.1", port))?;
        sock.set_nonblocking(true)?;
        Ok(Self { sock, buf: vec![0u8; 65536] })
    }

    /// The next packet, if one is waiting (never blocks). Undecodable
    /// datagrams are skipped.
    pub fn try_recv(&mut self) -> Option<Packet> {
        loop {
            match self.sock.recv_from(&mut self.buf) {
                Ok((n, _)) => {
                    if let Some(p) = Packet::decode(&self.buf[..n]) {
                        return Some(p);
                    }
                }
                // WouldBlock: nothing waiting. Windows also reports a
                // previous send's ICMP "port unreachable" here: skip it.
                Err(e) if e.kind() == io::ErrorKind::ConnectionReset => continue,
                Err(_) => return None,
            }
        }
    }

    /// Sends a packet to `127.0.0.1:<port>` from this socket.
    pub fn send_to(&self, p: &Packet, port: u16) -> io::Result<()> {
        self.sock.send_to(&p.encode(), ("127.0.0.1", port)).map(|_| ())
    }
}

/// Reads a telemetry file written by [`Sender::file`].
pub fn read_file(path: &std::path::Path) -> io::Result<Vec<Packet>> {
    let mut f = io::BufReader::new(std::fs::File::open(path)?);
    let mut out = Vec::new();
    let mut len = [0u8; 4];
    loop {
        match f.read_exact(&mut len) {
            Ok(()) => {}
            Err(e) if e.kind() == io::ErrorKind::UnexpectedEof => break,
            Err(e) => return Err(e),
        }
        let mut b = vec![0u8; u32::from_le_bytes(len) as usize];
        if f.read_exact(&mut b).is_err() {
            break; // cut short
        }
        if let Some(p) = Packet::decode(&b) {
            out.push(p);
        }
    }
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn step() -> Step {
        let mut s = Step { frame: 42, t: 12.5, latency_ns: 900, speed: 13.9, ttc: f32::INFINITY, lead_gap: f32::NAN, state: 1, ..Default::default() };
        s.buses[0] = BusSnap { now: 12.5, busy_total: 1.1, nominal_bps: 500_000, delivered: 4000, faulted: [1, 2, 3], backlog: 0, faults: vec![FaultInfo { id: 0x25, kind: 1, delay: 0.0, p: 0.2, from: 10.0, until: f64::INFINITY }] };
        s.tracks.push(Track { id: 7, x: 30.0, y: -0.4, vx: 10.0, vy: 0.0, half_length: 2.2, width: 1.9, flags: track_flags::CONFIRMED | track_flags::LEAD });
        s.actors.push(Actor { id: 9, kind: 1, x: 32.2, y: -0.4, yaw: 0.01, length: 4.5, width: 1.9, vx: 10.0 });
        s
    }

    #[test]
    fn packets_round_trip() {
        let s = step();
        let packets = vec![
            Packet::Hello { contract: 3, feature: "acc".into(), host: "GTA V".into(), vehicle: "Sultan".into(), recording: "r.mcap".into() },
            Packet::Step(Box::new(s.clone())),
            Packet::Frames { frame: 42, frames: vec![CanFrame { bus: ADAS, id: 0x301, t: 12.49, data: (0..16).collect() }] },
            Packet::Event { frame: 42, text: "engaged".into() },
            Packet::Control(Control::Inject { bus: CHASSIS, id: Some(0x25), kind: 0, delay: 0.0, p: 1.0, duration: 2.0 }),
            Packet::Control(Control::Inject { bus: ADAS, id: None, kind: 2, delay: 0.04, p: 0.5, duration: f64::INFINITY }),
            Packet::Control(Control::Clear { bus: None }),
        ];
        for p in &packets {
            let back = Packet::decode(&p.encode()).expect("decodes");
            match (p, &back) {
                // NaN != NaN: compare the step field by field via its bytes.
                (Packet::Step(a), Packet::Step(b)) => {
                    assert!(b.lead_gap.is_nan());
                    let (mut a, mut b) = ((**a).clone(), (**b).clone());
                    a.lead_gap = 0.0;
                    b.lead_gap = 0.0;
                    assert_eq!(a, b);
                }
                _ => assert_eq!(p, &back),
            }
        }
        assert!(Packet::decode(b"nope").is_none());
        assert!(Packet::decode(&packets[1].encode()[..20]).is_none(), "a cut packet is rejected");
    }

    #[test]
    fn frames_are_chunked() {
        let frames: Vec<CanFrame> = (0..1000).map(|i| CanFrame { bus: 0, id: i, t: 0.0, data: vec![0; 8] }).collect();
        let p = frame_packets(1, frames);
        assert_eq!(p.len(), 3);
        assert!(p.iter().all(|p| p.encode().len() < 65_000));
    }

    #[test]
    fn udp_and_file_transport() {
        let mut rx = Receiver::bind(0).ok().or_else(|| Receiver::bind(47399).ok()).unwrap();
        let port = rx.sock.local_addr().unwrap().port();
        let mut tx = Sender::udp(&[port]);
        tx.send(&Packet::Event { frame: 1, text: "hi".into() });
        let mut got = None;
        for _ in 0..200 {
            got = rx.try_recv();
            if got.is_some() {
                break;
            }
            std::thread::sleep(std::time::Duration::from_millis(5));
        }
        assert_eq!(got, Some(Packet::Event { frame: 1, text: "hi".into() }));

        let path = std::env::temp_dir().join(format!("adas-telemetry-{}.bin", std::process::id()));
        let mut f = Sender::file(&path).unwrap();
        f.send(&Packet::Event { frame: 1, text: "a".into() });
        f.send(&Packet::Step(Box::new(step())));
        f.flush();
        drop(f);
        let back = read_file(&path).unwrap();
        assert_eq!(back.len(), 2);
        let _ = std::fs::remove_file(path);
    }
}
