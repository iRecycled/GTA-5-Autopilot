//! Test commands over UDP on localhost (port 47310), so in-game tests run
//! from a script without key presses or window focus (`gta-test`).
//!
//! One command per datagram, plain text; each gets one reply datagram
//! (`ok ...` or `err ...`):
//!
//! | Command | Does |
//! | --- | --- |
//! | `status` | `ok ready in_vehicle=0/1 vehicle=NAME speed=M/S engaged=0/1 spot=NAME` |
//! | `car MODEL` | spawn MODEL and get in (as F8) |
//! | `tp SPOT` | teleport to the named test spot (or its index) |
//! | `lead DIST SPEED` | a lead car DIST m ahead at SPEED m/s (as F11) |
//! | `stopped DIST` | a stopped car DIST m ahead (as F4) |
//! | `lead_brake` | the lead brakes hard (as F5) |
//! | `clear` | remove the lead |
//! | `engage` / `cancel` | the engage request on / off |
//! | `button up\|down\|gap` | one ACC button press |
//! | `drive THR BRK STEER SECS` | the driver's inputs for SECS s (steer +left); fractional values count as an analogue device |
//! | `creep back\|forward SPEED SECS` | the driver holds SPEED m/s (parking speeds) for SECS s; any `drive` ends it |
//! | `signal left\|right\|off` | the turn signal |
//! | `overtake left\|right DIST SPEED` | a car DIST m behind in the next lane, driving straight at SPEED m/s |
//! | `cross left\|right DIST SPEED` | a car from 25 m to that side, crossing DIST m behind the rear bumper at SPEED m/s |
//! | `obstacle behind\|ahead GAP` | a parked van broadside, GAP m from the bumper |
//!
//! Test tooling only: nothing here is part of the adapter contract.

use std::net::{SocketAddr, UdpSocket};

pub const PORT: u16 = 47310;

#[derive(Clone, Debug, PartialEq)]
pub enum Cmd {
    Status,
    Car(String),
    Tp(String),
    Lead(f32, f32),
    Stopped(f32),
    LeadBrake,
    Clear,
    Engage,
    Cancel,
    Button(u32),
    Drive { throttle: f32, brake: f32, steer: f32, secs: f32 },
    /// backwards?, speed (m/s), seconds.
    Creep(bool, f32, f32),
    /// 1 left, −1 right, 0 off.
    Signal(i8),
    /// side (1 left, −1 right), distance behind (m), speed (m/s).
    Overtake(f32, f32, f32),
    Cross(f32, f32, f32),
    /// behind?, gap (m).
    Obstacle(bool, f32),
}

fn side(w: Option<&str>, line: &str) -> Result<f32, String> {
    match w {
        Some("left") => Ok(1.0),
        Some("right") => Ok(-1.0),
        _ => Err(format!("{line}: left|right")),
    }
}

pub fn parse(line: &str) -> Result<Cmd, String> {
    let w: Vec<&str> = line.split_whitespace().collect();
    let num = |i: usize| -> Result<f32, String> { w.get(i).ok_or(format!("{line}: missing argument {i}"))?.parse::<f32>().map_err(|_| format!("{line}: bad number")) };
    let rest = || w[1..].join(" ");
    Ok(match w.first().copied().unwrap_or("") {
        "status" => Cmd::Status,
        "car" if w.len() == 2 && w[1].chars().all(|c| c.is_ascii_alphanumeric() || c == '_') => Cmd::Car(w[1].to_string()),
        "tp" if w.len() >= 2 => Cmd::Tp(rest()),
        "lead" => Cmd::Lead(num(1)?, num(2)?),
        "stopped" => Cmd::Stopped(num(1)?),
        "lead_brake" => Cmd::LeadBrake,
        "clear" => Cmd::Clear,
        "engage" => Cmd::Engage,
        "cancel" => Cmd::Cancel,
        "button" => Cmd::Button(match w.get(1).copied() {
            Some("up") => adapter_api::buttons::SPEED_UP,
            Some("down") => adapter_api::buttons::SPEED_DOWN,
            Some("gap") => adapter_api::buttons::GAP,
            _ => return Err(format!("{line}: button up|down|gap")),
        }),
        "drive" => Cmd::Drive { throttle: num(1)?.clamp(0.0, 1.0), brake: num(2)?.clamp(0.0, 1.0), steer: num(3)?.clamp(-1.0, 1.0), secs: num(4)?.max(0.0) },
        "creep" => Cmd::Creep(
            match w.get(1).copied() {
                Some("back") => true,
                Some("forward") => false,
                _ => return Err(format!("{line}: creep back|forward SPEED SECS")),
            },
            num(2)?.clamp(0.0, 5.0),
            num(3)?.max(0.0),
        ),
        "signal" => Cmd::Signal(match w.get(1).copied() {
            Some("left") => 1,
            Some("right") => -1,
            Some("off") => 0,
            _ => return Err(format!("{line}: signal left|right|off")),
        }),
        "overtake" => Cmd::Overtake(side(w.get(1).copied(), line)?, num(2)?, num(3)?),
        "cross" => Cmd::Cross(side(w.get(1).copied(), line)?, num(2)?, num(3)?),
        "obstacle" => Cmd::Obstacle(
            match w.get(1).copied() {
                Some("behind") => true,
                Some("ahead") => false,
                _ => return Err(format!("{line}: obstacle behind|ahead GAP")),
            },
            num(2)?,
        ),
        _ => return Err(format!("unknown command: {line}")),
    })
}

pub struct Remote {
    sock: Option<UdpSocket>,
    buf: [u8; 512],
}

impl Remote {
    pub fn new() -> Self {
        let sock = UdpSocket::bind(("127.0.0.1", PORT)).ok();
        if let Some(s) = &sock {
            let _ = s.set_nonblocking(true);
        }
        Self { sock, buf: [0; 512] }
    }

    pub fn listening(&self) -> bool {
        self.sock.is_some()
    }

    /// Commands waiting this frame (never blocks).
    pub fn poll(&mut self) -> Vec<(Result<Cmd, String>, SocketAddr)> {
        let mut out = Vec::new();
        let Some(s) = &self.sock else { return out };
        for _ in 0..32 {
            match s.recv_from(&mut self.buf) {
                Ok((n, from)) => out.push((parse(String::from_utf8_lossy(&self.buf[..n]).trim()), from)),
                Err(e) if e.kind() == std::io::ErrorKind::ConnectionReset => continue,
                Err(_) => break,
            }
        }
        out
    }

    pub fn reply(&self, to: SocketAddr, text: &str) {
        if let Some(s) = &self.sock {
            let _ = s.send_to(text.as_bytes(), to);
        }
    }
}
