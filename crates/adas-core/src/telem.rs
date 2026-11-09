//! What the core sends to its viewers (`telemetry`), and the panel's faults.
//! Shared by the live core and `replay`, so both produce the same stream.

use adapter_api::{ActorList, Command, VehicleSample};
use can::{Bus, Fault, FaultKind};
use record::FaultRec;
use telemetry::{BusSnap, CanFrame, Control, FaultInfo, Step, Track, track_flags};

use crate::{Core, State};

fn bus_snap(b: &Bus) -> BusSnap {
    BusSnap {
        now: b.now(),
        busy_total: b.busy_total,
        nominal_bps: b.cfg.nominal_bps,
        delivered: b.delivered_total,
        faulted: b.faulted,
        backlog: b.backlog() as u32,
        faults: b
            .faults()
            .iter()
            .map(|f| {
                let (kind, delay) = match f.kind {
                    FaultKind::Drop => (0, 0.0),
                    FaultKind::Corrupt => (1, 0.0),
                    FaultKind::Delay(d) => (2, d),
                };
                FaultInfo { id: f.id.unwrap_or(u32::MAX), kind, delay, p: f.p, from: f.from, until: f.until }
            })
            .collect(),
    }
}

/// The step the core just ran, for the viewers.
pub fn step(core: &Core, s: &VehicleSample, actors: &ActorList, cmd: &Command, latency_ns: u32, step_ns: u32) -> Step {
    let v = core.vehicle();
    let hud = core.hud();
    let (lead, threat) = core.lead_ids();
    Step {
        frame: s.header.frame,
        t: s.header.game_time_us as f64 / 1e6,
        latency_ns,
        step_ns,
        speed: s.speed,
        yaw_rate: s.yaw_rate,
        steer_angle: s.steer_angle,
        driver_throttle: s.driver_throttle,
        driver_brake: s.driver_brake,
        driver_steer: s.driver_steer,
        sample_flags: s.flags,
        bus_speed: v.ego.speed,
        bus_yaw_rate: v.ego.yaw_rate,
        bus_steer_angle: v.ego.steer_angle,
        accel: core.accel_estimate(),
        cmd_flags: cmd.flags,
        throttle: cmd.throttle,
        brake: cmd.brake,
        steer: cmd.steer_angle,
        state: match core.state() {
            State::Off => 0,
            State::Active => 1,
            State::Override => 2,
        },
        hud_flags: hud.flags,
        hud_message: hud.message,
        set_speed: hud.set_speed,
        time_gap: hud.time_gap,
        lead_gap: hud.lead_gap,
        lead_speed: hud.lead_speed,
        ttc: hud.ttc,
        brake_gain: core.brake_gain(),
        aeb_activations: core.aeb_activations(),
        buses: [bus_snap(&v.chassis), bus_snap(&v.adas)],
        checksum_errors: v.stats.errors,
        radar_latency_frames: v.stats.radar_latency_frames,
        tracks: core
            .tracker()
            .tracks()
            .iter()
            .map(|t| {
                let mut flags = 0;
                if t.confirmed {
                    flags |= track_flags::CONFIRMED;
                }
                if lead == Some(t.id) {
                    flags |= track_flags::LEAD;
                }
                if threat == Some(t.id) {
                    flags |= track_flags::THREAT;
                }
                Track { id: t.id, x: t.x(), y: t.y(), vx: t.vx(), vy: t.vy(), half_length: t.half_length, width: t.width, flags }
            })
            .collect(),
        actors: actors
            .as_slice()
            .iter()
            .map(|a| telemetry::Actor { id: a.id, kind: a.kind as u8, x: a.x, y: a.y, yaw: a.yaw, length: a.length, width: a.width, vx: a.vx })
            .collect(),
    }
}

/// The CAN frames of the step the core just ran.
pub fn frames(core: &Core) -> Vec<CanFrame> {
    core.vehicle()
        .last_frames
        .iter()
        .map(|(bus, f)| CanFrame { bus: if *bus == "adas" { telemetry::ADAS } else { telemetry::CHASSIS }, id: f.id, t: f.delivered, data: f.data.clone() })
        .collect()
}

/// A panel control as a recordable fault; `now` = the bus time it starts (s).
pub fn fault_from_control(c: &Control, now: f64) -> FaultRec {
    match *c {
        Control::Inject { bus, id, kind, delay, p, duration } => FaultRec {
            op: FaultRec::INJECT,
            bus: bus as u32,
            id: id.unwrap_or(u32::MAX),
            kind: kind as u32,
            delay,
            p: p.clamp(0.0, 1.0),
            from: now,
            until: if duration.is_finite() { now + duration.max(0.0) } else { f64::INFINITY },
        },
        Control::Clear { bus } => FaultRec { op: FaultRec::CLEAR, bus: bus.map_or(u32::MAX, |b| b as u32), ..Default::default() },
    }
}

/// Applies a recorded fault to the core's buses.
pub fn apply_fault(core: &mut Core, f: &FaultRec) -> Result<(), String> {
    let v = core.vehicle_mut();
    match f.op {
        FaultRec::INJECT => {
            let kind = match f.kind {
                0 => FaultKind::Drop,
                1 => FaultKind::Corrupt,
                2 => FaultKind::Delay(f.delay.max(0.0)),
                k => return Err(format!("unknown fault kind {k}")),
            };
            let id = (f.id != u32::MAX).then_some(f.id);
            let bus = u8::try_from(f.bus).map_err(|_| format!("no bus {}", f.bus))?;
            v.inject_fault(bus, Fault { id, kind, p: f.p, from: f.from, until: f.until })
        }
        FaultRec::CLEAR => {
            v.clear_faults(u8::try_from(f.bus).ok().filter(|&b| b != u8::MAX));
            Ok(())
        }
        op => Err(format!("unknown fault op {op}")),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Feature;

    #[test]
    fn a_panel_fault_applies_and_clears() {
        let mut core = Core::new(Feature::Acc(Default::default()), false);
        let c = Control::Inject { bus: 0, id: Some(0x25), kind: 1, delay: 0.0, p: 2.0, duration: 1.5 };
        let f = fault_from_control(&c, 10.0);
        assert_eq!((f.from, f.until, f.p), (10.0, 11.5, 1.0));
        apply_fault(&mut core, &f).unwrap();
        assert_eq!(core.vehicle().chassis.faults().len(), 1);
        assert_eq!(core.vehicle().chassis.faults()[0].kind, FaultKind::Corrupt);
        let forever = fault_from_control(&Control::Inject { bus: 1, id: None, kind: 0, delay: 0.0, p: 1.0, duration: f64::INFINITY }, 3.0);
        apply_fault(&mut core, &forever).unwrap();
        assert!(core.vehicle().adas.faults()[0].until.is_infinite());
        apply_fault(&mut core, &fault_from_control(&Control::Clear { bus: Some(0) }, 0.0)).unwrap();
        assert!(core.vehicle().chassis.faults().is_empty() && core.vehicle().adas.faults().len() == 1);
        apply_fault(&mut core, &fault_from_control(&Control::Clear { bus: None }, 0.0)).unwrap();
        assert!(core.vehicle().adas.faults().is_empty());
        assert!(apply_fault(&mut core, &FaultRec { op: 1, bus: 7, ..Default::default() }).is_err());
    }
}
