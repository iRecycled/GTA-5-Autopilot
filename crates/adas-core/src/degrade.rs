//! The degradation matrix (D21): one table from each missing adapter
//! capability or sensor fault to the features it makes unavailable, and to
//! what the driver sees. Evaluated every step.

use adapter_api::{AdapterInfo, caps};

/// Why a feature is unavailable (or degraded). The HUD shows the first one.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Reason {
    /// No adapter info yet: no ego geometry, no sensor layout.
    NoAdapterInfo,
    /// The host can't apply pedal commands.
    NoPedals,
    /// The host gives no actor list: the radar has nothing to see.
    NoActors,
    /// The host can't hold the car at a standstill: no stop-and-go.
    NoHold,
    /// The host can't steer.
    NoSteer,
    /// No chassis CAN traffic for a while: no ego speed.
    ChassisSilent,
    /// No radar scan for a while.
    RadarSilent,
    /// A rear corner radar has sent nothing for a while.
    SideRadarSilent,
    /// The host gives no range fans: no parking sensors.
    NoRangeFans,
    /// A parking sensor reports no data, or the park ECU is silent.
    ParkSensorFault,
    /// The host doesn't say when reverse is selected.
    NoGear,
    /// ACC doesn't run in reverse.
    Reversing,
}

impl Reason {
    /// What the driver reads on the HUD.
    pub fn text(self) -> &'static str {
        match self {
            Reason::NoAdapterInfo => "starting up",
            Reason::NoPedals => "no pedal control on this host",
            Reason::NoActors => "no radar data on this host",
            Reason::NoHold => "no standstill hold on this host",
            Reason::NoSteer => "no steering control on this host",
            Reason::ChassisSilent => "vehicle data lost",
            Reason::RadarSilent => "radar blocked",
            Reason::SideRadarSilent => "side radar blocked",
            Reason::NoRangeFans => "no parking sensors on this host",
            Reason::ParkSensorFault => "parking sensor fault",
            Reason::NoGear => "no gear signal on this host",
            Reason::Reversing => "not available in reverse",
        }
    }

    /// The HUD message code (adapter-api `hud_message`).
    pub fn code(self) -> u32 {
        use adapter_api::hud_message as m;
        match self {
            Reason::NoAdapterInfo => m::STARTING_UP,
            Reason::NoPedals => m::NO_PEDALS,
            Reason::NoActors => m::NO_RADAR_DATA,
            Reason::NoHold => m::NO_STANDSTILL_HOLD,
            Reason::NoSteer => m::NO_STEERING,
            Reason::ChassisSilent => m::VEHICLE_DATA_LOST,
            Reason::RadarSilent => m::RADAR_BLOCKED,
            Reason::SideRadarSilent => m::SIDE_RADAR_BLOCKED,
            Reason::NoRangeFans => m::NO_PARK_SENSORS,
            Reason::ParkSensorFault => m::PARK_SENSOR_FAULT,
            Reason::NoGear => m::NO_GEAR,
            Reason::Reversing => m::REVERSING,
        }
    }
}

/// What may run this step.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Availability {
    pub acc: Result<(), Reason>,
    /// ACC may stop and hold the car (else it cancels below `ACC_MIN_SPEED`).
    pub stop_and_go: bool,
    pub aeb: Result<(), Reason>,
    /// Features that steer (the test driver now; LKA later).
    pub steer: Result<(), Reason>,
    /// Plain longitudinal control without sensors (the speed profile).
    pub drive: Result<(), Reason>,
    /// The radar has no line-of-sight checks: everything in its beam counts as
    /// visible (occlusion isn't modelled). A degradation, not a loss.
    pub los_degraded: bool,
    /// Blind spot detection.
    pub bsd: Result<(), Reason>,
    /// Rear cross-traffic alert.
    pub rcta: Result<(), Reason>,
    /// Park assist.
    pub park: Result<(), Reason>,
    /// The surround features may brake (the host has pedal control); else
    /// they only warn.
    pub surround_brake: bool,
}

/// Without stop-and-go, ACC works only above this speed (m/s), as on cars
/// without it.
pub const ACC_MIN_SPEED: f32 = 30.0 / 3.6;
/// Silence that counts as a lost bus or sensor (frames).
pub const CHASSIS_TIMEOUT_FRAMES: u64 = 18; // 0.25 s at 72 fps
pub const RADAR_TIMEOUT_FRAMES: u64 = 36; // 0.5 s, 10 scans at 20 Hz

/// Health of what the core receives.
#[derive(Clone, Copy, Debug, Default)]
pub struct Health {
    /// Frames since the last chassis frame arrived (`u64::MAX` = never).
    pub chassis_age: u64,
    /// Frames since the last radar scan arrived (`u64::MAX` = never).
    pub radar_age: u64,
    /// The same for the rear corner radars (left, right).
    pub rear_corner_age: [u64; 2],
    /// The parking sensors are running (the core asked for them), and their
    /// last cycle's age (frames) and whether a sensor gave no data.
    pub park_running: bool,
    pub park_age: u64,
    pub park_fault: bool,
}

/// The park ECU sends every 40 ms; this much silence is a fault (frames).
pub const PARK_TIMEOUT_FRAMES: u64 = 18; // 0.25 s at 72 fps

pub fn evaluate(info: Option<&AdapterInfo>, h: Health) -> Availability {
    let Some(info) = info else {
        let no = Err(Reason::NoAdapterInfo);
        return Availability {
            acc: no,
            stop_and_go: false,
            aeb: no,
            steer: no,
            drive: no,
            los_degraded: false,
            bsd: no,
            rcta: no,
            park: no,
            surround_brake: false,
        };
    };
    let has = |c: u32| info.capabilities & c != 0;
    let chassis_ok = h.chassis_age <= CHASSIS_TIMEOUT_FRAMES;
    let radar_ok = h.radar_age <= RADAR_TIMEOUT_FRAMES;
    let rear_ok = h.rear_corner_age.iter().all(|&a| a <= RADAR_TIMEOUT_FRAMES);
    let park_ok = !h.park_running || (h.park_age <= PARK_TIMEOUT_FRAMES && !h.park_fault);

    // The matrix: first unmet requirement wins.
    let need = |reqs: &[(bool, Reason)]| reqs.iter().find(|(ok, _)| !ok).map_or(Ok(()), |(_, r)| Err(*r));
    let longitudinal = [
        (has(caps::PEDALS), Reason::NoPedals),
        (has(caps::ACTORS), Reason::NoActors),
        (chassis_ok, Reason::ChassisSilent),
        (radar_ok, Reason::RadarSilent),
    ];
    Availability {
        acc: need(&longitudinal),
        stop_and_go: has(caps::HOLD),
        aeb: need(&longitudinal),
        steer: need(&[(has(caps::STEER), Reason::NoSteer), (chassis_ok, Reason::ChassisSilent)]),
        drive: need(&[(has(caps::PEDALS), Reason::NoPedals), (chassis_ok, Reason::ChassisSilent)]),
        los_degraded: !has(caps::LINE_OF_SIGHT),
        bsd: need(&[(has(caps::ACTORS), Reason::NoActors), (chassis_ok, Reason::ChassisSilent), (rear_ok, Reason::SideRadarSilent)]),
        rcta: need(&[
            (has(caps::ACTORS), Reason::NoActors),
            (has(caps::GEAR), Reason::NoGear),
            (chassis_ok, Reason::ChassisSilent),
            (rear_ok, Reason::SideRadarSilent),
        ]),
        park: need(&[(has(caps::RANGE_FANS), Reason::NoRangeFans), (chassis_ok, Reason::ChassisSilent), (park_ok, Reason::ParkSensorFault)]),
        surround_brake: has(caps::PEDALS),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn info(c: u32) -> AdapterInfo {
        AdapterInfo { capabilities: c, ..Default::default() }
    }
    const ALL: u32 = caps::STEER | caps::PEDALS | caps::HOLD | caps::ACTORS | caps::LINE_OF_SIGHT | caps::RANGE_FANS | caps::GEAR;
    const OK: Health = Health { chassis_age: 0, radar_age: 0, rear_corner_age: [0, 0], park_running: true, park_age: 0, park_fault: false };

    #[test]
    fn everything_available_with_full_capabilities() {
        let a = evaluate(Some(&info(ALL)), OK);
        assert_eq!((a.acc, a.aeb, a.steer), (Ok(()), Ok(()), Ok(())));
        assert!(a.stop_and_go && !a.los_degraded);
    }

    #[test]
    fn the_matrix() {
        assert_eq!(evaluate(None, OK).acc, Err(Reason::NoAdapterInfo));
        assert_eq!(evaluate(Some(&info(ALL & !caps::PEDALS)), OK).acc, Err(Reason::NoPedals));
        assert_eq!(evaluate(Some(&info(ALL & !caps::ACTORS)), OK).aeb, Err(Reason::NoActors));
        let no_hold = evaluate(Some(&info(ALL & !caps::HOLD)), OK);
        assert!(no_hold.acc.is_ok() && !no_hold.stop_and_go);
        assert_eq!(evaluate(Some(&info(ALL & !caps::STEER)), OK).steer, Err(Reason::NoSteer));
        assert!(evaluate(Some(&info(ALL & !caps::LINE_OF_SIGHT)), OK).los_degraded);
        let silent = evaluate(Some(&info(ALL)), Health { chassis_age: 100, ..OK });
        assert_eq!((silent.acc, silent.steer), (Err(Reason::ChassisSilent), Err(Reason::ChassisSilent)));
        let blind = evaluate(Some(&info(ALL)), Health { radar_age: 100, ..OK });
        assert_eq!(blind.aeb, Err(Reason::RadarSilent));
        assert!(blind.steer.is_ok(), "steering doesn't need the radar");
        assert!(blind.bsd.is_ok() && blind.rcta.is_ok(), "nor do the surround features");
    }

    #[test]
    fn the_surround_rows() {
        let a = evaluate(Some(&info(ALL)), OK);
        assert_eq!((a.bsd, a.rcta, a.park), (Ok(()), Ok(()), Ok(())));
        assert!(a.surround_brake);
        let side = evaluate(Some(&info(ALL)), Health { rear_corner_age: [0, 100], ..OK });
        assert_eq!((side.bsd, side.rcta), (Err(Reason::SideRadarSilent), Err(Reason::SideRadarSilent)));
        assert!(side.acc.is_ok() && side.park.is_ok());
        assert_eq!(evaluate(Some(&info(ALL & !caps::GEAR)), OK).rcta, Err(Reason::NoGear));
        assert_eq!(evaluate(Some(&info(ALL & !caps::RANGE_FANS)), OK).park, Err(Reason::NoRangeFans));
        assert_eq!(evaluate(Some(&info(ALL)), Health { park_fault: true, ..OK }).park, Err(Reason::ParkSensorFault));
        // Not running (at speed): an old cycle isn't a fault.
        assert!(evaluate(Some(&info(ALL)), Health { park_running: false, park_age: 9999, ..OK }).park.is_ok());
        // No pedals: warn only.
        assert!(!evaluate(Some(&info(ALL & !caps::PEDALS)), OK).surround_brake);
    }
}
