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
}

pub fn evaluate(info: Option<&AdapterInfo>, h: Health) -> Availability {
    let Some(info) = info else {
        let no = Err(Reason::NoAdapterInfo);
        return Availability { acc: no, stop_and_go: false, aeb: no, steer: no, drive: no, los_degraded: false };
    };
    let has = |c: u32| info.capabilities & c != 0;
    let chassis_ok = h.chassis_age <= CHASSIS_TIMEOUT_FRAMES;
    let radar_ok = h.radar_age <= RADAR_TIMEOUT_FRAMES;

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
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn info(c: u32) -> AdapterInfo {
        AdapterInfo { capabilities: c, ..Default::default() }
    }
    const ALL: u32 = caps::STEER | caps::PEDALS | caps::HOLD | caps::ACTORS | caps::LINE_OF_SIGHT;
    const OK: Health = Health { chassis_age: 0, radar_age: 0 };

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
        let silent = evaluate(Some(&info(ALL)), Health { chassis_age: 100, radar_age: 0 });
        assert_eq!((silent.acc, silent.steer), (Err(Reason::ChassisSilent), Err(Reason::ChassisSilent)));
        let blind = evaluate(Some(&info(ALL)), Health { chassis_age: 0, radar_age: 100 });
        assert_eq!(blind.aeb, Err(Reason::RadarSilent));
        assert!(blind.steer.is_ok(), "steering doesn't need the radar");
    }
}
