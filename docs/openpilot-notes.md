# What we borrow from openpilot (D13)

2026-11-09

openpilot and opendbc's current source (master), compared with what this project does, for the phases that build on it. Paths are in `commaai/openpilot` (under `openpilot/`) and `commaai/opendbc`.

## Engagement and override

| | openpilot | Us (D11) |
| --- | --- | --- |
| Brake | Disengages on the rising edge, or while held if moving (`selfdrived.py`, `pedalPressed`) | Disengages while pressed (> 10%). Engaging with the brake held at a standstill is refused |
| Gas | Overrides longitudinal control, no disengage (`gasPressedOverride`, `ET.OVERRIDE_LONGITUDINAL`); disengaging on the accelerator is an option (`DisengageOnAccelerator`) | Same default: overrides while pressed |
| Steering | Overrides lateral control (`steerOverride`, `ET.OVERRIDE_LATERAL`); some cars also disengage on a steering rising edge (`steerDisengage`) | Overrides lateral control; debounced per device (key 150 ms, stick 15% for 100 ms) |
| Cancel button | Disengages (`buttonCancel`) | Disengages (F3, or the engage request dropping) |
| A fault while engaged | **Soft disable:** "TAKE CONTROL IMMEDIATELY", control kept for `SOFT_DISABLE_TIME = 3` s, then off; back to enabled if the fault clears in time (`selfdrived/state.py`). **Immediate disable:** off at once | Takeover request: visual at once, chime from 1.5 s, then a **controlled stop** with hazards from 4 s. The request stands until the driver answers |

**What we take:**
- Brake on the rising edge or while moving. This would let a driver engage with the brake held at a standstill. It's a small change in `driver.rs`, worth doing when stop-and-go engagement matters.
- Two severities. An immediate disable fits faults where continuing is unsafe; we don't distinguish yet.
- Our controlled stop goes beyond openpilot on purpose: it is D11's "escalate to a minimal-risk manoeuvre", openpilot simply hands back.

## Driver monitoring (Phase 3 option)

`selfdrive/monitoring/policy.py`. Alert levels 1 to 3, in seconds of inattention:

| Policy | Alert 1 | Alert 2 | Alert 3 |
| --- | --- | --- | --- |
| Camera (vision) | 5 | 8 | 13 |
| Hands on the wheel only | 5 | 15 | 25 |

At level 3, openpilot says "DISENGAGE IMMEDIATELY". There is also a no-response timeout of 5 s. Repeated level-3 alerts lock engagement out for 1, 5, 15 and then 30 minutes.

For us, the steering-wheel-only timings fit, since there's no driver camera. Our takeover escalation is the natural end of that alert ladder.

## Longitudinal (Phase 1 tuning)

| | openpilot (Toyota) | Us |
| --- | --- | --- |
| Acceleration limits | `ACCEL_MAX = 1.5` m/s² (2.0 with the raised limit), `ACCEL_MIN = -3.5` (`toyota/values.py`) | `accel_max = 2.0`, ACC braking up to 3.5 |
| Time gaps | Relaxed 1.75 s, standard 1.45, aggressive 1.25 (`long_mpc.py`, `get_T_FOLLOW`) | 1.0, 1.5, 2.0, 2.5 s, default 2.0 |
| Jerk | Penalised in the MPC (jerk factor 1.0, aggressive 0.5) | Hard limits: reference 2.5 m/s³, command jerk-limited |

**What we take:**
- Our limits sit in the same band. 2.0 m/s² matches Toyota's raised limit.
- Our default gap is longer than openpilot's standard. That's fine for testing; a 1.45 s option could replace 1.5.

## Lateral (Phase 3)

| | openpilot / opendbc | Us |
| --- | --- | --- |
| Comfort limits | ISO 11270: `ISO_LATERAL_ACCEL = 3.0` m/s², `ISO_LATERAL_JERK = 5.0` m/s³. The controller caps at `MAX_LATERAL_ACCEL` ≈ 3.6 m/s² and `MAX_LATERAL_JERK` ≈ 3.6 m/s³, with road bank allowed for (`opendbc/car/lateral.py`) | None yet: no lateral planner |
| Toyota LTA (angle) limits | Steering-wheel angle ≤ 94.9°. Rate up 0.3 → 0.15° per 10 ms frame from 5 to 25 m/s (about 30 → 15 °/s); rate down 0.36 → 0.26 (`toyota/values.py`, `ANGLE_LIMITS`) | EPS model: road wheel ≤ 0.6 rad (about 509° at the wheel) at 0.8 rad/s (about 680 °/s at the wheel), at any speed |
| Learned online | `paramsd.py`: steer ratio (0.5–2× the car's value), tyre stiffness, angle offset, road roll, by a Kalman filter. `torqued.py`: lateral-acceleration factor, offset and friction for torque-controlled cars | The brake gain (D19); the steering ratio is a constant 14.8 |

**What we take** (all for Phase 3):
- Plan steering within ISO 11270: 3 m/s² lateral acceleration, and lateral jerk around 3.6 to 5 m/s³.
- **Tighten the EPS model to TSS2's real limits** before lane keeping. Our EPS is about 20 times faster than the car would accept, and has five times the range. Make the rate limit speed-dependent as opendbc's is, and cap the steering-wheel angle near 95°.
- Learn the steering ratio and angle offset online, as `paramsd` does, the same way we already learn the brake gain. GTA cars vary, so a constant ratio won't hold.
- Our single brake gain has the problem `torqued` avoids by fitting a curve: GTA's brakes aren't linear. Fit a pedal-to-deceleration curve in the Phase 4 calibration drive.
