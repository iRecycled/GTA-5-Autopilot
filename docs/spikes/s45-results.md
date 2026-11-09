# Spikes S4 and S5 results: GTA vehicle actuation and state (Enhanced)

2026-11-08 · **Pass**, with a changed D5

## Questions
- **S4:** can the adapter hold and track a commanded steering angle, drive the pedals, and hold the car at a standstill, on several vehicles?
- **S5:** where do the steering angle and the wheel speeds live in vehicle memory on Enhanced, and are they the same across vehicles?

## Setup

| | |
| --- | --- |
| Game | GTA V Enhanced, `VER_EN_1_0_1158_13` (as in [S1](s1-results.md)) |
| Code | [`spikes/s45-vehicle`](../../spikes/s45-vehicle/src/lib.rs); `cargo xtask deploy s45` |
| Vehicles | Sultan (AWD sedan), Baller (SUV), Futo (light RWD coupé), Mule (box truck) |
| Place | Airport taxiway (teleport tool, F7), flat and straight |

The spike's test tools (keys):
- **F7:** teleport.
- **F8:** spawn the next test car; the player is made invincible.
- **F9:** steering sweep. Injects a sine on the steering control, snapshots 12 KB of the vehicle struct every frame, and ranks every float field by correlation with the input.
- **F10:** pedal test.
- **F11:** scripted drive test:
  1. Speed up to 10 m/s on simulated throttle.
  2. Track the same angle command three ways.
  3. Brake to a stop.
  4. Try three standstill holds against creep throttle.

  Throughout, it snapshots the wheel structs and logs RPM and gear.

No one touches the controls during a test.

## Results

### Steering (S4, S5)

| | Sultan | Baller | Futo | Mule |
| --- | --- | --- | --- | --- |
| Smoothed steering input at `vehicle+0x9D4` (±0.948) | ✓ | ✓ | ✓ | ✓ |
| Steering angle (rad) at `vehicle+0x9DC` | ✓ | ✓ | ✓ | ✓ |
| Angle ÷ input (full-lock ratio) | 0.611 | 0.611 | 0.872 | 0.646 |
| Lag from our write to the angle | 1 frame | 1 frame | 1 frame | 1 frame |
| Write only: a 0.04 rad step settles at | 0.0355 | 0.0355 | 0.0426 | 0.0353 |
| Write + integral correction: the step settles at | 0.0400 | 0.0400 | 0.0400 | 0.0400 |
| Write + correction: RMS error over steps and a 0.8 Hz sine | 0.0012 rad | 0.0012 | 0.0012 | 0.0012 |

- **Writing the angle field does nothing.** GTA recomputes it every frame (we wrote 0.3; next frame it read 0.13).
- **Writing the smoothed input field every frame steers the car.** The game turns it into an angle in the same frame, and it skips GTA's own input smoothing (about 5 frames, 70 ms).
- **Offsets are the same on every vehicle tested.** Only the lock ratio differs, and the adapter estimates it on line from the two fields.
- **Write-only is off by a few percent,** depending on the vehicle, because the game pulls the input back toward the player's control each frame. Also injecting the matching steering control didn't remove the bias (Sultan 0.0355 either way). An integral correction on the measured angle (gain 0.3 per frame) removed it on every vehicle.
- **Sign:** positive input and angle mean left, which is ISO 8855. The steering control (`INPUT_VEH_MOVE_LR`) is right-positive.

### Pedals (S4)

- **Simulated input works only on unblocked controls.** Once `DISABLE_CONTROL_ACTION` blocks a control, `SET_CONTROL_VALUE_NEXT_FRAME` on it does nothing, for both pedals and steering. This rules out the old D5 plan: block the player's controls, inject ours, and read the player through the blocked control.
- **Injected value *v* gives throttle (*v* − 0.25) / 0.75.** That is a 25% deadzone. The brake behaves the same way.
- **Keyboard W is digital:** throttle goes from 0 to 1.0 in one frame. A gamepad trigger is analogue; the user will test it.
- **Holding the brake at a standstill drives the car backwards** (to 14.5 m/s in one test).

### Standstill hold (S4)

Each hold ran for 2 s with creep throttle (0.27) pushing against it, on flat ground:

| Hold | Sultan | Baller | Futo | Mule |
| --- | --- | --- | --- | --- |
| `SET_VEHICLE_HANDBRAKE` | 0.05 m | 0.10 m | 0.10 m | 0.09 m |
| `SET_VEHICLE_BRAKE` | rolled to 3.2 m/s | 3.5 m/s | 4.0 m/s | 1.7 m/s |
| None | 6.8 m/s | 6.9 m/s | 8.7 m/s | 3.3 m/s |

The handbrake native holds the car. It is the adapter's equivalent of a real car's electronic parking brake or auto-hold.

### Wheel speeds, RPM, gear (S5)

- **Wheel array:** a pointer at `vehicle+0xC30` with the wheel count at `+0xC38` (4 on every vehicle tested). The same layout as recent Legacy builds.
- **Wheel spin speed** is at `wheel+0x170`, in rad/s, negative when rolling forward. The slope against vehicle speed gives the rolling radius, and it matches wheel size:

  | | Sultan | Baller | Futo | Mule |
  | --- | --- | --- | --- | --- |
  | Radius from the `+0x170` slope | 0.32 m | 0.45 m | 0.31 m | 0.49 m |

  It behaves like a real wheel-speed sensor:
  - **Wheelspin:** during the Futo's full-throttle launch, the rear (driven) wheels spun at −48.9 rad/s while the fronts turned at −21.2.
  - **Lock-up:** under full brake, the wheels drop to 0 while the car is still doing 9 m/s. GTA has no ABS, and on the Mule they lock one at a time.
  - **Cornering:** left and right wheels differ slightly in turns.

  This is the signal ABS, traction control and odometry need.
- **`wheel+0xB0`** tracks vehicle speed × −0.889 on every wheel and vehicle, whatever the wheel size. It is probably a ground-contact speed. It isn't needed yet.
- **RPM and gear** come from natives, so they need no offset: `_GET_VEHICLE_CURRENT_REV_RATIO` (0.2 at idle to 1.0) and `_GET_VEHICLE_CURRENT_DRIVE_GEAR`.
- **Yaw rate and body velocity** also come from natives: `GET_ENTITY_ROTATION_VELOCITY` and `GET_ENTITY_SPEED_VECTOR`.

## Decisions

- **D5 changed** (PRD on PR #12):
  - **Steering:** a per-frame write of the smoothed input, with an integral correction on the measured angle. The core still sends an angle; the adapter converts it with the vehicle's lock ratio.
  - **Pedals:** simulated input on unblocked controls, through the deadzone mapping.
  - **Driver input:** read from the keyboard and gamepad through Windows.
  - **Standstill hold:** the handbrake native.
  - **Fallback:** the code hook on the game's steering update stays as the fallback. It isn't needed so far.
- **Safe state (D24):** stopping the steering write hands the steering back to the player at once. GTA's own smoothing then takes over from wherever the input is.

## Portability

Everything here stays in the GTA adapter (`gta-edition` for offsets, the GTA plugin for natives):
- the offsets;
- the deadzone;
- the handbrake hold;
- the integral correction.

The core sees an angle command and a standstill-hold request, and gets back angle, wheel speeds, yaw rate and RPM. Other hosts fill the same contract natively. BeamNG, for example, exposes steering, wheel speeds and a parking brake directly.

## Open items

- Gamepad trigger scale (the user's Xbox controller), which sets the driver-override threshold (D11).
- Re-run the F9 sweep and F11 drive test after every game update. The offsets are found by the sweep, not hard-coded.
- Behaviour on slopes, at higher speeds (lock ratio at speed), and with the speed-sensitive steering of a gamepad: Phase 0 integration tests.
- Teleporting into the airport raises a wanted level even with `SET_MAX_WANTED_LEVEL(0)`. A second teleport from inside the car clears it. This only affects test tooling.
