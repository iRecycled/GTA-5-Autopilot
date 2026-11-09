# Spikes S3 and S6 results: the adapter ↔ core link and the ray budget

2026-11-08 · **S3 pass · S6 pass, with design constraints**

Both spikes ran in the same GTA sessions, through one plugin ([`spikes/s36-link`](../../spikes/s36-link/src/lib.rs)) and the first real core process ([`crates/adas-core`](../../crates/adas-core/src/main.rs)).

## Questions
- **S3:** is a shared-memory round trip between the GTA adapter and a separate core process fast enough? The pass mark is a p99 under 0.5 ms at 60 and 144 fps.
- **S6:** do the line-of-sight checks for the five radars and the ultrasonic fans fit in 0.5 ms per frame in heavy traffic? And do parked vehicles appear in GTA's vehicle list?

## Setup

| | |
| --- | --- |
| Game | GTA V Enhanced, `VER_EN_1_0_1158_13`; Sultan; player invincible |
| Contract | [`crates/adapter-api`](../../crates/adapter-api/src/lib.rs) v1: `Header`, `VehicleSample`, `Command`, a seqlock mailbox per channel, a named Windows file mapping, and an auto-reset event that wakes the core |
| Core | `adas-core` runs [`control::SpeedController`](../../crates/control/src/longitudinal.rs), the S4 speed controller made sim-neutral, over the link |
| Timing mode | Reactive (D23). Each frame the adapter writes a sample, signals the core, and waits up to 1 ms for the command that answers this sample. It applies the command in the same frame |
| Frame rate | The game simulates at **72 fps**. The overlay's ~143 includes DLSS Frame Generation, which inserts generated frames, so scripts tick at 72 Hz, as S1 also found. The 60 fps case uses a cap in the plugin (F6) |

## S3 results

**Round trip** is measured from sample write to seeing the answering command, in the adapter:

| Run | Core mode | Frames | p50 | p99 | Max | Over 0.5 ms |
| --- | --- | --- | --- | --- | --- | --- |
| 72 fps, core driving (×2) | event | 7,619 | 8 µs | 70 µs | 139 µs | 0 |
| 60 fps, core driving | event | 3,180 | 10 µs | 22 µs | 38 µs | 0 |
| 72 fps, core driving | spin | 2,965 | 0.8 µs | 1.5 µs | 203 µs | 0 |

- The core's own step (the controller) takes 0.1–0.6 µs.
- **Event mode is the default.** The core sleeps between frames and still answers within the same frame with plenty of room. Spin mode is 10× faster but uses a whole CPU core.
- **The speed controller behaves the same over the link as it did in-plugin (S4).** At 60 fps, the error over the last 2 s of each segment was −0.01 to +0.15 m/s, and the car came to a clean stop and held.
- **Watchdog (D24):**
  - The core process was killed while driving at 8 m/s.
  - The adapter re-applied the last command for 6 frames, then at frame 7 (about 97 ms) released throttle, brake and hold and showed TAKE OVER.
  - The car then coasted down under the player's control.
- **Restart (D24):** a fresh core bumps the epoch and starts **disengaged**. It engaged only on a new request, and never on its own.

**S3 passes.** The p99 is 22–70 µs against a 0.5 ms limit, at both frame rates the game actually runs. A 144 Hz *simulation* would need Frame Generation off. Nothing here depends on the frame rate: at 144 Hz the frame is 6.9 ms, about 100 times the p99.

## S6 results

The ray pass, every frame:
1. List all vehicles.
2. Find the targets inside any radar's field of view, and pick the closest covering radar for each.
3. Schedule one line-of-sight ray per target, oldest result first, for a 4-frame refresh.
4. Below 3 m/s, add 12 ultrasonic sensors × 3 rays for a 5-frame refresh.

The radar layout matches D25, except that the front radar used ±30° instead of ±10°. That puts more targets in view, so the numbers are conservative.

**Run 1** (unscheduled; every candidate and every ultrasonic ray started every frame) found two problems:
- **GTA allows only about 10 async shape tests in flight.** `START_SHAPE_TEST_LOS_PROBE` returns 0 past that. About 63 of the 73 starts per frame failed. Failed starts are cheap, under 1 µs each.
- **2.5 ms per frame** downtown, with 157 vehicles listed.

**Run 2** (scheduled, with a timing breakdown) took 25 s per mode. The ego car stood in downtown traffic with 105–163 vehicles listed, 20–34 targets in view, and ultrasonics on in every frame:

| Mode | Pass p50 | Pass p99 | Max | Rays per frame | Result age, worst target (p50 / p99) |
| --- | --- | --- | --- | --- | --- |
| Native positions, async rays (≤ 8 in flight) | 38 µs | 59 µs | 273 µs | 8 | 5 / 6 frames |
| Memory positions, async rays | 86 µs | 124 µs | 279 µs | 8 | 6 / 6 frames |
| Memory positions, sync rays (12 per frame) | 176 µs | 255 µs | 364 µs | 12 | 5 / 5 frames |

The table excludes the 1-in-30 frames that sampled the parked count (see below).

- **The cost was `GET_PED_IN_VEHICLE_SEAT`, not the rays.** It takes about 60 µs per vehicle, so 7–11 ms for 130–160 vehicles. Run 1 called it every frame. Driver presence must be sampled a few vehicles per frame, or read another way.
- **The native `GET_ENTITY_COORDS` beats reading positions from memory** (about 0.1–0.15 µs vs 0.45 µs per vehicle: 13–19 µs vs 63–72 µs for 120–150 vehicles), because `ReadProcessMemory` is a system call. The plain native is both faster and less GTA-specific, so it is the choice.
- **A synchronous ray costs about 8 µs** here (12 took 95 µs at p50). That's cheaper than S1's 15–40 µs measurement.
- **Listing vehicles** (`worldGetAllVehicles`) costs about 10 µs for 160 vehicles. Reading earlier results costs about 6 µs.
- **Parked vehicles appear in the vehicle list:** 49–75 of the 105–163 listed had no driver.
- **Async alone refreshes the worst target about every 6 frames** (12 Hz at 72 fps), and the average target about every 2.5 frames. The radars are specified at 20 Hz (D25), a 3.6-frame period.

**S6 passes, with these design rules** for the GTA adapter:
1. **Positions from `GET_ENTITY_COORDS`.** Never call a per-vehicle native that isn't needed every frame. Sample `GET_PED_IN_VEHICLE_SEAT` a few vehicles per frame.
2. **One ray per target, not per radar.** Schedule rays oldest first, within GTA's async limit (8 in flight).
3. **Top up with synchronous rays** (about 8 µs each) when more are due than the async limit allows. That keeps the worst target at the 20 Hz radar rate.
   - *Estimate, not measured:* about 10 radar rays per frame at 34 targets, plus about 7 ultrasonic rays per frame. The pass would cost 8 async rays plus about 9 sync rays, roughly 110–150 µs at p99.
4. **Ultrasonics only at parking speeds**, as on real cars.

## Gaps

- **The ego car was stationary in traffic.** The downtown teleport put it against a planter, so the AI couldn't drive it. Per-frame cost depends on vehicle count and targets in view, and both were at their heaviest here. What's untested is driving *through* traffic, where targets enter and leave view faster. That needs a teleport that snaps to a road node (`GET_CLOSEST_VEHICLE_NODE`), and comes with the Phase 0 adapter.
- **The async/sync hybrid in design rule 3** is estimated from the measured parts, not run as one mode.
- **A 144 Hz simulation** wasn't run, because Frame Generation is on.

## What this changes

- **D4 is confirmed:** separate process, shared memory, an event wake, and the reactive timing mode.
- **D24 is verified in-game:** the 6-frame watchdog and the epoch restart.
- **D25 holds**, with the design rules above in the GTA adapter. Nothing in the core changes. A host with native radar sensors (BeamNG) skips all of this and fills the same channel.
- **The adapter's frame budget** (≤ 1 ms, Phase 0 milestone) is on track:
  - link round trip p99 ≤ 70 µs;
  - ray pass p99 about 60–150 µs;
  - sampling a handful of natives.
