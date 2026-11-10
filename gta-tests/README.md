# In-game tests

One command runs every test here on every car you name:

```
cargo xtask gta                          # every test, on the Sultan
cargo xtask gta --car sultan,baller      # on each car
cargo xtask gta gta-tests/takeover.toml  # one test
cargo xtask gta --restart                # after an adapter change: closes GTA, deploys, restarts it
```

It builds the core, the runner and the adapter, and deploys the adapter if it changed. If GTA isn't running it starts it; the Straight To Story Mode mod skips the menus. Then it runs each test:

1. spawns the car and teleports to the test spot, through the adapter's test commands (UDP 47310, no key presses or window focus);
2. starts a fresh `adas-core`, which records to `recordings/gta-tests/`;
3. runs the steps, waiting on what the core's telemetry reports;
4. scores the run against `[pass]` and prints a table.

The panel (`adas-panel`) and the Rerun views (`adas-viz`) can watch while it runs.

## Writing a test

```toml
name = "AEB: stopped car from 50 km/h"
spot = "airport taxiway"          # the default
steps = [
  "engage",
  "set 50",                       # presses the speed buttons until 50 km/h
  "wait speed>=48 25",            # waits up to 25 s
  "stopped 70",                   # a stopped car 70 m ahead
  "wait event:AEB braking 15",
  "wait stopped 15",
]

[pass]
min_aeb = 1
stop_gap_min = 2.0                # ground-truth gap to the car ahead at the end (m)
stop_gap_max = 15.0
```

**Steps:**

| Step | Does |
| --- | --- |
| `engage`, `cancel` | turn the ACC engage request on or off |
| `set KPH` | set the ACC speed |
| `gap SECONDS` | set the time gap |
| `lead DIST KPH` | spawn a lead car DIST m ahead at KPH |
| `stopped DIST` | spawn a stopped car DIST m ahead |
| `lead_brake` | the lead brakes hard |
| `clear` | remove the test car ahead |
| `drive THR BRK STEER SECS` | the driver's keys: W and S 0–1, steering −1 to 1 (left positive). Fractional values count as a gamepad. In GTA, S drives backwards from a standstill |
| `creep back\|forward M/S SECS` | the driver holds a parking speed (m/s), backwards or forwards; any `drive` ends it |
| `signal left\|right\|off` | the turn signal |
| `overtake left\|right DIST KPH` | a car DIST m behind in the next lane, driving straight at KPH (a fixed profile, no AI driver) |
| `cross left\|right DIST KPH` | a car from 25 m to that side, crossing DIST m behind the rear bumper at KPH |
| `obstacle behind\|ahead GAP` | a van parked broadside, its near side GAP m from the bumper |
| `fault adas\|chassis drop\|crc\|delay SECS` | a bus fault, as the engineering panel injects it |
| `sleep SECS` | wait a fixed time |
| `wait COND [TIMEOUT]` | wait until `speed>=KPH`, `speed<=KPH`, `stopped`, `engaged`, `off`, `reverse` or `event:TEXT` (default 30 s) |
| `check_gap SECS` | measure the time-gap error to the lead for SECS |

**Pass criteria:**
- `min_aeb`, `max_aeb`: limits on the number of AEB activations;
- `no_impact` (on by default);
- `stop_gap_min`, `stop_gap_max`: the ground-truth gap to the car ahead at the end;
- `end_kph_below`: the car must end slower than this;
- `events` / `no_events`: text that must or must not appear in the core's events;
- `max_gap_error_s`: the largest time-gap error `check_gap` may measure;
- `min_rcta`, `max_rcta`, `min_park`, `max_park`: limits on RCTA and park assist brake activations;
- `min_room`: the ground-truth room at the end from the bumper (rear when reversing) to the nearest actor in the car's path (m);
- `max_lane_rms`, `min_centering`, `max_takeovers`: lane centering's RMS offset from the lane centre while it steers (m), the share of the engaged time it steered, and the number of takeover requests;
- `min_engaged`: the share of the run, from the first engagement to the end, that the system stayed engaged. A drop-out fails it, such as the adapter releasing a core that stopped answering.

The core reports the surround features as events: "BSD: vehicle in the left blind spot", "BSD warning: signalling left into an occupied lane", "RCTA: cross traffic from the right", "RCTA braking", "park assist braking".

A failed step fails the test and the runner moves on to the next one. Each run's core log and recording are in `recordings/gta-tests/`; `replay` and `adas-viz --replay` open the recording.
