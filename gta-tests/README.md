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
| `drive THR BRK STEER SECS` | the driver's inputs: 0–1 pedals, steering −1 to 1 (left positive). Fractional values count as a gamepad |
| `fault adas\|chassis drop\|crc\|delay SECS` | a bus fault, as the engineering panel injects it |
| `sleep SECS` | wait a fixed time |
| `wait COND [TIMEOUT]` | wait until `speed>=KPH`, `speed<=KPH`, `stopped`, `engaged`, `off` or `event:TEXT` (default 30 s) |
| `check_gap SECS` | measure the time-gap error to the lead for SECS |

**Pass criteria:**
- `min_aeb`, `max_aeb`: limits on the number of AEB activations;
- `no_impact` (on by default);
- `stop_gap_min`, `stop_gap_max`: the ground-truth gap to the car ahead at the end;
- `end_kph_below`: the car must end slower than this;
- `events` / `no_events`: text that must or must not appear in the core's events;
- `max_gap_error_s`: the largest time-gap error `check_gap` may measure.

A failed step fails the test and the runner moves on to the next one. Each run's core log and recording are in `recordings/gta-tests/`; `replay` and `adas-viz --replay` open the recording.
