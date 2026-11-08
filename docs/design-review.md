# Design review

2026-11-08 · Nick Peterson

## Verdicts at a glance

Keep 7, change 8. D12 is now decided as recommended. Most changes refine a decision rather than reverse it. The two that matter most: lidar and camera data can't travel over CAN (D9), and steering through simulated controller input will feel laggy (D5).

**Outcome:** every recommendation below was accepted on 2026-11-08 and applied to the PRD's decision log as D1 to D21.

**Superseded in part:** a second, independent review (GitHub issues #1–#11) revised several of these decisions on 2026-11-08 and added D22–D29. Where this page and the PRD disagree, the PRD wins. This page is kept as the record of the first review.

| ID | Decision | Verdict | The change, in one line |
| --- | --- | --- | --- |
| D1 | Rust core | Keep | Speed comes from the architecture; Rust buys safety and tooling |
| D2 | Enhanced edition | Change | Enhanced stays the target; Legacy becomes the fallback if spike S2 fails |
| D3 | ReShade graphics hook | Keep | Fix the shim's C interface now; add a no-injection screen-capture fallback for the camera |
| D4 | Separate core process | Keep | Add a plan for sharing GPU frames across processes (shared resources + fence) |
| D5 | Simulated input for controls | Change | Hybrid: pedals by input, steering by angle command through a modelled power-steering unit |
| D6 | Ground truth first | Change | Sensor error model required from Phase 2; ground truth kept as a scoring oracle |
| D7 | Pretrained YOLO | Change | Fine-tune on GTA's free auto-labels; pick a detector whose license fits D14 |
| D8 | Overlay + Rerun | Keep | Optionally add Foxglove for replaying recordings |
| D9 | Copy a real car's CAN | Change | Several buses plus an Ethernet channel for lidar and camera; real-car layout for the chassis bus only |
| D10 | Game time | Change | Stamp both game time and a high-resolution wall clock |
| D11 | Override rules | Change | openpilot-style rules, thresholds per input device, escalating takeover requests |
| D12 | Target PC | Keep | Decided: separate gaming PC, RTX 4070 Ti + Ryzen 7 7800X3D |
| D13 | openpilot later | Change | Borrow its pieces now; run it whole later under WSL2 |
| D14 | Private until Phase 3 | Keep | Keep; watch dependency licenses |
| D15 | Wheel support later | Keep | Keep deferred; design override around a driver-torque signal so a wheel drops in |

## D1 · Core language: keep Rust

The premise needs one correction: Rust won't make the system fast; the architecture will. The real bottlenecks are the game's frame cadence, GPU readback and inference. C++ and Rust both hit them equally well. Rust still wins on safe multithreading in the core, Cargo, and no garbage-collector pauses.

| Option | Good | Bad |
| --- | --- | --- |
| **Rust everywhere + a small C++ shim** (current) | One language for all logic; safe concurrency; great tooling (Rerun, `ort`, `nalgebra`) | FFI glue for Script Hook V; C++ shim for ReShade; slower compiles |
| C++ in-game, Rust core | Natural fit for ReShade and the Script Hook V headers | Two languages and two build systems; memory-safety bugs in the riskiest code (hooks) |
| C++ everywhere | Every GTA example and SDK is C++ | Most unsafe concurrency; weaker package tooling |
| C# (Script Hook V .NET) in-game, Rust core | Fastest prototyping in-game | Garbage-collector pauses on the game thread, exactly where we have the least slack |

**Recommendation: keep.** Slow compiles are offset by D4: the core restarts without restarting GTA.

## D2 · Game edition: change to Enhanced target, Legacy fallback

Enhanced is the right target: better visuals help the camera models, and it's what you own and play. But it is the riskier platform. DX12 is harder to hook, its memory offsets are undocumented, all the research prior art is Legacy, and BattlEye must be bypassed at launch.

| Option | Good | Bad |
| --- | --- | --- |
| **Enhanced only** (current) | Best visuals; one target | If depth access fails (S2), lidar and vision are blocked with no plan B |
| Legacy only | DX11 is easy to hook; DeepGTAV-era code and offsets apply (Legacy has also had BattlEye since September 2024, bypassed the same way with `-nobattleye`) | Worse visuals; older edition, likely to get less mod attention over time |
| **Enhanced target, Legacy fallback** | Same plan, plus an exit if S2 fails; Script Hook V code is identical on both | Edition-specific code must be kept in one place |
| Support both permanently | Widest reach | Double testing for every patch |

**Recommendation: change to Enhanced target with a Legacy fallback.** Keep all edition-specific code (memory patterns, graphics hook) in one crate. PC owners can generally install both editions; check your launcher.

## D3 · Graphics hook: keep ReShade first, with two refinements

ReShade is the right first bet because it has already solved the hardest part: finding the right depth buffer among the dozens a frame uses. The weaknesses: it's another version-coupled dependency, its API is C++, and its depth detection on GTA Enhanced is unproven. The current plan has no fallback if only color capture works.

| Option | Good | Bad |
| --- | --- | --- |
| **ReShade add-on + C++ shim** (current) | Depth detection already done; overlay included; large community | C++ API; ReShade updates can break it; depth on Enhanced unverified |
| Pure-Rust DX12 hook (`hudhook` + our own resource tracking) | All Rust; no third-party render layer; full control | We must identify GTA's depth buffer ourselves, and redo it after patches |
| Script Hook V's present callback | Already loaded; no extra hook | Gives the swap chain only: no depth, no stencil; DX12 support unverified |
| Windows screen capture API (outside the game) | No injection at all; very stable; easy in Rust | Color only, no depth; about 1 frame of extra latency; captures HUD and overlay too |
| Calling ReShade's C++ interfaces from Rust directly | No C++ code to maintain | Hand-written vtable bindings; fragile across ReShade versions |

**Recommendation: keep, with two changes.**

1. Define the shim's C interface now, before S2, as about five functions: frame ready, depth handle, stencil handle, timestamp, overlay draw. Then ReShade and a pure-Rust hook are interchangeable behind it.
2. Add screen capture as a no-injection fallback for the camera. If S2 fails on depth, the camera features still ship and only lidar waits.

## D4 · Process model: keep the separate process, plan the GPU side

The separate process is right; the PRD just undersells why and skips its hardest part. The big win is development speed: GTA takes 1 to 2 minutes to load, and the core will be rebuilt hundreds of times. The gap: sending frames to a detector in another process means sharing GPU memory across processes, which the PRD never mentions.

| Option | Good | Bad |
| --- | --- | --- |
| **Separate process + shared memory** (current) | Rebuild and restart the core in seconds; a core crash leaves the game running; clean boundary | GPU frames need cross-process sharing; two processes to launch |
| Core inside the game process, on its own threads | Simplest data path; no IPC | Every core change means restarting GTA; a panic kills the game |
| Core as a hot-reloadable DLL inside the game | Fast reloads without IPC | Unloading Rust DLLs safely is unreliable (threads, statics, thread-local storage); crashes still kill the game |
| Separate process over the network (a second PC) | Could offload compute | Kills GPU sharing; adds milliseconds of jitter |
| Hybrid: emergency braking in-game, the rest outside | Shortest emergency path | Saves ~0.1 ms against a 16.7 ms frame; splits the brain in two |

**Recommendation: keep, and add the GPU plan.** The graphics hook marks its depth and color copies as shareable across processes. The core opens them on its own GPU device, and both sides synchronize on a shared GPU fence. This is standard DX12 practice and keeps frames off the CPU. Fold it into spike S7, and pull S7 forward to Phase 1 because it shapes the shim interface (D3).

## D5 · Actuation: change to a hybrid

This is the decision most likely to make the car feel slow. Simulated controller input passes through GTA's own steering smoothing, speed-dependent lock and auto-countersteer. The controller would then be steering a laggy, nonlinear filter instead of the wheels. Real lane-keeping systems send the power-steering unit an angle or torque command, and real pedals are close to direct.

| Option | Good | Bad |
| --- | --- | --- |
| **Simulated controller input** (current) | Uses supported game functions; no memory offsets; patch-proof | GTA's steering filter adds lag and nonlinearity; smooth small corrections are hard |
| Direct memory writes for all controls | Exact, immediate control of steering angle, throttle, brake | Offsets differ per build (S5); writes can fight the game's own input handling |
| Virtual gamepad driver (OS level) | No game modification, like the 2020 openpilot-in-GTA project | Still goes through the steering filter; the main Windows driver (ViGEmBus) has been retired |
| Speed- or force-setting functions | Trivial to use | Teleport-like physics; destroys the realism goal |
| **Hybrid: pedals by input, steering by angle written to memory** | Pedals behave naturally; steering gets a real angle command | Needs the steering offset from S5 |

**Recommendation: change to the hybrid.** Add a modelled power-steering unit in the actuator ECU with rate limits, torque limits and fault states. The car's feel then comes from our model, not GTA's input filter. While the system is engaged, block the player's controls from reaching the car and read them separately; those raw inputs become the driver-override signal (D11). Keep simulated input as the fallback if S5 finds no stable steering offset.

## D6 · Ground truth vs vision: keep the order, harden the ground truth

Starting with ground truth is right, but it has a trap. Features tuned on perfect data (no misses, no false alarms, no jitter) look smart until vision arrives, then brake for ghosts or miss pedestrians. The fix is to make ground truth imperfect on purpose from the start.

| Option | Good | Bad |
| --- | --- | --- |
| **Ground truth through Phase 4, then vision** (current) | Fast path to a drivable car | Features overfit to perfect data |
| Vision from day one | Realistic from the start | Months before anything drives; perception and control bugs get tangled |
| Ground truth forever, vision only for show | Simplest | Gives up the "how a real car sees" goal |
| **Ground truth through a sensor error model** | Misses, false positives, position noise and latency per sensor; tuned to real sensor specs | A real piece of work to build and tune |
| **Ground truth recorded alongside vision as a scoring oracle** | Measures vision accuracy for free, every drive | More recording volume |

**Recommendation: keep the order and add both.** Make the error model mandatory from Phase 2; calibrate it later from measured vision errors. Record ground truth on every drive so Phase 5 can score vision against it.

## D7 · Vision models: change to fine-tuning on auto-labels

The current plan wastes GTA's biggest advantage. The game can label its own frames: the stencil buffer marks every vehicle and pedestrian pixel, and the game knows every object's exact position. That's why researchers used GTA. One evening of driving produces thousands of labeled frames. A license problem is also hiding here: the popular Ultralytics YOLO releases are AGPL-3.0, which constrains any public release (D14).

| Option | Good | Bad |
| --- | --- | --- |
| **Pretrained YOLO, openpilot model for lanes** (current) | No training | Trained on real photos, so weaker on GTA; AGPL license |
| **Pretrained detector fine-tuned on GTA auto-labels** | Big accuracy gain for little effort; labels are free and exact | Needs a training loop (offline, Python is fine) |
| Apache-licensed detector (YOLOX, RT-DETR) instead of Ultralytics | No license constraints on release | Slightly less polished tooling |
| openpilot's driving model for lanes and path | Trained on millions of real miles; MIT license | Expects a specific camera geometry, so frames must be reshaped; may struggle with GTA's look |
| Classic lane detection (edges, line fitting) | No models | Brittle in shadows, rain and at junctions |
| Map lanes only | Already planned for Phase 4 | Not vision, so the camera-sees-lanes goal goes unmet |

**Recommendation: change.** Use an Apache-licensed detector fine-tuned on auto-labeled GTA frames. Map lanes are the baseline, and openpilot's model is a Phase 5 experiment against them. Revise the non-goal "no training our own networks" to "fine-tuning allowed; no training from scratch."

## D8 · Views: keep overlay + Rerun

The split is right: the in-game overlay is what the driver sees, and Rerun is what the engineer sees. One gap: Rerun is best for live viewing, while recordings are MCAP files, which Foxglove opens natively with ready-made plot, 3D and message panels.

| Option | Good | Bad |
| --- | --- | --- |
| **In-game overlay + Rerun** (current) | Rust-native logging in a few lines; live point clouds and timeline scrubbing | Rerun's own file format differs from our MCAP recordings |
| Overlay only | One screen | Cramped; point clouds and the CAN trace drawn over the game |
| Overlay + Foxglove | Opens MCAP directly; rich robotics panels | Live streaming needs a bridge; not Rust-native |
| Custom `egui` app | Exactly what we want | Weeks of UI work Rerun already does |

**Recommendation: keep.** Add Foxglove as an optional replay tool, which costs nothing because recordings are already MCAP. If D3 settles on ReShade, draw the overlay through ReShade's own overlay hook rather than a second ImGui hook.

## D9 · CAN design: change to several buses plus an Ethernet channel

This is the biggest technical flaw in the PRD: it puts everything on one CAN bus, and some of that data physically can't fit.

- **Lidar:** tens of thousands of points at 10 Hz is several MB/s. Classic CAN carries under 0.06 MB/s. Real cars send lidar over Automotive Ethernet.
- **Camera:** real cars carry video over dedicated serial links to the camera ECU, which puts only objects and lanes on CAN. Our design already does this in effect, since frames stay on the GPU; the PRD should say so.
- **Radar:** 5 radars × 32 tracks × 20 Hz is about 3,200 frames per second. That is roughly 80% of a 500 kbit/s bus before any chassis traffic. Real cars put radar on a private ADAS bus, often CAN FD.

| Option | Good | Bad |
| --- | --- | --- |
| **One bus, a real car's layout** (current) | Simple; realistic chassis messages | Lidar can't fit; radar overloads it |
| One bus, our own layout | Total freedom | Same capacity problem; less realism; openpilot is ruled out |
| **Several buses + gateway + Ethernet channel** | Matches real cars: chassis CAN, private ADAS CAN FD, body/cluster CAN, Ethernet for lidar, serial links for cameras | More to model; needs a gateway ECU |
| Everything over simulated Ethernet | Unlimited bandwidth | Loses the CAN realism you asked for |

**Recommendation: change.** Use a real car's layout (from opendbc) for the chassis bus so openpilot stays possible. Put radars on a private CAN FD bus, lidar on a modelled Ethernet channel, and cluster/body on a third bus, with a gateway ECU between them. The bus model then shows real bus load, and overloads become a testable fault.

## D10 · Time model: change to two clocks

Game time is right for control: it pauses with the game and scales in slow motion. But the latency budgets in the PRD are wall-clock targets, and game time can't measure them. Game time also only ticks once per frame at millisecond resolution. How GTA's game timer behaves in the pause menu and in slow motion is unverified.

| Option | Good | Bad |
| --- | --- | --- |
| **Game time** (current) | Correct across pause and slow motion | Can't measure real latency; millisecond resolution |
| Wall-clock time | Precise; measures latency | Controllers misbehave on pause and slow motion; replays drift |
| **Both on every message** | Game time drives control; wall clock measures latency and frame pacing | 8 more bytes per message |
| Lockstep (the game advances only when the core says so) | Repeatable runs | GTA isn't deterministic; freezing frames breaks its physics and audio |

**Recommendation: change to both clocks.** Add the pause and slow-motion check to spike S1.

## D11 · Engagement and override: change to openpilot-style rules

The current rules assume a "steering torque" we don't have. A keyboard steering key is all-or-nothing, so one tap would read as a full override. Gamepad sticks are analog, but GTA has no torque concept at all. The rules also say nothing about what happens when the driver ignores a takeover request.

| Option | Good | Bad |
| --- | --- | --- |
| **Current rules** (brake disengages all, torque threshold overrides steering) | Simple | Needs torque we can't measure; no escalation |
| **openpilot's model**: brake cancels all; gas pauses speed control; steering input pauses lane-keeping | Proven and well documented; realistic | Needs per-device thresholds |
| Tesla-style: steering input disengages lane-keeping fully | Clear to the driver | Annoying with a keyboard, where every tap disengages |
| Driver monitoring: periodic hands-on checks | Realistic Level 2 behavior | Nagging gets in the way of testing; make it toggleable |

**Recommendation: change.** Adopt openpilot's model with thresholds per input device: gamepad above about 15% for 100 ms, keyboard held longer than 150 ms. Add escalating takeover requests: visual, then chime, then a controlled stop with hazard lights. Driver monitoring is an optional toggle.

## D12 · Target PC: decided, a separate gaming PC

Decided: GTA and the core run on a dedicated gaming PC with an RTX 4070 Ti and a Ryzen 7 7800X3D, not the Docker server. That removes the GPU contention and the risk to house services, and the GPU fully supports TensorRT. What it means for the design:

| Resource | What it means | Action |
| --- | --- | --- |
| GPU compute (Ada generation, TensorRT FP16) | A small detector at 640 px should take a few ms per frame (estimate), well inside the 8 ms budget | S7 measures it; headroom could later fit a lane-segmentation model |
| VRAM: 12 GB (16 GB if it's the Ti Super) | The tightest resource. GTA Enhanced with ray tracing at high settings can use most of it | Budget: game ≤ 9 GB, perception ≤ 2 GB, 1 GB margin. Develop with ray tracing off or low; DLSS frees VRAM |
| CPU: 8 cores, 16 threads, one chiplet with 3D V-Cache | Excellent for GTA; one chiplet means no cross-chiplet latency on shared memory | The core uses 2 to 3 threads; the 100 Hz tick thread runs at high priority |
| Separate from the Docker server | No shared GPU or power budget | None |

**Recommendation: keep,** with the VRAM split written into the performance requirements (done).

## D13 · openpilot: change to borrowing pieces now

Running openpilot whole is a fun Phase 6 stretch goal. Its pieces are useful much earlier, and waiting means re-deriving what it already solved. openpilot is MIT-licensed and runs on Linux, so it would need WSL2 on the gaming PC.

| Option | Good | Bad |
| --- | --- | --- |
| **Run it whole, later** (current) | A real-world brain as a benchmark against ours | Only possible if D9 copies a supported car |
| Never | No Linux dependency | Throws away proven designs |
| **Borrow pieces now: opendbc layouts, engagement rules, controller tuning, driving model** | Proven designs from day one; nothing to run | Must read and adapt Python code into Rust |
| Run it whole, early | Fastest "it drives" | Then it's openpilot driving, not our stack; slows our own architecture |

**Recommendation: change to borrowing now.** Running it whole stays a Phase 6 option.

## D14 · Release: keep private until Phase 3

Private first is right. Rockstar tolerates single-player, non-commercial mods, and an unfinished mod published early draws bug reports. The one thing to do now is track licenses: an AGPL dependency (D7) would force the whole public release to be AGPL.

| Option | Good | Bad |
| --- | --- | --- |
| **Private until Phase 3** (current) | Freedom to break things | No outside feedback |
| Public from day one | Contributors, visibility | Support load while unstable |
| Never public | No obligations | A shame for a project like this |

**Recommendation: keep,** and use only permissively licensed dependencies so any license stays open.

## D15 · Steering wheel and pedals: keep deferred, design for it

Deferring is fine, but a wheel is the most realistic way to experience this project. With force feedback, the wheel would turn by itself as the autopilot steers, and a real torque reading would make override detection genuine. Add one rule now so it drops in later.

| Option | Good | Bad |
| --- | --- | --- |
| **Later** (current) | Focus | Risk of designs that assume a keyboard |
| Now | Most realistic from the start | Force feedback work before anything drives |
| Never | Simpler | Loses the best demo |

**Recommendation: keep deferred.** Design override (D11) around a "driver torque" signal: estimated from a keyboard or gamepad today, measured from a wheel later. Note that the Manual Transmission mod also takes over controls, so the two can't run together.

## Decisions the PRD is missing

Six decisions should join the log. D16 and D20 belong in Phase 0, because changing them later touches every module.

| New ID | Decision | Options | Recommendation | Needed by |
| --- | --- | --- | --- | --- |
| D16 | Coordinate frames and units | GTA's world frame / the automotive standard (ISO 8855: x forward, y left, z up) | ISO 8855 and SI units inside the core; convert once at the plugin boundary | Phase 0 |
| D17 | Bus topology | Follows from D9 | Chassis CAN, ADAS CAN FD, body CAN, an Ethernet channel and a gateway ECU, all defined in one topology file | Phase 1 |
| D18 | Sensor error model | Per-sensor noise only / full model (misses, false positives, latency, weather) | Full model with parameters from real datasheets, one config per sensor | Phase 2 |
| D19 | Vehicle dynamics per car | One tuning for all / hand-tuned per car / automatic identification | An automatic calibration drive (steps and sweeps) that fits a model per car; GTA cars handle very differently | Phase 2 |
| D20 | Testing strategy | Manual driving / replay tests / scenario suite | Replay recorded drives through the core on every build (no GTA needed), plus a scenario suite run before each phase gate | Phase 0 |
| D21 | Degradation behavior | Per feature, ad hoc / one matrix | One matrix: for each sensor fault, which features go unavailable and what the driver sees | Phase 2 |

Next step: tell me which verdicts you accept, and I'll update the PRD's decision log, sensor and architecture sections to match.
