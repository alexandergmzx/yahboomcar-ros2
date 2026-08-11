# Simulation: what it can and cannot tell you

The car has never driven on a floor under this stack. SLAM, Nav2 and RViz all need a
robot that moves, so a simulated one was built to develop them against — and this document
exists mainly to stop its results being cited for things it cannot possibly know.

**The one-line version: simulation validates the STACK, not the ROBOT.**

---

## The design decision that makes it worth trusting

`yahboomcar_sim` replaces the **firmware**, not the software. It publishes exactly what
the board publishes and subscribes to exactly what the board subscribes to — the contract
in [`CLAUDE.md`](../CLAUDE.md):

```
pub  /odom_raw  11 Hz      sub  /cmd_vel
pub  /imu       25 Hz
pub  /scan      12 Hz
pub  /battery    1 Hz
```

Everything above that line — `yahboomcar_bringup`, `imu_filter_madgwick`, the EKF, the
governor, the deadman, `laser_odometry`, SLAM, Nav2, RViz — runs **completely
unmodified**. `sim_bringup_launch.py` literally includes `yahboomcar_bringup_launch.py`;
it is not a copy.

There is no `use_sim_time`, no simulation branch, no sim-only parameter file anywhere in
the stack. So anything made to work here is the real stack working. **Swapping in the car
is turning one process off and the robot on.**

Verified: rates measured at 11.991 / 10.999 / 25.006 / 1.000 Hz against the firmware's
12 / 11 / 25 / 1, and the scan is 360 beams at 1.000° over 0.12–8.0 m, matching the real
device measured from the bags.

## Faithful to the awkward parts

Only behaviours **measured on the real car** are modelled, because a simulator that
invents plausible dynamics is worse than one that is honestly simple — it produces
confident numbers about a robot that does not exist.

| behaviour | why it is modelled |
|---|---|
| **No command watchdog** | the real firmware holds a commanded speed indefinitely (measured 45 s). Modelling it means `tools/test_failsafe.py` and `cmd_vel_deadman` can be exercised with no car |
| **Strafe produces exactly zero** | measured; the chassis is differential despite the mecanum-sounding vendor executable |
| **BEST_EFFORT sensor QoS** | a RELIABLE subscriber silently receives nothing. Reproducing it means that trap is hit in simulation rather than on the floor |
| **IMU with no orientation** | 6-axis, no magnetometer. `imu_filter_madgwick` has to supply the attitude, exactly as on the car |
| **`slip` parameter** | `slip:=1.0` is the car on its stand: wheels turn, body still. The case that separates the two EKF configs |

**Proof it is faithful where it matters:** `tools/test_failsafe.py` run against the
simulator reaches the *same verdict* as against the car — no firmware watchdog, both cases
FAIL — and the deadman closes it in both.

---

## What each environment can answer

| question | 2D sim | Isaac | floor |
|---|---|---|---|
| Do the configs load and plugins resolve? | **yes** | yes | yes |
| Is the TF tree complete? | **yes** | yes | yes |
| Does SLAM build a closed map? | **yes** | yes | yes |
| Does Nav2 accept and reach a goal? | **yes** | yes | yes |
| Do lifecycle nodes activate? | **yes** | yes | yes |
| Does the deadman stop a runaway? | **yes** (timing optimistic) | yes | yes |
| Real stopping distance | no | **no** | **yes** |
| Odometry accuracy | no | **no — 1.87× anomaly** | **yes** |
| Whether 8 m range holds on a dark wall | no | partly | **yes** |
| Real latency | **no** | no | **yes** |
| Wheel slip on a dirty floor | only if injected | partly | **yes** |

### The specific numbers not to quote from simulation

- **Latency.** The deadman stops the sim in 435–526 ms and the car in 693–762 ms. The
  ~200 ms gap is the Wi-Fi and micro-ROS agent hop that the simulator does not model. Sim
  timings are **optimistic**, and by roughly that much.
- **Odometry.** The 2D sim has no wheel dynamics at all, so odometry is exact by
  construction unless `slip` is set. Isaac carries an unexplained **1.87× effective wheel
  radius**. Neither can be cited for an accuracy claim.
- **Stopping distance.** There is no physics here — no inertia, no traction limit, no
  motor spin-up. Braking remains a floor measurement and nothing in simulation
  substitutes for it.
- **A map built in simulation is a map of the simulation.** `sim_arena_4x4.pgm` is
  evidence that SLAM runs and its parameters are sane. It is not a map of your room.

---

## Running it

```bash
# The robot, plus the real bringup stack
ros2 launch yahboomcar_sim sim_bringup_launch.py
ros2 launch yahboomcar_sim sim_bringup_launch.py slip:=1.0    # car on its stand

# Mapping (the canonical robot1 SLAM launch, D-19)
ros2 launch yahboomcar_config slam_launch.py rviz:=true
ros2 run nav2_map_server map_saver_cli -f my_map

# Navigation against a saved map
ros2 launch yahboomcar_nav navigation_dwb_launch.py maps:=/abs/path/my_map.yaml
./tools/nav2_smoke.py --x 0.8 --y 0.0
```

The vendor `yahboomcar_nav slam_toolbox_launch.py` still exists in the vendor tree, but
its lifecycle manager bonds with a 30 s timeout, and under fleet-session load that bond
falsely timed out and kill/respawned SLAM mid-session (D-19) — use the canonical launch
above, whose manager runs bond-free.

### Fun mode

```bash test:skip
./tools/simctl start --backend isaac --fun     # then:  ./tools/simctl teleop
```

Obstacle braking OFF (the governor's stop/slow distances go to 0, making the obstacle
rules unreachable), speed caps at the firmware maxima (1.0 m/s forward **and**
reverse, 5.0 rad/s yaw). It is a governor *preset*, not a bypass: the deadman stays
armed, a stale scan still stops the robot, and teleop keeps the single-writer
discipline. The patrol is suppressed — fun is a driving mode. **SLAM runs, so the map
builds while you drive** (`--no-slam` turns it off); on `--backend isaac` the map will
smear on turns, because `/odom_raw` yaw over-reports ~3× under turn slip — an open
finding in [`slam-research/isaac-scan-quality.md`](slam-research/isaac-scan-quality.md).
**A fun-mode map is for looking at. It is never evidence.** Real collisions exist only
on the isaac backend; the 2D simulator has no contact physics.

> SLAM was briefly forced OFF in fun mode (2026-08-10) on the argument that a crash map
> is garbage and its corrections yank the view. That removed the map from the one view
> built for watching yourself drive, and it was reported as "still no map on rviz". The
> view-yanking was really the fixed frame (now `odom`) and the scan filter starving
> `/scan`; both are fixed. A crash map not being *evidence* is a documentation matter,
> which is what the sentence above is. Simulation-only
by construction: `simctl` refuses the car's domain structurally, and the hardware
launch defaults are untouched.

Fun sessions open the **drive view** (`yahboomcar_config/rviz/drive.rviz`): a
chase camera following `base_footprint`, anchored to the `odom` world frame so the
scan stays glued to the world while the camera follows the car (a robot-anchored
fixed frame made the scan swim against the map — measured and reverted). simctl
opens RViz only once `/odom` is flowing, so the view is never red. If `arena_fun.usd` exists, fun isaac
sessions load it — **feather boxes** (0.02 kg vs the calibrated 0.12): the car
punches through at full speed (zone entry measured at 1.05 m/s, penetration to
0.07 m of the box centre). Build it once:

```bash test:skip
~/isaac/env_isaaclab/bin/python tools/build_arena.py --box-mass 0.02 --out arena_fun.usd
```

Calibration results only ever come from the canonical `arena.usd`; the fun variant
lives next to it, never in place of it.

The arena defaults to the planned 4×4 m room with four 0.3 m boxes near the corners,
defined once in `yahboomcar_sim/arena.py` and imported by `tools/arena_observability.py`
AND `tools/build_arena.py` (the Isaac room), so the backends cannot drift apart. They
had: until 2026-08-08 the 2D room carried three mid-room boxes while Isaac had four by
the corners, and no SLAM map could match both expectations at once.

### What a session leaves behind

Since 2026-08-10, `simctl start … stop` records itself — the audit trigger
was a manual fun session whose close-box destabilization could not be
diagnosed afterwards because it left **no bag, no saved map, no `/cmd_vel`
record**, and the next `start` used to overwrite the previous session's
logs (`sh()` wrote flat names with mode `'w'`; the overnight diagnosis
sessions survived only by hand-copying).

Every session now gets one directory, named by its correlation id:

```text
MicroROS-assets/logs/sessions/<stamp>-<backend>[-fun]-d<domain>/
  session.json        flags, git SHA, bag path+sha256, map, duration,
                      health counters (queue-full drops, relay drops,
                      pacing trims, laser-odometry degeneracies)
  events.log          ISO-stamped timeline: start, bag open/close, map
                      save, stop — append-only
  simctl-*.log        every component's log, no longer overwritten
  map-<id>.{yaml,pgm} saved at stop while slam_toolbox is still alive
  lens-history-*.json the SLAM lens's metric ring, filed on lens exit
```

The bag (in `MicroROS-assets/bags/<id>/`, mcap, 1 GB splits) carries the
sensor topics AND `/cmd_vel` + `/cmd_vel_raw` — the command stream is what
lets "the map went weird at t=93 s" be correlated with what was being
commanded at t=93 s. `--no-bag` opts out; recording is skipped (and says
so, and writes it in the manifest) below 5 GB of free disk. Recording is
fail-open everywhere: a recording problem lands in `session.json`'s
`errors` list and never breaks the session it was recording. The flat
`logs/simctl-*.log` names remain valid as symlinks to the newest session.
Counters distinguish "no evidence" (`null`, log absent) from "zero events"
(`0`) — a 2D session reports `relay_dropped_scans: null`, not a fake zero.

### The Isaac session's EKF (default: pn-fix, since 2026-08-10)

Isaac sessions run `yahboomcar_config` `bringup_corrected_launch.py` with
`ekf_sim_pnfix.yaml` by default: wheel-vx + IMU-yaw twist fusion with the
numerically identified process noise (the vendor chain attenuated a clean
25 Hz gyro to 0.727× with a 2.8 s lag; the identified matrix reads
transfer 1.001 / lag 20 ms — full derivation and live A/B in
`docs/slam-research/near-wall-stability.md`). Approved as default by Alex
after live driving. `--ekf vendor` restores the vendor fusion (and its turn
smear) for comparison; `corrected`, `corrected-novyaw` and `n4-pure` remain
as study variants. SIM-ONLY: the 2D backend keeps its bundled vendor
composition (no contact physics, nothing to fix), and the hardware fusion
question is separate — the real gyro is intermittently faulty and any
gyro-leaning fusion there is gated on `sensor_health.py --rotate-window`.

### Watching SLAM properly: the lens

```bash test:skip
./tools/slam_lens.py             # then open http://localhost:8765/  (domain 66)
./tools/slam_lens.py --domain 68 --sim-time    # watching a replay
```

One browser canvas with the map, the scan endpoints at their TF-resolved pose
(colored hit/miss against the map), the SLAM pose, a ground-truth ghost, and a
pure-odometry ghost — plus four live metrics (scan→map fit, pose-vs-truth,
odom/truth yaw ratio, scan staleness, TF@stamp) each tied to a failure this
repo has measured. RViz shows you *a* picture; the lens shows you whether the
sensor, the prior, and the map still agree, which is the question a smeared
map actually poses. Negative-controlled on 2026-08-10 (injected `--slip 0.4`
read 1.666× on the yaw tile — theory says 1.667). Read-only: it subscribes
and looks up TF, publishes nothing, so it can watch any session without being
able to disturb it. KNOWN LIMIT: the stale-scans tile counts bit-identical
messages, which catches 2D-style duplication but NOT Isaac's render-pacing
behavior (measured 2026-08-10: 0/3330 bit-identical). The content-lag tile is
sim-only and must refuse static motion; a parked robot makes time-offset fitting
unobservable. Guarded bag analysis found median −0.04 s and did not demonstrate
material lag. Treat the tile as diagnostic context, not a map-failure verdict.

Manual sessions ARE archived automatically since 2026-08-10 — see "What a
session leaves behind" above. (This paragraph previously warned the opposite:
rolling logs overwritten per start, no bag, no command record. That world is
gone; the warning survived here by accident and contradicted the section
above — caught by Alex's audit. The one thing still worth doing by hand for
a diagnosis-worthy moment: note the wall-clock time, so the session's
events.log and bag can be cut to the right window quickly.)

---

## Two Jazzy breakages found here, that would each have cost a floor session

**1. `slam_toolbox` is a lifecycle node on Jazzy and starts `unconfigured`.** It reads its
parameters, logs its stack size, and then does *nothing* — never subscribes to `/scan`,
never publishes `/map`, and reports no error at all. The tell is `ros2 topic info /scan`
showing zero subscribers. `slam_toolbox_launch.py` therefore starts
`nav2_lifecycle_manager` with `autostart` to drive it through configure and activate.

**2. AMCL will not publish `map→odom` without an initial pose, and that aborts the whole
Nav2 bringup.** The failure surfaces four messages downstream of its cause:

```
amcl:              AMCL cannot publish a pose or update the transform
global_costmap:    Invalid frame ID "map" ... frame does not exist
global_costmap:    Failed to activate ... transform did not become available
lifecycle_manager: Failed to bring up all requested nodes. Aborting bringup.
```

Fixed with `set_initial_pose: true` in `params/dwb_nav_params.yaml`, so bringup is
deterministic instead of depending on someone clicking *2D Pose Estimate* in RViz inside
the activation timeout. This is the **fifth** Jazzy breakage in the vendor Nav2 config,
after the four in [`porting-notes.md`](porting-notes.md).

---

## What was actually verified in simulation

| result | evidence |
|---|---|
| Stack runs unmodified on the sim | bringup, EKF, governor, deadman, `laser_odometry` all started with no flags |
| Sim reproduces the no-watchdog behaviour | `test_failsafe.py` gives the same verdict as the car |
| SLAM builds a closed map | 4.08 × 4.08 m against a 4 × 4 m arena — the dimensions are an independent check on the geometry |
| Map saves and reloads | `sim_arena_4x4.pgm` / `.yaml`, 2 cm resolution |
| Nav2 activates fully | all seven lifecycle nodes `active`, zero aborts |
| **Nav2 reaches a goal** | Navigated, peak 0.26 m/s. **Accuracy RETRACTED** — the 177 mm figure compared an `odom`-frame pose against a `map`-frame goal and ignored the action status. See `tools/nav2_smoke.py`. |
| The smoke test can fail | exits 2 with the reason named when Nav2 is absent |

## The braking tool, dry-run end to end before the floor

`tools/measure_braking.py` had never executed its full chain — calibrate, drive, measure,
subtract the run-up, fit, envelope. Its fit was unit-tested against synthetic numbers, but
a bug anywhere else would have surfaced *on the floor*, spending the session it was meant
to serve.

So the simulator was given a **known** braking response — `decel: 1.5 m/s²`,
`dead_time: 0.25 s`, an explicitly-labelled test fixture in `physics.py`, not a model of
this car — and the whole protocol was run against it with `--sim-tape` substituting
ground truth for a tape measure.

**It found two real bugs and one measurement bias:**

1. **The dead timer re-armed every tick**, so it never expired and the robot never moved.
   The entire first dry run reported 0 mm stopping distances. Caught in the fixture, not
   the tool, but it would have looked exactly like a tool failure.

2. **`integrate()` summed only whole samples inside the window.** `/odom_raw` is 11 Hz, so
   including or excluding one sample at each end costs up to 90 ms of travel — an error
   **proportional to speed**. This one matters on the floor too: odom is 11 Hz there as
   well. Fixed by interpolating at both boundaries.

3. Ground truth was published at the firmware's 11 Hz, adding the same speed-proportional
   error to the "tape". Moved to physics rate — it is a debug channel, not part of the
   contract.

**The lesson worth carrying to the floor:** a linear-in-speed measurement bias is
*indistinguishable from dead time*. It lands in `T_stop` and is stolen from the quadratic
term, so a few millimetres of bias wrecks the deceleration estimate:

| | `T_stop` (truth 250 ms) | `a` (truth 1.5 m/s²) |
|---|---|---|
| whole-sample integration | 297 ms | **0.54** |
| boundary-interpolated | 283 ms | **1.14**, 90% CI [0.93, 1.48] |

Same data, same fit, one integration fix — and `a` moves by more than a factor of two. On
the floor the analogous bias is calibration error in `k`, which multiplies a run-up far
longer than the stop. It is why the calibration gate is strict.

Simulator results live in `braking_runs_sim.json`, never the hardware store.

## Honest limits

- **Nav2 tuned here will need retuning on the floor.** No wheel dynamics, no latency. What
  transfers is that the configuration is *valid* and the plugins resolve — which is worth
  a great deal, because that was four Jazzy breakages ago.
- **The governor and Nav2 have not been reconciled.** `nav2_smoke.py` runs Nav2
  **ungoverned** on purpose: the governor would stop for obstacles Nav2 is planning
  through, and the test would report a Nav2 failure that is really a governor success. It
  reports the peak speed Nav2 commanded so the conflict is visible. On the real floor,
  running Nav2 means running it unprotected — do not, until the stopping envelope is
  measured.
- **AMCL's initial pose is hard-coded to the origin.** Correct for the sim and for a car
  starting where its map was built; wrong if the robot is placed anywhere else.
