# yahboomcar-ros2 — the robot1 first-party stack

The safety layer, localization, simulators, digital-twin bridge, tuned
configuration, diagnostic tooling and measured documentation for **robot1**,
the fleet's Yahboom MicroROS car. The car's ESP32-S3 factory firmware **is**
the robot — a frozen micro-ROS client over WiFi/UDP that cannot be rebuilt —
so everything here runs on the ground station and treats the board as a
fixed, measured contract.

This repo is Alex's authorship only, extracted from the MicroROS working
repo with full history (fleet decision D-12; 63 commits follow every file —
`git log --follow` verified at extraction). Vendor runtime packages and the
course-PDF archive stay in the MicroROS checkout permanently (unlicensed
vendor code, R-05 — never republished here). Licensed MIT (© 2026
Alexander Gomez). Fleet-level decisions live in the robot-fleet repo;
session rules and the full constraint text live in [CLAUDE.md](CLAUDE.md).

**This README is executable.** Every fenced command below is run by
[`tools/test_readme.py`](tools/test_readme.py); blocks that cannot run in CI
say so on their fence tag and in the prose beside them.

## Read this before touching the car

| constraint | consequence |
|---|---|
| **No command watchdog** in the firmware — a commanded speed is retained indefinitely [measured three ways, 2026-08-06] | any crash leaves the car driving; WiFi loss is unmitigable from the ground station. `cmd_vel_deadman` covers publisher death (**693–762 ms, measured on hardware**) and nothing else |
| **The gyro is intermittently faulty** — fails to a confident 0.000000 that any filter would happily fuse | `tools/sensor_health.py --rotate-window` is the only decisive check; run it before every session where yaw matters |
| **6-axis IMU, no magnetometer** | yaw *rate* only — never a heading (fleet D-06) |
| **The vendor EKF believes the wheels** — 2239 mm imagined travel on a stand [measured] vs 6 mm with the corrected config | use `yahboomcar_config/param/ekf_corrected.yaml`; its A/B evidence sits next to it |

Floor sessions: hand on the power switch, always. The full argument is
[docs/safety-case.md](docs/safety-case.md).

## Index

- [What's in the box](#whats-in-the-box)
- [Quickstart (simulation)](#quickstart-simulation)
- [Watching SLAM properly: the lens](#watching-slam-properly-the-lens)
- [Running the real car](#running-the-real-car)
- [The firmware contract](#the-firmware-contract)
- [Tools catalog](#tools-catalog)
- [Documentation index](#documentation-index)
- [Status: delivered and not delivered](#status-delivered-and-not-delivered)
- [Repo relationships and layout](#repo-relationships-and-layout)
- [Conventions](#conventions)
- [License](#license)

## What's in the box

| package | what it does | headline fact |
|---|---|---|
| [`yahboomcar_safety/`](yahboomcar_safety/) | `cmd_vel_governor` (stale-scan stop, stale/non-finite command stop, lateral zeroing, speed caps, obstacle stop/slow zones) + `cmd_vel_deadman`, composable on `/cmd_vel_raw` → `/cmd_vel` | deadman reaction **693–762 ms [measured on hardware]**; fun preset lifts caps to firmware maxima but keeps the deadman armed |
| [`yahboomcar_localization/`](yahboomcar_localization/) | ICP laser scan matcher publishing `/odom_laser` with degeneracy detection (withholds output when the room can't answer); deliberately broadcasts **no TF** — the EKF owns `odom→base_footprint` | **UNVALIDATED on a moving floor robot** — not to be fused into anything safety-bearing until it has been |
| [`yahboomcar_sim/`](yahboomcar_sim/) | 2D firmware-contract simulator: `fake_robot` publishes the exact topic contract, analytic 4×4 m arena shared with Isaac, slip injection (`--slip`) | the stack runs against it **unmodified** — simulation validates the STACK, not the robot |
| [`yahboomcar_twin/`](yahboomcar_twin/) | Isaac Sim visual twin bridge: `/joint_states` from `/odom_raw` + servo topics | wheel angles are visual only, never an odometry source |
| [`yahboomcar_config/`](yahboomcar_config/) | first-party config: the canonical SLAM launch (D-19, `bond_timeout: 0.0`), `ekf_corrected.yaml`, tuned `slam_toolbox.yaml`, RViz views, the authored arena reference geometry | the corrected EKF's stand A/B: vendor **2239 mm** phantom travel vs **6 mm** corrected [measured 2026-08-06] |

## Quickstart (simulation)

The one entry point is `simctl` — it sources the fleet environment itself,
brings up the 2D simulator with the real bringup stack, safety, laser
odometry, the canonical SLAM launch and RViz, and tears it all down in
order (robots are zeroed BEFORE teardown):

```bash test:lifecycle
./tools/simctl start
./tools/simctl stop
```

`simctl start` runs the canonical SLAM launch itself — do **not** launch
`slam_launch.py` again on top: two `/slam_toolbox` nodes fight over
`map→odom` and the map flickers apart.

**Every session records itself.** A start creates a session directory
(`MicroROS-assets/logs/sessions/<id>/` — the id correlates everything), all
component logs land inside it (the flat `logs/simctl-*.log` names stay valid
as symlinks), a capped bag records the sensor AND command topics
(`--no-bag` to opt out; skipped automatically under 5 GB free disk), the map
is saved at stop, and `session.json` carries flags, git SHA, bag checksum
and the session's health counters. See "What a session leaves behind" in
[docs/simulation-guide.md](docs/simulation-guide.md). Inspect a running
session with:

```bash test:sim
./tools/simctl status
```

Drive it yourself (interactive keyboard, shown not run — the vendor keyboard
starts at 0.2 m/s, press `q` to raise it):

```bash test:skip
./tools/simctl teleop
```

Other `simctl` verbs: `patrol` (drive an already-running sim), `estop`
(publish zeros to a domain, explicitly). The Isaac backend is
single-occupancy for the whole machine (two instances can take down the PC),
gated by `/tmp/fleet-isaac.lock`, so it is shown here and deliberately not
run by the README suite:

```bash test:skip
./tools/simctl start --backend isaac --no-isaac-gui
./tools/simctl start --backend isaac --fun     # feather boxes, braking off, caps at firmware maxima
./tools/simctl start --backend isaac --fun --ekf pn-fix   # EXPERIMENTAL fusion, see below
```

`--ekf pn-fix` runs the numerically identified EKF fusion instead of the
vendor config: IMU-owned yaw with corrected process noise (transfer 1.001 /
lag 20 ms vs the vendor chain's 2.8 s lag —
[the study](docs/slam-research/near-wall-stability.md)). Measured live:
worst near-wall map jump 3308 → 236 mm, patrol map at truth-prior quality.
It is NOT the default: it missed the absolute wall-grind gate (residual
instability under 0.3 m standoff, suspected wheel-vx contact lie —
unmeasured), so `vendor` remains default until the morning decision.
Expect visibly stabler turning; still avoid grinding the walls.

Isaac SLAM maps are **not usable evidence** (see
[status](#status-delivered-and-not-delivered)). The full backend comparison,
fun-mode physics, and the two Jazzy breakages that would each have cost a
floor session are in [docs/simulation-guide.md](docs/simulation-guide.md).

The ROS-free test suite (metrics, scorers, gates — no simulator needed):

```bash test:auto
python3 -m pytest tools/tests/ -q
```

## Watching SLAM properly: the lens

RViz shows *a* picture; the lens shows whether the sensor, the odometry
prior and the map still agree — which is the question a smeared map actually
poses. One process, zero build steps, read-only by construction:

```bash test:longrunning
./tools/slam_lens.py
```

Then open <http://localhost:8765/>: the occupancy grid, the scan endpoints
at their TF-resolved pose (colored hit/miss against the map), the SLAM pose,
a ground-truth ghost and a pure-odometry ghost, all in one frame — plus six
live metric tiles (scan→map fit, pose-vs-truth divergence, odom/truth yaw
ratio, duplicate scans, content lag, TF health), each tied to a failure this
repo has measured. Negative-controlled before first use: an injected
`--slip 0.4` read **1.666×** on the yaw tile against a theoretical 1.667.

## Running the real car

Hardware sections are not tested in CI — every command below needs the
robot, an operator within reach of the power switch, and the floor rules in
[docs/first-floor-procedure.md](docs/first-floor-procedure.md). The gyro
check comes first, every boot where yaw matters, because a stationary robot
cannot reveal the fault:

```bash test:hardware
./tools/sensor_health.py --rotate-window
```

Bringup with the corrected EKF (requires the car on `ROS_DOMAIN_ID=20`,
matching the board):

```bash test:hardware
ros2 launch yahboomcar_config bringup_corrected_launch.py
```

Floor SLAM (hand-push walk first, then governed teleop) is a procedure, not
a command: [docs/floor-slam-session.md](docs/floor-slam-session.md), nested
in [docs/first-floor-procedure.md](docs/first-floor-procedure.md), under the
delivery contract in [docs/slam-delivery-plan.md](docs/slam-delivery-plan.md).

## The firmware contract

Board: ESP32-S3, factory firmware = micro-ROS client over WiFi/UDP:8090,
node `/YB_Car_Node`, `ROS_DOMAIN_ID` must match the board (currently 20 —
never a default in any tool here). No camera exists on this robot.

| direction | topic | type | rate |
|---|---|---|---|
| pub | `/odom_raw` | `nav_msgs/Odometry` | 11 Hz |
| pub | `/imu` | `sensor_msgs/Imu` | 25 Hz |
| pub | `/scan` | `sensor_msgs/LaserScan` | 12 Hz |
| pub | `/battery` | `std_msgs/UInt16` (÷10 = volts) | 1 Hz |
| sub | `/cmd_vel` | `geometry_msgs/Twist` | — |
| sub | `/beep`, `/servo_s1`, `/servo_s2` | `UInt16` / `Int32` | — |

Subscribe to sensor topics with BEST_EFFORT sensor QoS — it is safe against
any offer. (The current agent OFFERS RELIABLE on all four sensor topics and
a RELIABLE subscriber received 1826/1827 scans [measured, fleet OI-20]; the
BEST_EFFORT rule stands because the reverse arrangement silently starves.)

## Tools catalog

Every tool self-sources the fleet environment via the single shared
resolver [`tools/_layout.py`](tools/_layout.py). One line each; run any of
them with `--help` for the full story.

**Session drivers**

| tool | one line |
|---|---|
| `simctl` | start/stop/status/teleop/patrol/estop for both sim backends; the session entry point |
| `sim_patrol.py` | governed open-loop laps so there is something to watch (refuses real hardware) |
| `sim_runner.py` | the Isaac backend: firmware contract from an OmniGraph, encoder-honest `/odom_raw` |
| `build_arena.py` | authors the Isaac USD arena from the shared `yahboomcar_sim.arena` constants |
| `urdf_to_usd.py`, `isaac_twin_setup.py`, `isaac_twin_verify.py`, `twin_motion_sequence.py`, `verify_twin.py` | twin construction and its verification chain |

**Instruments and gates**

| tool | one line |
|---|---|
| `slam_lens.py` | live map/scan/pose alignment in a browser with six diagnosis tiles (above) |
| `slam_preflight.py` | gate 2 of the SLAM delivery plan: TF availability at scan stamps, streaks, frames, duplicate publishers |
| `score_slam_map.py` | gate 6: map usefulness vs a written-first reference; rejects the 14–18-pixel map that once shipped as a success |
| `replay_slam_bag.py` | gate 7: same-bag A/B replays so parameter comparisons are bit-identical-fair |
| `sensor_health.py` | liveness/variance/rate per sensor; `--rotate-window` is THE gyro-fault check |
| `sensor_agreement.py`, `check_odom_vs_imu.py`, `check_odom_properties.py` | cross-sensor consistency (the odometry yaw lie is measured by these) |
| `scan_characterize.py` | passive `/scan` fingerprint: rates, geometry, per-angle validity, QoS |
| `check_isaac_contract.py` | measures a running simulator against the firmware contract from outside |
| `check_rviz_render.py` | proves a display is actually drawn, by pixels, with its own negative controls |
| `check_body_moved.py`, `arena_observability.py`, `lidar_view.py`, `measure_latency.py` | smaller lenses on motion, arena coverage, scans and transport |

**Bench and calibration** — `umbmark.py`, `ekf_ab_test.py`,
`measure_braking.py`, `bench_motion.py`, `car_selftest.py`,
`test_failsafe.py`, `test_launch_preflight.py`, `nav2_smoke.py`.

**Hardware provisioning** (hardware only, never run unattended) —
`provision_board.py`, `board_reset.py`.

**Shared internals** — `_layout.py` (the one path resolver),
`_cmd_vel_safety.py` (every driving tool's hardware refusal),
`_dds_shm.py` (one definition of "stale" shared memory),
`_scan_frame_relay.py` / `_scan_rate_probe.py` (Isaac scan hygiene),
`_slam_lens_core.py` (the lens's ROS-free metric core), `_tf_source.py`,
`_bench_driver.py`.

## Documentation index

| doc | the question it answers |
|---|---|
| [safety-case.md](docs/safety-case.md) | why the safety layer is shaped the way it is, with the measurements |
| [slam-delivery-plan.md](docs/slam-delivery-plan.md) | **binding**: what counts as delivered physical SLAM, the gates, the forbidden claims |
| [slam-research/](docs/slam-research/) | the SLAM evidence trail — `findings.md` (2D tuning), `isaac-scan-quality.md` (the smeared-map convictions, including the render-pacing runaway) |
| [slam-runs/](docs/slam-runs/) | one report per SLAM attempt, failed attempts kept; reference geometries; the run template |
| [simulation-guide.md](docs/simulation-guide.md) | what each backend can and cannot answer, fun mode, the lens, Jazzy traps |
| [sensor-fusion-research.md](docs/sensor-fusion-research.md) | why the corrected EKF fuses what it fuses |
| [odometry-calibration.md](docs/odometry-calibration.md) | UMBmark on this chassis |
| [rviz-guide.md](docs/rviz-guide.md) | the RViz signatures that look like bugs and aren't |
| [first-floor-procedure.md](docs/first-floor-procedure.md), [floor-slam-session.md](docs/floor-slam-session.md) | the operator procedures for the real car |
| [bench-report-20260808.md](docs/bench-report-20260808.md) | the hardware bench evidence |
| [handoff-*.md](docs/) | session handoffs — the running record of what landed, what failed, what is parked |
| [research-log.md](docs/research-log.md), [handoff-audit.md](docs/handoff-audit.md) | the long log, and the ledger of retracted claims |

## Status: delivered and not delivered

- **Physical SLAM is NOT DELIVERED.** No physical map exists. A green RViz,
  an active lifecycle node, a `/map` publisher or a simulator map is not
  acceptance evidence ([docs/slam-delivery-plan.md](docs/slam-delivery-plan.md)
  is binding; final SLAM handoffs lead with `DELIVERED` / `NOT DELIVERED` /
  `BLOCKED ON HARDWARE`).
- **Simulation validates the stack, not the robot.** The stack runs
  unmodified against `yahboomcar_sim`; that is the claim it supports.
- **Isaac SLAM maps are not usable evidence** (OPEN): `/odom_raw` yaw runs
  ~2.8–3.0× true under turn slip and the map smears on the first hard turn.
  Proven by two-run same-bag replay: as-recorded maps FAIL (≈7×7 m for the
  4×4 room), truth-prior replays PASS every scorer row (4.16/4.14 m, IoU
  0.776) — [docs/slam-runs/isaac-fun-20260810.md](docs/slam-runs/isaac-fun-20260810.md).
- **A fun-mode map is never a delivery artifact**, by contract.

## Repo relationships and layout

```text
robot-fleet/                     the fleet repo (decisions D-nn, OI-nn, R-nn)
├── ground_station/install/     THE build/run environment (A2.3)
└── src/
    ├── yahboomcar-ros2/         THIS repo — first-party only
    └── MicroROS -> ../MicroROS  vendor runtime + course archive (R-05, read-only)
                                 also MicroROS-assets/ (bags, logs, maps, USD)
```

Assets and the twin USD are resolved via `MICROROS_ASSETS` /
`YAHBOOM_USD_DIR` or the fleet-layout default — never copied into this repo.
Domain conventions: **20** hardware (never a tool default), **66** simulator,
**68** replays, scratch domains per concurrent session.

## Conventions

- **Evidence style**: measured vs assumed is always marked; negative results
  are stated in **bold**; rejected alternatives are recorded, not deleted.
  Fleet ids (D-nn decisions, OI-nn open items, R-nn rejects) apply here.
- **This README is a test fixture**: `./tools/test_readme.py` extracts and
  runs every fenced command (`--list` to preview, `--coverage` for the
  summary). An untagged command block fails the suite — silence is how
  untested commands get into documentation.
- [CLAUDE.md](CLAUDE.md) is the authority for session rules (including the
  unattended-session hard rules) and the full constraint text.

## License

MIT — see [LICENSE](LICENSE). © 2026 Alexander Gomez. The vendor's unlicensed
code is deliberately absent from this repo (R-05); do not add it.
