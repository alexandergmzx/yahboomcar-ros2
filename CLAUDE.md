# CLAUDE.md

Yahboom MicroROS car (robot1) — the FIRST-PARTY stack, extracted with full
history from the MicroROS working repo (fleet D-12, session 4).

**The relationship:** vendor runtime packages (bringup, description, ctrl,
nav, base_node, multi, …) and the course-PDF archive live in the
**MicroROS checkout** (`../MicroROS` from this repo's fleet position),
served into the fleet workspace via the farm, permanently — they are
unlicensed vendor code (R-05) and are never republished here. THIS repo is
Alex's authorship only: safety, localization, the simulators, the tools,
the measured docs, and `yahboomcar_config` (first-party config + robot1's
canonical SLAM launch, D-19). Archive-specific guidance (PDF→Jazzy command
tables, MicroROS-assets fetching) stays in the MicroROS CLAUDE.md.

## The firmware contract (the board is the robot)

Board: ESP32-S3, factory firmware = micro-ROS client over WiFi/UDP:8090,
node `/YB_Car_Node`. **It cannot be rebuilt** (vendor ships .bin only) and
**it has NO command watchdog — a commanded speed is retained indefinitely
[measured three ways, 2026-08-06]**.

| Direction | Topic | Type | Rate |
|---|---|---|---|
| pub | `/odom_raw` | `nav_msgs/Odometry` | 11 Hz |
| pub | `/imu` | `sensor_msgs/Imu` | 25 Hz |
| pub | `/scan` | `sensor_msgs/LaserScan` | 12 Hz |
| pub | `/battery` | `std_msgs/UInt16` (÷10 = volts) | 1 Hz |
| sub | `/cmd_vel` | `geometry_msgs/Twist` | — |
| sub | `/beep`, `/servo_s1`, `/servo_s2` | UInt16 / Int32 | — |

No camera exists on this robot. Subscribe to sensor topics with BEST_EFFORT
sensor QoS (safe against any offer). The old claim that a RELIABLE subscriber
silently starves **did not reproduce on 2026-08-08**: the current agent OFFERS
RELIABLE/VOLATILE on all four sensor topics and a RELIABLE subscriber received
1826/1827 scans [measured; fleet OI-20].

## Hard constraints (the safety-relevant core; full text in docs/)

- **No command watchdog** ⇒ any crash leaves the car driving; WiFi loss is
  unmitigable from the ground station. `cmd_vel_deadman` covers publisher
  death (**693–762 ms, measured on hardware**) and nothing else. Floor
  sessions: hand on the power switch, always.
- **The gyro is INTERMITTENTLY FAULTY** — fails to a confident 0.000000
  that any filter would fuse. `tools/sensor_health.py --rotate-window` is
  the only decisive check; run it before every session where yaw matters.
- **The vendor EKF believes the wheels** (2239 mm imagined travel on a
  stand [measured] vs 6 mm with `yahboomcar_config/param/ekf_corrected.yaml`).
  The corrected config is the fleet's; its A/B evidence sits next to it.
- **6-axis IMU, no magnetometer** — yaw rate only, never a heading (the
  decision robot2/robot3 inherit as D-06).
- The stack runs unmodified against `yahboomcar_sim` — simulation
  validates the STACK, not the robot (`docs/simulation-guide.md`).
- `ROS_DOMAIN_ID` must match the board (currently 20; fleet A2.4 marks it
  transitional).

## Layout (post-extraction)

```
yahboomcar_safety/        governor + deadman (measured on hardware)
yahboomcar_localization/  ICP scan matcher w/ degeneracy (fleet odometry source)
yahboomcar_sim/           firmware-contract 2D simulator
yahboomcar_twin/          Isaac digital-twin bridge
yahboomcar_config/        first-party config + robot1 SLAM launch (D-12/D-19)
tools/                    simctl, sensor_health, test_failsafe, provisioning, …
docs/                     safety-case, sensor-fusion-research, slam-research, …
```

Tools are extraction-aware through ONE shared resolver, `tools/_layout.py`
(added 2026-08-08 after the Isaac repair found 18 tools still carrying
private stale copies of the old `yahboomcar_ws` paths): the fleet env is
`ground_station/install` (A2.3); **MicroROS-assets** (bags, logs, firmware)
and the twin **USD** stay in the MicroROS checkout, resolved via
`MICROROS_ASSETS` / `YAHBOOM_USD_DIR` env vars or the `../MicroROS`
fleet-layout default — never extracted (R-05).

## Quickstart (fleet layout)

```bash test:lifecycle
cd ../../ground_station && source /opt/ros/jazzy/setup.bash && source install/setup.bash
../src/yahboomcar-ros2/tools/simctl start        # 2D sim + real stack + canonical SLAM (D-19)
../src/yahboomcar-ros2/tools/simctl stop         # zeroes robots BEFORE teardown
```

`simctl start` runs the canonical `yahboomcar_config slam_launch.py` itself —
do NOT launch it again on top (two `/slam_toolbox` nodes fight over
`map->odom` and the RViz map flickers apart). To run SLAM by hand:
`simctl start --no-slam`, then `ros2 launch yahboomcar_config slam_launch.py
rviz:=true`.

SLAM on the REAL car (hand-push walk first, then governed teleop):
`docs/floor-slam-session.md`, nested in `docs/first-floor-procedure.md`.

## Physical SLAM is NOT delivered — read the contract before tuning

`docs/slam-delivery-plan.md` is binding. **No physical map exists yet.** A green RViz,
an active lifecycle node, a `/map` publisher, a simulator map, or a parameter that
loaded is not acceptance evidence, and the previous pass was rejected for presenting
exactly those. Before changing a SLAM parameter, run the gates in that document, in
order; the tooling for them is `tools/slam_preflight.py` (TF/timing, gate 2),
`tools/score_slam_map.py` (map usefulness, gate 6 — it rejects the 14–18-pixel map that
shipped before), and `tools/replay_slam_bag.py` (same-bag A/B, gate 7). Run reports go
in `docs/slam-runs/`, failed attempts included. Final handoffs on SLAM lead with
`DELIVERED`, `NOT DELIVERED`, or `BLOCKED ON HARDWARE`.

Evidence style is this repo's export: measured vs assumed marked, negative
results in bold, rejected alternatives recorded. The fleet added bracket
tags and D/OI/R ids; both conventions apply here going forward.

## Unattended sessions (operator asleep/away) — hard rules

These bind ANY session running without an operator who can answer. If unsure
whether a rule applies: it applies. They rank above task completion — a task
finished by breaking one of these is a failed task.

- **git is local-only tonight. `git push` does not exist.** No pushes, no PRs,
  no remote branch creation, no fetching-and-merging. Remotes are
  human-reviewed surfaces; nothing unreviewed leaves this machine. Morning
  review decides what publishes.
- **History is append-only.** New branch per session (`<purpose>-<date>`),
  one concern per commit, finding/OI IDs in messages. Never amend, rebase,
  `reset --hard`, `clean -fd`, or delete branches. A wrong commit is repaired
  by a new commit that says it repairs it.
- **A commit is a reliable checkpoint or it doesn't happen.**
  `colcon build --packages-select <touched>` plus the touched packages' tests
  green BEFORE each commit. Work that can't reach green stays uncommitted in
  the tree and is reported in the handoff doc — never committed "to save it".
  Session ends with a clean tree or a documented dirty one, nothing silent.
- **Isaac Sim is single-occupancy, machine-wide.** Two instances can take
  down the whole PC — killing every other session's work, not just yours.
  Before any `simctl start --backend isaac`: acquire `/tmp/fleet-isaac.lock`
  (write PID + session name; a lock whose PID is dead is stale and may be
  removed) AND verify no kit/isaac process is running. If busy: poll every
  5 min for max 45 min, then PARK every isaac-dependent task and continue
  with what doesn't need the GPU. Never launch a second instance to "check".
  On session end and on EVERY failure path: `simctl stop`, verify the
  process actually died, release the lock. Orphaned kit processes hold GPU
  memory for the next victim.
- **Domain hygiene**: scratch ROS_DOMAIN_ID per concurrent session (67/69
  convention). Domain 20 is the hardware fleet domain — never used unattended.
- **No hardware while unattended.** No flashing, no serial, no GPIO, nothing
  past `--dry-run`. Hardware requires the operator's hands within reach of
  the power switch (floor rules, D-08 consequence).
- **Resource check before long jobs.** Free disk before bag recording (cap
  and split bags — an unbounded bag fills the disk by 4am); free RAM before
  sim bringup (the 3-robot stack is ~1.75 GB [measured]).
- **Bounded retries.** The same command failing twice for the same reason
  closes that path: record it, move on. No retry loops.
- **Park, don't decide.** Judgment calls (safety semantics, OI status,
  anything touching D-nn/R-nn, preference questions) go to the handoff doc's
  "morning decisions" list. Evidence tags and OI-close rules apply at
  3am exactly as at 3pm.
- **The handoff doc is the one mandatory deliverable** — written even when,
  especially when, the session fails early.