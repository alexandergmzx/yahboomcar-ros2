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

No camera exists on this robot. BEST_EFFORT sensor QoS — a RELIABLE
subscriber silently receives nothing.

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

Tools are extraction-aware (see the header block in `tools/simctl`): the
fleet env is `ground_station/install` (A2.3); **MicroROS-assets** (bags,
logs, firmware) stays in the MicroROS checkout, resolved via
`MICROROS_ASSETS` env var or the `../MicroROS` fleet-layout default —
assets are never extracted.

## Quickstart (fleet layout)

```bash test:lifecycle
cd ../../ground_station && source /opt/ros/jazzy/setup.bash && source install/setup.bash
../src/yahboomcar-ros2/tools/simctl start        # 2D sim + real stack
ros2 launch yahboomcar_config slam_launch.py     # canonical robot1 SLAM (D-19)
../src/yahboomcar-ros2/tools/simctl stop         # zeroes robots BEFORE teardown
```

Evidence style is this repo's export: measured vs assumed marked, negative
results in bold, rejected alternatives recorded. The fleet added bracket
tags and D/OI/R ids; both conventions apply here going forward.
