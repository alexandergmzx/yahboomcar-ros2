# Floor SLAM session — live lidar + growing map in RViz, real car

The goal: you move with the car on the floor, and RViz shows what the lidar sees around
it (the LaserScan display, live) and the map SLAM is building behind it. Two sessions,
in this order:

- **Session A — hand-push mapping walk.** You push the car; NO motor commands exist
  anywhere. Zero kinetic risk, and it already delivers the whole "what is near the
  robot" view. Can be run today.
- **Session B — governed keyboard teleop.** The car drives; you walk with it. This one
  is nested inside [`first-floor-procedure.md`](first-floor-procedure.md), because
  mapping needs yaw, and yaw on the floor is earned through that procedure's staging
  table — it is not enabled by this document.

What is already measured, so this runbook composes rather than hopes:

- SLAM against the real sensor stream is proven: real bringup +
  `yahboomcar_config slam_launch.py`, 300 s, **0 lifecycle flaps**, 296 map updates
  ([`bench-report-20260808.md`](bench-report-20260808.md) §6 — car static; motion is
  what these sessions add).
- The deadman covers a dying publisher (693–762 ms, measured). **WiFi loss is
  unmitigable** — the firmware retains the last command indefinitely. Hand on the power
  switch whenever motors are enabled; that is the actual stop.

---

## Preflight (passive — car powered, on its spot; nothing moves)

Run from the fleet layout, every terminal:

```bash
cd ../../ground_station
source /opt/ros/jazzy/setup.bash && source install/setup.bash
export ROS_DOMAIN_ID=20
export FASTDDS_DEFAULT_PROFILES_FILE=$PWD/fastdds_udp_only.xml   # bench §5: removes
                                       # shm stale-segment blinding at zero measured cost
```

1. **Agent up, exactly one:** `docker ps | grep micro-ros-agent` — if missing:
   `docker run -d --net=host microros/micro-ros-agent:jazzy udp4 --port 8090 -v4`
2. **Sensors alive, not just present:** `./tools/car_selftest.py --sensors-only`
   (subscribes only; PASS/FAIL table against the firmware contract).
3. **The gyro, decisively:** `./tools/sensor_health.py --rotate-test` — rotate the car
   by hand when asked (`--rotate-window 20` is the non-interactive variant). **A FAULT
   aborts the session**: the gyro fails to a confident 0.000000, the EKF fuses it as
   "not rotating", and every turn you make smears the map. The fault is intermittent
   (3 of 5 informative runs), which is why this runs before *every* session, not once.
4. **Battery ≥ 7.4 V:** `ros2 topic echo /battery --once` ÷ 10
   (staging table in the first-floor procedure).

## Stack (one command per terminal, all on domain 20)

```bash
ros2 launch yahboomcar_bringup yahboomcar_bringup_launch.py   # madgwick + EKF + TF + description
ros2 run yahboomcar_localization laser_odometry               # the room's own opinion of motion
ros2 launch yahboomcar_safety safety_launch.py                # governor + deadman
ros2 launch yahboomcar_config slam_launch.py rviz:=true       # SLAM (bond-free, D-19) + the view
```

`laser_odometry` is not decoration: `slam_debug.rviz` shows `/odom` (EKF, yellow) and
`/odom_laser` (lidar, cyan) together, and the two diverging is the most informative
thing this robot can tell you — it is wheel slip happening live. On the stand the wheels
claimed 1.602 m while the lidar saw 0.128 m.

Safety runs even in Session A: the governor and deadman are inert without a command
source, and their startup logs double as the preflight's `NO DEADMAN` / `BYPASSED`
checks.

In RViz, everything green means: RobotModel from the description, scan points around
the car, map growing where the scan has been. If nothing renders and the TF display
complains about `map` — that frame is SLAM's output; give it a few seconds of scans, or
set Fixed Frame to `odom` to separate "robot broken" from "SLAM not started"
([`rviz-guide.md`](rviz-guide.md), failure signature 1).

## Session A — the hand-push mapping walk

No teleop, no patrol, nothing publishing commands — verify with the governor log
(`BYPASSED` names any rogue publisher). Then:

- Push slowly and **turn gently**. The scan is 12 Hz; a fast hand-spin outruns what the
  matcher can track, and a hand-spin is exactly the motion the intermittent gyro makes
  worst. If the map jumps wildly on a turn, stop, and re-run the gyro check.
- Watch the LaserScan display for "what is near the robot" — 0.5 s decay, so the cloud
  trails and a `map->odom` correction is visible as one coherent step
  ([`rviz-guide.md`](rviz-guide.md), signature 3, measured up to 125 mm).
- Pushing rolls the wheels, so wheel odometry stays live — this is a real odom+scan
  SLAM session, not scan-matching-only.

Save anything worth keeping before teardown:

```bash
ros2 run nav2_map_server map_saver_cli -f floor-walk-$(date +%Y%m%d)
```

## Session B — governed teleop (after the first-floor gates)

Prerequisites, from [`first-floor-procedure.md`](first-floor-procedure.md): the full
"Before the car touches the floor" checklist, and the staging table at least through the
step that re-enables yaw — mapping is turning, and turning on the floor is earned there.

```bash
ros2 launch yahboomcar_safety safe_teleop_launch.py max_speed:=0.05
```

The vendor keyboard (u i o / j k l), remapped onto `/cmd_vel_raw` so the governor
speed-limits every command against `/scan` before the firmware sees it. The keyboard
needs its own tty; if keys do nothing, use the split-terminal fallback in the launch
docstring. Raise `max_speed` only per the staging table.

- One command source at a time. Kill the patrol/behaviour layers before this; the
  governor's `BYPASSED` log is the witness.
- **Hand on the power switch.** Walk with the car. The deadman covers a crashed
  keyboard; the switch covers everything else.

## Stop sequence (both sessions)

1. Release keys / stop pushing; kill the teleop (Ctrl+C in its terminal).
2. The deadman zeroes `/cmd_vel` within ~0.5 s of silence; confirm at rest:
   `ros2 topic echo /odom_raw --once` → `twist.twist.linear.x` ≈ 0.
3. Save the map (above), then Ctrl+C the launches. The car's agent and node stay up —
   they belong to the board.
4. Power switch off. (Big hammer at any point: `./tools/simctl stop` also zeroes the
   car's domain, by design, without killing its agent.)

## Abort immediately if

The first-floor procedure's list applies verbatim (zero-command ignored ≈ 1 s, governor
or deadman logs stop, `/scan` under ~6 Hz, uncommanded motion, anyone enters the area),
plus one SLAM-specific trigger: **the map jumps wildly mid-walk** — that is the
intermittent gyro failing confident-flat mid-session. Stop, re-run
`sensor_health.py --rotate-test`, and distrust everything mapped after the jump.

Abort means **power switch**, not Ctrl+C.
