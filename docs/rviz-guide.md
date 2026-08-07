# RViz for this robot: which display answers which question

```bash
rviz2 -d yahboomcar_ws/src/yahboomcar_nav/rviz/nav2_debug.rviz
# or:  ros2 launch yahboomcar_nav slam_toolbox_launch.py rviz:=true
```

RViz shows a great deal and explains none of it. This is a map from *symptom* to *cause*
for the failures this stack actually produces.

---

## The displays, and why each is there

| display | topic | the question it answers |
|---|---|---|
| **TF** | `/tf` | is the frame chain complete? First thing to break, last thing checked |
| **LaserScan** | `/scan` | is the lidar alive, and does the room look right? |
| **Map** | `/map` | has SLAM/AMCL got a map, and is it the right size? |
| **RobotModel** | `/robot_description` | is the URDF loaded and are the joints sane? |
| **Local/Global Costmap** | `…/costmap` | does Nav2 believe there are obstacles where there are? |
| **Global/Local Plan** | `/plan`, `/local_plan` | did the planner produce a path, and is the controller following it? |
| **Odometry (EKF)** | `/odom` | where the fused estimate thinks the robot is |
| **Odometry (lidar)** | `/odom_laser` | where the *room* says the robot is |

Covariance rendering is deliberately **off** for `/odom_laser`. The node inflates its
covariance along whichever direction the scan geometry fails to constrain, so switching
the ellipse on in a poorly conditioned spot draws a metres-wide blob over everything. The
covariance is still published and still used by the EKF — it is simply not a thing to
draw on top of a 4 m room.

### The pair worth watching, specific to this robot

`/odom` and `/odom_laser` are shown together — yellow and cyan — because **they disagreeing
is the most informative thing this robot can tell you.** `/odom` is the EKF's fused
estimate; `/odom_laser` comes from matching scans against the room. When the wheels slip,
the yellow trail runs away and the cyan one stays put.

On the stand that divergence is total: the wheels claimed **1.602 m** while the lidar saw
**0.128 m**. Watching them separate on the floor is watching slip happen live, and it is
the one failure the encoders cannot report.

---

## Three failure signatures worth recognising on sight

### 1. Nothing renders, "Fixed Frame [map] does not exist"

The `map` frame is published by SLAM or AMCL. If neither is running or activated, it does
not exist and RViz shows an empty world.

**Check:** set *Fixed Frame* to `odom`. If the robot and scan appear, the robot is fine and
the problem is upstream in SLAM/AMCL — not in the sensor, which is where people look next.

Two specific causes on Jazzy, both documented in
[`simulation-guide.md`](simulation-guide.md):

- `slam_toolbox` starts **unconfigured** and silently does nothing. `ros2 lifecycle get
  /slam_toolbox` should say `active`.
- AMCL will not publish `map→odom` without an initial pose, which aborts the whole Nav2
  bringup. Fixed with `set_initial_pose` in the params.

### 2. LaserScan display is empty but `ros2 topic hz /scan` shows data

**QoS.** The firmware publishes `/scan` BEST_EFFORT, and a RELIABLE subscriber receives
**nothing at all, with no error anywhere**. The bundled config already sets *Reliability
Policy: Best Effort* on the LaserScan display; if you build one from scratch, this is the
trap.

The same trap bites any node you write. It is why every tool in this repo uses
`qos_profile_sensor_data` for sensor topics.

### 3. The scan "resets" or jumps, most obviously at corners

It is not the scan. Measured on the simulator: `/scan` holds **12.0 Hz with 360/360 valid
returns** straight through the turns, never dropping a message.

What moves is **`map→odom`** — SLAM's correction for accumulated odometry error —
measured jumping by up to **125 mm** (mean 3.8 mm). RViz draws the scan in the `map`
fixed frame, so when SLAM revises that transform, everything in that frame steps at once
and it reads as the scan resetting.

**Corners are where it is worst** because rotation is where odometry error accumulates
fastest, and a small yaw error puts distant scan points a long way out. A 125 mm jump at
2 m range is about 3.6° of yaw correction — an entirely ordinary amount to discover after
a 90° turn.

**Confirm it in one move: set *Fixed Frame* to `odom`.** The scan will go smooth, because
`odom` is continuous by construction, and the *map* will jump instead. If the scan is
still ragged in `odom`, then the problem really is the sensor.

### 4. The map drifts away from the scan, or jumps

The `map→odom` transform is SLAM's correction for accumulated odometry error, so it moves
whenever SLAM decides the wheels have lied. A little continuous motion is normal; a jump
means a loop closure just fired.

**Sustained drift in one direction means odometry is systematically wrong** — which on
this robot usually means slip. Confirm with `/odom` vs `/odom_laser`: if the yellow trail
is longer than the cyan one, the wheels are over-reporting.

---

## Getting a robot in front of it

No floor required:

```bash
ros2 launch yahboomcar_sim sim_bringup_launch.py     # simulated robot + real stack
ros2 launch yahboomcar_nav slam_toolbox_launch.py rviz:=true
```

On the real car, replace the first line with `yahboomcar_bringup_launch.py` and read
[`first-floor-procedure.md`](first-floor-procedure.md) first — the firmware has no command
watchdog, and RViz will not save you from that.

## Driving from RViz

**2D Pose Estimate** publishes `/initialpose` (AMCL only; meaningless while SLAM is
running). **2D Goal Pose** publishes `/goal_pose`, which `bt_navigator` acts on.

⚠️ A goal sent from RViz goes to Nav2, and **Nav2 publishes straight to `/cmd_vel`,
bypassing the safety governor entirely** — see [`safety-case.md`](safety-case.md). In
simulation that is fine. On the floor it means clicking in RViz drives an unprotected
robot.
