# Safety case — what is protected, what is not

**This is not a safety-rated system.** It is a lidar speed limiter on *some* command
paths. This document exists so that partial coverage is stated rather than implied,
because the dangerous failure here is not the gap itself — it is believing the gap is
closed.

Written after an external audit found the governor was bypassed by everything in the
repository, including my own test tools.

---

## Protected paths

Commands here pass through `cmd_vel_governor`, which limits them against `/scan` before
they reach the firmware.

| Path | How |
|---|---|
| `tools/car_selftest.py` | publishes `/cmd_vel_raw` by default |
| `tools/twin_motion_sequence.py` | publishes `/cmd_vel_raw` by default |
| Keyboard teleop via `ros2 launch yahboomcar_safety safe_teleop_launch.py` | vendor node remapped to `/cmd_vel_raw` |

Both tools accept `--direct` to bypass deliberately. That flag exists for an elevated
bench with no governor running; it is not for floor use.

## NOT protected

These publish straight to `/cmd_vel` and reach the motors with **no obstacle limiting
whatsoever**. This is a deliberate scope decision — the course PDFs stay correct — not an
oversight.

| Path | Note |
|---|---|
| `ros2 run yahboomcar_ctrl yahboom_keyboard` | the form documented in the PDFs |
| `ros2 run yahboomcar_ctrl yahboom_joy` | joystick teleop |
| `calibrate_linear`, `calibrate_angular` | drive 1 m / rotate on command |
| `laser_Avoidance`, `laser_Tracker`, `laser_Warning` | vendor autonomous behaviours |
| Nav2 (`navigation_dwb_launch.py`) | full autonomous navigation |
| Anything else copied from the course PDFs | by design |

The governor logs an **ERROR naming any node** it sees publishing to `/cmd_vel`, so a
bypass is visible in the log rather than silent. Watch for it.

To protect a vendor node ad hoc:

```bash
ros2 run <pkg> <node> --ros-args -r /cmd_vel:=/cmd_vel_raw
```

## Limits that remain even on protected paths

These are properties of the design, not bugs, and they bound what the governor can claim:

- **Forward sector only.** The governor watches ±45° ahead. **Nothing guards the rear**,
  and reverse is deliberately unrestricted.
- **System response time has been measured ELEVATED, which makes it a LOWER BOUND, not a
  conservative value.** Two runs of `tools/measure_latency.py` on the live car:

  | term | run A | run B | transfers to the floor? |
  |---|---|---|---|
  | scan interval p95 | 147 ms | 113 ms | **yes** — sensor + radio, no load |
  | governor loop | 50 ms | 50 ms | **yes** — software |
  | command→motion p95 | 247 ms | 230 ms | **no** — contains unloaded spin-up |
  | **T (p95)** | **444 ms** | **393 ms** | lower bound only |

  At 0.30 m/s, run B's T is 118 mm of travel before braking begins. The two runs differ
  by 51 ms almost entirely in the scan term, which is the link variability already
  documented below.

- **The wheels were off the ground, so they spun up unloaded.** `/odom_raw` is
  encoder-derived, so it registers motion as soon as the wheels turn — not as soon as the
  *robot* moves. On the floor the motors must first overcome static friction, rolling
  resistance and the chassis inertia, so command→motion takes **longer** there.
  An earlier version of this document claimed spin-up made the figure conservative. That
  was backwards, and it was the safety-relevant direction to get wrong.

- **A threshold sweep separates the load-independent part.** Timing to reach 10/25/50/75%
  of commanded speed gave 219/230/274/326 ms — a 107 ms spread, well above the 91 ms
  `/odom_raw` sampling interval, so spin-up is genuinely visible rather than lost in
  quantisation. Extrapolating to zero threshold gives **~195 ms of comms + firmware**,
  which *is* load-independent and does carry to the floor. The remaining ~35 ms at the
  25% threshold is unloaded spin-up, and that is the term that grows.

  The 195 ms intercept still contains up to one sampling interval of quantisation, so the
  true comms+firmware delay is bounded between roughly **104 and 195 ms**.

- **Reaction still dominates braking, and the floor can only widen the gap.** The braking
  term at 0.30 m/s is 45 mm even at a pessimistic `a = 1.0 m/s²` and 11 mm at `a = 4.0`,
  against ≥118 mm of reaction. A 4× error in `a` moves the total by ~34 mm.
  **The latency is the safety problem; the brakes are not.**

- **Deceleration `a` is still unmeasured** — it needs floor space. The simulated figure
  (~0.39 m from a wall at 0.25 m/s) came from a simulated 0.355 kg mass and a friction
  coefficient I chose, and predicts nothing about the real robot.

- **What was timed is command→motion STARTS. Safety depends on command→motion STOPS.**
  These are different quantities and both are load-dependent, but not in the same
  direction: load *opposes* starting and *assists* stopping. So the start figure is not a
  usable proxy for the stop figure in either direction.

  Phase 2 sidesteps the decomposition entirely. Tape-measuring total stopping distance at
  several speeds and fitting `d = v·T_stop + v²/(2a) + C` separates both parameters at
  once — `T_stop` from the linear term, `a` from the quadratic — and measures them under
  exactly the load that matters. No elevated figure is carried forward.
- **The taper creeps rather than hard-stopping.** Speed scales linearly to zero *at*
  `stop_distance`, so a steady approach decelerates asymptotically and may never command
  an exact zero. It stops short of contact — measured at 0.394 m in simulation — but a
  fast intrusion is a different case and is not characterised.
- **It trusts `/scan`.** A lidar reporting confidently wrong ranges defeats it entirely.
  Stale or empty scans stop the robot; *wrong* scans do not.
- **The radio link has been unreliable.** Rates have been observed between one third and
  full spec. A stalled link means stale scans, which stops the robot — the fail-closed
  path — but it also means teleop stops responding.
- **No authorisation boundary.** There is no SROS2 keystore or enclave configuration.
  Any ROS 2 participant reachable on the same domain can publish `/cmd_vel` and drive the
  car. Audit finding #7, open.

## What has actually been tested

| Claim | Evidence |
|---|---|
| Governor logic is correct | 26 unit tests, incl. stale scan, all-NaN scan, sector edges, rear obstacles |
| It limits real commands with real lidar | bench, live car: 0.39 m ahead → 0.25 m/s request throttled to **0.022 m/s** |
| Stale input stops the robot | bench: output went to 0.000 after commands ceased |
| Bypass is detected and named | bench: governor logged the offending node |
| It stops before contact | **simulation only**, 0.394 m from a wall |
| Scan interval, governor loop | measured, live car, 2 runs; load-independent so they transfer |
| Command→motion latency | measured **elevated only**; a lower bound on the floor value |
| Comms + firmware delay | bounded to 104–195 ms by a threshold sweep |
| Braking response / deceleration `a` | **NOT MEASURED** |
| Real stopping distance | **NOT MEASURED** |

## Before driving on the floor

1. Measure real braking distance at several speeds with a tape measure.
2. Decide whether the unprotected vendor paths are acceptable, or remap them too.
3. Keep the car's lidar clear — a blocked scanner is the failure the governor cannot see
   past, since it stops on *stale* data but trusts *wrong* data.
