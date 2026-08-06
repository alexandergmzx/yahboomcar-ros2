# Safety case — what is protected, what is not

**This is not a safety-rated system.** It is a lidar speed limiter on *some* command
paths. This document exists so that partial coverage is stated rather than implied,
because the dangerous failure here is not the gap itself — it is believing the gap is
closed.

Written after an external audit found the governor was bypassed by everything in the
repository, including my own test tools.

---

## ⚠️ The firmware has NO command watchdog. Measured, three ways.

**A commanded speed is retained indefinitely.** The car was commanded to 0.15 m/s, all
publishing then ceased, and it held 0.15 m/s for the full 45 s the probe watched — never
decaying, never timing out. It stopped only when an explicit zero was sent.

`tools/test_failsafe.py` confirms this across every way the command path can die
(car elevated, 2026-08-06). **Reconfirmed after a full power cycle of the board**, with
no deadman running, so it is a property of the firmware and not of a wedged session:

| Loss mode | Result |
|---|---|
| Publisher ceases, no zeros sent | **FAIL** — still driving |
| Governor `SIGKILL`ed mid-motion | **FAIL** — no zero escapes, firmware does not expire it |
| micro-ROS agent frozen 6 s | **FAIL** — still at 0.145 m/s on reconnect |

`geometry_msgs/Twist` carries no timestamp and no expiry, so nothing in the message says
"stale". The firmware must expire retained commands on its own, and it does not. The
vendor config surface (`config_robot.py`) exposes no timeout parameter — wifi, UDP,
baudrate, namespace, car type, domain ID, servo offsets, two PID sets, and nothing else —
so **this is not configurable**, and Yahboom ships no source to patch.

### What this means

**Any crash of any component leaves the car driving until it hits something or the
battery dies.** The governor's protection is entirely contingent on the governor
remaining alive. This is not a bug to fix; it is a property of the hardware to design
around.

### The mitigation, and its measured bound

`cmd_vel_deadman` watches `/cmd_vel` and zeros it when the last command was a move and
nobody has spoken since. It is a separate process specifically so it can outlive a
governor crash.

**Re-running the same fail-safe test with the deadman running flips both observable cases**
(same elevated car, same 0.15 m/s). Both configurations were re-measured back to back
after the power cycle, so the comparison is not across board states:

| Loss mode | Without deadman | With deadman |
|---|---|---|
| Publisher ceases | drove 45 s+, never stopped | **stopped in 693 ms** |
| Governor `SIGKILL`ed | drove indefinitely | **stopped in 762 ms** |

762 ms is the worst upper bracket, from a report the tool now generates with its own
provenance fields (`deadman_active`, `stop_mechanism`, `firmware_has_watchdog`) rather
than a key called `watchdog_bound_s` that read as firmware protection this car does not
have. An earlier back-to-back pair gave 687 / 742 ms; the ~20 ms difference is link
jitter, not a change in behaviour. It decomposes as the deadman's 0.5 s silence timeout
plus roughly 200 ms of command→motion-stop latency, and it is **tunable** — the timeout
is a launch argument. It has not been lowered because the radio link is demonstrably
jittery (scan gaps to 147 ms measured), and a timeout tight enough to trip on normal
jitter would produce spurious stops that teach the operator to distrust it.

**This is a deadman-mediated stop, not a firmware watchdog.** It holds only while the
deadman process is alive *and* the link is up. It is not a property of the car.

At 0.30 m/s, 762 ms is **229 mm of coast**; at the 0.05 m/s first-floor cap, 38 mm. That
is a **crash-case** term, separate from the normal stopping envelope — in ordinary
operation the governor commands the stop itself and the reaction time is T, not this.

### What remains uncovered is uncoverable from this machine

| Failure | Covered by deadman? |
|---|---|
| Governor crashes / `SIGKILL` | **yes — measured, 762 ms** |
| Teleop or a test tool dies mid-command | **yes — measured, 693 ms** |
| The deadman itself dies | no |
| This PC loses power, freezes, or is put to sleep | **no** |
| Wi-Fi drops, or the agent dies | **no — measured, case 3** |

Wi-Fi loss is the one that matters most, because it is the most likely, and it is
**unmitigable in software**: when the link goes, nothing on this machine can reach the
board at all. The only remaining stop is the physical power switch.

**Therefore every floor session requires a hand on the power switch, at a speed slow
enough to catch the car on foot.** That is a permanent operating constraint, not a
temporary one pending more work.

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

- **Forward sector only.** The governor watches ±45° ahead. **Nothing guards the rear or
  the sides.** Reverse and rotation stay *possible* — blocking them would trap the car
  against whatever it is trying to escape — but they are now bounded rather than passed
  through: reverse is capped at `max_reverse_speed` (0.10 m/s) because it is sensor-blind,
  and yaw is capped at `max_yaw_near` (0.4 rad/s) inside `stop_distance` because rotating
  sweeps the footprint corners past what the forward sector cannot watch. Lateral motion
  is zeroed outright: the chassis is differential, and the filter should not depend on the
  firmware ignoring a command it cannot govern.
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

- **Reaction appears to dominate braking — but that rests on an ASSUMED `a`.** Under
  assumed values, the braking term at 0.30 m/s is 45 mm at a pessimistic `a = 1.0 m/s²`
  and 11 mm at `a = 4.0`, against ≥118 mm of reaction, so a 4× error in `a` would move
  the total by only ~34 mm.

  That argument is worth stating and worth distrusting. `a` has never been measured on
  this robot, on this floor, and an audit showed the planned measurement is
  ill-conditioned at low speeds: 0.5 mm of systematic bias moved a fitted `a` from 1.0 to
  2.5 m/s² *with essentially zero residual*. Until `tools/measure_braking.py --fit`
  reports `identifiable: True`, **treat stopping distance as unmeasured**, not as a minor
  term. The unmeasured-quantity statement governs the test decision; the
  reaction-dominates argument does not.

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

## TODO: command arbitration (open design question, deliberately not built)

Bypass detection is an **alarm, not an interlock** — the governor logs an ERROR naming
the offending node and nothing more. Raised here rather than implemented, because the
obvious fixes are all wrong in an instructive way:

- **ROS 2 cannot prevent a bypass.** Any participant on the domain may publish `/cmd_vel`.
  The only real enforcement is SROS2 permissions, and the enforcement point would have to
  be the **agent's DDS side** — the board is a Micro-XRCE-DDS v2.x client with no security
  story of its own, so nothing can be enforced on the firmware.
- **Making the governor hard-fault on bypass punishes the wrong person.** It would brick
  governed teleop the moment anyone ran a command out of the course PDFs, while the
  bypassing node kept driving the car regardless. Strictly worse than the alarm.
- **Zeroing on bypass turns two publishers into a fight**, at 20 Hz, over a car.

What this actually is: several sources want to command one actuator, and there is no
arbiter. That is a **command-arbitration** problem, and it belongs with sensor fusion
rather than with the safety filter. Until it is designed, the operating rule stands:
**only run governed launch files, and watch the log for BYPASSED.**

## What has actually been tested

| Claim | Evidence |
|---|---|
| Governor logic is correct | 26 unit tests, incl. stale scan, all-NaN scan, sector edges, rear obstacles |
| It limits real commands with real lidar | bench, live car: 0.39 m ahead → 0.25 m/s request throttled to **0.022 m/s** |
| Stale input stops the robot | bench: output went to 0.000 after commands ceased |
| Bypass is detected and named | bench: governor logged the offending node |
| **Firmware retains commands forever** | live car, 3 modes + a 45 s probe: never stops without an explicit zero |
| **Deadman stops a dead publisher** | live car: 693 ms (publisher ceases), 762 ms (governor SIGKILLed) |
| Deadman covers link loss | **NO — measured to fail.** Nothing on this PC can |
| Stop paths leak no translation | unit test over all 5 stop paths, both axes |
| Reverse and near-obstacle yaw bounded | unit tests; **never exercised on hardware** |
| It stops before contact | **simulation only**, 0.394 m from a wall |
| Scan interval, governor loop | measured, live car, 2 runs; load-independent so they transfer |
| Command→motion latency | measured **elevated only**; a lower bound on the floor value |
| Comms + firmware delay | bounded to 104–195 ms by a threshold sweep |
| Braking response / deceleration `a` | **NOT MEASURED** |
| Real stopping distance | **NOT MEASURED** |

## Before driving on the floor

Follow [`first-floor-procedure.md`](first-floor-procedure.md). It replaces the checklist
that used to sit here, which was circular — it required braking distance to be measured
before any floor test, and that measurement *is* a floor test. The first session's whole
job is that measurement, at 0.05 m/s, with a hand on the power switch.
