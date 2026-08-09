<!-- PROVENANCE (fleet D-12): ported from ../MicroROS/docs/first-floor-procedure.md at
     commit 80fa059 (2026-08-07, "Dry-run the braking protocol end to end in simulation").
     Alex-authored content that predated the extraction; this copy is the first-party
     home and the one docs here link to. Deliberate divergences from the source, each
     minimal: (1) the Bring-up sourcing lines use the fleet layout instead of the old
     yahboomcar_ws paths; (2) the battery-pack README link points at the MicroROS
     checkout, where that README lives; (3) a duplicated "Abort immediately if" heading
     in the source was dropped. Nothing else was changed. -->

# First floor session — procedure

The car has never driven on the floor under this stack. This document is the procedure
for the first sessions, and it exists because the previous checklist was circular: it
said "measure braking distance before driving on the floor", and that measurement *is* a
floor test.

**Read [`safety-case.md`](safety-case.md) first.** The single most important fact:

> The firmware has no command watchdog. A commanded speed is retained indefinitely —
> measured, three ways. If the Wi-Fi drops, nothing on this PC can stop the car.

`cmd_vel_deadman` covers a publisher dying (measured: 762 ms). It does **not** cover link
loss, agent death, or this PC freezing. For those the only stop is the power switch,
which is why the first sessions run slowly enough to walk the car down.

---

## Before the car touches the floor

- [ ] **Clear area, ≥ 3 × 3 m**, hard flat floor. No cables, no rugs, no chair legs.
- [ ] **Soft perimeter** — cushions, folded cardboard, anything that absorbs a 0.05 m/s
      nudge. Not a wall you care about.
- [ ] **You are standing, not seated**, within one step of the car.
- [ ] **You know where the power switch is** and can reach it without moving furniture.
- [ ] **Battery at or above 7.4 V.** `ros2 topic echo /battery --once`, divide by 10.

      | | volts | meaning |
      |---|---|---|
      | full | 8.4 | 2S pack fully charged |
      | **session gate** | **7.4** | nominal; start a floor run at or above this |
      | tool floor | 7.0 | `car_selftest.py` refuses to command motion below this |

      This car ships a **7.4 V (2S)** pack — see the MicroROS checkout's README. An
      earlier version of this line demanded 11 V, which would have rejected every
      healthy battery this robot has ever had. The 7.4 V gate leaves margin above the
      7.0 V floor, because a pack that sags under motor load is how you get a brownout
      mid-run, and mid-run a brownout looks exactly like a fault.
- [ ] **Sensor health passes.** `./tools/sensor_health.py --rotate-test` — you rotate the
      car by hand and the gyro must respond. **The rotate test is not optional on this
      robot**: the gyro is intermittently faulty (confirmed 3 of 5 informative runs), and
      at rest a working gyro and a broken one both read exactly `0.000000`, so a
      stationary check cannot tell them apart. Neither a power cycle nor a serial reset
      has recovered it. Without a live gyro, calibration cannot be witnessed and the
      braking gate will refuse to record runs.
- [ ] **Gyro settled** — car on the ground, powered, stationary 5 s before anything moves.
- [ ] **Lidar unobstructed.** The governor stops on *stale* data but trusts *wrong* data;
      a partly blocked scanner is its worst input.
- [ ] **Exactly one agent running.** `docker ps | grep micro-ros-agent`.
- [ ] **Governor confirms the deadman.** Look for
      `safety companion present on /cmd_vel: /cmd_vel_deadman`. Its *absence* is the
      danger, and the governor warns `NO DEADMAN` if it is missing.
- [ ] **Nothing ELSE is publishing `/cmd_vel`.** A `BYPASSED` error naming any other node
      is an abort. Nav2, calibration nodes, the laser behaviours and
      `twin_motion_sequence.py` must all be stopped. The deadman is expected there and
      is not a bypass — verify with `./tools/test_launch_preflight.py`, which needs no
      car and fails if this launch cannot pass its own preflight.

## Bring-up

```bash
# Fleet layout (the extraction moved this repo out of yahboomcar_ws; A2.3):
cd ../../ground_station
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=20

ros2 launch yahboomcar_safety first_floor_launch.py
```

That profile is 0.05 m/s, no yaw, no reverse, deadman mandatory. Confirm in its output:

- `governor up: ... max 0.05 m/s`
- `deadman up: zeroing /cmd_vel after 0.5s of silence`

If you do not see both lines, stop.

## Session 0 — odometry scale (five minutes, no driving)

```bash
./tools/measure_braking.py --calibrate
```

**A fresh calibration is now required before any braking run.** The tool refuses to
record runs when the active calibration is short (< 1 m), curved (> 5°), lacking a gyro
record, or has an implausible `k`. Warning was not enough: every run subtracts
`k × run-up` from a tape total, so a 5% error in `k` puts 5% of the *run-up* — far longer
than the stop — straight into the stopping distance. A 1.8 m run-up at 5% is 90 mm of
error on a stop of perhaps 50 mm.

The five calibrations recorded on 2026-08-06 all predate the gyro check and are therefore
rejected. Redo one.

Motors stay off. Push the car by hand along a measured edge and enter the tape distance.
This is the only way to learn the odometry scale without traction or braking involved,
and every later run depends on it — the tool refuses to record runs without it.

Push at least **1.5 m**: a fixed ±10 mm tape error is 2% of a 0.5 m push and 0.5% of a
2 m one. Push **straight** — the gyro now measures how far the heading wandered, and a
curved push makes the wheels trace an arc while your tape measures the chord.

**Do several, and take the LOWEST `k`, not the average.** Slip during a push always makes
odometry under-report, so `k` is only ever biased upward. It is a one-directional error,
and averaging it in is averaging in contamination. `--list-calibrations` flags short and
curved pushes and recommends accordingly.

Measured 2026-08-06 over five pushes: `k` fell 1.0849 → 1.0040 as pushes got longer and
cleaner, converging on ~1.00. **Linear odometry on this robot is accurate to about
0.4%.** That is not in tension with the ~7.7% figure in
[`handoff-audit.md`](handoff-audit.md) — that one is *yaw*, measured against the gyro, and
is a different quantity entirely. Distance and heading are calibrated separately.

## Session 1 — straight-line stopping distance

The only job. Everything else is a way to lose the car under the furniture.

```bash
./tools/measure_braking.py --speed 0.05 --runs 5
```

Per run: put the car on the **START** mark, press Enter, let it drive, and when the tool
prints `ZERO commanded` let it coast to rest. Then tape-measure **START to final rest**
and type it in millimetres.

You are *not* asked to mark where the stop was commanded. Nobody can mark a moving
robot's position at the instant a command is sent — reaction time alone is ~250 ms, which
at 0.10 m/s is 25 mm, larger than the thing being measured. Instead the tool subtracts the
run-up (steady-speed odometry, scaled by the calibration) from your tape total. Odometry
is never trusted during braking, because locked or slipping wheels make encoders
under-report exactly when it matters.

Then repeat at **0.10** and **0.15 m/s**.

## Abort immediately if

- the car does not respond to a zero command within about a second
- the governor or deadman logs stop appearing (a dead node is invisible until you need it)
- `/scan` rate drops below ~6 Hz, or the LED goes to 1 s blink (reconnecting)
- the car moves in any direction you did not command
- anything enters the area

Abort means **power switch**, not Ctrl+C. Ctrl+C is a request; the switch is not.

## Raising the cap

Each step is earned by the previous one, and the gate is *prediction*, not survival:

| Step | Cap | Gate to pass before the next |
|---|---|---|
| 1 | 0.05 m/s | ≥ 5 runs recorded |
| 2 | 0.10 m/s | ≥ 5 runs recorded |
| 3 | 0.15 m/s | ≥ 5 runs, and `--fit` now reports **`identifiable: True`** |
| 4 | 0.20 m/s | measured stop inside the **envelope** predicted at step 3 |
| 5 | 0.30 m/s | envelope still holds at 0.20 |

**The fit must pass every validity check, not just have enough speeds.** A later audit
fed the tool stopping distances that *decreased* with speed and got `identifiable: True`
with `T_stop = 5.109 s`; and a set giving `T_stop = −0.295 s`, also `identifiable: True`.
Neither is a hard case — both are impossible — and the earlier check had no opinion,
because it tested the number and spread of speeds and treated those as sufficient.

`--fit` now reports each condition by name and refuses unless all hold:

| check | why |
|---|---|
| ≥ 3 distinct speeds, ≥ 2.5× spread | `T_stop` and `a` cannot be separated otherwise |
| ≥ 3 runs at every speed | a mean of one run is not a mean |
| `T_stop > 0` | the robot does not brake before being told to |
| `a > 0` and finite | otherwise it is not a stopping model |
| distance rises with speed | **a faster robot cannot stop shorter** |
| ≥ 90% of bootstrap fits physical | if the data cannot pin the sign, nothing is identified |
| lower confidence bound on `a` above zero | — |

**No envelope is produced at all from a fit that fails.** A number derived from impossible
data is worse than no number, because it will be used.

Systematic bias remains the subtler trap: 0.5 mm of it moved fitted `a` from 1.0 to 2.5
*while leaving essentially zero residual*. A good-looking fit is not evidence, so the bias
sweep is printed and folded into the envelope.

**A few millimetres of systematic bias will wreck `a`, not `T_stop`.** Dry-running the
whole protocol against a simulator with *known* values showed it: a speed-proportional
measurement error is indistinguishable from dead time, so it lands in `T_stop` and is
taken out of the quadratic term. Fixing one integration boundary moved the recovered
deceleration from **0.54 to 1.14 m/s² against a truth of 1.5** — on identical data.

That is why the calibration gate is strict: an error in `k` multiplies the *run-up*, which
is far longer than the stop, and arrives as exactly this kind of bias.

**Size against the conservative envelope, never the point estimate.** The envelope is the
worst of three things plus a 100 mm margin:

- the 95th percentile of the bootstrap prediction, over **physical samples only**;
- the worst fit across the systematic-bias sweep, which the bootstrap cannot see;
- **the worst stopping distance actually observed at or below that speed.**

That last one needs no theory and cannot be argued with: a model may not predict a stop
shorter than one that has already happened. It is what catches a bad envelope even when
everything else has gone wrong — the audit produced a "conservative" 33 mm figure at
0.20 m/s from data whose own worst observed stop was 50 mm.

"It stopped" is not the gate. A model that predicts correctly can be trusted at a speed
you have not yet tried; one that merely stopped cannot, and 0.30 m/s is exactly where
extrapolating a wrong model costs the most.

## What this procedure does not make safe

- **Rear and sides.** The sector faces forward. Steps 1–2 forbid reverse and yaw for that
  reason; steps 3–4 re-enable them bounded, still unsensed.
- **Moving obstacles.** The model assumes the obstacle stays put. A person walking toward
  the car adds their closing speed to the requirement.
- **Wrong lidar data.** Stale scans stop the car; confidently wrong ones do not.
- **Link loss.** Unmitigable. This is what the power switch is for.
