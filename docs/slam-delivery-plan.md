# Physical SLAM: delivery plan and acceptance contract

**Provenance.** Ported 2026-08-10 from `../MicroROS/docs/slam-delivery-plan.md`
(authored 2026-08-09 by Alex's audit of the `deb670a` SLAM pass, which ran in the
MicroROS checkout). The substance of the acceptance contract is unchanged. What is
adapted: file paths and the launch chain, because the first-party stack lives HERE
post-extraction (D-12) and this repo owns the pieces the runs depend on — the bond-free
canonical SLAM launch (D-19), the corrected EKF, the safety chain, and the tooling
below. The sibling copy remains the record of the audit itself.

**Current verdict: physical SLAM NOT DELIVERED.** Offline tooling exists (2026-08-10);
no physical map has been produced. See `docs/slam-runs/` for the run record — currently
empty of physical runs, which is itself the honest state.

---

## Why the previous pass was rejected as a delivery

Useful configuration research that solved the comfortable parts and stopped before the
requested result:

- the commit subject said "tested against real slip", but the mapping runs used the 2D
  simulator's `--slip 0.4` knob. The 92% stand figure is real wheel/body disagreement,
  and a car with its wheels off the floor is still not a physical mapping run;
- every configuration ran once, weaker than the report's own planned two-run protocol;
- the Huber A/B bag omitted `/odom_raw`, so its independent witness could not be scored,
  and it was not repeated;
- HuberLoss showed no demonstrated map-quality improvement — retained as a reasonable
  default, not as an established win;
- the reported maps held **14–18 occupied pixels**. Correct outer dimensions do not make
  an occupancy grid useful;
- RViz `Global Status: Ok` proved the display configuration was tidy;
- no new physical `.pgm`/`.yaml`, pose graph, run report, or replayable physical bag;
- the next hardware SLAM launch (`bench-slam-20260808.log`) emitted **135 message-filter
  drops**, mostly `queue is full`, with no evidence of a saved map. That is the first
  thing to diagnose; more tuning before that is premature.

This is not permission to hide the earlier research. It is a ban on presenting setup,
simulation, or UI cleanup as the requested physical outcome.

## Definition of done

Physical SLAM is delivered only when all of these exist:

1. a current-boot gyro hand-rotation PASS, or a SLAM input configuration that excludes
   the failed gyro, justified by a recorded A/B;
2. a machine-readable preflight proving **≥99% of scans resolve `odom → laser_frame` at
   the scan's own timestamp** over 60 s, with zero post-warm-up queue overflows;
3. one motors-off / manual-push physical bag containing `/scan`, `/odom_raw`, `/odom`,
   `/imu`, `/imu/data`, `/tf`, `/tf_static`, `/map` and the commanded-velocity topics;
4. **two independent physical runs** around the same measured route, starting and ending
   at the same taped pose. Replaying one bag twice is not two physical runs;
5. per run: a saved `.pgm`, `.yaml`, serialized pose graph, raw run report, and bag
   checksum. Large bags stay git-ignored; checksums and reports are tracked;
6. a quantitative scorer passes the acceptance table for both maps and compares their
   repeatability;
7. the chosen parameter set wins an A/B replay of the **same physical bag**, or the
   report states plainly that it did not beat the baseline and the baseline stays;
8. the final map and report are committed under stable names. No vendor map is
   overwritten.

## The tooling, and which gate each one serves

Built 2026-08-10, offline, with no hardware contact:

| gate | tool | negative control |
|---|---|---|
| 2 — TF/timing | `tools/slam_preflight.py` | a deliberately missing TF link fails the gate by name |
| 6 — map usefulness | `tools/score_slam_map.py` | `--self-test`: corrupted map rejected; the 16-pixel map rejected on occupancy while its bounding box is still correct |
| 7 — fair A/B | `tools/replay_slam_bag.py` | refuses a bag missing `/scan` or `/tf`; records the bag SHA-256 in every row |
| odometry prior | `yahboomcar_config/launch/bringup_corrected_launch.py` | drift tests assert the copied vendor composition still matches the vendor source |

Run reports and scorer schemas: `docs/slam-runs/`.

## Phase 0 — safety and evidence

- Record `git rev-parse HEAD`, ROS distro, domain, battery, params SHA-256 and
  wall-clock start/end in every report.
- **Do not start Nav2. No autonomous patrol on the physical robot.**
- The first physical run is motors-off, pushed by hand. That exercises the real lidar,
  encoders, clocking, TF and scan matching without relying on a firmware watchdog that
  does not exist.
- Powered mapping waits on separate gates: a valid fail-safe report, a current gyro
  disposition, and a measured low-speed stopping distance. Until then the only powered
  wheel work is elevated.
- Never delete or overwrite a failed bag or log. Timestamped paths only.

```text
MicroROS-assets/bags/slam-physical-YYYYMMDD-HHMMSS/    # ignored, retained locally
MicroROS-assets/logs/slam-physical-YYYYMMDD-HHMMSS.log # ignored, retained locally
docs/slam-runs/YYYYMMDD-HHMMSS.md                      # tracked summary + checksums
```

## Phase 1 — diagnose the hardware scan queue, before touching `slam_toolbox.yaml`

```bash
cd ../../ground_station && source /opt/ros/jazzy/setup.bash && source install/setup.bash
export ROS_DOMAIN_ID=20
export FASTDDS_DEFAULT_PROFILES_FILE=$PWD/fastdds_udp_only.xml

../src/yahboomcar-ros2/tools/sensor_health.py --rotate-test        # gyro, THIS boot
ros2 launch yahboomcar_config bringup_corrected_launch.py          # corrected EKF
../src/yahboomcar-ros2/tools/slam_preflight.py --seconds 60 --json <report>.json
```

Then, and only if the preflight passes, SLAM stationary for 60 s: acceptance is `/map`
updating **and** zero post-warm-up queue overflows. Lifecycle `active` is not
acceptance.

**The open frame question.** `/odom_raw.header.frame_id` is the literal string
`odom_frame` [deserialized from `bench-matrix-20260808-093840`, 2026-08-09] while the
EKF and SLAM configurations use `odom`, and no bridging static transform exists in
either checkout [grep, both]. `slam_preflight.py` reports it and deliberately does not
rename it. Prove whether `robot_localization` rejects, transforms, or accepts that pose
component before changing anything; if normalization is needed, do it once, in a tested
relay or a consistent frame configuration — never with competing TF broadcasters.

**Latency is measured, not tuned away.** Existing bags range 6–80 ms in some and
0.33–0.51 s in others. Do not "fix" a failing gate by raising `transform_timeout` until
the warnings stop.

## Phase 2 — the first physical map, no motor commands

Follow `docs/floor-slam-session.md` Session A (hand-push walk), plus:

1. tape-measure the room first and write `docs/slam-runs/<room>-reference.yaml`: at least
   two wall spans, one doorway/opening, two obstacle dimensions or offsets;
2. mark the start pose;
3. record `/scan /odom_raw /odom /imu /imu/data /tf /tf_static /map /cmd_vel
   /cmd_vel_raw /slam_toolbox/graph_visualization`;
4. push slowly through a loop with translations and turns both ways, pausing at
   distinctive geometry. Do not lift the chassis — wheel contact is part of the test;
5. save map and pose graph under unique names
   (`yahboomcar_config/maps/physical_YYYYMMDD_run1.*`);
6. return to the taped start pose before stopping; record the reported and physical
   error.

If the run fails, diagnose the bag. Do not substitute a simulator run.

## Phase 3 — score usefulness, not existence

`./tools/score_slam_map.py --map ... --reference ... --run-facts ...`

| metric | required |
|---|---|
| map artifact | nonempty YAML and PGM; image path resolves |
| occupancy content | ≥100 occupied pixels and ≥5% known cells |
| measured wall spans | each within max(0.15 m, 5%) of tape |
| doorway/obstacle feature | present and within 0.15 m of measured width/offset |
| wall quality | median thickness ≤0.12 m; no duplicate parallel wall >0.20 m |
| physical loop closure | taped pose reported within 0.10 m and 5° |
| runtime health | no post-warm-up queue overflows; map updated throughout motion |
| repeatability | two aligned maps within 5% on spans, occupied-cell IoU ≥0.65 |

Thresholds may be tightened after measurement. They may **not** be loosened after a
failed run without recording the failed value, the physical reason, and Alex's approval.

## Phase 4 — tune fairly, on replayable physical evidence

`./tools/replay_slam_bag.py --bag <physical bag> --params <arm> --out <map>`

- baseline = the current parameters; compare `None` vs `HuberLoss` and at most one
  further justified parameter, one at a time;
- ≥2 runs per configuration, same bag SHA-256 in every row;
- winner by Phase-3 scores. Inside repeatability noise ⇒ prefer the default and say
  there was **no demonstrated improvement**;
- do not bundle RViz changes with map-algorithm claims.

## Phase 5 — repeat physically, then consider powered mapping

Repeat Phase 2 independently for run 2. If both pass Phase 3, commit the selected map,
both run reports, reference geometry, scorer outputs and tests.

Powered mapping is a later gate: regenerated fail-safe report, current gyro decision,
measured stopping distance, first-floor governor/deadman preflight, 0.05 m/s cap, hand
on the power switch. Nav2 stays out of scope until its late-goal cancellation race is
fixed (partially addressed in `b946b40`; unexercised against a live Nav2).

## Claims that may and may not be made

Allowed before completion: "SLAM lifecycle activates in simulation" · "a simulator
publishes a map" · "the hardware TF preflight failed at X%" · "this parameter loaded,
but map improvement is unproven".

Forbidden before completion: "physical SLAM works" · "a map was delivered" without a
committed `.pgm`/`.yaml` and run report · "HuberLoss improved SLAM" without same-bag A/B
scores outside repeatability noise · "tested against real slip" when the run used
simulator injection or an elevated chassis · "RViz is green, therefore SLAM works" ·
"map size is correct, therefore the map is good".

## Handoff format

Lead with exactly one of `DELIVERED`, `NOT DELIVERED`, or `BLOCKED ON HARDWARE`, then
exact commands, commit SHA, artifact links, measured scores, failed attempts, remaining
safety limitations, and confirmation that no ROS processes or stale DDS segments were
left behind.
