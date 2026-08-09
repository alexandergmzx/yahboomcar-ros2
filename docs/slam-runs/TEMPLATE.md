# Physical SLAM run — YYYY-MM-DD HH:MM

Copy to `docs/slam-runs/YYYYMMDD-HHMMSS.md`. One file per attempt, **including failed
attempts** — a failed run is evidence and stays in the record; it is not replaced by a
cleaner narrative later.

**Verdict:** `DELIVERED` | `NOT DELIVERED` | `BLOCKED ON HARDWARE`

---

## Provenance

| field | value |
|---|---|
| git SHA | |
| branch | |
| ROS distro | jazzy |
| `ROS_DOMAIN_ID` | |
| bringup launch | `yahboomcar_config bringup_corrected_launch.py` |
| EKF config | `ekf_corrected.yaml` / `ekf.yaml` (vendor) |
| SLAM launch | `yahboomcar_config slam_launch.py` |
| SLAM params SHA-256 | |
| battery start / end | V / V |
| wall clock start / end | |
| motion | manual push / governed teleop |
| operator present | yes/no |

## Phase gates

Tick only what was actually run and passed. An untried gate is blank, not a dash.

- [ ] **Gyro, this boot** — `./tools/sensor_health.py --rotate-test` PASS
      (or: SLAM input configuration excluding the gyro, justified by a recorded A/B)
      → verdict:
- [ ] **TF/timing preflight** — `./tools/slam_preflight.py --seconds 60 --json <report>`
      → exact-time success: ____ % · longest failure run: ____ scans · PASS/FAIL
- [ ] **Stationary SLAM, 60 s** — `/map` updates, zero post-warm-up queue overflows
      → overflows: ____
- [ ] **Reference geometry measured** before the run → file:
- [ ] **Physical bag recorded** with `/scan /odom_raw /odom /imu /imu/data /tf /tf_static
      /map /cmd_vel /cmd_vel_raw /slam_toolbox/graph_visualization`
      → path: · SHA-256:
- [ ] **Map + pose graph saved** under unique names → `.pgm` · `.yaml` · `.posegraph`
- [ ] **Returned to the taped start pose** → reported error: ____ m / ____ deg
- [ ] **Scored** — `./tools/score_slam_map.py --map ... --reference ... --run-facts ...`
      → PASS/FAIL (paste the table below)

## Scorer output

```text
(paste the score_slam_map table here, including any FAIL rows)
```

## What actually happened

Narrative. Include what went wrong, what was retried, and anything surprising. If a
threshold was missed, record the measured value and the physical reason — a threshold is
never loosened after a failed run without recording the failed value and getting the
user's approval.

## Artifacts

| artifact | path | tracked? |
|---|---|---|
| bag | `MicroROS-assets/bags/slam-physical-.../` | no (checksum below) |
| SLAM log | `MicroROS-assets/logs/slam-physical-....log` | no |
| preflight JSON | | yes |
| map `.pgm` / `.yaml` | `yahboomcar_config/maps/physical_..._runN.*` | yes |
| pose graph | `MicroROS-assets/maps/physical_..._runN.posegraph` | no |
| scorer JSON | | yes |

Checksums:

```text
bag:    sha256  ...
params: sha256  ...
```

## Remaining limitations

State them plainly. At minimum, carry forward whichever of these still apply:

- the firmware has no command watchdog; Wi-Fi loss is unmitigable from the ground station
- the gyro is intermittently faulty and a stationary robot cannot reveal it
- `ekf_corrected.yaml` was validated on a stand, and (until a run says otherwise) not in
  real motion
- no measured stopping distance ⇒ powered mapping stays gated
- Nav2 out of scope

## Cleanup confirmation

- [ ] no ROS processes left (`pgrep`)
- [ ] no stale DDS segments (`./tools/simctl status`)
- [ ] car powered down / left safe
