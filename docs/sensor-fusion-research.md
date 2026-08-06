# Sensor fusion: what each sensor can actually witness

Companion to [`research-log.md`](research-log.md). Everything below that carries a number
was measured on this robot or raytraced exactly; where something is inferred it says so.

Written after the odometry calibration, where deciding the scale factor `k` kept coming
back to "believe the tape measure" — because **every estimate this robot makes of its own
motion comes from a wheel or from integrating its own opinion.**

---

## 1. What each sensor observes

| sensor | observes | cannot observe | characteristic failure |
|---|---|---|---|
| wheel encoders | wheel rotation | ground motion | **slip** — reports wheels, not world |
| IMU gyro | yaw *rate* | absolute heading | drift; no magnetometer to correct it |
| IMU accel | body acceleration | position, without integrating twice | tilt leaks gravity; brake-dive |
| lidar | pose **relative to the room** | anything in a degenerate direction | geometry-dependent, not time-dependent |

The lidar is categorically different. Everything else answers *"how much do I think I have
moved"* by accumulating; the lidar answers *"where am I relative to that wall"*. It is
therefore the only sensor that can **bound** drift rather than accumulate it — and the
only independent witness when the encoders and the tape disagree.

### Integration order decides what the IMU is good for

Bias enters displacement as ½·b·t² and velocity as b·t. With a realistic ~0.05 m/s²
residual after static-bias removal:

| use | duration | drift | verdict |
|---|---|---|---|
| distance over a calibration push | 4 s | ~400 mm | **useless** — worse than what it checks |
| distance over a braking event | 0.3 s | ~2 mm | **good** |
| Δv over a braking event | 0.3 s | ~15 mm/s | **good** — single integration |

So *"use the IMU as a second opinion on distance"* is right for braking and wrong for the
push, and the difference is duration alone.

**Brake-dive limits it further, in the dangerous direction.** The chassis pitches forward
under braking, tilting the accelerometer into gravity. One degree leaks 0.171 m/s²:

| speed | decel | 1° of dive as % of signal |
|---|---|---|
| 0.05 m/s | 0.17 m/s² | **103%** |
| 0.10 m/s | 0.33 m/s² | 51% |
| 0.30 m/s | 1.00 m/s² | 17% |

Static bias subtraction cannot remove it — the tilt happens *during* the event — and it
biases deceleration **high**, i.e. predicts a shorter stop than reality. The IMU becomes a
useful braking witness exactly as speeds get dangerous, and is useless at the speed the
floor tests start from.

---

## 2. Fusion only helps when failures are independent

This is the whole justification, and the vendor config violates it.

`yahboomcar_bringup/param/ekf.yaml` fuses, from `/odom_raw`, **both pose and twist**:

```yaml
odom0_config: [true, true, false,      # x, y
               false, false, true,     # yaw
               true, true, false,      # vx, vy
               false, false, true, ...] # vyaw
```

Pose is the integral of twist. Feeding both puts one measurement in twice, and the filter
treats two perfectly correlated inputs as two independent ones. The covariance then
shrinks faster than the evidence justifies: the filter becomes **confident, not accurate**.

The IMU block repeats the error and compounds it — `imu0_config` enables both `yaw` and
`vyaw`. The ICM-42670-P is **6-axis, with no magnetometer**, so its "yaw" is not an
observation of anything absolute; it is the gyro integrated by the driver. Fusing it feeds
integrated drift back in wearing the clothes of an independent fix. And with `/odom_raw`
yaw *also* enabled, the filter had **two dead-reckoned yaw sources and no world reference**
— averaging two drifting estimates while reporting the confidence of a redundant pair.

It also fuses `vy`, which is identically zero on this differential chassis.

Corrected in [`ekf_corrected.yaml`](../yahboomcar_ws/src/yahboomcar_bringup/param/ekf_corrected.yaml),
alongside the vendor file rather than replacing it: encoders contribute **twist only**, the
IMU **yaw rate only**, and `/odom_laser` contributes **pose** — the only world-referenced
input available. **Not yet validated on hardware.**

### The same error, reached from the other direction

An ICP scan matcher seeded from wheel odometry returns its seed unchanged in any direction
the geometry does not constrain. Fusing that with wheel odometry would double-count
exactly as above. That is why `scan_matcher.match()` defaults to an unseeded coarse-to-fine
schedule rather than taking an odometry prior — keeping the estimate independent is the
entire point of having it.

---

## 3. Degeneracy — and a prediction of mine that was wrong

A scan matcher can only resist motion **along a surface normal**. The information a scan
carries about translation is `Σ n nᵀ` over observed normals (Censi, ICRA 2007); its
eigenvalues say how much information exists in each direction. Two parallel walls give one
eigenvalue ≈ 0: motion between them is not noisy but *unobservable*, and ICP will still
converge and still return a confident-looking number.

**I predicted the planned 4×4 m arena would be close to that worst case, and that boxes
would be what made translation observable there. Both were wrong.**
[`tools/arena_observability.py`](../tools/arena_observability.py) raytraces exact scans:

| room | median isotropy | verdict |
|---|---|---|
| **4.0 × 4.0 m** | **0.914** | **well conditioned** |
| 4.0 × 3.0 m | 0.733 | well conditioned |
| 8.0 × 2.0 m | 0.230 | marginal |
| 12.0 × 1.0 m | 0.058 | **degenerate** |
| 30.0 × 30.0 m | 0.103 | marginal — far walls out of range |

The lidar reaches 8 m and the room is 4 m, so it sees **all four walls from every
position**, and four walls constrain both axes. Adding boxes slightly *reduced* isotropy
(0.924 → 0.772 with three), because box faces are axis-aligned too while occluding wall
returns.

Degeneracy needs an aspect ratio near **8:1**, or a room **larger than the sensor range**.
A big empty hall is the hazard; a small square room is not.

Boxes are still worth having — they break the rotational symmetry of a square room, where
a matcher that loses track can relocalise into the wrong quadrant with total confidence,
and they are what the safety governor needs to see. Just not for observability.

---

## 4. What the measurements showed

`tools/sensor_agreement.py`, on bags already on disk.

### Slip is detectable, and large

On `twin_dataset` (car on its centre stand, so translational slip is 100% by
construction): encoders reported **1.602 m** travelled, the lidar **0.128 m** — 92%
apparent slip, correctly flagged. The wheels turned and the world did not move.

### A dead channel that would have poisoned any filter

`/imu` `angular_velocity.z` in that bag is **exactly zero across all 1256 samples, std
0.000000**. Read naively that says "the body did not rotate". It actually says the gyro
reported nothing.

This is the most dangerous input a filter can receive, because **it agrees with everything
and never objects**. An EKF would have fused it as a confident zero-rotation measurement.
Zero-variance channels are now detected and excluded by name.

### Where the three witnesses disagree, and an unresolved scale

On the desk-motion bag (`selftest-20260806-004826`, 40 s, live gyro):

| | value |
|---|---|
| encoder yaw / IMU yaw | **1.112** — encoders over-read ~11% |
| lidar/gyro slope, 38 genuinely turning pairs | **1.556** |
| lidar/encoder path length | 0.684 |

The 11% encoder over-read in yaw is consistent with the ~7.7% recorded in
[`handoff-audit.md`](handoff-audit.md). The lidar/gyro scale disagreement is **open**, and
two things prevent calling it: the gyro can only be integrated between *bag arrival*
times, because firmware and host clocks are unsynchronised and header stamps are no better,
and scan inter-arrival jitter is 113–147 ms p95.

**Summing per-pair ICP over a run is not the way to compare.** 464 of 503 pairs on that bag
were under 0.05 rad/s; summing over them accumulates ICP noise as a random walk. The sum
implied a 25% lidar *under*-read while the regression over turning pairs implied the
opposite. The tool now reports the regression and says the sum is not informative.

---

## 5. Sources consulted

| source | why | what it changed |
|---|---|---|
| Censi, *On achievable accuracy for range-finder localization*, ICRA 2007 | is scan-match uncertainty derivable rather than tuned? | gave `Σ n nᵀ` as the translational information matrix — the basis of the degeneracy detector and of the published covariance |
| Moore & Stouch, *A generalized extended Kalman filter implementation for robot_localization*, IAS-13 2014 | what the vendor filter is actually doing | confirmed per-input `*_config` semantics; made the pose+twist double-count legible |
| Julier & Uhlmann, covariance intersection | how wrong is fusing correlated estimates? | named the failure mode: correlated inputs fused naively give an inconsistent, overconfident covariance |
| Chetverikov et al., Trimmed ICP | outlier rejection for scan matching | adopted the idea, then **rejected fixed-fraction trimming** after measuring that it biases rotation low (below) |
| Borenstein & Feng, UMBmark, SPIE 1995 | already used in `odometry-calibration.md` | the systematic-vs-random error distinction that motivates comparing sensors rather than repeating one |

---

## 6. Bugs this work surfaced

Recorded because each was silent, plausible-looking, and would have propagated into a
state estimate.

1. **Fixed-fraction trimming biases rotation low.** Under rotation, residual grows with
   distance from the centre — so the points carrying the rotational signal are exactly the
   ones with the largest residuals, and trimming to the closest 80% discards them. It
   converged to a stable **0.0852 rad against a true 0.12** and stayed there through every
   stage: a 29% under-estimate that looked like clean convergence. Replaced with robust
   median + 3·MAD rejection.

2. **The scan-match frame convention was inverted.** `match(prev, curr)` returns the
   *scene* transform, which is the inverse of the robot's: turn left, and the room appears
   to turn right. Encoders and gyro both read ≈ +1.8 rad while the lidar read −1.3. The
   magnitude was plausible; **only the sign gave it away.**

3. **Integer division silently disabled subsampling.** `355 // 180 == 1`, so a "cap at 180
   points" decimated by 1 — not at all. Timing was identical at 355 and "180" points until
   it was found.

4. **ICP missed real-time by 6.6×.** p95 546 ms against an 83 ms scan interval, dominated
   by *iteration count* rather than point count. Now bounded by an explicit wall-clock
   budget that returns the best estimate so far and flags it, because an estimator that
   silently misses its deadline just drops scans until it looks like it is working.
   Measured after: mean 21 ms, p95 55 ms, 426/426 pairs matched.

---

## 7. Open, and honest limits

- **`/odom_laser` has never run on a robot driving across a floor.** Bags and the elevated
  car only. It must not enter a safety path until it has.
- **`ekf_corrected.yaml` is untested on hardware.** Written from the analysis above, not
  from a measured improvement.
- **The lidar/gyro yaw scale disagreement (≈1.56) is unresolved**, and the clock problem
  above may be the whole of it.
- **Scan distortion is not corrected.** At 12 Hz a scan is not an instantaneous snapshot;
  the robot moves during the sweep. A known error source, untreated.
- **Fusion cannot rescue a degenerate direction.** If every sensor is blind to it,
  combining them adds confidence without adding information — the specific way this could
  make things *worse*.
- **scipy is unusable on this machine**, so none of it is available to lean on:
  `spatial`, `optimize`, `linalg` and `stats` all raise `numpy.dtype size changed`,
  because a pip numpy 2.2.1 in `~/.local` shadows apt's 1.26.4 while apt's scipy 1.11.4 is
  built against the numpy 1.x ABI. Everything here is plain numpy.
