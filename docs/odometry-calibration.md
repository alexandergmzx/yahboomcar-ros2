# Odometry calibration — UMBmark field protocol

Follow this at the robot. Method is **UMBmark** (Borenstein & Feng, SPIE 1995), the
standard benchmark for differential-drive odometry; see [`research-log.md`](research-log.md)
for sources.

**Why bother:** cartographer here runs `use_odometry = true` and `use_imu_data = false`
(`yahboomcar_nav/params/lds_2d.lua`), so **map quality rides entirely on odometry**. And
we already have a suspicious number — commanding 0.15 m/s produced **0.170** on
`/odom_raw`, ~13% high. Mapping before fixing that bakes the error into the map.

---

## What you need

| | |
|---|---|
| **Floor space** | **3 × 3 m clear** (2 × 2 m path + ~0.5 m margin) |
| Tape measure | mm markings |
| Masking tape | mark the start pose |
| Time | 30–40 min for all 10 runs |
| Battery | charge first — 10 runs plus turns is a real load |

**Surface matters.** This is a 4-wheel skid-steer: every turn is slip, and slip depends
on the surface. **Calibrate on the surface you will actually drive on**, and don't mix
carpet and hard floor within a run set.

### Marking the start

Put tape down as a cross, and mark the robot's reference point (centre of `base_footprint`)
and its heading. You are measuring where it *returns to*, so the start mark has to be
unambiguous to ~5 mm.

```
        ^ +x (robot's forward at start)
        |
   -----+-----  tape cross = origin
        |
     start pose, heading +x

   2 m square, driven CW then CCW:

   D<--------C          A ---> B  leg 1
   |         ^          B ---> C  leg 2   (turn 90° on the spot between legs)
   |         |          C ---> D  leg 3
   v         |          D ---> A  leg 4
   A-------->B
```

---

## Procedure

Per the paper, scaled to **L = 2 m**, at **0.15 m/s** (paper uses 0.2; slower is safer
on a skidding chassis).

For each run:

1. Place the robot on the start mark. Power on, leave it **still for 5 s** so the gyro
   initialises — skip this and the whole run is garbage.
2. Start bringup and reset odometry to zero at the mark.
3. Drive the square: **2 m straight, stop, turn 90° on the spot, stop**, four times.
4. On return, **measure with the tape** the actual offset from the origin, in the
   robot's *starting* frame: `x` = forward, `y` = left. Record both, in mm, with sign.
5. Record what odometry *claims* the final position was.

**5 runs clockwise, then 5 counter-clockwise. 10 total.**

> Do not skip the CCW set to save time. The two dominant errors **cancel in one direction
> and add in the other** — that is the entire reason this test is bidirectional. Five
> CW runs alone can show near-zero error while both errors are large.

`tools/umbmark.py` walks you through the runs and does all the arithmetic. Use it rather
than the tally sheet below unless you prefer paper.

### Tally sheet

Absolute return position error, measured minus origin, in **mm**:

| Run | Direction | x (fwd) | y (left) | notes |
|---|---|---|---|---|
| 1 | CW | | | |
| 2 | CW | | | |
| 3 | CW | | | |
| 4 | CW | | | |
| 5 | CW | | | |
| 6 | CCW | | | |
| 7 | CCW | | | |
| 8 | CCW | | | |
| 9 | CCW | | | |
| 10 | CCW | | | |

---

## The maths

Cluster centres, n = 5 per direction:

```
x_cg,cw  = mean(x_1..x_5  CW)     y_cg,cw  = mean(y_1..y_5  CW)
x_cg,ccw = mean(x_6..x_10 CCW)    y_cg,ccw = mean(y_6..y_10 CCW)
```

Accuracy figure — this is the number that summarises the robot:

```
r_cg,cw  = sqrt(x_cg,cw²  + y_cg,cw²)
r_cg,ccw = sqrt(x_cg,ccw² + y_cg,ccw²)
E_max,syst = max(r_cg,cw, r_cg,ccw)
```

Correction factors. **α is the sum, β is the difference** — that asymmetry is what the
bidirectional run buys you:

```
α = (x_cg,cw + x_cg,ccw) / (−4L) · 180/π        → wheelbase error   (Type A)
β = (x_cg,cw − x_cg,ccw) / (−4L) · 180/π        → wheel diameters   (Type B)

R  = (L/2) / sin(β/2)
Ed = (R + b/2) / (R − b/2)        b = 0.135 m nominal track
Eb = 90 / (90 − α)                b_actual = Eb · b
```

**Cross-check, do not skip:** the same α and β can be derived from y instead of x —

```
β = (y_cg,cw + y_cg,ccw) / (−4L) · 180/π
α = (y_cg,cw − y_cg,ccw) / (−4L) · 180/π
```

If the x- and y-derived values disagree materially, the run set is contaminated by
**non-systematic** error (a bump, a cable, a wheel catching). Repeat rather than trusting
the number. `umbmark.py` reports both and flags disagreement.

---

## Applying the correction

Two constraints specific to this robot:

- **No firmware knob.** Odometry is computed inside the closed-source firmware.
  `config_robot.py` exposes motor PID and servo offsets, nothing for odometry.
- **No per-wheel data.** Only body twist reaches us on `/odom_raw`, so the per-wheel
  `c_L`/`c_R` factors from the paper cannot be applied as written.

So corrections go in a small ROS node upstream of the EKF, applying what our data
supports:

| Measured | Applied as |
|---|---|
| average wheel diameter error | scale on `v_x` |
| `Eb` (wheelbase) | scale on `ω_z` |
| `Ed` (unequal wheels) | yaw bias proportional to `v_x` — "straight" curves |

**Only build this if the numbers justify it.** A few percent is not worth an extra node
in the chain; ~13% is.

## Expected results, and what is *not* a problem

- **`Eb` will probably be large.** UMBmark assumes 2-wheel differential; this is 4-wheel
  skid-steer, where turning is slip by design. The effective track that makes odometry
  work is meaningfully wider than the physical 0.135 m. That is physics, not a bad
  measurement.
- The paper reports roughly an **order-of-magnitude** improvement in `E_max,syst` after
  correction. Re-run the full 10 afterwards to confirm; a correction that doesn't
  measurably improve `E_max,syst` should be reverted, not kept on faith.
- Numbers are **surface-specific**. Note the surface next to the result.

## Then

Once `E_max,syst` is known and any correction is in place:

1. Record `floor_dataset` — the first bag with physically real odometry.
2. Cartographer mapping, save the map.
3. Nav2: expect lifecycle to reach `active` with the robot running, then 2D Goal Pose.
