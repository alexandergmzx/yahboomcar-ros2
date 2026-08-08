# SLAM output and RViz clutter: audit, research, and fixes

"SLAM and RViz look messy" turned out to be two independent problems that happen to
surface together: scan-matching/map quality (`slam_toolbox.yaml` tuning) and viewer
clutter (`nav2_debug.rviz` showing displays with no publisher during a pure-SLAM run).
Fixed in the same pass because they were reported together, but tested independently
below — a map-quality fix and a viewer-config fix have no reason to be entangled.

## 1. What's being fixed, and what is NOT a bug

Not in scope, and already documented: the `map→odom` "jump" RViz shows at corners
(measured up to 125 mm, worst at corners) is SLAM correcting for real odometry drift, not
a rendering bug — see [`rviz-guide.md`](../rviz-guide.md) §3. If that's what "messy" meant,
this doc doesn't change it; it's expected behavior, not a defect.

What this doc *does* address:

1. `slam_toolbox.yaml` has no robust loss function for the pose-graph solver, despite
   being otherwise carefully tuned for this robot's measured wheel slip.
2. `nav2_debug.rviz` carries Nav2-only displays and, in some invocation paths, a
   diagnostic-odometry display with no live publisher — both render as red/warning
   entries in the Displays panel.

## 2. Current-state audit

### `slam_toolbox.yaml` vs. stock `mapper_params_online_async.yaml`

Both read directly off this machine (`ros-jazzy-slam-toolbox` 2.8.5,
`/opt/ros/jazzy/share/slam_toolbox/config/mapper_params_online_async.yaml`), not from
memory or docs:

| parameter | stock default | this repo | why |
|---|---|---|---|
| `resolution` | 0.05 m | 0.02 m | 4×4 m arena; 5 cm grid would fit the whole room in 80 cells |
| `minimum_travel_distance` | 0.5 m | 0.10 m | small/slow robot; stock threshold barely takes any scans across 4 m |
| `minimum_travel_heading` | 0.5 rad | 0.10 rad | same reasoning |
| `correlation_search_space_dimension` | 0.5 m | 0.3 m | sized (in-file comment) to one lidar-tick's motion — see §4b, this reasoning doesn't match the actual scan-processing interval |
| `correlation_search_space_smear_deviation` | 0.1 | 0.03 | tightened alongside the search space |
| `distance_variance_penalty` | 0.5 | 0.3 | loosened: wheel odometry is measured untrustworthy (92% slip on the stand) |
| `angle_variance_penalty` | 1.0 | 0.6 | same |
| `min_pass_through` | 2 | *(unset, stock)* | not tuned in this pass — see §4d |
| `occupancy_threshold` | 0.1 | *(unset, stock)* | not tuned in this pass — see §4d |
| `check_min_dist_and_heading_precisely` | false | *(unset, stock)* | stock default already suits "rotational odometry is poor" (per upstream docs) — this robot's gyro is intermittently faulty, so leaving this at its default is itself correct, not an oversight |
| `ceres_loss_function` | None | **None → HuberLoss (this fix)** | see §4a |

Everything else in the file is either identical to stock or a small rate/timeout tweak
matched to the ~12 Hz lidar; not relevant to map cleanliness.

### `nav2_debug.rviz` displays with no live publisher, by invocation path

| display | topic | `slam_toolbox_launch.py` alone | `tools/simctl start` |
|---|---|---|---|
| Local Costmap, Global Costmap | `.../costmap` | no publisher | no publisher (simctl never starts Nav2) |
| Global Plan, Local Plan | `/plan`, `/local_plan` | no publisher | no publisher |
| `Navigation 2` panel | — | inert | inert |
| Odometry (lidar `/odom_laser`) | `/odom_laser` | **no publisher** (`laser_odometry` isn't launched here) | **live** — `simctl`'s `cmd_start` launches `laser_odometry` unconditionally at step `[3/6]`, not gated by any flag |

This table is why the fix keeps the EKF/lidar odometry pair in the new `slam_debug.rviz`
(it's genuinely live under the primary `simctl start` workflow) but drops the Nav2-only
displays entirely (no invocation path this repo has ever gives them a publisher).

## 3. External research (cited, cross-checked against what's installed on this machine)

- **`github.com/SteveMacenski/slam_toolbox`** README, "Configuration" section
  (https://github.com/SteveMacenski/slam_toolbox#configuration) — the HuberLoss
  recommendation quoted in §4a. Cross-checked against the actually-shipped
  `config/mapper_params_online_async.yaml` on the `ros2` branch, which matches what's
  installed locally.
- **Ceres loss-function enum values confirmed by binary inspection, not docs alone**:
  `strings /opt/ros/jazzy/lib/libceres_solver_plugin.so` → `None`, `HuberLoss`,
  `CauchyLoss`.
- **`ros-perception/laser_filters`** (https://github.com/ros-perception/laser_filters) —
  `LaserScanRangeFilter`, `LaserScanSpeckleFilter`. `ros-jazzy-laser-filters` 2.0.9 is
  already installed on this machine — documented here as a follow-up option, not
  implemented (§4d).
- **`nav2_bringup`'s reference view**,
  `/opt/ros/jazzy/share/nav2_bringup/rviz/nav2_default_view.rviz` (matches
  https://github.com/ros-navigation/navigation2/blob/main/nav2_bringup/rviz/nav2_default_view.rviz) —
  read directly, not inferred from the C++ header. It confirmed the TF display's frame
  allowlist mechanism actually used in a working config is `Frames: / All Enabled: false`
  with each wanted frame listed explicitly, **not** the `Filter (whitelist)` regex
  property (that field exists on the class but is left `""`, unused, even in this
  reference file). An earlier draft of this fix cited the regex property, inferred from
  `rviz_default_plugins/displays/tf/tf_display.hpp` alone — corrected after actually
  reading a real `.rviz` file instead of trusting the header.

## 4. Fix recommendations, with rationale

### 4a. `ceres_loss_function: HuberLoss` — implemented

Unset in this repo's config (defaults to `None`), despite the file otherwise being
extensively tuned for exactly the situation the `slam_toolbox` maintainer names as the
reason to set it: *"If you have an abnormal application or expect wheel slippage, I might
recommend a HuberLoss function... a really good catch-all."* This robot has measured ~92%
wheel slip on a stand. `HuberLoss` reweights outlier residuals in the *global* pose-graph
solve (Ceres) — a different layer from the existing loose local-matcher penalty block,
which is left unchanged (see the rationale comment in `slam_toolbox.yaml` itself).

**Open, unproven**: whether the already-loose penalty block blunts `HuberLoss`'s effect by
keeping most residuals under its transition point. Not resolved by reasoning — see §6.

### 4b. `correlation_search_space_dimension` — tested, not blind-committed

The in-file comment justifies `0.3` as "one lidar-tick's motion" (~25 mm at 0.3 m/s over
1/12 s). But `slam_toolbox` only processes a scan once `minimum_travel_distance` /
`minimum_travel_heading` (0.10 m / 0.10 rad) is crossed since the last processed scan —
several ticks, not one — and under measured slip the true pose can diverge from the
odometry-reported prior by more than the tick-based figure assumes. This is presented as a
**hypothesis**, tested empirically in §6, not asserted as fact.

### 4c. RViz split — implemented

New `yahboomcar_nav/rviz/slam_debug.rviz`, the new default for
`slam_toolbox_launch.py rviz:=true` and for `tools/simctl start`. Drops the Nav2-only
displays and the `Navigation 2` panel (§2 table: no publisher in any invocation path);
trims the TF tree to `map`/`odom`/`base_footprint`/`laser_frame` via the confirmed
`Frames`/`All Enabled: false` allowlist; keeps the EKF/lidar odometry pair, since it's live
under `simctl start`. `nav2_debug.rviz` is unmodified and still available via
`rviz_cfg:=.../nav2_debug.rviz` for combined SLAM+Nav2 debugging sessions.

### 4d. Escalation only — not implemented in this pass

`min_pass_through: 3` (speckle suppression) and `ros-perception/laser_filters`
pre-filtering (`LaserScanRangeFilter`, `LaserScanSpeckleFilter`) are standard techniques
for noisy/slipping robots. Both packages are confirmed already installed on this machine,
but installed isn't the same as demonstrated-necessary — per this repo's habit of updating
only what's demonstrably broken, these are documented here as the next thing to try if
§4a/§4b don't resolve the reported messiness, not implemented speculatively.

## 5. What changed, file by file

- `yahboomcar_ws/src/yahboomcar_nav/params/slam_toolbox.yaml` — added `ceres_loss_function: HuberLoss`.
- `yahboomcar_ws/src/yahboomcar_nav/rviz/slam_debug.rviz` — new file.
- `yahboomcar_ws/src/yahboomcar_nav/launch/slam_toolbox_launch.py` — added `rviz_cfg` launch arg, defaulting to `slam_debug.rviz`.
- `tools/simctl` — RViz step now launches `slam_debug.rviz` instead of `nav2_debug.rviz`.
- `docs/rviz-guide.md`, `docs/porting-notes.md` — one-line pointers to this doc.

## 6. Verification protocol and results

Two independent witnesses per configuration, not one — a map's bounding-box size can be
correct while its walls are internally doubled/fuzzy, the same class of blind spot this
repo's own audit record flagged when an "at rest" check looked at translation only and
missed rotation:

- **Witness A — map geometry**: `map_saver_cli`, then bounding box (`width_px ×
  resolution`) against the known 4×4 m arena. A row-band wall-thickness sub-metric was
  tried first and abandoned: with this drive pattern the map's trinary PGM export has only
  ~15 confidently "occupied" pixels total (everything else is "unknown" or "free" —
  trinary export has no intermediate confidence value), too sparse for a band count to
  mean anything. Whole-map occupied/unknown/free pixel counts are reported instead, as a
  coarser signal.
- **Witness B — sensor agreement**: `ros2 bag record /odom_raw /odom /odom_laser /imu
  /scan` during the same driven loop, then `tools/sensor_agreement.py <bag>` — an existing
  tool, independent of whatever `slam_toolbox` itself computed. (First run of this
  protocol recorded `/odom` instead of the `/odom_raw` this tool actually requires; fixed
  for every run after — see the `huber_dim03` row.)

Driven with `tools/sim_patrol.py --laps 2 --side 1.0` (built for exactly this: "for
demonstrations and for exercising SLAM"), which forces repeated corners.

**All results below are single runs, not the two-per-configuration this doc originally
called for** — time-boxed after the search-space result below turned out to be a clear
enough "don't change it" signal that further repeats stopped being the highest-value next
step. Take the exact numbers as indicative, not statistically tight.

| run | search dim | ceres loss | slip | map dims | occ/unk/free px | lidar/encoder transl. | lidar/IMU yaw | notes |
|---|---|---|---|---|---|---|---|---|
| `baseline_dim03` | 0.3 | None | 0 (default) | 4.04×4.04 m | 17 / 39312 / 1475 | 1.037 (−4%) | 0.957 | true pre-fix config |
| `huber_dim03` | 0.3 | HuberLoss | 0 (default) | 4.04×4.04 m | 14 / 39287 / 1503 | — | — | Witness B bag was missing `/odom_raw` (see above); map witness only |
| `huber_dim05` | 0.5 | HuberLoss | 0 (default) | 4.04×4.06 m | 15 / 39525 / 1466 | 1.038 (−4%) | 0.958 | low-slip driving barely stresses the mechanism §4b theorizes about |
| `slip04_dim05` | 0.5 | HuberLoss | 0.4 | 4.12×4.12 m | 18 / 40939 / 1479 | 0.475 (**+53% apparent slip**) | 0.907 | stock search-space width, under real slip |
| `slip04_dim03` | 0.3 | HuberLoss | 0.4 | 4.04×4.04 m | 17 / 39324 / 1463 | 0.469 (**+53% apparent slip**) | 0.897 | this repo's tuned search-space width, under the same slip |

**Result: §4b's hypothesis was not supported.** Under low/default slip, `dim=0.3` and
`dim=0.5` are indistinguishable — expected, since gentle floor driving doesn't stress
the mechanism being tested. Under `slip=0.4` (chosen because §4b is specifically about
slip-driven odometry error, and gentle driving turned out not to exercise it — confirmed
by the ~53% apparent-slip readings on both slip runs, sanity-checking that the injected
slip actually took effect and was independent of the SLAM parameter under test), the
*narrower* `dim=0.3` produced a map closer to the true 4.00×4.00 m arena (4.04×4.04) than
the wider stock `dim=0.5` (4.12×4.12) — the opposite of what was predicted. The
sensor-agreement numbers move in the same small direction (yaw agreement 0.897 vs 0.907,
essentially a tie at this sample size). **Decision: left `correlation_search_space_dimension`
at its original 0.3, unchanged.** The in-file comment's reasoning ("one lidar tick's
motion") is still not quite the right justification for the number — that critique in §4b
stands as written — but the number itself already tests fine, and there is no measured
case for moving it. Changing a value that isn't demonstrably broken, on reasoning that
turned out not to predict the measured outcome, is exactly the mistake this repo's own
audit record warns against.

`ceres_loss_function: HuberLoss` was kept despite not producing a visible difference in
these particular runs (`baseline_dim03` vs `huber_dim03`: map metrics are close, and the
`huber_dim03` bag can't be compared on Witness B due to the recording mistake noted
above). It's kept because it is the maintainer's named, low-risk, structurally-justified
recommendation for exactly this robot's situation (measured 92% slip on the stand — a much
more severe condition than anything reproduced here in a moving-robot test), not because
this doc's own limited testing demonstrated a map-quality win. That distinction is
deliberate, not glossed over: an unfalsified but also unconfirmed change, landed on
authority + low risk rather than on this doc's own evidence.

**RViz clutter fix — verified with real evidence, not just static review:**

- `ros2 topic info` while the stack was running confirmed `/local_costmap/costmap`,
  `/local_plan`, `/plan` at **Publisher count: 0** (RViz's own subscription was the only
  reason they appeared in `ros2 topic list` at all) — the literal mechanism behind the
  red/warning entries in `nav2_debug.rviz`. `/odom_laser` had **Publisher count: 1**,
  confirming the correction in §2/§4c to keep that display.
- A real display was available (`DISPLAY=:0`), so both configs were actually opened in
  `rviz2` and screenshotted, not just reasoned about. `slam_debug.rviz` rendered with
  **Global Status: Ok** (green), `TF` showing `Filter (whitelist)`/`Filter (blacklist)`
  correctly present-but-empty and a collapsed `Frames` allowlist (matching the confirmed
  `nav2_bringup` reference pattern from §3), and no Nav2-only entries in the Displays
  tree at all. `nav2_debug.rviz`'s equivalent session showed the `Navigation 2` panel
  stuck on all-`unknown` status fields the whole time SLAM ran — a second, independent
  symptom of the same root cause (no Nav2 running), beyond the red-icon displays.

## 7. Open questions and honest limits

- **The search-space hypothesis in §4b did not survive contact with measurement.** It's
  left in this doc rather than deleted, specifically so the reasoning and the result that
  contradicted it stay attached to each other — a plausible-sounding argument that turned
  out wrong is more useful on the record than quietly removed.
- Every result in §6 is a single run per configuration, not the two-per-configuration
  originally planned. The slip-vs-no-slip *direction* of effect (53% apparent slip only
  appearing when `--slip 0.4` was set) is a real, mechanism-level sanity check and not
  noise. The *small* dim=0.3-vs-0.5 gap under slip (4.04 vs 4.12 m, ~2%) is not repeated
  enough to rule out noise — treat "0.3 is at least as good as 0.5" as reasonably solid,
  and "0.3 is measurably better" as not established.
- `huber_dim03`'s bag is missing `/odom_raw`, a recording mistake caught and fixed for
  every later run — that one row has no Witness B. Not repeated, to keep total sim time
  bounded once the higher-value search-space question was answered.
- The `HuberLoss` / loose-penalty-block interaction flagged in §4a remains genuinely
  untested: no run in §6 isolated it directly (all HuberLoss runs already carry the loose
  penalty block, since that block was never toggled).
- All testing here is 2D-simulator-only, per this repo's standing caveat that simulation
  validates the stack, not the robot — none of this has been run against the real car.
