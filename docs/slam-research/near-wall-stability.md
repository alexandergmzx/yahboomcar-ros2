# Near-wall SLAM stability: the mechanism, the survey, and the same-bag A/B

**2026-08-10 afternoon. Context:** with rotation behaving at operator speeds, Alex's
fun-mode sessions now destabilize when the robot gets close to a wall ("went really
good until I got too close to a wall"). This document is the survey of the numeric
methods that exist for exactly this failure, the measurement of which mechanism is
actually active in this stack, and the same-bag A/B that tests the candidate fixes.
Style per house rules: measured vs assumed marked, negative results in bold,
rejected/failed arms kept.

## The failure, located and measured

`tools/…/wall_moment_probe.py` (scratchpad; preserved with the run artifacts) over
three bags — Alex's organic run and two controlled wall-approach sessions (scripted
approach → wall-follow → retreat, closed-loop on truth, governed via /cmd_vel_raw):

| bag | worst map→odom jump | at wall distance | jumps >50 mm |
|---|---|---|---|
| organic `20260810-125354` | **746 mm / 21.0°** at t=204.6 s | 0.367 m | 5 |
| controlled A `20260810-131057` | **3308 mm / 31.5°** | 0.085 m | — |
| controlled B `20260810-131558` | **1875 mm / 20.8°** | 0.098 m | — |

The coupling with wall distance is monotonic (organic bag, 600-scan sample):

| truth dist to wall | scans | valid beams (med) | isotropy (med) | map→odom jump p95 |
|---|---|---|---|---|
| 1.0–2.0 m | 465 | 358 | 0.962 | 40 mm |
| 0.6–1.0 m | 24 | 355 | 0.812 | 62 mm |
| 0.3–0.6 m | 112 | 348 | **0.711** | **71 mm (max 746)** |
| <0.3 m | 13 | 345 | 0.657 | 35 mm* |

*the <0.3 m row is 13 scans; the catastrophic jump landed in the 0.3–0.6 band as the
robot crossed it.

**Beam starvation is substantial under sustained proximity but still not the
driver [measured, corrected by the verification pass]**: the organic bag's
brief pass kept ~345/358 valid beams, but the controlled contact bags lose
32–41% of beams in the <0.3 m band (medians 211 and 243 of 358). Even so,
200+ beams keep seeing the other three walls — enough constraint that pure
starvation cannot explain a metre-scale re-anchor. What degrades alongside is
the constraint GEOMETRY (normals isotropy 0.96 → 0.66) and, under ~0.12 m
(`range_min`), the near wall's returns die entirely while the far walls
remain — the scan is then consistent with a family of poses slid along the
invisible wall.

## Survey: what the field does about this

- **Eigenvalue degeneracy sensing** (Zhang & Singh's line;
  [real-time degeneracy sensing & compensation](https://arxiv.org/html/2412.07513),
  [DARE-SLAM](https://arxiv.org/pdf/2102.05117)): eigendecompose the match
  Hessian/covariance; treat directions whose eigenvalue drops below a floor as
  unobservable — don't update along them, or inflate their covariance.
  **This repo already implements exactly this** in
  `yahboomcar_localization/laser_odometry_node.py` (covariance inflated along the
  weak eigenvector, degenerate matches dropped, not published) — but it protects
  only `/odom_laser`, which no sim path fuses. slam_toolbox has no equivalent.
- **Degeneration-aware weighting** (same survey line): dynamically RAISE the
  motion-prior weight when scan geometry is degenerate — i.e., the penalty terms
  are the knob that already exists in slam_toolbox
  ([params reference](https://github.com/SteveMacenski/slam_toolbox/blob/ros2/README.md)).
- **Pre-filtering**: range floors and speckle/footprint filters ahead of the
  matcher ([laser_filters practice](https://johntgz.github.io/2022/01/09/the_ultimate_guide_to_laser_filters/));
  in-toolbox, `min_pass_through` suppresses speckle at the occupancy layer — the
  escalation `docs/slam-research/findings.md` §4d pre-authorized and nobody had
  tried yet.

## The stack-specific insight: the penalty rationale has inverted

`slam_toolbox.yaml`'s penalty block is deliberately LOOSE
(`distance_variance_penalty 0.3` / `angle_variance_penalty 0.6` vs stock 0.5/1.0),
documented as "odometry is the least trustworthy input available." That was written
against the raw encoder lie. At the slow speeds where rotation now behaves, the
prior is decent — and looseness is precisely the freedom the matcher uses to slide
along a wall whose along-direction the scan cannot pin. §4a
(`findings.md`) asked whether the loose penalties blunt matching; the arms below
are its isolation test.

## The A/B (same-bag replays, `replay_slam_bag.py`, domain 68, map→odom stripped uniformly)

Arms:

- **N0** — canonical `slam_toolbox.yaml`, bag as recorded (baseline).
- **N1** — penalties to stock + floor raised: `distance_variance_penalty 0.5`,
  `angle_variance_penalty 1.0`, `minimum_distance_penalty 0.7`.
- **N2** — N1 + `min_pass_through: 3`, and the bag's `range_min` raised to
  0.16 m (kills the flickering near bin the RTX lidar half-loses anyway —
  validity 63–89% below 0.5 m).

Scored by `score_slam_map` against the authored arena reference (the wall rows —
spans and duplicate-wall extent — are exactly what a re-anchoring event inflates).

### Controlled bag A (worst case: 0.085 m standoff, 3.3 m live jump)

| arm | spans (m) | dup wall (m) | wall thick (m) | verdict |
|---|---|---|---|---|
| N0 | 9.36 × 7.20 | 0.46 | 0.040 | **FAIL** — the live failure reproduces offline |
| N1 | 8.64 × 6.84 | 1.32 | 0.020 | **FAIL** |
| N2 | 8.32 × 6.60 | 1.22 | 0.020 | **FAIL** |

**The parameter hypothesis is FALSIFIED at close range**: stock-restored
penalties (N1) and penalties + range-floor + speckle suppression (N2) do not
rescue a <0.1 m wall encounter; all three maps show the same structure — one
crisp room, then a FAN of progressively rotated wall copies (side-by-side
render preserved with the artifacts). A fan is not a one-time re-anchor: the
prior was continuously sweeping while scans matched onto each successive copy.

## The actual mechanism: a wall-blocked body under spinning wheels, not geometry

The fan demanded a rotating prior, and the bag has it. During bag A's jump
window (t≈146–153 s), `/odom_raw` yaw rate vs truth yaw rate, per second:

| t (s) | odom wz (rad/s) | truth wz (rad/s) | ratio |
|---|---|---|---|
| 146 | 0.026 | 0.004 | 6× |
| 147 | 0.704 | 0.027 | **26×** |
| 149 | 0.600 | 0.048 | 12× |
| 152 | 0.701 | 0.059 | 12× |

The free-floor encoder lie is ~2.9×. **In the blocked-at-the-wall regime it
runs ~6–26×** (bag B independently: peak 26× at t=127, one 118× second at
t=138 [verification pass]): the body barely rotates while the wheels spin
at command, the encoders honestly report a rotation that never happened,
the vendor EKF fuses it as pose AND twist, and slam_toolbox receives a
prior sweeping tens of degrees per second. Whether the blocking constraint
is literal chassis-wall contact is INFERRED from the kinematic mismatch,
not directly measured (no contact signal was queried; the URDF half-extents
leave ~0.5–1.5 cm of nominal clearance at the 0.085 m standoff — a shoved
feather box in the gap is equally consistent). What is measured is the
mismatch itself, and that suffices: the prior sweeps, the map fans. Fun
mode's braking-off is the only reason a sub-0.35 m standoff is reachable.

Penalty regimes at both tested points of the axis (loose canonical, stock
N1) fail to veto the sweep while still permitting real motion — two points
do not prove NO penalty setting could, but loosening and tightening both
failing points away from that axis entirely. The isotropy degradation
measured above is real but SECONDARY: geometry weakens exactly when the
prior corrupts, removing the matcher's ability to resist.

Two regimes, one channel: the organic bag's 746 mm event shows **no
blocked-wheel signature** — its window reads 1.9–2.5× during a genuine fast
rotation [verification pass re-probe], i.e. the ordinary free-floor lie at
speed, mostly healed by loop closure (its offline N0 nearly passes). The
catastrophic fan requires the sustained blocked regime, seen in both
controlled bags. Same wheel-yaw channel, two severities.

### Arm N4 — the fix candidate, tested offline

The IMU is bolted to the BODY: wall-contact wheel spin never enters its yaw
channel (Isaac publishes true body rate; the real ICM-42670-P measures the
body too — same physics). `ekf_corrected.yaml` already fuses IMU yaw-rate
and wheel vx as twist-only with a rejection threshold. Arm N4 rebuilds the
bag's `odom→base_footprint` by dead-reckoning exactly that input set
(IMU yaw integrated at 25 Hz, odom vx projected along it) and replays through
the CANONICAL slam yaml:

| bag | arm | spans (m) | dup wall (m) | verdict |
|---|---|---|---|---|
| A (worst) | N4 imu-yaw prior | **4.30 × 4.26** | **0.20** | near-PASS: span rows over the ±0.20 m bar by 0.10 / 0.06 m; every other row PASS (89.4% known, wall 0.060 m) |
| B | N4 imu-yaw prior | 4.50 × 4.58 | 0.68 | partial rescue: spans re-scaled from 8.56 m to ~4.5, dup wall still FAIL |

From 9.36 m of fanned room to 4.30 m with two rows centimetres off the bar —
using the same scans, the same contact, the same canonical SLAM parameters,
and an integrator far cruder than the real corrected EKF (25 Hz Euler,
no rejection gate, no laser input). The yaw channel was the fault.

### Cross-bag confirmation (organic + controlled B)

| bag | N0 (canonical) | N1 (tight penalties) |
|---|---|---|
| organic (0.367 m near-miss) | 4.16 × 4.36, dup 0.16 — FAIL by one span row (+0.16 m) | 4.16 × 4.48, dup 0.16 — FAIL, **slightly worse** |
| controlled B (0.098 m contact) | 8.56 × 4.42, dup 0.24 — FAIL | 8.64 × 4.70, dup 0.36 — FAIL, **slightly worse** |

**N1 never changes a verdict, on any of the three bags.** Row-level effects
are mixed — on bag A it shrank spans (9.36→8.64) while tripling duplicate
wall (0.46→1.32); on the organic and B bags its span rows are worse by
2.8% and 6.3%, beyond the 0.5% repeatability baseline (with the honest
caveat that that baseline was measured between PASS-regime maps and its
transfer to fan-failure maps is unestablished — no same-config replay
replicate exists). The §4a question is answered at the level that matters:
the loose penalty block is NOT the cause of the near-wall failure, and
tightening it rescues nothing. Note also the organic bag's N0 nearly passes
offline — Alex's live 746 mm event was a borderline transient largely
healed by loop closure; the catastrophic regime requires the sustained
blocked-wheel condition.

## Verdict and recommendation

**The near-wall instability is corrupted wheel odometry under an external
constraint, not scan geometry and not SLAM parameters.** Ranked:

1. **Convicted**: near-wall instability repeats across all 3 bags and both
   live sessions; the blocked-wheel mechanism (yaw lie ~6–26× vs ~2.9×
   free-floor, fused as pose and twist by the vendor EKF, prior sweeping
   the map into a fan) is directly evidenced in the TWO controlled bags.
   The organic event is the same wheel-yaw channel in its ordinary
   free-floor regime at speed (1.9–2.5×) — milder, loop-closure-healed.
2. **Falsified**: penalty tightening (N1) and range-floor + speckle (N2)
   rescue nothing on any bag (verdict-level; row effects mixed, see
   cross-bag table). Do NOT ship a `slam_toolbox_nearwall.yaml`; the
   canonical yaml survives on evidence. (The §4d escalation remains untried
   for its ORIGINAL sparse-map purpose; nothing here retires it there.)
3. **Demonstrated**: structurally excluding the wheel-yaw channel (N4:
   IMU yaw + wheel vx only) rescues the worst bag to centimetres of the
   bar and re-scales bag B from 8.6 m to 4.5 m. The bag-B residual is
   PLAUSIBLY the wheel-vx channel lying under the same blocked condition
   (phantom forward travel) — plausible because it is the one channel N4
   retains, but **unmeasured**: no vx-vs-truth probe was run, and that is
   the first check for whoever picks this up.

**Recommendation (Alex's call — touches the sim's default fusion, morning
decision 5 now with teeth):** run sim sessions on the corrected-EKF fusion,
with eyes open about what it is and is not. `ekf_corrected.yaml` is NOT
N4's architecture: it still fuses wheel vyaw (its header says so — twist
vx AND vyaw), merely gated by `odom0_twist_rejection_threshold: 1.542`,
alongside IMU yaw-rate. Whether that Mahalanobis gate actually fires on a
26× lie depends on the covariances the firmware stamps and was tested by
no arm here — and when it fires it rejects the WHOLE odom twist (vx and
vyaw together), a behavior difference from N4. The stronger variant, which
IS N4's structure, is `odom0_config` with vyaw=false (wheel yaw excluded
outright, rotation twist from the IMU alone) — one line, testable in the
same A/B harness. The full corrected stack already exists in-tree: simctl
launches `laser_odometry` unconditionally and `ekf_corrected.yaml` fuses
`/odom_laser` as its pose input; only the bringup wiring (vendor `ekf.yaml`
hardcoded) keeps sim sessions on the vulnerable config. Validation shape
when approved: one fun wall-approach session per EKF variant (vendor /
corrected-as-is / corrected+vyaw-off), scored per this document's method —
the session-recording infrastructure makes each attempt a complete record.

Secondary, cheaper mitigations, in order: keep fun-mode boxes/walls
approaches above ~0.15 m when a usable map matters (operator guidance —
below `range_min` the near wall is invisible regardless of fusion);
a contact heuristic (odom-vs-IMU yaw-rate disagreement > 3× for > 0.5 s ⇒
governor stop) would make contact self-announcing rather than silent.

## Bounding factors, stated plainly

- The Isaac near-field residual dropout (~10%, OPEN) bounds how good ANY
  configuration can be under ~0.5 m; the range-floor arm sidesteps rather than
  fixes it.
- These are SIMULATOR measurements. The real lidar's near-field validity is
  ~98.6%; the real car's odometry lies differently (~7.7% yaw). The penalty
  question must be re-asked on hardware bags before any hardware retune —
  the delivery plan's gates apply, and this document does not touch them.
- A scan-geometry gate in front of slam_toolbox (drop scans below an isotropy
  floor, the laser_odometry approach promoted to the SLAM input) was designed as
  arm N3 and NOT RUN — parked unless N1/N2 prove insufficient, because a gate
  that drops scans interacts with the governor's stale-scan stop and must be
  designed with that coupling in mind.

## Provenance and verification

Sessions `20260810-131057-isaac-fun-d66` / `20260810-131558-isaac-fun-d66`
(controlled, scripted approach; full session records incl. bags with
/cmd_vel) and Alex's organic `20260810-125354-isaac-fun-d66`. All arm maps,
scorer JSONs, the N0/N1/N2 fan render, the N0-vs-N4 comparison figure, the
probe scripts and the arm parameter files are preserved under
`MicroROS-assets/maps/near-wall-20260810/`.

Every numeric claim in this document was adversarially re-checked by an
independent three-lens verification pass (tables vs scorer JSONs; mechanism
claims re-probed from the bags; overclaim hunt) before commit. Eight of its
findings forced corrections — among them: the beam-count generalization was
wrong (organic-only), the contact-signature range was trimmed (6–26×, not
10–26×), the organic event carries NO blocked-wheel signature (1.9–2.5×,
free-floor regime), physical contact is inferred rather than measured, and
the corrected-EKF config was being credited with N4's structure it does not
have. The pre-correction claims are in git history; the corrections are the
document above.

## Live A/B (same evening): the offline rescue does NOT transfer as-configured

`simctl --ekf {vendor,corrected,corrected-novyaw,n4-pure}` was implemented and
two live wall-approach sessions run (all auto-recorded; /odom_laser added to
session bags for the second):

| live variant | map (stop) | dup wall | worst jump | jumps >100 mm |
|---|---|---|---|---|
| vendor (baseline bags) | 9.36 × 7.20 | 0.46 | 3308 mm | few (5 >50 mm organic) |
| corrected-novyaw (`20260810-164606`) | 5.62 × 8.50 | 0.54 | 1259 mm | **23** |
| n4-pure (`20260810-165133`) | 5.92 × 9.26 | 1.46 | 2240 mm | **73** |

**Both corrected variants FAIL live, and instability scales with IMU reliance**
— the opposite of the offline ranking. The bag convicts the difference:
raw IMU yaw rate is honest (median ratio to truth 1.012, corr 0.763 while
turning) but the EKF's yaw OUTPUT under-rotates at **0.727× with corr 0.531**.
The offline N4 arm integrated the IMU rate at gain 1.0; the live filter
low-pass-filters it, and with no absolute yaw source the attenuation never
recovers — a permanently lagging prior that slam_toolbox fights everywhere,
not only at walls. **No sign/frame error** (ruled out by the same probe).

Consequences, applied:

- **The vendor default was NOT flipped** — the win criteria did their job.
  `--ekf` ships as an EXPERIMENTAL flag, default `vendor`, so nothing changes
  without opt-in. The param variants stay, labeled (novyaw: variant;
  n4pure: diagnostic-only, nothing bounds drift).
- The offline causality stands (same-bag N4 rescue is a replay fact); what
  failed is THIS EKF CONFIGURATION's transfer of it. The next lead is
  specific and measured: find why `ekf_filter_node` attenuates a clean
  25 Hz yaw-rate measurement by ~27% (process-noise vs measurement-covariance
  balance; the sim IMU's stamped covariances; `frequency: 10` against a
  25 Hz input) — offline, against the recorded n4-pure bag, BEFORE any
  further live sessions. Tuning it blind live would be exactly the
  tune-before-diagnose failure the delivery plan forbids.

## Numeric identification (night session 2026-08-10n): the 0.727× convicted

Offline system-identification harness (scratchpad `ekf_offline_harness.py`,
preserved with the artifacts): the REAL corrected chain (madgwick + ekf_node
+ statics, `use_sim_time`) fed the recorded n4-pure bag's sensors on domain
68, EKF output probed live, transfer computed vs the bag's ground truth.
**Instrument gate passed first**: the harness reproduces the live phenomenon
(ratio 0.760 vs live 0.727) and adds the decisive number — the EKF yaw
output lags truth by **2.8 s**. A SLAM prior 2.8 s behind a turning robot
can only fan.

One hypothesis per arm, bounded list, all run:

| arm | change | ratio | corr | lag |
|---|---|---|---|---|
| baseline | ekf_n4pure as-is | 0.760 | 0.702 | 2.80 s |
| H1 | stamp IMU cov (0.02 rad/s)² | **0.612** | 0.619 | 2.9 s — **worse**: the stamped variance exceeds the zero-substitution epsilon, LOWERING trust |
| **H2a** | process noise q(yaw,vyaw) ×10 | 1.008 | 0.935 | 0.22 s |
| **H2b** | process noise q(yaw,vyaw) ×100 | **1.001** | **0.994** | **0.02 s** |
| H3 | filter frequency 10→25 Hz | 0.731 | 0.610 | 2.84 s — dead |
| H4 | fuse raw /imu (bypass madgwick) | 0.757 | 0.702 | 2.80 s — **madgwick exonerated** |
| H5 | dynamic_process_noise on | 0.759 | 0.684 | 2.78 s — dead |
| novyaw+pn | H2b matrix + /odom_laser pose (odom1) kept | **0.415** | 0.629 | −0.02 s — the laser-pose input HALVES the yaw transfer at high measurement trust; THIS row is why `ekf_sim_pnfix.yaml` has no odom1 (one bag, one config — hardware must re-ask) |

**The cause is the process-noise/measurement-covariance balance, exactly as
robot_localization's own reference params warn** (the [ADVANCED] note at
`/opt/ros/jazzy/share/robot_localization/params/ekf.yaml:178-188`: fused
velocity is a weighted average of prediction and measurement; "sluggish
convergence... especially noticeable with LIDAR data during rotations";
remedy = inflate process noise). Our corrected configs never set
`process_noise_covariance`, so the defaults (q_yaw 0.06, q_vyaw 0.02) let
the constant-velocity prediction dominate a clean 25 Hz gyro. At ×100 the
filter finally believes its sensor: unit gain, 20 ms lag — the offline-N4
behavior achieved INSIDE robot_localization, no custom node needed.

Alex's framing was right: this was a numerical-methods problem — a filter
gain, corrected by a measured ×100 on two process-noise terms — not a
robotics problem.

## Live G3 (night session): dominance without the gate — default NOT flipped

`--ekf pn-fix` (`ekf_sim_pnfix.yaml`, committed dcce295), three live sessions
(all auto-recorded; the map_saver 2 s subscription race hit twice — maps
recovered offline from the session bags, infra nit filed):

| metric | vendor live wall | pn-fix wall #1 (`182351`) | pn-fix wall #2 (`183430`) |
|---|---|---|---|
| worst map→odom jump | 3308 mm / 31.5° | 550 mm / 8.0° | **236 mm / 1.7°** |
| jumps >100 mm | (fans) | 23 | 18 |
| stop/recovered map | 9.36 × 7.20, dup 0.46 | 4.96 × 4.58, dup 0.34 | 4.50 × 4.58, dup 0.28 |
| open-space jump p95 | 199 / 228 mm (wall bags A/B; the organic free-drive bag reads 40 mm) | 23 mm | 22 mm |
| patrol map (`182829`) | 7.24 × 7.08 FAIL | — | 4.16 × 4.20, dup 0.14 — ≈ the truth-prior gold standard (4.16 × 4.16) |

**pn-fix strictly dominates vendor in every measured dimension** — worst
jump 14× smaller, patrol at gold-standard quality, open space better —
and the residual instability is CONCENTRATED toward the wall (wall #2:
p95 210 mm under 0.3 m, 143 mm at 0.3–0.6 m, 22–31 mm beyond 0.6 m — the
probe's ±1 s window smears contact events into neighbour bins). It is not
strictly confined: the patrol session's TF stream shows 8 jumps >100 mm in
open space (worst 179 mm at 0.76 m) even though its MAP is gold-standard —
the map is the deliverable and the map is clean, but "stable everywhere but
the wall" would overstate the TF stream. The leading explanation for the
in-band residual remains the wheel-vx channel feeding phantom translation
during the grind — PLAUSIBLE and still UNMEASURED (no vx-vs-truth probe
exists yet; it is decision 12 for a reason).
But the G3 criteria (map PASS-or-≤0.05 m over, near-wall jumps <100 mm —
set in the session plan before the live runs, though only in the plan file,
not repo-registered: future gates belong in the run report BEFORE execution)
were **not met in two attempts, and the arm is closed** per the bounded-retry
rule. The Isaac default therefore REMAINS `vendor` tonight;
flipping on a dominance argument instead of the agreed bar is a morning
decision, not a 3 A.M. one.

What would close the gap, in evidence order: (a) don't grind the wall —
away from it pn-fix's MAPS are already at gold-standard quality (its TF
stream still carries occasional >100 mm corrections, see above);
(b) a contact detector zeroing wheel-vx trust during the blocked regime
(odom-vs-IMU disagreement, decision 13's alarm made into a fusion gate);
(c) the delivery-plan-grade fix nobody simulated yet: braking ON sessions
never enter the band at all.

*(Night-session sections verified by a second adversarial two-lens pass:
25 claims checked, 7 corrections forced and applied — among them the vendor
open-space provenance cell, the omitted novyaw+pn arm row, the "confined"
overstatement against the patrol TF stream, and the unregistered-gate
wording. Pre-correction text is in git history.)*

## The speed layer (evening round, Alex driving): scan fixed, aggression layer named

Alex live-approved pn-fix ("it autocorrects the map to keep tracking the real
car") and reported: at higher speed "the scan destabilized first, then the
rest of SLAM." Their session bag (`20260810-190343`) pinned it: pacing had
bottomed at 3.0 renders/s again, and the slow→fast walls-fit rms went
0.023 → 0.074 m — at 3 renders/s a 12 Hz revolution assembles across a
~0.33 s render boundary, a tens-of-cm discontinuity INSIDE single scans at
driving speed. (Content lag as originally framed did NOT spike at speed —
lag_frac was 0.034 in fast windows; the seam discontinuity is the correct
mechanism, and the earlier "content lag" framing is superseded by it.)

**Fix landed (decisions 2+4 closed, commit f1c1902):** trim policy extracted
pure + tested (8 tests; two of the four recorded walks are replay fixtures, the other two cited in the docstring); floor =
SCAN_HZ makes the downward walk structurally impossible; a divergence guard
covers the remaining upward regime; the false '0.0 of 12 Hz' warning is
gone. **Live validation (`20260810-195417`, scripted external patrol via sim_patrol at 0.6 m/s + 1.0 rad/s under fun mode — the manifest reads patrol:false because fun suppresses simctl's OWN patrol, and max_speed:0.35 does not govern fun; commit f1c1902's 'fast patrol' shorthand carries the same nuance):
pacing held 12.0 all session, relay drops 10→1, fast-window fit rms
0.026 ≈ slow quality — the scan layer Alex reported is FIXED.**

**The layer behind it, named and parked:** at that same aggression the MAP
still fails (5.70 × 6.66, jump p95 ~950 mm, with clean scans). Two offline
arms on the same bag both died: F25 (filter at 25 Hz) made it WORSE
(dup wall 2.80 — noisier prior); NOVX (drop wheel vx, gyro-only prior)
lost tracking to a near-empty map — slam_toolbox here cannot live without a
translation prior. Shared clue: the EKF-vs-truth yaw CORRELATION collapses
to ~0.5 at this aggression even at unit ratio — and the plant itself is
chaotic there (achieved-yaw variance 0.48/0.18/0.05 rad/s across sessions
at fixed command, long documented). Caveat for the next pass: the pn-fix
baseline transfer on THIS bag was not separately harnessed (the live
session was pn-fix; its offline control is missing — run it first).
Suspects, in order: skid-regime wheel-vx lie (still unmeasured — the
decision-12 probe covers it), plant chaos at commanded 1.0 rad/s fun
turns (beyond anything the floor procedure would command), EKF vx noise
pollution at aggression. This is a NEW question, not a regression in the scoped sense: fun-at-max
never passed a map gate in any era. The full no-regression sweep (organic
and normal-speed patrol re-run under the new stack) has NOT been done —
only the unit suite and the wall session below back the claim so far.

**Wall no-regression under the full new stack** (`20260810-200741`, pn-fix
default + pacing floor): worst near-wall jump **204 mm / 2.3°** — the best
wall result of the whole arc (pre-pacing best 236, vendor 3308) — 10 jumps
>100 mm, map 4.24 × 4.24 / dup 0.20. No regression anywhere; strict
improvement. Two riders: pacing settled HIGH this session (17.7 renders/s —
the floor plus an under-reading rate probe trims upward now), and at that
rate the seam corruption woke up: **the relay dropped 400 corrupted scans
live and the map survived** — the drop path's first live catch, closing the
2026-08-09 "never observed live post-fix" caveat. Corruption rate vs render
rate is now a measurable curve someone should draw before narrowing the trim
ceiling (parked, decision 19).
