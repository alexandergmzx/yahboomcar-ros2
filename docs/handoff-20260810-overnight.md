# Overnight handoff — 2026-08-10 (~01:30–morning), branch `slam-lens-20260810`

**SLAM: NOT DELIVERED** (unchanged — tonight was simulator diagnosis, which cannot
deliver physical SLAM by construction). The question Alex asked — *why is the
`--backend isaac --fun` map messy and not respecting the robot's movement* — is
answered with measurements, a reproduction bag, replay attribution, and a new live
instrument. **No robot-stack code was changed** (Alex's explicit scope: diagnose
only); every proposed fix is parked below.

## The answer, in one paragraph

The fun-mode Isaac smear is **two independent faults stacked, plus one scope
decision**. (1) `/odom_raw` yaw over-reports rotation ~2.8× under turn slip
[measured three ways tonight: 2.75× live lens, 2.82× bag `check_odom_vs_imu`,
0.354 median IMU/odom] — the vendor EKF fuses that lie as pose AND twist, so
slam_toolbox's rotation prior is ~180% wrong, and each hard turn re-lays the
room at a new angle: the live map scored 7.30 × 6.74 m for a 4.0 m room with
1.44 m of duplicate parallel wall and *crisp 0.020 m walls* — clean scans,
poisoned prior. (2) NEW, never previously diagnosed: `sim_runner`'s
render-pacing feedback loop diverges (23.08→3.43/s on 08-09, 6.57→2.92/s
tonight) because /scan rate stops responding to render pacing in this regime;
at the bottom, ~4.4 published revolutions share one rendered world state, and
**~27% of scans carry content ≥0.2 s older than their stamp** [content-lag
probe] — up to ~14° of injected orientation error per stale scan during a
0.6 rad/s turn. (3) Fun's feather boxes get shoved through a world slam_toolbox
assumes static — REAL but not separable this session (the walls landed three
times; box-level attribution needs a session whose walls land once).

## Evidence chain (full report: `docs/slam-runs/isaac-fun-20260810.md`)

- New instrument `tools/slam_lens.py` (committed, tested 23/23 + suite 63/63):
  live map+scan+pose+truth alignment in a browser with fit / divergence /
  yaw-ratio / staleness / TF tiles. **Negative-controlled before use**: healthy
  2D read fit 0.97, ratio 1.00; injected `--slip 0.4` read ratio 1.666 vs
  theory 1.667; killed EKF read TF 0%, divergence 1.18 m↗.
- Reproduction bag `MicroROS-assets/bags/isaac-fun-slam-20260810` (262.9 s,
  sha256 78e05356c823cacf…), 5-lap governed patrol, headless, fun arena.
- Replay attribution (same bag, domain 68, live `map→odom` stripped uniformly):
  - **arm 1 (as recorded): FAIL, reproduces offline** — 7.24 × 7.08 m,
    dup wall 1.10 m. The smear is deterministic from the inputs.
  - **arm 3 (odom→base rebuilt from `/sim/ground_truth`): PASS, every row** —
    4.16 × 4.16 m (authored 4.00 ± 0.20), dup wall 0.10 m, 91.9% known cells,
    wall thickness 0.060 m. Same scans, same 27% content lag, same shoved
    boxes; only the prior changed. **The encoder-yaw lie is the dominant
    cause; pacing + boxes fit inside a passing map's residual.** Fixing
    pacing alone would not have fixed the map.
  - arm 2 (dedup scans) was CANCELLED honestly: predicted ~70–75% duplicate
    scans, measured **0/3330 bit-identical and 0/3330 near-identical** — the
    prediction was wrong, and the staleness instead presents as content lag
    (median best time-offset −0.08 s; 27% ≥0.2 s; 17/124 at the −0.6 s sweep
    limit; rms at best offset 0.021 m).
- **Repeatability run (second Isaac boot, bag 97c7f1760b0ec827…): everything
  replicated** — yaw lie 2.94×/2.99×, pacing bottomed 2.91/s, 0 duplicates,
  lag tail p10 −0.36 s, relay drops 0. Arms: as-recorded **FAIL again**
  (6.98 × 6.74 m, dup 1.42), truth-prior **PASS again** (4.14 × 4.14 m).
  Formal repeatability arm3-vs-arm3b: spans 0.5% apart, IoU 0.776 — PASS.
- **Lens grew a content-lag tile** during the night (commits ae7d9da,
  3354ef3): time-offset walls-fit, sim-only, live-verified at both ends —
  and its first live incident (a static robot reading "80% stale") was
  caught, guarded (refuse when the pose barely moves across the window),
  and pinned by test. Suite 69/69.

## Negative results tonight (bold per house style)

- **Duplicate-scan staleness does not exist on Isaac even at 2.92 renders/s**
  (0/3330 bit-identical, 0/3330 within 5 mm) — bit-hashing is the wrong
  detector for render-pacing staleness; the lens's stale tile is Isaac-blind.
- **Scan content corruption did not occur this session** (relay dropped
  0/3331) — the smear needs no corrupted scans.
- **`slam_preflight` read 0.00% TF success on a session whose TF was healthy
  by the consumer's question** (lens deferred check: 100%) — its immediate
  `can_transform` in the scan callback is unsatisfiable when scan latency is
  3.8 ms and the EKF publishes at 10 Hz. Valid for hardware (scans arrive
  0.1–0.5 s late), invalid as a low-latency-sim gate.

## Morning decisions (parked, not decided — in rough priority order)

1. **Isaac encoder-yaw fidelity** (pre-existing OPEN, now the convicted primary
   cause): options (a) map wheel speeds through the measured slip line so
   /odom_raw lies at the real car's ~7.7% scale, or (b) re-tune PhysX
   friction/feedforward. Touches pinned constants → Alex's call. Arm 3's
   result quantifies exactly how much map quality this buys.
2. **Render-pacing controller divergence** (NEW): proposed shape — floor
   `render_hz >= SCAN_HZ`, and treat pacing moving >2× while measured rate
   moves <10% as divergence (hold + warn instead of dividing again).
   `sim_runner.py:938-961`.
3. **Fun-should-map-well** (Alex's stated stance, contradicts
   slam-delivery-plan's "fun maps are never evidence"): after 1+2 land,
   re-measure; if box-shove smear remains the limiter, options are heavier fun
   boxes, kinematic (unshovable) boxes, or accepting fun maps as
   walls-only-scored. Needs the delivery-plan sentence amended either way.
4. **The `_counts['scan']` false WARNING** (30 spurious "0.0 of 12 Hz" lines
   per session; the OmniGraph never publishes /scan — the relay child does).
   Trivial fix, big log-noise win. `sim_runner.py` status block.
5. **Every simctl path runs the vendor EKF** (`ekf.yaml`, the 2239 mm-phantom
   config) while `bringup_corrected_launch.py` sits unused — on Isaac this
   compounds cause 1 (lie enters as pose AND twist + IMU-yaw echo). Decide
   whether sim sessions should run the corrected config (and whether
   `laser_odometry` must then be fused, cf. the odom1-has-no-publisher gap
   noted in the exploration of 2026-08-10).
6. **slam_preflight deferred-lookup mode** so gate 2 is meaningful on
   low-latency backends; keep the immediate mode for hardware. Also its
   duplicate-publisher probe listed `slam_toolbox` twice on /tf with one
   process running — check whether one node exposing two /tf publishers is
   real or a graph-cache artifact.
7. **Port the lens into fleet-console** — Alex asked for the webapp to serve
   the fleet repos; fleet-console's "SLAM map view + lidar overlay" is its
   scheduled home. The metric core (`tools/_slam_lens_core.py`) is ROS-free
   and lifts as-is; the console's React shell replaces `slam_lens.html`.
8. **Lens stale-tile Isaac-blindness**: add a content-lag metric (needs truth
   + arena — sim-only) or label the tile 2D-only.

## Session hygiene (unattended rules)

- Isaac lock: acquired before start, refreshed with sim_runner PID, released
  after verified teardown (no kit/omniverse processes). `simctl stop` clean:
  17 terminated / 0 remaining, 85 stale DDS segments cleared.
- Domains: 66 (sim), 68 (replays), 67 (lens smoke). Domain 20 untouched.
  No hardware, no flashing, no serial.
- git: local only, append-only, new branch `slam-lens-20260810`. Commits so
  far: lens (tests 63/63 green first), arena reference yaml, docs (this
  handoff + run report + addendum + guide section). No push (rule).
- Bags: capped (360 s timeout + 1 GB splits), disk checked before (127 G free).
- Bounded retries respected: lens smoke-test rerun once after a real fix;
  no retry loops. The `pkill -f` self-match trap cost three compound commands
  (documented in session memory).

## Tree state at handoff

Clean at final commit; see `git status` on branch `slam-lens-20260810`.
All run artifacts (arm maps, scorer/replay/preflight JSONs, both lens
screenshots, and the four scratch probe scripts) are preserved under
`MicroROS-assets/maps/isaac-fun-20260810/` — the assets home, never
committed (R-05 convention), listed in the run report's Artifacts section.
