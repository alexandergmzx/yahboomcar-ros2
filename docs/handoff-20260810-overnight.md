# Overnight handoff — 2026-08-10 (~01:30–morning), branch `slam-lens-20260810`

**SLAM: NOT DELIVERED** (unchanged — tonight was simulator diagnosis, which cannot
deliver physical SLAM by construction). The question Alex asked — *why is the
`--backend isaac --fun` map messy and not respecting the robot's movement* — is
answered with measurements, a reproduction bag, replay attribution, and a new live
instrument. **No robot-stack code was changed** (Alex's explicit scope: diagnose
only); every proposed fix is parked below.

## Updated verdict — speed and clearance qualify the turn failure

The fun-mode Isaac map's primary failure is **turning at ordinary/high manual
speed**; straight legs and slower motion are substantially more stable. That
matches Alex's two manual-driving observations and the bags:
straight-line truth speed is only about 5% above wheel odometry (median
truth/odom 1.049 and 1.050 in the two runs), while `/odom_raw` yaw over-reports
turning by 2.82× and 2.95×. The vendor EKF fuses that angular lie as pose and
twist, so each hard turn gives slam_toolbox a rotation prior roughly 180–195%
too large and the room is laid down again at a new angle. Slower commands
plausibly reduce wheel slip and the angular mismatch presented per scan, but
that speed dependence is operator evidence, not yet a recorded A/B. The live
map's 7.30 × 6.74 m span, 1.44 m duplicate wall and crisp 0.020 m wall strokes
are the result: clean local scans placed at bad headings. Replacing the
odometry prior with truth makes both runs pass (4.16 × 4.16 and 4.14 × 4.14 m),
so the prior is the dominant cause. The replay replaces full SE(2), not yaw
alone, but the small linear-scale error, enormous yaw error, and turn-only
manual symptom make yaw the strongly supported component; a yaw-only replay
remains the strict isolation arm.

There is a second operating boundary: Alex reports that even the slow map can
destabilize when the robot gets too close to a fun-mode box. This does **not**
yet identify a cause. Candidate mechanisms are chassis/box contact increasing
slip, movement of a 0.02 kg box violating SLAM's static-world assumption, and
the lidar's residual near-field dropout. The latest rolling manual-session logs
(12:02–12:06) show 2 laser-odometry degeneracies in 2,885 scans, one scan-relay
rejection, and 89 slam_toolbox queue-full drops, but contain no `/cmd_vel`
record, bag, or saved map, so none can be correlated to the close-box moment.

`sim_runner`'s render-pacing controller also diverges (23.08→3.43/s on 08-09,
6.57→2.92/s in run 1, 8.22→2.91/s in run 2), but its claimed map impact is
**retracted**. The original “27% ≥0.2 s stale / up to 14°” result came from an
unguarded time-offset probe that scored a static robot. Applying the later
mandatory static-motion guard leaves only 1/77 run-1 and 2/78 run-2 moving
samples at ≥0.2 s, with median offset −0.04 s in both. Fix pacing for simulator
honesty, but do not present content lag as a demonstrated smear cause. Fun's
movable boxes remain real but were not separable in these runs.

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
    dup wall 1.10 m. The failure class reproduces from the recorded inputs.
  - **arm 3 (odom→base rebuilt from `/sim/ground_truth`): PASS, every geometry row** —
    4.16 × 4.16 m (authored 4.00 ± 0.20), dup wall 0.10 m, 91.9% known cells,
    wall thickness 0.060 m. Same recorded scan stream and shoved boxes; the
    odometry prior was replaced. **The turn-yaw lie is the dominant cause.**
    Fixing pacing alone would not have fixed the map.
  - arm 2 (dedup scans) was CANCELLED honestly: predicted ~70–75% duplicate
    scans, measured **0/3330 bit-identical and 0/3330 near-identical** — the
    duplicate prediction was wrong. The first content-lag result was also
    wrong: its unguarded probe included static samples. Guarded re-analysis is
    median −0.04 s, with 1/77 moving samples at ≥0.2 s in run 1 and 2/78 in
    run 2; no −0.6 s boundary hits.
- **Repeatability run (second Isaac boot, bag 97c7f1760b0ec827…): the map
  attribution replicated** — yaw lie 2.94×/2.99×, pacing bottomed 2.91/s,
  0 duplicates, relay drops 0. Arms: as-recorded **FAIL again**
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
- **The session does not demonstrate material timestamp/content lag.** The
  guarded moving-sample results are median −0.04 s and only 1.3% / 2.6% at
  ≥0.2 s. The earlier 27% figure is retracted as static-pose degeneracy.
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
2. **Render-pacing controller divergence** (NEW, map impact unproven): proposed shape — floor
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
8. **Content-lag evidence discipline**: keep the metric sim-only, keep the
   static-motion guard mandatory, and store guarded machine-readable results
   before making a scan-age claim.

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

## Evidence retention correction after the manual follow-up

The two overnight bags are retained and checksummed, but **not every manual run
log is saved**. `simctl` writes stable names such as `simctl-isaac.log` and
`simctl-slam.log`; each new start overwrites the preceding session. The files
currently present belong to the latest 12:02–12:06 manual run. That run has no
bag or map artifact, and earlier manual-session logs cannot be reconstructed
from the asset tree. Treat all manual speed/clearance findings as operator
observations until a timestamped bag records commands, scans, odometry, truth,
TF and map together.

## Tree state at handoff

The original overnight handoff was committed cleanly. This corrected verdict is
currently an uncommitted documentation/instrument-description update on branch
`slam-lens-20260810`; see `git status`.
All run artifacts (arm maps, scorer/replay/preflight JSONs, both lens
screenshots, and the four scratch probe scripts) are preserved under
`MicroROS-assets/maps/isaac-fun-20260810/` — the assets home, never
committed (R-05 convention), listed in the run report's Artifacts section.

## Afternoon addendum (2026-08-10 ~12:15–12:45): session data coverage

Alex's audit: not all run data is saved. Confirmed, and worse — `simctl`
wrote every log to a flat name with mode `'w'`, so **each start destroyed
the previous session's logs**; the 12:02 close-box session left no bag, no
map, no `/cmd_vel`; the overnight evidence survived only by hand-copying.

Landed (branch `slam-lens-20260810`):

- **Salvaged first**: the 12:02 logs → `logs/sessions/20260810-1202-isaac-fun-MANUAL-salvaged/`
  before any new start could destroy them.
- **`tools/_session_record.py`** (+15 tests): session dirs named by a
  correlation id, atomic `session.json` manifests, ISO-stamped `events.log`
  timeline, flat-name symlinks (real files renamed aside, never deleted),
  health-counter parsers pinned to the REAL log formats (the salvaged
  session is the fixture), bag checksums shared-shape with replay tables.
  Counters report None for "log absent", never a fake 0.
- **simctl records by default**: session dir for every start (failed starts
  included — their logs are evidence too), capped mcap bag WITH `/cmd_vel` +
  `/cmd_vel_raw` (the audit's headline gap; `--no-bag` opts out; auto-skip
  under 5 GB free disk), stop grew steps 3/7 (SIGINT the recorder, wait for
  metadata.yaml, checksum) and 4/7 (save the map while slam_toolbox lives),
  manifest finalized with counters + duration on every exit path, status
  shows the live session + recorder. All recording is fail-open.
- **slam_lens files its metric history** into the session dir on exit
  (`--dump` to override).
- **Discovery bug found & fixed on the way**: `sim_target` treated the
  EXISTENCE of `/sim/ground_truth` as simulator evidence, but a topic
  appears in the graph for a mere subscriber — Alex's noon lens tab
  (read-only, still open) made every `start` refuse with "already running".
  Now requires a PUBLISHER (`get_publishers_info_by_topic`).

Verified: 2D round-trip session `20260810-123046-2d-d66` — bag closed
(sha f093d5406243ee1d…, 10 topics incl. 2371 `/cmd_vel` + 1562
`/cmd_vel_raw` msgs), `replay_slam_bag --inspect` says replayable, map
saved at stop, counters captured (40 queue-full drops), events timeline
complete; negative paths say what they skipped and why
(`--no-bag`/`--no-slam` session `20260810-123420-2d-d66`); two sessions
side-by-side with nothing overwritten; suite green (84 tests + README
suite re-run).

Morning-decisions additions:

9. **Session retention**: ~25 MB/session accumulates in
   `MicroROS-assets/{logs/sessions,bags}`; no pruning policy exists. Manual
   for now, deliberately — deleting evidence is an operator's call.
10. **Alex's noon lens needed SIGKILL** (SIGTERM ignored after ~30 min
   idle) — shutdown robustness after long idle, one reproduction, unfiled.

## Afternoon addendum 2 (~13:00–14:30): near-wall stability answered

Alex: rotation now behaves; the map destabilizes near walls. Full study in
`docs/slam-research/near-wall-stability.md` (every number adversarially
verified by a 3-lens pass before commit; 8 corrections applied and listed).

One line: **the wall doesn't break SLAM — blocked wheels do.** With the body
constrained at the wall (fun braking off), the encoder yaw lie runs ~6–26×
(vs 2.9× free-floor); the vendor EKF fuses the sweep; the map fans. Proven
by 9 same-bag replay arms over 3 bags: penalties (N1) and range-floor+
speckle (N2) rescue nothing — **§4a answered: the loose penalty block is
not the cause** — while excluding the wheel-yaw channel (N4, IMU-yaw prior)
takes the worst bag from 9.36 m of fanned room to 4.30 m, centimetres off
the bar. No canonical yaml change is warranted; none was made.

Morning decisions, appended:

11. **EKF fusion for sim sessions** (upgrades decision 5): A/B the three
    variants — vendor / `ekf_corrected.yaml` as-is (gate untested under
    the blocked regime) / corrected + `odom0` vyaw=false (N4's structure,
    one line) — one fun wall-approach session each, scored per the study.
12. **Measure the wheel-vx lie under the blocked regime** (the bag-B
    residual's untested attribution; first check for whoever picks this up).
13. **Contact self-announcement**: odom-vs-IMU yaw-rate disagreement >3×
    for >0.5 s ⇒ governor stop — makes the blocked regime loud instead of
    silently corrupting the map.

## Evening addendum (~16:40–17:10): the EKF fix attempt — honest negative

Alex: still unstable; approved implementing the corrected-EKF fix with a
validate-then-flip gate. Result: **the gate refused the flip, correctly.**

- Landed: `--ekf {vendor,corrected,corrected-novyaw,n4-pure}` on simctl's
  Isaac path (default vendor, UNCHANGED); `ekf_corrected_novyaw.yaml`
  (variant) + `ekf_n4pure.yaml` (diagnostic); `/odom_laser` now in session
  bags. Config package built, 84+9 tests green.
- Measured live: both corrected variants FAIL the wall A/B and are MORE
  jumpy than vendor away from walls (23 and 73 jumps >100 mm vs vendor's
  handful). Convicted from the n4-pure bag: the EKF attenuates the honest
  IMU yaw rate to **0.727× (corr 0.531)** — the offline N4 rescue integrated
  at gain 1.0, and that difference is the whole offline/live gap. No
  sign/frame error (ruled out).
- Study updated with the live table and the specific next lead
  (docs/slam-research/near-wall-stability.md, "Live A/B" section).

Decision 11 UPDATED, not closed: the EKF-fusion switch is blocked on one
measured question — why does ekf_filter_node low-pass a clean 25 Hz yaw-rate
input by 27%? Diagnose OFFLINE against bag `20260810-165133` (process noise
vs stamped measurement covariance vs the 10 Hz filter rate) before any
further live session. Until then: the practical stability guidance for fun
driving stands — the instability is the wheel channel under blocked-body
conditions plus, in the corrected variants, this filter attenuation; vendor
remains the default and the wall remains the boundary.

## Night session 2 (2026-08-10n, branch `ekf-numerics-20260810n`): the numerics

Alex's directive: research/test/try the numeric-methods angle overnight.
**Alex was right — the whole failure was a filter gain.** Full trail in
`docs/slam-research/near-wall-stability.md` (two new sections); artifacts in
`MicroROS-assets/maps/near-wall-20260810/ekf-numerics/`.

- **Identified**: offline system-id harness (reproduced the live 0.727x
  first, then measured the missing number: EKF yaw lags truth 2.8 s). Six
  bounded arms → process noise convicted (upstream's own [ADVANCED] warning);
  q(yaw,vyaw)x100 → transfer 1.001 / corr 0.994 / lag 0.02 s. H1 covariance
  stamping made it WORSE (epsilon substitution), madgwick exonerated.
- **Offline map gates: all three bags rescued** by the fixed filter's real
  TF (wallA 4.22x4.22, wallB 4.20x4.22, organic FULL PASS 4.16x4.18 — a bag
  even vendor failed). Laser-pose fusion measured HARMFUL at high trust
  (0.415) → excluded from the promoted sim config `ekf_sim_pnfix.yaml`.
- **Live G3: dominance without the gate.** pn-fix beat vendor everywhere
  (worst wall jump 3308→236 mm, patrol MAP at truth-prior quality 4.16x4.20 (its TF stream still shows 8 jumps >100mm in open space — see study),
  open-space p95 22 mm) but missed the absolute bar (<100 mm jumps, map
  PASS) in two wall attempts — residual confined to the <0.3 m grind band
  where wheel-vx feeds phantom translation. **Arm closed per bounded
  retries; the Isaac default REMAINS vendor.** Commits: dcce295 (+ final
  study/handoff commit).

Morning decisions, updated/added:

11. → RESOLVED into 14: the corrected-EKF question is now the pn-fix question.
14. **Flip Isaac default to `--ekf pn-fix`?** The dominance table says yes,
    the absolute gate (set in the session plan before the runs; not repo-registered — noted) said not-tonight. Alex's call with the
    table in front of them. If flipped: README+guide updates + one more
    confirmation pair.
15. **Contact-gated vx trust**: the last band's fix — zero wheel-vx trust
    when odom-vs-IMU yaw disagreement exceeds ~3x (decision 13's alarm as a
    fusion gate). Would attack the only regime pn-fix doesn't fix.
16. **map_saver race**: stop's map save failed 2/3 sessions tonight
    ("Failed to spin map subscription", 2 s internal timeout); maps were
    recovered offline from session bags each time. Fix shape: retry once
    or pass a longer timeout in simctl's step 4/7.
17. **Hardware note**: everything tonight is SIM-validated. The pn-fix
    reasoning (process-noise vs measurement-covariance balance) transfers
    to hardware IN PRINCIPLE, but the real gyro is intermittently faulty
    (the D-06/gyro-zero constraint) — a fusion leaning this hard on the
    gyro NEEDS the sensor_health rotate-window gate before every hardware
    session, and the laser-pose question must be re-asked there.

Session hygiene: 3 live sessions (of 4 budgeted), lock taken/PID'd/released
each time, verified dead each teardown; domains 66/68 only; bags capped;
all sessions auto-recorded; Alex's four audit-edit files remain uncommitted
and untouched; suite green at every commit.

## Evening round 2 (Alex driving, ~19:00–20:30): approved, defaulted, next layer named

Alex live-verdict on pn-fix: "behaves great... it autocorrects the map to
keep tracking the real car" → **decision 14 APPROVED and executed**: pn-fix
is the Isaac default (06238db; vendor one flag away; banner/README/guide
updated; suites green). Decision 11 closed into it.

Alex's fast-speed report ("the scan destabilized first") diagnosed from
their own session bag and FIXED: the pacing runaway's intra-scan seam
(decisions 2+4 closed, f1c1902 — floor at SCAN_HZ, divergence guard, false
warning gone; 8 new tests; live-held 12.0 renders/s with fast-window scan
quality at slow-window levels).

NEW decision 18 — **the aggression layer**: at 0.6 m/s + 1.0 rad/s fun
turns the map still fails with clean scans (jump p95 ~950 mm); F25 and
NOVX arms both dead offline (worse / lost tracking); yaw corr collapses to
~0.5 at unit ratio; plant itself chaotic at that command level. First
moves for the next session: the missing pn-fix offline control on bag
20260810-195417, then the decision-12 vx probe extended to skid regimes.
Note: this aggression exceeds anything the floor procedure commands on the
real car — a scope question ("should fun-at-maximum map?") rides along, as
it did for the boxes (decision 3).

Also landed on the way: slam_lens port-fallback (an operator lens tab on
8765 is normal and never ours to kill — found by the README suite
colliding with Alex's live tab).

Round-2 wall no-regression (`20260810-200741`, full new stack): worst jump
**204 mm** — best of the arc — map 4.24×4.24/dup 0.20 PASS-shaped. Rider:
pacing settled at 17.7/s and the seam corruption woke at that rate — the
relay's drop path FIRED LIVE for the first time (400 drops, map survived),
closing the 08-09 caveat. NEW decision 19: draw corruption-rate-vs-render-
rate before narrowing the trim ceiling toward SCAN_HZ.

## 2026-08-11 early: the gate deadlock — audit hypothesis proven, fixed, confirmed

Alex's audit suspect (`odom0_twist_rejection_threshold` deadlocking on the
1.0 m/s step) PROVEN by same-bag A/B: path ratio 0.47 gated → 0.99 ungated,
yaw unchanged. Gate removed from ekf_sim_pnfix.yaml (imu0 gate retained);
live max-speed confirm: path 0.969, best max-aggression map of the arc
(4.20×4.24/dup 0.26, 0.06 m over the dup bar). Decision 18 largely
dissolves into this; residual is margin, not mystery. Branch
gate-fix-20260811, ready for Alex's merge call.
