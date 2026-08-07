# Audit handoff

> **External audit completed.** Its findings held up: I verified each one rather than
> taking them on trust, and every claim I checked was correct. The section
> "Post-audit status" at the end records what was closed and what remains open.
> Where earlier sections of this file contradict that one, the later section wins.

For a reviewer (human or LLM) checking this work. Written to help you find my errors, not
to defend the work. Where I am unsure, it says so; where I was wrong, it says that too.

**One-line summary:** the ROS 2 Humble→Jazzy port is real and well tested; the Isaac Sim
work was rushed, wasteful, and produced a false "working" claim; the robot **demonstrably
drives, and its odometry agrees with an independent gyro to within ~8%**; and the radio
link is **degraded to roughly half its specified rates**, which is still unexplained and
undermines any timing-sensitive measurement.

---

## Start here: the claims most likely to be wrong

Check these first. They are ordered by how much rests on them.

| # | Claim | Status | Why it deserves scrutiny |
|---|---|---|---|
| 1 | `/odom_raw` reflects real wheel rotation | ✅ **RESOLVED — measured** | Hand-spin test, `selftest-20260806-004042`: turning a wheel by hand with nothing commanded produced peak twist **1.5446** and pose change **0.221 m**. Encoders are real; odometry is **not** open-loop. UMBmark, cartographer and the twin rest on a sound foundation after all. |
| 2 | "The motors move" | ✅ **RESOLVED — measured** | `selftest-20260806-004114`: commanded 0.12 → `/odom_raw` 0.127 forward, 0.131 back, yaw 0.693 for a commanded 0.6. Since claim #1 establishes the encoders are genuine, odometry responding to commands means the wheels physically turned. The inference is now sound; it was not before. |
| 2b | Earlier "all tests failed" | ❌ **INVALID RUN** | Three runs (`003440`, `003546`, `003918`) received **zero** messages because `ROS_DOMAIN_ID` was unset and defaulted to 0 while the board is on 20. One of them still printed "odometry is probably open-loop" from no data. Those logs prove nothing about the robot. |
| 2c | The robot physically moves, and odometry is accurate | ✅ **measured, independently** | `selftest-20260806-004826`, the first recording **with traction**: IMU yaw 0.719 rad/s against odometry 0.693 — the gyro is independent of the encoders, so this confirms the *body* moved, not just the wheels. Time-aligned over the turn: correlation **+0.977**, median IMU/odom ratio **0.928**. Every elevated run reads IMU yaw exactly 0.000, which is the correct contrast. Reproduce: `tools/check_body_moved.py`, `tools/check_odom_vs_imu.py`. |
| 3 | The chassis is differential, not mecanum | ✅ measured | A commanded strafe produced *exactly* zero on every `/odom_raw` axis. Claim #1 now confirms the encoders are genuine, so this is a real firmware behaviour rather than an artefact. |
| 4 | Isaac twin "articulates and tracks" | ⚠️ **PARTLY FALSE, now fixed** | Joints did track, but the robot was **falling through the floor** and I never checked base pose. Two causes: ground authored as `UsdGeom.Plane` (visual only, no collider) and wheels imported with no collision geometry (`collision_from_visuals` defaults False; the URDF has no `<collision>`). Both fixed; the fall detector is **negative-tested** via `build_arena.py --break-floor`, which must exit 1. |
| 4b | Sim arena drives and stops | ✅ measured | `drive-straight` 0.935 m for a commanded 1.0 with 1 mm lateral drift; `rotate` 51° with 71% slip; `obstacle-stop` halts 0.395 m from the wall with no measurable coast. Reproduce: `tools/sim_runner.py --test all`. |
| 4c | Sim UMBmark validates the calibration pipeline | ❌ **NO — and this is the interesting one** | Injecting a 10% left/right wheel imbalance should give Ed≈1.1053; the run reports **1.0013**. Two reasons, both real: corner turns close the loop on ground-truth yaw and so cancel the dead-reckoning error UMBmark exists to measure, and ~71% rotational slip suppresses differential steering (a 10% mismatch should curve a 1 m leg ~43°; it barely deflects). **The arena cannot validate the Ed pathway as configured.** The maths is validated separately by synthetic round-trip; the physical pathway is not. |
| 5 | Nav2 params fixed for Jazzy | ✅ measured | All eleven nodes reach `inactive` (configured). Activation was never reached — it needs a live robot. Do not read this as "navigation works". |
| 6 | Workspace builds on Jazzy | ✅ measured | 13/13 packages, 37/37 launch files parse. Easy to re-run and the most solid result here. |

## Corrections I had to make to my own claims

The clearest signal about where more errors hide. Each of these was stated confidently
and was wrong:

1. **"Nav2 params are accepted without rejection."** Wrong. Parameter *declaration*
   succeeded; plugin *loading* failed four different ways. I had earlier predicted
   `plugin_lib_names` would break, then wrongly retracted that prediction.
2. **"`base_node_X3` turns `/odom_raw` into `/odom` + TF."** Wrong. It publishes TF only,
   has no publisher at all, and is not in the bringup launch.
3. **"The workspace builds with zero source changes."** True but misleading — the
   bringup package compiled perfectly and was functionally broken.
4. **"The twin articulates."** A false positive: joints settling under gravity on
   zero-stiffness drives, passing a "did anything move" threshold.
5. **"3 stray bridges remain."** A measurement artifact — my `grep` was matching its own
   command line. Actual count was zero.
6. **"The robot's sensor rates are degraded."** Half true. Two of my own measurement bugs
   (single-threaded `spin_once`, and RELIABLE QoS against BEST_EFFORT publishers) made a
   real ~40% degradation look worse than it was.

## Known defects, open

- **Effective rolling radius in sim is 1.87x the geometric one** — 0.0458 m measured
  against 0.0245 from both the STL and the imported mesh bbox. A driven wheel cannot
  propel a body faster than pure rolling, so this points at an angular-unit or
  contact-radius mismatch between `apply_action(joint_velocities=...)` and the PhysX
  drive, not at real physics. The controller uses the measured value so commands match
  reality; `sim_runner.py --calibrate` re-measures it. **Unexplained, and worth an
  auditor's attention** — it may indicate a units bug that also distorts other results.
- **Simulated masses are low**: 0.355 kg total against a real car nearer 1 kg, taken
  from the URDF's inertial blocks. Affects contact forces and therefore slip.
- **Rotation is strongly non-linear**: below roughly 1.2 rad/s of wheel speed the robot
  does not turn at all (static friction is never broken); at 4 rad/s it turns 111° in 2 s.
  Any test that commands slow rotation will silently do nothing.
- **Stopping distance is unmeasured.** The safety governor limits commands; it cannot
  beat the robot's braking. Nobody should call it safe until that number exists.
- **`yahboomcar_multi` is unverified** — needs two robots.
- **`transforms3d` was installed but broken — NOW FIXED** (0.4.2, see porting-notes).
  Originally: apt 0.4.1 calls `np.maximum_sctype`,
  removed in NumPy 2.0). Nothing imports it, so it is inert.
- **A 15 mm frame disagreement**: the cartographer launch places `laser_frame` 0.094079 m
  above `base_link`; the URDF's `radar_Joint` says 0.078934 m.
- **The radio link runs at roughly half spec and is unexplained.** Measured across runs:
  `/scan` 4.1–8.9 Hz against 12, `/imu` 5.3–16.5 against 25, `/odom_raw` 3.1–8.3 against
  11, while the 1 Hz `/battery` topic is unaffected — the signature of bandwidth, not
  firmware. Battery is healthy at 8.3 V. Both PC and car now share the 2.4 GHz band;
  earlier measurements had the PC on 5 GHz. **Any timing-sensitive result should be
  distrusted until this is understood**, including the UMBmark protocol, which assumes
  odometry samples arrive promptly.
- **Odometry over-reports yaw by ~7.7%** (median IMU/odom ratio 0.928 on the desk run).
  For a skid-steer this is the effective-track error UMBmark calls `Eb`. Note this is far
  *smaller* than the "expect a large Eb" I predicted in
  `docs/odometry-calibration.md` — that prediction should be treated as unsupported until
  a proper UMBmark run either confirms or replaces this single-bag estimate.
- **This car has no servos.** `jq1_Joint`/`jq2_Joint` exist in the vendor URDF but the
  Standard chassis has no gimbal; they are only on the Vision version. The twin animates
  them anyway, and an earlier self-test reported `servos PASS` for hardware that is not
  present. Now `--servos` opt-in, but the twin still needs them gated.

## Verified, with the command to re-run

Do not trust the table; run the commands.

| Result | Reproduce |
|---|---|
| 13/13 packages build | `cd yahboomcar_ws && colcon build --symlink-install` |
| 37/37 launch files parse | `ros2 launch <pkg> <file> --show-args` across the tree |
| Safety governor logic | `cd yahboomcar_ws/src/yahboomcar_safety && PYTHONPATH=. pytest test/ -q` → 26 pass |
| Governor live behaviour | run `cmd_vel_governor`, publish synthetic `/scan` + `/cmd_vel_raw` |
| Firmware topic rates | `./tools/car_selftest.py --sensors-only` |
| UMBmark maths | `./tools/umbmark.py compute --demo` — round-trips α=0.55°, β=1.30° |
| Body really moved (not just wheels) | `./tools/check_body_moved.py MicroROS-assets/bags/*` |
| Odometry vs gyro agreement | `./tools/check_odom_vs_imu.py MicroROS-assets/bags/selftest-20260806-004826` |
| Agent bridges the firmware | run the `:jazzy` agent, `ros2 node list` → `/YB_Car_Node` |
| USD has 9 meshes, 6 movable joints | needs the 5.1.0 importer path first |

## What I changed outside this repository

Relevant because an audit of the repo alone will miss these.

| Change | Detail | Reversible? |
|---|---|---|
| Removed ROS 2 Humble | `apt remove '~nros-humble-*'` — 157 orphaned `jammy` packages, unusable on noble | yes, reinstall |
| Installed Jazzy packages | navigation2, nav2-bringup/common, cartographer-ros, slam-toolbox, imu-tools, joy-linux, `python3-transforms3d` | yes |
| **Installed then deleted Isaac Sim 6.0.1** | 13 GB downloaded, 27 GB at `~/isaacsim`, now removed. **This was pure waste** — 5.1.0 already existed at `~/isaac/env_isaaclab` | deleted |
| ESP-IDF registry | corrected `~/.espressif/esp_idf.json` (a v5.5-dev tree was labelled 6.0.0); retired `idf-env.json` | backups kept |
| Deleted ~1.2 GB | redundant `~/esp/esp-idf-v5.4.1.zip`, empty `~/esp/v5.2/` | no |
| Desktop entry | added then removed `~/.local/share/applications/IsaacSim.desktop` | removed |

Not touched: `Development/omniverse_twin` (read only).

## Process failures worth weighing

Not just *what* was wrong but *how* it went wrong, since that predicts the rest:

- **I did not check for an existing Isaac install** before downloading 13 GB, despite
  having checked free disk space. Worse, `omniverse_twin/tools/isaac_5_1_ros_camera.py`
  already demonstrated the `app.update()` pattern I then spent hours rediscovering.
- **I left 14 stray processes running** across test runs, all publishing conflicting
  commands, because `kill %1` kills a shell job and not the process it spawned.
- **My verification repeatedly checked the thing I had built rather than the thing that
  mattered** — joint angles but not base pose; "did it move" rather than "did it track".
- **Several bugs were in my test harnesses, not the code under test**, and in three
  separate cases a harness sampled across a state transition and reported failure. I
  caught these, but only after asserting a wrong result at least once.

## How to attack this

If I were auditing it, in this order:

1. **Run `--handspin`.** If odometry is open-loop, a large amount of downstream work is
   void, and everything in `docs/odometry-calibration.md` needs rewriting.
2. **Re-run the safety governor tests, then try to break them.** Adversarial scans:
   all zeros, all NaN, a single spurious near return, ranges below `range_min`. This is
   the component where being wrong is dangerous rather than merely embarrassing.
3. **Diff `yahboomcar_ws/src` against the vendor drop** in
   `MicroROS-assets/ROS_Source_Code/`. Every change should be justified in
   `docs/porting-notes.md`; anything unexplained is suspect.
4. **Check the Nav2 param edits against Jazzy's own reference** at
   `$(ros2 pkg prefix nav2_bringup)/share/nav2_bringup/params/nav2_params.yaml`.
5. **Ignore prose, re-run commands.** The commit messages are detailed but they are still
   my account of my own work.

## Repository map

```
docs/porting-notes.md        Humble->Jazzy changelog, includes my corrections
docs/research-log.md         every external source, why, and what it changed
docs/odometry-calibration.md UMBmark protocol  (depends on claim #1)
docs/handoff-audit.md        this file
tools/car_selftest.py        measures the car, --handspin settles claim #1
tools/umbmark.py             calibration maths, validated by round trip
tools/provision_board.py     safe board config (the vendor's script writes junk on import)
tools/urdf_to_usd.py         BROKEN: targets the deleted 6.0.1 API
yahboomcar_ws/src/           13 packages; yahboomcar_safety and _twin are mine
MicroROS-assets/             git-ignored vendor drop, bags, logs
```

Branch `jazzy-port`, ~30 commits. Commit messages state what was measured versus
inferred; where they disagree with this document, trust this document — it was written
later and with less to prove.


---

## Post-audit status

An external audit (Codex) reviewed the repository. Ten findings; all verified as
correct. One was worse than reported — the laserscan node passes `angle_increment`
twice, so `angle_min` never reaches the conversion at all.

### Closed

| # | Finding | What was done |
|---|---|---|
| 4 | Governor bypassed by everything | **Scoped fix.** `car_selftest` and `twin_motion_sequence` now publish `/cmd_vel_raw`; `safe_teleop_launch.py` runs the governor with the vendor keyboard remapped. The governor now **names** bypassing publishers at ERROR. Verified on the live car: real lidar at 0.39 m throttled a 0.25 m/s request to **0.022 m/s**, and withheld input drove output to zero. Coverage is deliberately partial — see `docs/safety-case.md`. |
| 8 | laserscan node broken | Five defects fixed (angle_min never passed; 135 **radians** added to every angle; no headers; undefined variable on shutdown; node name colliding with `robot_pose_publisher_ros2`). 9 tests pin each. |
| 10 | Manifests | `exec_depend` added across 12 packages from real imports; descriptions written. |
| — | Docs asserting falsehoods | `CLAUDE.md` no longer claims the twin syncs chassis pose. |

### Open, deliberately

| # | Finding | Why it is still open |
|---|---|---|
| 1, 2, 3, 5 | Twin is not a twin; asset paths disagree; live twin still falls; verifier cannot fail | Deferred by decision. `isaac_twin_setup.py` and `isaac_twin_verify.py` now carry **KNOWN BROKEN** banners so a passing run is not mistaken for a working twin. Design is settled: 5.1.0 has `ROS2SubscribeTransformTree`, which takes `articulationRoots` and a `frameNamesMap` and drives prims from `/tf`; with a **kinematic** base posed from `odom`→`base_footprint`, the falling bug becomes structurally impossible rather than merely patched. |
| 6 | Sim physics untrustworthy for calibration | Agreed and documented. The 1.87× effective-radius anomaly is unexplained and flagged; the arena is for visualisation and collision experiments, not controller or odometry validation. |
| 7 | No SROS2 authorisation boundary | Real gap. Enclaves and keystores are a deliberate piece of work, not a bolt-on. Recorded in `docs/safety-case.md`. |
| 9 | Braking distance unmeasured | Needs the floor. This is the number that gates real driving and it does not exist. |
| 10 | **Licences** | Left as `TODO` **on purpose**. This is third-party Yahboom code shipping no licence declaration; writing one into `package.xml` is a legal assertion nobody here can make, and a fabricated licence is worse than an obvious gap because it looks settled. Each manifest carries a comment explaining this. **Blocks redistribution** until Yahboom clarifies. |

### Vendor packages: what "unprotected" means

The vendor course nodes (`yahboom_keyboard`, `yahboom_joy`, `calibrate_*`, `laser_*`,
Nav2) publish `/cmd_vel` directly and are **not** speed-limited. This was a scope
decision so commands copied from the course PDFs keep working as documented. The
governor logs an ERROR naming any such publisher, so the gap is visible rather than
silent.

---

# Session 2 handoff (2026-08-06)

Everything below happened after the audit above. Same purpose as the rest of this file:
written to help you find my errors, not to defend the work. Two external audits landed
during this session and both found real blocking defects; their findings are recorded as
findings, not as things I noticed.

## The single most important result

**The firmware has no command watchdog. A commanded speed is retained indefinitely.**

Measured three ways with the car elevated (`tools/test_failsafe.py`), then reconfirmed
after a full power cycle so it is a property of the firmware and not of a wedged session:
commands ceasing, the governor `SIGKILL`ed, and the agent frozen all leave the car
driving. A follow-up probe held 0.15 m/s for the full 45 s it watched with nothing
publishing. It stops only on an explicit zero, and `config_robot.py` exposes no timeout,
so it is not configurable.

**Any crash of any component leaves the car driving. Wi-Fi loss is unmitigable from this
machine.** `cmd_vel_deadman` covers a publisher dying — measured 693 ms and 762 ms — and
covers nothing else. That is a permanent operating constraint, not a gap pending work.

**Attack this first.** If the watchdog claim is wrong, the entire safety case is
mis-scoped. `./tools/test_failsafe.py` reproduces it in about two minutes on an elevated
car. `--negative-test` proves the detector cannot false-positive a stop.

## Claims from this session most likely to be wrong

| claim | how to attack it |
|---|---|
| Corrected EKF: 6 mm vs vendor 2239 mm under pure slip | `tools/ekf_ab_test.py`; both runs bagged in `MicroROS-assets/bags/ekf-ab-*`. Stand only — it proves nothing about driving |
| Lidar sees 92% slip on `twin_dataset` | `tools/sensor_agreement.py <bag>`; ground truth is structural (wheels off the ground) |
| ICP recovers per-scan motion to ~2.7 mm | 48 unit tests against synthetic transforms; the real-scan figure is unverified against ground truth |
| A bare 4×4 m room is well conditioned (isotropy 0.914) | `tools/arena_observability.py`; raytraced, not simulated. **I predicted the opposite** |
| Deadman stops the car in 762 ms | one bench session; the bound holds only while that process lives AND the link is up |
| Linear odometry is accurate to ~0.4% | five hand pushes, best-of not mean — see below |

## Claims I made and had to retract, this session

Recorded because the pattern matters more than any single error: **each was plausible,
self-consistent, and wrong.**

1. **"Elevated latency measurement is conservative."** Backwards. Unloaded wheels spin up
   almost instantly and `/odom_raw` is encoder-derived, so the measurement is a *lower*
   bound on the floor value — the safety-relevant direction to get wrong. Caught by the
   user, not by me.
2. **"A 4×4 m room is close to the worst case for scan matching."** Asserted in a plan, in
   docstrings and in a commit message. Measurement showed median isotropy 0.914 — well
   conditioned — and that adding boxes slightly *hurt*. The lidar reaches 8 m and the room
   is 4 m, so it sees all four walls from everywhere.
3. **"Trimming rejects outliers harmlessly."** Fixed-fraction trimming discards the points
   furthest from the centre of rotation, which are exactly the ones that see rotation. It
   converged to a stable 0.0852 rad against a true 0.12 and looked like clean convergence.
4. **The scan-match frame convention was inverted.** `match(prev, curr)` returns the scene
   transform, the inverse of the robot's. Encoders and gyro read ≈ +1.8 rad while the
   lidar read −1.3. The magnitude was plausible; only the sign exposed it.
5. **"Summing per-pair ICP gives total yaw."** It measures noise: 464 of 503 pairs were
   near-stationary, and the sum implied a 25% lidar under-read while a regression over the
   38 genuinely turning pairs implied the opposite.
6. **"transforms3d was the pip install that broke things."** It is apt-installed and was
   not the cause. It *was* broken, for a different reason, and that mattered more.
7. **"`map_gmapping_launch.py` cannot work on Jazzy — `slam_gmapping` was never ported."**
   Propagated into four files: `porting-notes.md`, the launch file's own docstring,
   `slam_toolbox_launch.py`'s docstring, and `tools/test_readme.py`'s rationale — where it
   was the motivating example for a tool built to stop exactly this. It was never tested.
   `slam_gmapping` *had* been ported earlier in this same repo, in seven `.h`→`.hpp`
   include lines, and `porting-notes.md` recorded that a hundred lines below the claim it
   contradicts. Running it takes 45 s: it scan-matches through 31 map updates and
   publishes an occupancy grid with 604 occupied cells.

   **This one is different from 1–6 in a way worth naming.** Those were false *positives*
   — claiming something worked when it did not. This was a false *negative*: writing off a
   working feature, which no amount of careful verification of working things will ever
   catch, because nobody tests what they have already declared dead. It was found only
   because the README suite demanded a tag for every command and this one had no defensible
   answer for why it was untestable. **"This is broken" needs evidence exactly as much as
   "this works" does.**

## Defects found by external audit, not by me

Both audits were right about everything I checked.

- **The mandatory floor launch failed its own preflight.** The deadman must publish
  `/cmd_vel`; the governor called every other `/cmd_vel` publisher a bypass; the procedure
  says abort on bypass. Unpassable, and I built both halves. Unit tests could not catch it
  — each node was correct alone. Now `tools/test_launch_preflight.py`.
- **The battery gate demanded 11 V on a 7.4 V pack.** Would have rejected every healthy
  battery.
- **The braking protocol could not have produced a defensible result.** Timer-based rather
  than mark-based, recorded commanded rather than measured speed, and two low speeds
  cannot separate `T_stop` from `a` — the auditor showed 0.5 mm of bias moving `a` from
  1.0 to 2.5 with essentially zero residual. The tool now reproduces that demonstration
  itself and refuses to report `a` from fewer than three well-spread speeds.
- **`failsafe_report.json` said `watchdog_bound_s`**, which reads as firmware protection
  this robot does not have.
- **`verify_twin.py --live` checked synthetic fixture waypoints**, so it tested nothing.

## Open, and honest

- **Stopping distance is unmeasured.** The floor gate. Everything about speed staging
  depends on it, and `docs/safety-case.md` no longer claims braking is the minor term,
  because that rested on an assumed `a`.
- **`/odom_laser` has never run on a moving robot.** Bags and the stand only.
- **`ekf_corrected.yaml` is bench-validated under slip only.**
- **The lidar/gyro yaw scale disagreement (~1.556) is unresolved.** The body cannot rotate
  on the stand, so it waits for the floor. The gyro can only be integrated between bag
  arrival times, and scan jitter is 113–147 ms p95, which may be the whole explanation.
- **Scan distortion is uncorrected.** At 12 Hz a scan is not an instantaneous snapshot.
- **The URDF and the vendor's own static TF disagree** about lidar height by 15 mm
  (0.0789 vs 0.094079). Irrelevant to planar work, unresolved for 3D.
- **`cv2`/`image_geometry` is left broken**, deliberately: this robot has no camera.
- Still never run: **Nav2, SLAM**, the twin as a launch against the real car. Still absent:
  **SROS2**, command arbitration, licences. Still unexplained: the **1.87× simulated wheel
  radius**.

## What I changed outside this repository, this session

- `pip install --user --break-system-packages "scipy>=1.14"` — every compiled scipy
  submodule was broken machine-wide by a pip numpy shadowing apt's.
- `pip install --user --break-system-packages --upgrade transforms3d` (0.4.1 → 0.4.2) —
  it was broken by the same cause, and took `tf_transformations`, `tf2_geometry_msgs` and
  `tf2_sensor_msgs` with it, so all Python TF maths was down.
- Created `~/.venvs/microros` as the going-forward policy.
- Replaced the `uros-udp` agent container after my own agent-freeze test corrupted the
  XRCE session. Identical config, captured from `docker inspect` first.

Reverse the pip installs with `pip uninstall`; apt's versions are untouched on disk.
