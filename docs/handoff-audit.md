# Audit handoff

For a reviewer (human or LLM) checking this work. Written to help you find my errors, not
to defend the work. Where I am unsure, it says so; where I was wrong, it says that too.

**One-line summary:** the ROS 2 Humble→Jazzy port is real and well tested; the Isaac Sim
work was rushed, wasteful, and produced a false "working" claim; and the single most
load-bearing assumption — that `/odom_raw` reflects physical wheel rotation — is **still
unverified**.

---

## Start here: the claims most likely to be wrong

Check these first. They are ordered by how much rests on them.

| # | Claim | Status | Why it deserves scrutiny |
|---|---|---|---|
| 1 | `/odom_raw` reflects real wheel rotation | ❌ **UNVERIFIED** | Never checked against the physical robot. If the firmware computes odometry open-loop from `cmd_vel`, then UMBmark calibration, cartographer (`use_odometry = true`), and the twin's wheel animation all rest on a number that never observed the world. `tools/car_selftest.py --handspin` settles it and has not been run. |
| 2 | "The motors move" | ⚠️ **INFERRED ONLY** | I asserted this from `/odom_raw` values. The operator watching the robot did not see motion. Both can be true if odometry is open-loop. Unresolved. |
| 3 | The chassis is differential, not mecanum | ✅ measured | A commanded strafe produced *exactly* zero on every `/odom_raw` axis. Strong, but note it shares dependency #1: if odometry is open-loop, this shows only that the firmware ignores `linear.y`, which still supports the conclusion. |
| 4 | Isaac twin "articulates and tracks" | ⚠️ **PARTLY FALSE** | Joints did track. But the robot was **falling through the floor** the whole time and I never checked base pose. The verification was blind to the most obvious possible failure. |
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

- **Isaac twin falls through the floor.** Ground was created with `UsdGeom.Plane`
  (visual only, no collider) while the robot imports with `fix_base=False`. Not yet fixed.
- **`tools/urdf_to_usd.py` targets the Isaac 6.0.1 importer API.** That install has been
  deleted; 5.1.0 uses the older command-based API. The tool will fail until updated.
- **Stopping distance is unmeasured.** The safety governor limits commands; it cannot
  beat the robot's braking. Nobody should call it safe until that number exists.
- **`yahboomcar_multi` is unverified** — needs two robots.
- **`transforms3d` is installed but broken** (apt 0.4.1 calls `np.maximum_sctype`,
  removed in NumPy 2.0). Nothing imports it, so it is inert.
- **A 15 mm frame disagreement**: the cartographer launch places `laser_frame` 0.094079 m
  above `base_link`; the URDF's `radar_Joint` says 0.078934 m.

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
