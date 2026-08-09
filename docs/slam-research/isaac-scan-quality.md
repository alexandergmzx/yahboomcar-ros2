# Isaac lidar scan quality: the smeared-map defect, measured and filtered

**2026-08-09. Context:** Alex reported Isaac-backend SLAM maps come out smeared/unusable
while 2D-backend maps are clean. This document is the measurement trail that convicted
the cause, the hypotheses that died on the way (kept, per this repo's convention), and
the fix. All measurements: bare isaac sessions (`simctl start --backend isaac
--no-isaac-gui --no-safety --no-slam --no-rviz --no-patrol`), system-python probes
driving via the shared `_cmd_vel_safety` guards, `/sim/ground_truth` as the only
rotation reference (`/odom_raw` is encoder-derived and lies under slip BY DESIGN —
measured 1.392 rad/s claimed vs 0.479 rad/s true in the same window).

## The conviction

Consecutive-scan circular cross-correlation (360 beams, 1°/beam) against ground truth:

- **Static body**: consecutive scans should be near-identical. Measured (one session):
  48/93 pairs at shift 0 — and **13 pairs at ±85–86° with rms-after-best-shift ~1.4 m**.
- **Rotating at a true 0.42 rad/s**: the good pairs sit at −2/−3° per scan — exactly
  −Δyaw, the correct sensor-frame signature — interleaved with the same ±85°-class
  outliers.
- Scan-vs-arena-raycast rms (raycast from `yahboomcar_sim.arena` at the truth pose) is
  **bimodal: p10 0.18 m / p90 1.43 m**, ~50% good in a bad session.
- The bad scans arrive in **bursts** (36 consecutive was measured).
- Scans carry **−1.0 no-return sentinels** — finite, and poison to any consumer that
  treats them as ranges.

So the raw stream is a MIXTURE: correct sensor-frame revolutions interleaved with
revolutions assembled with a wrong sweep-phase origin — consistent with the measured,
unexplained ~72-messages-per-render-second emission (≈1.2 messages per render at the
calibrated pacing: the assembly seam lands inside messages). slam_toolbox ingests every
scan at full confidence; a quarter-turn-rotated scan every few messages destroys the
map on the first turn, and a static car maps fine — exactly the reported symptom, and
exactly why bench §6 (static hardware SLAM) never saw it.

**Session nondeterminism, measured:** back-to-back sessions on the same build produced
a ~50% corrupted stream, then a 100% clean one (303/303 scans ≤ 0.141 m rms vs
raycast, median 0.080). Any fix has to be per-scan, not per-session.

## Hypotheses tested and rejected

- **Time dilation / stamp lies** — stamps advance at wall rate (slope 1.0000 on /scan
  and /odom_raw); forward pose-path vs twist·stamped-dt ratio 0.999. Dead.
- **Duplicate scans (stale content, fresh stamp)** — 0/614 bit-identical pairs. Dead.
- **Content world-locked in orientation** — an early probe showed content rotating at
  0.061 rad/s under commanded 0.5, which fit world-locking; a counter-rotation relay
  built on it made things worse. The probe had compared content against /odom_raw
  (which lies) and against sessions where the body barely gripped; with ground truth
  measured simultaneously, good scans are sensor-framed. **The first fix shipped
  against a confounded measurement; the measurement discipline that caught it is the
  same probe, re-referenced.** Dead.
- **Mirrored (CW) azimuth indexing** — a spawn scan matched the arena raycast as-is at
  shift +1 (rms 0.28 m); mirrored was strictly worse. Dead.
- **Robot tilting (lidar plane cutting the floor)** — roll/pitch exactly 0.00° ± 0.00
  during the garbage-dominated window. Dead.

## The fix: validate, never mutate

`tools/_scan_frame_relay.py`, spawned by `sim_runner.py` next to the rate probe. The
RTX helper now publishes `scan_isaac_raw`; the relay compares every message against a
raycast of the **shared arena** (`yahboomcar_sim.arena` — the same single source
`build_arena.py` builds the USD from) at the ground-truth pose, excluding −1 sentinels,
and republishes only scans under a 0.6 m rms gate (the empty middle of the bimodal
distribution) as `/scan`. Corrupted revolutions are dropped; sim_runner's closed-loop
rate trim measures the FILTERED /scan and renders faster to hold the 12 Hz contract.

Fail-open, loudly: if ≥90% of a 300-message window fails, the room geometry itself is
wrong (rebuilt USD, robot outside the arena) and filtering disables itself with an
error — a burst (measured up to 36) or a 50% mixture can never trip it. Ground-truth
use is backend-internal: the relay is part of the simulated firmware, using the
simulator's own state to emit its own sensor honestly; the stack sees only `/scan`.

Verified live: in a clean session the filter passes everything (303/303, delivered
rate == raw rate). **A corrupted session has not yet been caught live post-fix** — the
drop path follows arithmetically from the measured distributions (52% fail < 90%
fail-open bar; gate 0.6 splits p10 0.18 from p90 1.43), but the burst-drop behaviour
under SLAM should be confirmed the next time a session boots corrupted:
`grep dropped` in the relay's output, and the map gate below.

## Related observations, out of scope here

- Achieved body yaw rate at a fixed commanded 0.5 rad/s varied 0.48 / 0.18 / 0.05
  rad/s across sessions and scene states (one 0.05 case was the robot wedged against
  the +x wall after undirected probe driving). The yaw-slip feedforward's calibration
  (`compensate_yaw`) predates today's arena rebuild; if turning realism matters to a
  result, re-run its calibration first.
- `/sim/ground_truth` publishes z = 0 always (hardcoded); irrelevant to SLAM, noted so
  nobody trusts it for height.
- The `_scan_rate_probe` child was missing from simctl's SIM_PROCS kill list and
  outlived sessions unnoticed; fixed alongside (both children are listed now).

## Acceptance gate — RUN, AND FAILED, for a now-measured second cause

An Isaac full-default session (SLAM + patrol, 4 min) with the filter in place produced
a map spanning **7.90 × 8.96 m for the 4 × 4 room** — 2/4 boxes matched, one smeared
into four blobs. The scans were clean; what remains is the ODOMETRY PRIOR:

- Under turn slip, `/odom_raw` yaw runs ~2.9× the true rate (1.392 vs 0.479 rad/s
  [measured]) because `compensate_yaw` deliberately spins the wheels ~3.6× to achieve
  true body rotation against the rear-slide friction model, and the encoders honestly
  report that spin.
- The real car's encoder yaw error is **~7.7%** (handoff-audit, vs gyro). The sim's
  encoder lie is therefore ~25× the size of the lie it exists to reproduce, the vendor
  EKF fuses it, and slam_toolbox receives a rotation prior ~190% wrong — no matcher
  recovers from that, and the map smears exactly as observed.
- The 2D backend never sees this because `fake_robot` defaults `slip:=0`.

**This is left OPEN deliberately.** The fix is a design decision about encoder
fidelity, not a bug patch: options are (a) mapping wheel speeds through the measured
slip line (`YAW_GAIN`/`YAW_LOSS`) so `/odom_raw` lies at the real car's scale, with
the run-to-run grip variance as the honest residual error; or (b) re-tuning the PhysX
friction/feedforward pair so the wheels genuinely spin near-truth. Either touches the
measured constants the feedforward tests pin — Alex's call. Until then: **the Isaac
backend validates the stack against clean scans, but its SLAM maps are NOT
usable evidence**; the 2D backend remains the SLAM validation rig.
