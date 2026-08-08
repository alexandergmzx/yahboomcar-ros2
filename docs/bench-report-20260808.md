# Bench report — 2026-08-08 (robot1 elevated, wheels free)

Session: Isaac backend repair + live bench (lidar, motors, encoders, IMU, safety
chain, transport A/B, SLAM lifecycle). Car elevated at its center on the bench,
tracks free, domain 20. **Every velocity in this file is FREE-SPIN** (unloaded
wheels): signs, symmetry, brackets and latching transfer to the floor; magnitudes
and spin-up times do not (floor is slower — see measure_latency.py's caveat).

Tags: [measured] = number from this bench today; [prior] = earlier recorded
evidence; **negative results in bold**.

---

## 0. Preflight (passive)

| check | result |
|---|---|
| agent container | `uros-udp` up 43 h [measured] |
| board state on arrival | **silent — topics advertised, zero data on all four** [measured]. Agent log: last XRCE session established ~19 h earlier, nothing since. The known post-drop silence; recovered with ONE `tools/board_reset.py` EN pulse (no power cycle needed) |
| domain-20 topic inventory | exactly the firmware contract: /odom_raw /imu /scan /battery /cmd_vel /beep /servo_s1 /servo_s2 — no camera, no extras [measured] |
| battery | **8.3 V** at session start, still 8.3 V at every phase gate [measured] |
| rates vs contract (150 s window) | /scan 12.18 Hz (12) · /imu 23.85 (25) · /odom_raw 10.93 (11) · /battery 1.00 (1) — PASS [measured]. An 8 s snapshot taken ~40 s after the board reset read low (8.4/14.1/8.0); treat short windows right after a reset as spin-up transient, not contract evidence |
| delivered QoS | **all four sensor topics offer RELIABLE/VOLATILE** [measured], not the BEST_EFFORT the contract table documents — see §1 delta table |

Blind-probe observation: two `simctl stop` zero-verification probes earlier in the
session reported `nodata` for domain 20 while 70–75 stale `/dev/shm/fastrtps*`
segments existed; after cleanup the same graph answered. Consistent with the
OI-13 blinding mechanism [measured 2026-08-07]; today's instance is
correlational (not isolated), recorded as corroboration only.

## 1. /scan characterization — 150 s, default transport [all measured]

| quantity | value | vendor claim / prior | delta |
|---|---|---|---|
| mean rate | 12.18 Hz | 12 Hz | +1.5 % |
| jitter (host inter-arrival) | mean 82.1 ms, std 26.5, p95 123.1, max 532 | not claimed | — |
| points per revolution | 360 at 1.000°, span 360.0° | 360 | 0 |
| published range limits | [0.12, 8.00] m | [0.12, 8.0] | 0 |
| valid-return fraction | 95.3 % over 1831 revolutions | — | — |
| plausible ranges seen | [0.295, 3.429] m (bench room) | — | — |
| angular coverage | **no sector below 50 % valid**; worst sectors on this mount: 190–200° 70 %, 270–280° 79 %, 230–240° 82 %, 180–190°/220–230° 86 % | — | — |
| offered QoS | RELIABLE, VOLATILE, depth 0 | BEST_EFFORT (contract table) | **DELTA — recorded** |
| RELIABLE-subscriber test | received 1826/1827 msgs alongside a best-effort subscriber | "a RELIABLE subscriber silently receives nothing" [prior] | **trap did NOT reproduce on the current agent** |

**Range accuracy: UNTESTED.** No reference target was placed this session, and
accuracy is not improvised from room furniture.

The QoS delta and the non-reproducing starvation trap exceed the 10 % rule in
kind (not degree): recorded in the fleet register (OI-20). The safe practice —
subscribe BEST_EFFORT to sensor topics — is unchanged; a RELIABLE offer matches
both.

## 2. Motors / encoders — direction, step, latching [all measured, FREE-SPIN]

Direction/sign matrix (2.5 s steps, steady window = last 1.5 s):

| step | command | /odom_raw steady | sign vs REP-103 |
|---|---|---|---|
| +x | +0.12 m/s | vx +0.1189 ± 0.0050 | MATCHES |
| −x | −0.12 m/s | vx −0.1183 ± 0.0056 | MATCHES |
| +yaw | +0.80 rad/s | wz +0.7843 ± 0.0494 | MATCHES |
| −yaw | −0.80 rad/s | wz −0.8029 ± 0.0361 | MATCHES |

Left/right symmetry: |+x|/|−x| = **1.005**; |+yaw|/|−yaw| = **0.977**.
Strafe (±y) not commanded this session; the chassis is differential and ±y was
measured to produce exactly zero previously [prior].

Step response (3 s steps, ×3 per axis; brackets one 11 Hz interval wide):

| quantity | value |
|---|---|
| command → first /odom_raw motion | upper bounds 171, 203, 216, 219, 233, 375 ms (max 375) |
| command → 90 % of plateau | 261–422 ms |
| steady state vs commanded | 0.973–1.008 (x), 0.992–1.008 (yaw) |

FREE-SPIN lower bound: on the floor static friction and chassis inertia add an
unmeasured amount ([prior] reasoning in measure_latency.py stands).

Latching confirmation: driver publishing 0.12 m/s on /cmd_vel SIGKILLed (no
cleanup zero possible); **motion persisted the full 3 s window (100 % of 32
samples; last moving sample +2.93 s)**; zeroed by the harness finalizer, rest
verified on /odom_raw (peak 0.000). The no-watchdog finding [prior, measured
three ways] reproduced on hardware today.

## 3. Safety chain [measured]

Fleet-install `safety_launch.py` (governor + deadman, permissive distances —
bench lidar sees near walls; the subject is the crash path). Driver on
/cmd_vel_raw (the governed floor path) SIGKILLed mid-motion, ×5:

| run | stop bracket (ms) |
|---|---|
| 1 | [445, 537] |
| 2 | [469, 562] |
| 3 | [404, 492] |
| 4 | [427, 480] |
| 5 | [440, 534] |

Upper-bound spread **480–562 ms**, vs the 693–762 ms [prior]. Faster than the
prior and honestly so: on the governed path the governor's stale-command rule
(cmd_timeout 0.3 s) fires before the deadman's 0.5 s, so this measures the
CHAIN, with the governor first. The 693–762 ms prior remains the bound for
deadman-only stops (driver on /cmd_vel with no governor). Both mechanisms leave
the car verified at rest.

Estop path: motion latched (driver SIGKILLed), then `simctl estop --domain 20`:
at rest **[2041, 2126] ms after the estop process started** — that number
includes process/env startup and the tool's 1 s subscription settle, not just
publish→stop. Rest verified on /odom_raw.

Agent-loss case (case 3 of test_failsafe): **not re-run** — destructive (wedges
the board's XRCE client, needs a hand power-cycle), and the finding is already
decisive from 2026-08-06 [prior].

## 4. IMU under load [measured]

- Accelerometer: live at rest (accel-z std 0.0064) and throughout.
- **THE INTERMITTENT GYRO FAULT APPEARED**: gyro_z flat 0.000000 ± 0.000000
  through BOTH commanded rotations while /odom_raw showed ±0.78–0.80 rad/s.
  Bagged with timestamps: `MicroROS-assets/bags/bench-matrix-20260808-093840`
  (yaw steps ≈ 09:39:00–09:39:10). Not debugged live, per the standing rule.
- Yaw-rate sign vs commanded rotation: **UNTESTABLE this session** (the channel
  under test was dead). The sign check remains open for a session where
  `sensor_health.py --rotate-window` passes first.
- Consequence for today: nothing fused yaw (SLAM ran on scan matching + wheel
  odom TF); do not fuse /imu yaw until the fault is understood.

## 5. Transport A/B — OI-13 evidence [measured, 60 s legs, back-to-back]

| topic | default (shm+udp) | UDP-only profile | rate delta |
|---|---|---|---|
| /scan | 12.40 Hz, gap p95 102.8 max 283 ms | 12.39 Hz, p95 111.5 max 232 | −0.1 % |
| /imu | 24.30 Hz, p95 57.4 max 281 | 24.27 Hz, p95 60.8 max 197 | −0.1 % |
| /odom_raw | 11.06 Hz, p95 112.7 max 243 | 11.01 Hz, p95 115.9 max 232 | −0.5 % |
| /battery | 1.00 Hz | 1.00 Hz | 0 |

**Throughput is identical; worst-case gaps slightly better under UDP-only.**
The decision input is therefore not rates: shm's failure mode is the
stale-segment blinding (410 segments once [prior]; 35 at the 2026-08-07 failure
[measured]; 70–75 correlated with blind probes today), which UDP-only removes as
a class at zero measured cost. **Machine-wide recommendation: set
`FASTDDS_DEFAULT_PROFILES_FILE=ground_station/fastdds_udp_only.xml` for every
fleet process on this machine.** Pi/laptop legs of OI-13 remain untested here.

## 6. OI-14 hardware half — D-19 SLAM vs the real /scan [measured]

Real hardware bringup + `yahboomcar_config slam_launch.py` (bond disabled),
300 s against the live sensor stream, car static:

- **flap count: 0** (zero lifecycle transition events after activation;
  slam_toolbox present in 30/30 graph checks)
- /map: 296 updates, final 2.98 × 3.42 m — static, as expected for a static car
- observed: occasional message-filter drops ("queue is full", ~1 per 2.6 s) —
  scans outpacing TF interpolation; mapping continued throughout

The bond-disabled composition now has hardware evidence, not just sim gates.
OI-14's remaining open half is unchanged in kind — we still WANT supervision on
hardware — but "bond-disabled is stable against real sensors" is now [measured].

## 7. Isaac backend repair (task 1 summary; details in the commit)

- Symptom reproduced under a wall clock: **443 s to FAILED** while `sim_runner`
  had exited in **under 1 s** — stale pre-extraction path
  (`REPO/yahboomcar_ws/.../arena.usd`). The wait polled topics blind to process
  death; that is the "waits indefinitely" report.
- Readiness condition (named): >10 msgs on /scan (BEST_EFFORT sensor QoS) AND
  >10 on /odom (RELIABLE depth 10) within a 4 s probe window, re-polled every
  ~9 s; budget 420 s (isaac) / 90 s (2d).
- Root cause class: the D-12 extraction audit fixed simctl and test_sim.py but
  **eighteen other tools kept private stale copies of the old layout paths**.
  Fixed with one shared resolver (`tools/_layout.py`) and ports in every tool.
- Discovery (2b) and Isaac-itself (2c) ruled out: with paths fixed, the bridge
  published on the scratch domain with no profile changes in 20 s, and Isaac's
  own log showed a clean start.
- Hardening: every backend wait now runs through `wait_for` — names the unmet
  condition, tails the right log, and **fails immediately when the awaited
  backend process dies**. Exercised deliberately (broken `YAHBOOM_USD_DIR`):
  loud failure in 25 s naming the dead process and the log line.
- Gates: isaac lifecycle green from a fresh shell — start **72 s** (headless,
  vs the ~90 s budget), drive verified (70/70 odom samples moving), estop to
  vx = 0.002, clean stop; 2d re-run green at **50 s**.

## Slots not filled, with reasons

| slot | reason |
|---|---|
| lidar range accuracy | no reference target placed; not improvised |
| IMU yaw-rate sign | gyro fault active all session (bagged); channel dead |
| agent-loss stop case | destructive (wedges XRCE client); decisive [prior] evidence 2026-08-06 |
| strafe (±y) matrix column | differential chassis; exact zero [prior]; not re-commanded |
| OI-13 Pi/laptop legs | this machine only today |
| floor (loaded) magnitudes | bench is FREE-SPIN by definition |

Evidence files: `MicroROS-assets/logs/scan-char-*-20260808-*.{log,json}`,
`bench-*-20260808-*.log`, bags `bench-matrix-20260808-093840`,
`selftest-20260808-*`; structured results in
`yahboomcar_safety/bench_motion.json`.
