#!/usr/bin/env python3
"""Measure how far the car actually travels after being told to stop.

    ./tools/measure_braking.py --calibrate       # once: hand-push odometry scale
    ./tools/measure_braking.py --speed 0.05 --runs 5
    ./tools/measure_braking.py --fit             # analyse; nothing moves

READ docs/first-floor-procedure.md FIRST. Clear area, soft perimeter, hand on the power
switch.

THE MEASUREMENT PROBLEM
-----------------------
"Distance from zero-commanded to at-rest" cannot be tape-measured directly, because
nobody can mark the position of a moving robot at the instant a command is sent. Human
reaction alone is ~250 ms, which at 0.10 m/s is 25 mm -- larger than the quantity being
measured.

The obvious workaround is to integrate /odom_raw over the braking phase. That is exactly
wrong: odometry is encoder-derived, so if the wheels lock or slip while braking it
reports the wheels, not the ground. It under-reports precisely in the regime of interest,
and it fails silently.

So this tool never trusts odometry during braking:

    stopping distance  =  TOTAL(tape)  -  k * runup(odometry)

The tape measures the whole run, START mark to final rest. Odometry supplies only the
RUN-UP, which happens at steady speed where encoders are trustworthy. k is an odometry
scale factor from --calibrate, obtained by pushing the robot a tape-measured distance
with the motors off -- no traction, no slip, no braking.

WHAT THE IMU CAN AND CANNOT WITNESS
-----------------------------------
The obvious wish is for the IMU to give a second opinion on distance. It depends
entirely on DURATION, because double integration accumulates bias as 0.5*b*t^2. With a
realistic ~0.05 m/s^2 residual after static-bias removal:

    calibration push, ~4 s     ->  ~400 mm of drift on an 1830 mm push   USELESS
    braking event,  ~0.3 s     ->  ~2 mm on a ~50 mm stop                GOOD

So the IMU is not a witness for the push, and is one for braking. Short is why.

There is a second limit that runs the other way. The chassis pitches forward under
braking, tilting the accelerometer into gravity and ADDING apparent deceleration. One
degree of dive leaks 0.171 m/s^2, which against the actual decel is:

    at 0.05 m/s   103% of the signal      unusable
    at 0.10 m/s    51%                    indicative only
    at 0.30 m/s    17%                    useful

Static bias subtraction cannot remove it, because the tilt happens during the event. It
biases `a` HIGH, meaning shorter predicted stopping distance -- the dangerous direction.
So IMU distance is reported with the speed-dependent caveat and never overrides tape.

What IS robust at every speed is SINGLE integration. Delta-v drifts linearly, not
quadratically, so comparing the encoder's speed change against the IMU's is a direct
slip detector: encoders measure wheels, the IMU measures the body, and a gap between
them IS the slip. That is the check that matters on a dirty floor.

For the calibration push the gyro is the useful axis, not the accelerometer: integrating
yaw rate shows whether the push actually went straight. A curved push makes the wheels
travel an arc while the tape measures the chord, inflating odometry and biasing k low.

WHY SEVERAL SPEEDS, AND WHY LOW SPEEDS ALONE ARE NOT ENOUGH
------------------------------------------------------------
    d(v) = v*T_stop + v^2/(2a)

T_stop contributes linearly, `a` quadratically, so separating them needs a spread of
speeds. Over a narrow, low range the quadratic term is tiny and the split is
ill-conditioned: an audit showed that **0.5 mm of systematic measurement bias moved the
fitted `a` from 1.0 to 2.5 m/s^2 while leaving essentially zero residual.** A small
residual therefore proves nothing at all here.

Three defences, all reported by --fit:
  * bootstrap confidence intervals over runs, so random scatter is visible;
  * an explicit SYSTEMATIC BIAS sweep, refitting with +/-1 mm added to every measurement,
    because bootstrap cannot see a bias that affects all runs equally;
  * a conservative UPPER envelope (95th percentile of the bootstrap prediction plus a
    margin) which is what may be used for sizing -- never the point estimate.

The tool refuses to report `a` at all from fewer than three distinct speeds, and warns
when the speed range is too narrow to identify it.
"""
import argparse
import json
import math
import os
import sys
import threading
import time
from datetime import datetime

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _cmd_vel_safety import install_stop_handlers               # noqa: E402

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT_DIR = os.path.join(REPO, 'MicroROS-assets', 'logs')
# Hardware and simulator results live in SEPARATE stores. They shared one file, and a
# simulator dry run wrote synthetic calibrations and runs into the hardware evidence --
# the same class of mistake that had already put a simulated fail-safe result where a
# hardware one belonged. Separation is structural now, not a matter of remembering.
DATA_HW = os.path.join(REPO, 'yahboomcar_ws', 'src', 'yahboomcar_safety',
                       'braking_runs.json')
DATA_SIM = os.path.join(REPO, 'yahboomcar_ws', 'src', 'yahboomcar_safety',
                        'braking_runs_sim.json')
DATA = DATA_HW          # rebound in main() once --sim-tape is known

AT_REST = 0.02
N_REST = 3
MIN_SPEEDS = 3          # distinct speeds required before `a` is identifiable
MIN_SPREAD = 2.5        # max(v)/min(v) below this and the split is untrustworthy
MIN_RUNS_PER_SPEED = 3  # fewer than this and a per-speed mean means little
MIN_PHYSICAL_BOOT = 0.9  # fraction of bootstrap samples that must be physical
MARGIN_C = 0.10         # m: design margin added to the envelope


def load():
    if os.path.exists(DATA):
        with open(DATA) as f:
            return json.load(f)
    return {'odom_scale_k': None, 'calibrations': [], 'runs': []}


def save(store):
    with open(DATA, 'w') as f:
        json.dump(store, f, indent=2)


def calibration_problems(cal):
    """Why this calibration is unfit to derive stopping distances from. [] means fit.

    Warning about a bad calibration is not enough: every braking run subtracts
    k * run-up from a tape total, so a calibration that is 5% wrong puts 5% of the RUN-UP
    -- which is far longer than the stop -- straight into the stopping distance. A 1.8 m
    run-up at 5% is 90 mm of error on a stop of perhaps 50 mm.
    """
    bad = []
    if cal.get('odom_m', 0) < 1.0:
        bad.append(f'push only {cal.get("odom_m", 0)*1000:.0f} mm; a fixed 10 mm tape '
                   f'error is {10.0/max(cal.get("odom_m", 0.01)*1000, 1)*100:.1f}% of it')
    dy = cal.get('yaw_change_rad')
    span = cal.get('gyro_span_rad_s')
    abs_yaw = cal.get('abs_yaw_rad')
    if dy is None:
        bad.append('no IMU record, so nobody knows whether the push was straight')
    else:
        # A DEAD GYRO IS THE BEST-LOOKING CALIBRATION THIS TOOL CAN PRODUCE, and the old
        # gate passed it: yaw_change_rad == 0.0 satisfied "not None" and "<= 5 degrees".
        # This robot's gyro publishes exactly 0.000000 on all three axes when it fails --
        # confirmed in 3 of 5 informative bags, and CLAUDE.md calls it out as a confident
        # zero any filter will fuse. Audit finding.
        if span is None:
            bad.append('no gyro-span record, so a DEAD gyro cannot be told apart from a '
                       'straight push -- both integrate to zero yaw. Recalibrate with a '
                       'build that records gyro_span_rad_s')
        elif span < 1e-6:
            bad.append(f'gyro span is {span:.9f} rad/s across the whole push -- exactly '
                       'flat. That is this robot\'s documented gyro failure signature, '
                       'not evidence of straightness')
        elif abs(math.degrees(dy)) > 5.0:
            bad.append(f'push curved by {math.degrees(dy):+.0f} deg; the wheels traced an '
                       'arc while the tape measured the chord')
        elif abs_yaw is not None and math.degrees(abs_yaw) > 12.0:
            # Net yaw cannot see an S-shape: two opposite turns cancel to zero while the
            # wheels traced two arcs and the tape measured a chord.
            bad.append(f'push snaked: {math.degrees(abs_yaw):.0f} deg of TOTAL turning '
                       f'with only {math.degrees(dy):+.0f} deg net')
    if not 0.8 < cal.get('k', 0) < 1.3:
        bad.append(f'k = {cal.get("k", 0):.3f} is implausible')
    return bad


def active_calibration(store):
    """The calibration record the active pointer refers to, BY ID.

    Selection used to be by float k -- `abs(c.k - active_k) < 1e-12` -- which returns
    the FIRST of any calibrations sharing a scale factor, and two sessions on two
    floors can legitimately share one. The id is minted once and never recomputed, so
    it cannot collide. The float path survives only for stores predating ids.
    Audit finding (round 5; round 4 fixed the run matching but missed this selector).
    """
    cal_id = store.get('active_cal_id')
    if cal_id is not None:
        for c in store.get('calibrations', []):
            if c.get('cal_id') == cal_id:
                return c
        return None          # dangling pointer: refuse to guess by value
    k = store.get('odom_scale_k')
    if k is None:
        return None
    for c in store.get('calibrations', []):
        if abs(c.get('k', -1) - k) < 1e-12:
            return c
    return None


def runs_for_active_calibration(store):
    """-> (runs, skipped, k). Only runs computed under the CURRENT calibration.

    Every run stores `stop_distance_m = tape - k * runup`, so k is baked into the answer
    at record time. Runs taken under different calibrations therefore carry different
    systematic corrections, and fitting them together produces a model that looks
    perfectly plausible and describes nothing -- the fit had no way to know, because it
    was handed `store['runs']` wholesale. Audit finding.

    Runs predating the per-run `odom_scale_k` field are treated as foreign: it cannot be
    established what correction they carry, and guessing is how bad evidence survives.
    """
    k = store.get('odom_scale_k')
    if k is None:
        return [], list(store.get('runs', [])), None
    # MATCH BY IMMUTABLE ID, not by float. Runs used to be associated with their
    # calibration via abs(run.k - active.k) < 1e-9 -- but k is a MEASUREMENT, not an
    # identity: two different sessions on two different floors can legitimately produce
    # the same scale factor, and their runs would then silently pool into one model
    # carrying two different systematic corrections. The calibration's timestamp is
    # minted once at --calibrate and never recomputed, so it is the identity.
    # Audit finding.
    cal_id = store.get('active_cal_id')
    keep, skip = [], []
    for r in store.get('runs', []):
        if cal_id is not None:
            # Runs recorded before cal_id existed have no provenance and are foreign.
            (keep if r.get('cal_id') == cal_id else skip).append(r)
        else:
            # Legacy store with no active_cal_id: the float match is all there is, and
            # it stays only for reading OLD data -- every new calibration writes an id.
            rk = r.get('odom_scale_k')
            (keep if rk is not None and abs(rk - k) < 1e-9 else skip).append(r)
    return keep, skip, k


def _lstsq(v, d):
    import numpy as np
    A = np.vstack([v, v ** 2]).T
    (t, b), *_ = np.linalg.lstsq(A, d, rcond=None)
    return float(t), float(b)


def fit(runs, n_boot=2000, seed=0):
    """Fit d = T_stop*v + v^2/(2a) with bootstrap CIs and a bias sweep.

    RUNS ARE GROUPED BY COMMANDED SPEED, regressed on MEASURED speed.

    Grouping used to key on `round(measured_speed, 3)`, which is an identity no two real
    runs ever share: 0.049, 0.050 and 0.051 m/s became three separate "speeds" of one run
    each, so `enough_runs_per_speed` could never pass on floor data. Reproduced with a
    physically exact dataset (T_stop 0.25 s, a 1.5 m/s^2, 5 runs at each of 3 commanded
    speeds): the fit recovered 250 ms and 1.50 exactly and still reported
    identifiable=False over 9 spurious speeds. Audit finding.

    The commanded speed is the experimental STAGE and is already recorded on every run;
    the measured speed is the measurement and belongs on the x-axis, not in the key.
    """
    import numpy as np
    pts = [(r['measured_speed_m_s'], r['stop_distance_m'],
            r.get('commanded_speed_m_s'))
           for r in runs
           if r.get('stop_distance_m') is not None
           and r.get('measured_speed_m_s')]
    if len(pts) < 2:
        return {'error': f'only {len(pts)} usable run(s); need at least 2'}

    v = np.array([p[0] for p in pts])
    d = np.array([p[1] for p in pts])
    # Runs predating commanded_speed_m_s fall back to the measured value, which restores
    # the old (broken) grouping for those runs ONLY, and says so.
    legacy = sum(1 for p in pts if p[2] is None)
    stage = np.array([p[2] if p[2] is not None else round(p[0], 3) for p in pts])
    speeds = sorted({float(x) for x in stage})
    spread = max(speeds) / min(speeds) if min(speeds) > 0 else 1.0

    t_hat, b_hat = _lstsq(v, d)
    res = d - (t_hat * v + b_hat * v ** 2)

    out = {
        'n_runs': len(pts),
        'legacy_runs_without_commanded_speed': legacy,
        'distinct_speeds': speeds,
        'speed_spread': spread,
        'T_stop_s': t_hat,
        'residual_rms_m': float(np.sqrt(np.mean(res ** 2))),
        'residual_max_m': float(np.max(np.abs(res))),
        'points': [(float(vv), float(dd)) for vv, dd, _ in pts],
    }
    out['decel_m_s2'] = (1.0 / (2.0 * b_hat)) if b_hat > 0 else None

    # PHYSICAL VALIDITY, checked one named condition at a time.
    #
    # An earlier version called a fit `identifiable` on the strength of speed count and
    # spread ALONE, which let it bless data that could not describe any robot. An audit
    # fed it stopping distances that DECREASED with speed and got identifiable: True with
    # T_stop = 5.109 s; and a set giving T_stop = -0.295 s, also identifiable: True.
    # Neither is a hard case -- both are impossible, and the fit had no opinion.
    per_speed = {sp: [float(dd) for dd, st in zip(d, stage) if st == sp]
                 for sp in speeds}
    means = [float(np.mean(per_speed[sp])) for sp in speeds]

    checks = {
        'enough_speeds': len(speeds) >= MIN_SPEEDS,
        'enough_spread': spread >= MIN_SPREAD,
        # A per-speed mean from one or two runs is not a mean.
        'enough_runs_per_speed': all(len(per_speed[sp]) >= MIN_RUNS_PER_SPEED
                                     for sp in speeds),
        # Dead time cannot be negative: the robot does not begin stopping before it is
        # told to.
        't_stop_positive': t_hat > 0,
        # Deceleration must be positive and finite, or the model is not a stopping model.
        'decel_positive': b_hat > 0 and out['decel_m_s2'] is not None,
        # A faster robot cannot stop in a SHORTER distance. This is the check that
        # catches wholesale nonsense before any curve is fitted through it.
        'monotonic_in_speed': all(b >= a - 1e-9 for a, b in zip(means, means[1:])),
    }
    out['per_speed_mean_m'] = dict(zip(map(str, speeds), means))

    rng = np.random.default_rng(seed)
    boot = []
    for _ in range(n_boot):
        idx = rng.integers(0, len(v), len(v))
        if len({float(x) for x in stage[idx]}) < 2:
            continue
        try:
            boot.append(_lstsq(v[idx], d[idx]))
        except Exception:
            continue
    if boot:
        bt = np.array([x[0] for x in boot])
        bb = np.array([x[1] for x in boot])
        out['T_stop_ci'] = [float(np.percentile(bt, 5)), float(np.percentile(bt, 95))]
        # A bootstrap sample with a non-positive coefficient does not describe a robot
        # that stops. Counting how MANY are unphysical is itself the identifiability
        # test: if the data cannot pin the sign, the parameter is not determined.
        physical = (bb > 0) & (bt > 0)
        out['boot_physical_fraction'] = float(physical.mean())
        checks['bootstrap_mostly_physical'] = (
            float(physical.mean()) >= MIN_PHYSICAL_BOOT)
        if physical.sum() > 10:
            a_s = 1.0 / (2.0 * bb[physical])
            out['decel_ci'] = [float(np.percentile(a_s, 5)),
                               float(np.percentile(a_s, 95))]
            checks['decel_ci_positive'] = bool(np.percentile(a_s, 5) > 0)
        else:
            checks['decel_ci_positive'] = False
        # Only physical samples may inform an envelope.
        out['_boot'] = (bt[physical], bb[physical])
    else:
        checks['bootstrap_mostly_physical'] = False
        checks['decel_ci_positive'] = False

    out['checks'] = checks
    out['identifiable'] = all(checks.values())
    out['failed_checks'] = [k for k, v in checks.items() if not v]

    # Bootstrap cannot see a bias common to every run, which is the failure mode that
    # actually bit here. Refit with a fixed offset on all measurements instead.
    out['bias_sweep'] = []
    for mm in (-1.0, -0.5, 0.5, 1.0):
        try:
            t_b, b_b = _lstsq(v, d + mm / 1000.0)
            out['bias_sweep'].append({
                'bias_mm': mm, 'T_stop_s': t_b,
                'decel_m_s2': (1.0 / (2.0 * b_b)) if b_b > 0 else None})
        except Exception:
            pass
    return out


def envelope(f, speed, pct=95):
    """Conservative predicted stopping distance, or None when the data cannot support one.

    Three defences, because an earlier version had none of them and produced a 33 mm
    "conservative" envelope at 0.20 m/s from data whose own worst observed stop was
    50 mm -- predicting a stop shorter than one already measured.

    1. REFUSE an invalid fit outright. A number derived from data that cannot describe a
       robot is worse than no number, because it will be used.
    2. Take the worst across the SYSTEMATIC BIAS sweep, not just the nominal fit. The
       bootstrap cannot see a bias common to every run, and 0.5 mm of it moved fitted `a`
       from 1.0 to 2.5 in the case that prompted the sweep.
    3. Floor it at the WORST DISTANCE ACTUALLY OBSERVED at or below this speed. No model
       may predict a stop shorter than one that has already happened. This is the check
       that needs no theory and cannot be argued with.
    """
    import numpy as np
    if 'error' in f or not f.get('identifiable'):
        return None
    if '_boot' not in f or len(f['_boot'][0]) < 10:
        return None

    bt, bb = f['_boot']
    preds = bt * speed + bb * speed ** 2
    est = float(np.percentile(preds, pct))

    # (2) worst over the bias sweep -- and REFUSE if any sweep result is unphysical.
    #
    # This used to `if b.get('decel_m_s2') and b['T_stop_s'] > 0:`, silently SKIPPING
    # exactly the sweep results that matter. A negative T_stop under 1 mm of assumed
    # bias does not mean "ignore this sample", it means the split between T_stop and `a`
    # is not determined by this data at all -- which is the single failure mode the
    # sweep exists to expose. Skipping it made the envelope "the worst of the results
    # that happened to be physical", not "the worst of the sweep" as documented.
    # Audit finding.
    sweep = f.get('bias_sweep', [])
    if not sweep:
        return None
    for b in sweep:
        if not b.get('decel_m_s2') or b['T_stop_s'] <= 0:
            return None
        est = max(est, b['T_stop_s'] * speed + speed ** 2 / (2.0 * b['decel_m_s2']))

    # (3) never below anything already measured at or below this speed.
    observed = [d for v, d in f.get('points', []) if v <= speed + 1e-9]
    if observed:
        est = max(est, max(observed))

    return est + MARGIN_C


def report(f, say):
    if 'error' in f:
        say(f'  {f["error"]}')
        return
    say(f'  runs: {f["n_runs"]}   distinct speeds: {f["distinct_speeds"]}   '
        f'spread {f["speed_spread"]:.1f}x')
    say(f'  residuals: rms {f["residual_rms_m"]*1000:.1f} mm, '
        f'max {f["residual_max_m"]*1000:.1f} mm')
    say('  (a small residual does NOT mean the split is trustworthy -- see the bias '
        'sweep)')
    say('')
    ci = f.get('T_stop_ci')
    say(f'  T_stop = {f["T_stop_s"]*1000:6.0f} ms' +
        (f'   90% CI [{ci[0]*1000:.0f}, {ci[1]*1000:.0f}]' if ci else ''))
    if f.get('decel_m_s2'):
        aci = f.get('decel_ci')
        say(f'  a      = {f["decel_m_s2"]:6.2f} m/s^2' +
            (f'   90% CI [{aci[0]:.2f}, {aci[1]:.2f}]' if aci else ''))
    else:
        say('  a      = NOT IDENTIFIABLE (fitted quadratic term <= 0)')

    say('')
    say('  VALIDITY CHECKS')
    explain = {
        'enough_speeds': f'at least {MIN_SPEEDS} distinct speeds',
        'enough_spread': f'speed spread at least {MIN_SPREAD}x',
        'enough_runs_per_speed': f'at least {MIN_RUNS_PER_SPEED} runs at every speed',
        't_stop_positive': 'dead time positive (the robot cannot brake before being told)',
        'decel_positive': 'deceleration positive and finite',
        'monotonic_in_speed': 'stopping distance rises with speed (faster cannot be shorter)',
        'bootstrap_mostly_physical': f'at least {MIN_PHYSICAL_BOOT:.0%} of bootstrap fits physical',
        'decel_ci_positive': 'lower confidence bound on deceleration above zero',
    }
    for k, v in f.get('checks', {}).items():
        say(f'    [{"ok" if v else "FAIL"}] {explain.get(k, k)}')

    if not f['identifiable']:
        say('')
        say('  *** THIS FIT IS NOT USABLE. No envelope will be produced. ***')
        say(f'      failed: {", ".join(f["failed_checks"])}')
        say('      A model fitted to data that cannot describe a robot will still')
        say('      produce numbers, and they will be used. That is the failure this')
        say('      refuses.')

    if f.get('bias_sweep'):
        say('')
        say('  systematic bias sweep (same bias applied to EVERY measurement):')
        say('     bias    T_stop        a')
        for b in f['bias_sweep']:
            a = f'{b["decel_m_s2"]:.2f}' if b['decel_m_s2'] else '  n/a'
            say(f'    {b["bias_mm"]:+5.1f} mm  {b["T_stop_s"]*1000:6.0f} ms   {a:>6}')
        vals = [b['decel_m_s2'] for b in f['bias_sweep'] if b['decel_m_s2']]
        if len(vals) >= 2 and min(vals) > 0 and max(vals) / min(vals) > 1.5:
            say(f'    +/-1 mm of bias swings `a` by {max(vals)/min(vals):.1f}x. '
                'Do not use the point estimate.')

    say('')
    if not f['identifiable']:
        say('  STOPPING ENVELOPE: withheld, the fit is not usable (see above).')
        return
    say('  CONSERVATIVE STOPPING ENVELOPE')
    say('    = max(95th pct bootstrap, worst bias-swept fit, worst OBSERVED stop)'
        f' + {MARGIN_C*1000:.0f} mm')
    tested = f['distinct_speeds']
    for v in (0.05, 0.10, 0.20, 0.30):
        e = envelope(f, v)
        if e is None:
            continue
        extrap = '' if (tested and min(tested) <= v <= max(tested)) \
            else f'   EXTRAPOLATION ({v/max(tested):.1f}x beyond tested)'
        say(f'    {v:.2f} m/s -> {e*1000:5.0f} mm{extrap}')
    say('')
    say('  Use the envelope, never the point estimate. Extrapolated rows are not')
    say('  evidence -- they are what the next step must be measured against.')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--speed', type=float, default=0.05)
    ap.add_argument('--runs', type=int, default=5)
    ap.add_argument('--hold', type=float, default=4.0,
                    help='seconds at speed before commanding the stop')
    ap.add_argument('--max-speed', type=float, default=0.10)
    ap.add_argument('--i-accept-the-risk', action='store_true')
    ap.add_argument('--direct', action='store_true',
                    help='publish /cmd_vel, bypassing the governor. NOT for floor use.')
    ap.add_argument('--domain', type=int, default=20)
    ap.add_argument('--fit', action='store_true', help='analyse recorded runs and exit')
    ap.add_argument('--calibrate', action='store_true',
                    help='hand-push odometry scale calibration; motors stay off')
    ap.add_argument('--list-calibrations', action='store_true',
                    help='show every recorded calibration and which one is active')
    ap.add_argument('--use-calibration', type=int, metavar='N',
                    help='make calibration N active (see --list-calibrations)')
    ap.add_argument('--sim-tape', action='store_true',
                    help='SIMULATION ONLY: take the "tape" measurement from the '
                         'simulator ground truth instead of prompting. Refuses to run '
                         'against real hardware.')
    args = ap.parse_args()

    global DATA
    DATA = DATA_SIM if args.sim_tape else DATA_HW

    store = load()

    cals = store.get('calibrations', [])

    if args.list_calibrations or args.use_calibration is not None:
        import statistics
        ks = [c['k'] for c in cals]
        med = statistics.median(ks) if ks else None
        print(f'{len(cals)} calibration(s); active k = {store.get("odom_scale_k")}')
        print()
        print('  #  when              push      odom      k       vs median  notes')
        for i, c in enumerate(cals):
            notes = []
            if c['odom_m'] < 1.0:
                notes.append(f'SHORT ({10.0/(c["odom_m"]*1000)*100:.1f}%/10mm)')
            dy = c.get('yaw_change_rad')
            if dy is not None and abs(math.degrees(dy)) > 5.0:
                notes.append(f'CURVED {math.degrees(dy):+.0f}deg')
            elif dy is None:
                notes.append('no IMU record')
            dev = (c['k'] / med - 1) * 100 if med else 0.0
            if abs(dev) > 2.0:
                notes.append('OUTLIER')
            active = '*' if abs(c['k'] - (store.get('odom_scale_k') or -1)) < 1e-12 else ' '
            print(f' {active}{i}  {c["timestamp"]}  {c["tape_m"]*1000:6.0f}mm  '
                  f'{c["odom_m"]*1000:6.0f}mm  {c["k"]:.4f}  {dev:+6.2f}%   '
                  f'{", ".join(notes)}')
        if med:
            print()
            print(f'  median k = {med:.4f}   spread '
                  f'{(max(ks)/min(ks)-1)*100:.1f}%')
            print('  Slip during a push makes odometry UNDER-report, so k > 1. That is')
            print('  a ONE-DIRECTIONAL error, which changes how to combine these:')
            print('  the MINIMUM k among long, straight pushes is the least-contaminated')
            print('  estimate. Median and mean are both wrong here, because they average')
            print('  in contamination that only ever pushes one way.')
            good = [c for c in cals if c['odom_m'] >= 1.0
                    and (c.get('yaw_change_rad') is None
                         or abs(math.degrees(c['yaw_change_rad'])) <= 5.0)]
            if good:
                best = min(good, key=lambda c: c['k'])
                i = cals.index(best)
                print()
                print(f'  RECOMMENDED: calibration {i} (k = {best["k"]:.4f}) -- lowest k '
                      f'among {len(good)} push(es) over 1 m.')
                if abs(best['k'] - (store.get('odom_scale_k') or -1)) > 1e-12:
                    print(f'  Active is {store.get("odom_scale_k"):.4f}. '
                          f'Set it with --use-calibration {i}')
                else:
                    print('  That is already active.')
        if args.use_calibration is not None:
            if not 0 <= args.use_calibration < len(cals):
                print(f'\nno calibration {args.use_calibration}')
                return 2
            chosen = cals[args.use_calibration]
            store['odom_scale_k'] = chosen['k']
            # BOTH pointers, atomically. Setting only k left active_cal_id STALE, so
            # every later run was stamped with the identity of a calibration it was
            # not computed under -- corrupted provenance that looks perfectly healthy.
            # Audit finding, reproduced by the auditor.
            store['active_cal_id'] = chosen.get('cal_id')
            save(store)
            print(f'\nactive k set to {store["odom_scale_k"]:.4f} '
                  f'(calibration {args.use_calibration}, '
                  f'cal_id {store["active_cal_id"]})')
            if store['active_cal_id'] is None:
                print('  NOTE: this calibration predates cal_id. Runs recorded under it')
                print('  cannot be provenance-matched; prefer a fresh --calibrate.')
        return 0

    if args.fit:
        keep, skip, k = runs_for_active_calibration(store)
        print('=== fit over recorded runs ===')
        print(f'odometry scale k = {k}')
        if skip:
            print(f'  EXCLUDED {len(skip)} run(s) recorded under a DIFFERENT calibration')
            print('  (or with none recorded). stop_distance = tape - k*runup, so those')
            print('  runs carry a different systematic correction and cannot be pooled')
            print('  with these -- the combined model would look fine and mean nothing.')
        if not keep:
            print('  no runs match the active calibration; nothing to fit')
            return 1
        report(fit(keep), print)
        return 0

    if not args.calibrate and args.speed > args.max_speed \
            and not args.i_accept_the_risk:
        print(f'REFUSED: {args.speed} m/s exceeds --max-speed {args.max_speed}.')
        print('See the staging table in docs/first-floor-procedure.md.')
        return 2

    if not args.calibrate and store.get('odom_scale_k') is None:
        print('REFUSED: no odometry scale factor yet. Run --calibrate first.')
        print('Without it the run-up distance is unknown, and the stopping distance is')
        print('derived by subtracting the run-up from the tape measurement.')
        return 2

    # A calibration existing is not the same as a calibration being usable. Every run
    # subtracts k * run-up from a tape total, so error in k lands in the stopping
    # distance multiplied by a run-up far longer than the stop itself.
    if not args.calibrate:
        cal = active_calibration(store)
        problems = calibration_problems(cal) if cal else ['active k matches no '
                                                          'recorded calibration']
        if problems:
            print('REFUSED: the active calibration is not fit to derive stopping '
                  'distances from.')
            for b in problems:
                print(f'  - {b}')
            print()
            print('Run --calibrate again: motors off, push STRAIGHT along a measured')
            print('edge, at least 1.5 m. The gyro now witnesses whether it was straight.')
            print('--list-calibrations shows every recorded one and recommends the best.')
            return 2

    if os.environ.get('ROS_DOMAIN_ID') is None:
        os.environ['ROS_DOMAIN_ID'] = str(args.domain)

    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Imu

    lines = []

    def say(m=''):
        print(m, flush=True)
        lines.append(str(m))

    rclpy.init()
    node = Node('measure_braking')
    samples = []
    node.create_subscription(
        Odometry, '/odom_raw',
        lambda m: samples.append((time.time(), m.twist.twist.linear.x)),
        qos_profile_sensor_data)
    truth = []        # simulator ground truth, only present when a simulator is running
    imu = []          # (t, accel_x, gyro_z)
    node.create_subscription(
        Imu, '/imu',
        lambda m: imu.append((time.time(), m.linear_acceleration.x,
                              m.angular_velocity.z)),
        qos_profile_sensor_data)
    if args.sim_tape:
        node.create_subscription(
            Odometry, '/sim/ground_truth',
            lambda m: truth.append((m.pose.pose.position.x, m.pose.pose.position.y)), 10)
    topic = '/cmd_vel' if args.direct else '/cmd_vel_raw'
    pub = node.create_publisher(Twist, topic, 10)
    # `finally: hard_stop()` covers Ctrl+C and exceptions but NOT SIGTERM, whose default
    # action terminates the process before any cleanup runs -- and this tool commands a
    # REAL car at speed on a floor. Audit finding.
    install_stop_handlers(pub)

    ex = SingleThreadedExecutor()
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()

    stamp = datetime.now().strftime('%Y%m%d-%H%M%S')
    time.sleep(2.5)
    if not samples:
        print('FAIL: no /odom_raw. Car powered? Right domain? Agent up?')
        return 2

    if args.sim_tape:
        # Ground truth only exists in simulation. Refusing without it stops --sim-tape
        # ever being pointed at hardware, where there is nothing to substitute for a
        # tape measure and a fabricated one would be indistinguishable from a real one.
        names = [n for n, _ in node.get_node_names_and_namespaces()]
        if 'YB_Car_Node' in names or not truth:
            print('REFUSED: --sim-tape needs /sim/ground_truth from the simulator, and '
                  'must never run against hardware.')
            print('On the real robot the tape measure IS the ground truth; there is no '
                  'substitute, and a fabricated one would look exactly like a real one.')
            return 2
        print('*** --sim-tape: "tape" readings come from SIMULATOR GROUND TRUTH. ***')
        print('*** Results check the TOOL, and say nothing about any real robot.  ***')

    def imu_bias(t_from, t_to):
        """Mean accel-x and gyro-z while stationary: removes static tilt and gyro drift."""
        w = [(a, g) for t, a, g in imu if t_from <= t <= t_to]
        if not w:
            return None, None
        return (sum(x[0] for x in w) / len(w), sum(x[1] for x in w) / len(w))

    def imu_integrate(t_from, t_to, ab, gb):
        """-> (delta_v, distance, delta_yaw, abs_yaw, raw_gyro_span) over a window.

        Trapezoidal. /imu runs at 25 Hz, so a 0.3 s event is only ~7 samples and the
        integration itself is coarse; this is a cross-check, not a precision instrument.

        `abs_yaw` integrates |yaw rate|, i.e. TOTAL TURNING regardless of direction. Net
        yaw cannot see an S-shaped push whose two turns cancel -- that reads as perfectly
        straight while the wheels traced two arcs and the tape measured a chord.

        `raw_gyro_span` is max-min of the UNCORRECTED gyro over the window. It exists to
        tell a genuinely straight push apart from a DEAD gyro: this robot's gyro publishes
        exactly 0.000000 on all three axes when it fails, so a dead one integrates to a
        flawless zero yaw. Span is measured before bias removal because subtracting a bias
        computed from the same dead signal would hide it.
        """
        w = [(t, a - ab, g - gb) for t, a, g in imu if t_from <= t <= (t_to or 1e18)]
        raw = [g for t, _, g in imu if t_from <= t <= (t_to or 1e18)]
        if len(w) < 2:
            return None, None, None, None, None
        dv = d = dyaw = abs_yaw = 0.0
        v = 0.0
        for (t0, a0, g0), (t1, a1, g1) in zip(w, w[1:]):
            dt = t1 - t0
            dv += 0.5 * (a0 + a1) * dt
            d += abs(v) * dt + 0.5 * abs(0.5 * (a0 + a1)) * dt * dt
            v += 0.5 * (a0 + a1) * dt
            dyaw += 0.5 * (g0 + g1) * dt
            abs_yaw += abs(0.5 * (g0 + g1)) * dt
        span = (max(raw) - min(raw)) if raw else 0.0
        return dv, d, dyaw, abs_yaw, span

    def integrate(t_from, t_to=None):
        """Integrate |vx| over [t_from, t_to], INTERPOLATING at both boundaries.

        Taking only whole samples inside the window loses up to one sample period at each
        end, and /odom_raw runs at 11 Hz -- so the error is up to 90 ms of travel, which
        is PROPORTIONAL TO SPEED. A linear-in-v error is indistinguishable from dead time,
        so it lands in T_stop and steals from the quadratic term.

        Measured in the simulator dry run against known values (decel 1.5, dead time
        0.25): whole-sample integration over-reported stopping distance by +5, +8 and
        +17 mm at 0.05, 0.10 and 0.15 m/s. Interpolating the ends removes a bias that
        would otherwise corrupt the floor measurement in exactly the same way.
        """
        if t_to is None:
            t_to = samples[-1][0] if samples else t_from
        seg = [s for s in samples if t_from <= s[0] <= t_to]
        total = sum(abs(a[1]) * (b[0] - a[0]) for a, b in zip(seg, seg[1:]))

        def v_at(t):
            """Linear interpolation of |vx| at an arbitrary time."""
            before = [s for s in samples if s[0] <= t]
            after = [s for s in samples if s[0] >= t]
            if not before or not after:
                return None
            a, b = before[-1], after[0]
            if b[0] == a[0]:
                return abs(a[1])
            f = (t - a[0]) / (b[0] - a[0])
            return abs(a[1]) + f * (abs(b[1]) - abs(a[1]))

        # The partial slivers the whole-sample sum missed, at each end.
        if seg:
            v0, v1 = v_at(t_from), v_at(seg[0][0])
            if v0 is not None and v1 is not None:
                total += 0.5 * (v0 + v1) * (seg[0][0] - t_from)
            v0, v1 = v_at(seg[-1][0]), v_at(t_to)
            if v0 is not None and v1 is not None:
                total += 0.5 * (v0 + v1) * (t_to - seg[-1][0])
        return total

    # ------------------------------------------------------------- calibration
    if args.calibrate:
        say(f'=== odometry scale calibration  {stamp} ===')
        say('Motors stay OFF. Push the car by hand, in a straight line, along a')
        say('measured edge. No traction and no braking, so the encoders cannot slip.')
        try:
            input('Place the car at the start mark and press Enter... ')
        except EOFError:
            pass
        say('  holding still for 2 s to measure IMU bias...')
        t_bias0 = time.time()
        time.sleep(2.0)
        ab, gb = imu_bias(t_bias0, time.time())
        t0 = time.time()
        try:
            raw = input('Push it now, then type the tape distance in mm and press Enter: ')
            tape = float(raw.strip()) / 1000.0
        except (EOFError, ValueError):
            say('no measurement given; aborted')
            return 2
        t_end = time.time()
        odo = integrate(t0)
        if odo <= 0:
            say('FAIL: odometry recorded no movement. Did the car move? Is it powered?')
            return 2
        k = tape / odo
        say(f'  odometry {odo*1000:.0f} mm   tape {tape*1000:.0f} mm   k = {k:.4f}')

        # Straightness, from the gyro. A curved push makes the wheels trace an arc while
        # the tape measures the chord; arc/chord ~ 1 + theta^2/24 for small theta, so the
        # odometry reads long and k comes out low.
        dyaw = abs_yaw = gyro_span = None
        if gb is not None:
            _, _, dyaw, abs_yaw, gyro_span = imu_integrate(t0, t_end, ab or 0.0, gb)
        if dyaw is None:
            say('  straightness: NO IMU DATA -- cannot tell whether the push was straight')
        else:
            infl = (dyaw ** 2) / 24.0
            say(f'  straightness: net yaw {math.degrees(dyaw):+.1f} deg, '
                f'TOTAL turning {math.degrees(abs_yaw):.1f} deg')
            say(f'    implied arc-vs-chord inflation of odometry: {infl*100:.2f}%')
            say(f'    raw gyro span over the push: {gyro_span:.6f} rad/s')
            # A DEAD gyro integrates to a flawless zero and would otherwise certify the
            # most convincing calibration this tool can produce. This robot's gyro
            # publishes exactly 0.000000 on all three axes when it fails, confirmed in 3
            # of 5 informative bags. Audit finding: the old gate accepted it.
            if gyro_span < 1e-6:
                say('    *** GYRO IS FLAT: span is exactly zero across the whole push.')
                say('    That is this robot\'s documented failure signature, not a')
                say('    straight push. A dead gyro cannot witness anything. ***')
            elif abs(math.degrees(dyaw)) > 5.0:
                say('    *** the push CURVED. k is biased low; redo it against a '
                    'straight edge. ***')
            elif abs_yaw is not None and math.degrees(abs_yaw) > 12.0:
                say(f'    *** the push SNAKED: {math.degrees(abs_yaw):.0f} deg of total '
                    'turning with little net change. The wheels traced arcs while the '
                    'tape measured a chord. ***')
        if odo < 1.0:
            say(f'  *** SHORT PUSH ({odo*1000:.0f} mm). A fixed +/-10 mm tape error is '
                f'{10.0/(odo*1000)*100:.1f}% here. Push at least 1.5 m. ***')
        if not 0.5 < k < 2.0:
            say(f'  REFUSED: k={k:.3f} is implausible. Check the push was straight and')
            say('  that the tape figure is in millimetres.')
            return 2
        store.setdefault('calibrations', []).append(
            {'timestamp': stamp,
             # The IMMUTABLE identity runs are associated by. k is a measurement two
             # sessions can coincidentally share; this cannot be.
             'cal_id': stamp,
             'tape_m': tape, 'odom_m': odo, 'k': k,
             'yaw_change_rad': dyaw,
             # Both recorded so calibration_problems() can tell a straight push from a
             # dead gyro and from an S-shape. Net yaw alone cannot distinguish either.
             'abs_yaw_rad': abs_yaw,
             'gyro_span_rad_s': gyro_span,
             'arc_chord_inflation': ((dyaw ** 2) / 24.0) if dyaw is not None else None})
        store['odom_scale_k'] = k
        store['active_cal_id'] = stamp
        save(store)
        say(f'  saved. Odometry reads {(1/k - 1)*100:+.1f}% vs ground truth.')
        return 0

    # ------------------------------------------------------------- braking runs
    k = store['odom_scale_k']
    say(f'braking measurement  {stamp}')
    say(f'publishing to {topic}' + ('  *** GOVERNOR BYPASSED ***' if args.direct else ''))
    say(f'{args.runs} runs at {args.speed} m/s, odometry scale k = {k:.4f}')
    say('FLOOR TEST. Hand on the power switch.')

    def hard_stop():
        for _ in range(15):
            pub.publish(Twist())
            time.sleep(0.04)

    new = []
    try:
        for i in range(args.runs):
            say('')
            say(f'--- run {i+1}/{args.runs} ---')
            if not args.sim_tape:
                try:
                    input('  Car at the START mark, then press Enter... ')
                except EOFError:
                    pass

            # Stationary window first: removes static tilt from accel-x and gyro drift.
            t_bias0 = time.time()
            time.sleep(1.5)
            ab, gb = imu_bias(t_bias0, time.time())

            truth_start = truth[-1] if truth else (0.0, 0.0)
            t_start = time.time()
            t = Twist()
            t.linear.x = float(args.speed)
            end = time.time() + args.hold
            while time.time() < end:
                pub.publish(t)
                time.sleep(0.05)

            t_zero = time.time()
            for _ in range(3):
                pub.publish(Twist())
                time.sleep(0.02)
            say('  ZERO commanded')
            time.sleep(5.0)

            # Measured, not commanded: the achieved speed differs from the request, and
            # the fit needs the speed the robot actually had when told to stop.
            pre = [s[1] for s in samples if t_zero - 0.6 <= s[0] < t_zero]
            v_meas = (sum(abs(x) for x in pre) / len(pre) * k) if pre else None
            runup = integrate(t_start, t_zero) * k
            odo_stop = integrate(t_zero) * k
            rest_t = None
            after = [s for s in samples if s[0] >= t_zero]
            for j in range(len(after) - N_REST + 1):
                if all(abs(x) <= AT_REST for _, x in after[j:j + N_REST]):
                    rest_t = after[j][0] - t_zero
                    break

            say(f'  measured speed before stop: '
                f'{v_meas*1000:.0f} mm/s' if v_meas else '  speed: UNKNOWN')
            say(f'  run-up (odometry x k): {runup*1000:.0f} mm')
            say(f'  odometry during braking: {odo_stop*1000:.0f} mm  '
                '(NOT used -- encoders lie when wheels slip)')
            say(f'  time to rest: {rest_t*1000:.0f} ms' if rest_t
                else '  NEVER REACHED REST -- abort and check the deadman')

            # --- IMU cross-check over the braking window only ---
            dv_imu = d_imu = slip = None
            t_rest = (t_zero + rest_t) if rest_t else None
            if ab is not None and t_rest:
                dv_imu, d_imu, _, _, _ = imu_integrate(t_zero, t_rest, ab, gb)
            if dv_imu is not None and v_meas:
                # Encoders say the wheels lost v_meas. The IMU says the BODY lost
                # |dv_imu|. A gap between them is slip -- the wheels and the ground
                # disagreeing, which is exactly what a dirty floor produces.
                slip = (abs(dv_imu) - v_meas) / v_meas
                say(f'  IMU delta-v {abs(dv_imu)*1000:.0f} mm/s vs encoder '
                    f'{v_meas*1000:.0f} mm/s   -> {slip*100:+.0f}%')
                if abs(slip) > 0.30:
                    say('    *** WHEELS AND BODY DISAGREE BY >30% -- suspect slip. ***')
                # Distance: trustworthy only where brake-dive is small next to the decel.
                a_est = v_meas / rest_t if rest_t else 0.0
                dive_frac = (9.81 * 0.01745 / a_est) if a_est > 0 else 9.9
                verdict = ('useful' if dive_frac < 0.25 else
                           'indicative' if dive_frac < 0.6 else 'UNUSABLE')
                say(f'  IMU stopping distance {d_imu*1000:.0f} mm  '
                    f'[1 deg of brake-dive = {dive_frac*100:.0f}% of decel -> {verdict}]')
            elif ab is None:
                say('  IMU: no data (is /imu publishing?)')

            total = None
            if args.sim_tape:
                # Straight-line distance from where the run began to where it ended --
                # exactly what a tape between the two marks would read.
                if len(truth) > 2:
                    total = math.hypot(truth[-1][0] - truth_start[0],
                                       truth[-1][1] - truth_start[1])
                    say(f'  sim ground-truth total: {total*1000:.0f} mm')
            else:
                try:
                    raw = input('  tape: START mark to final rest, in mm '
                                '(blank to discard): ')
                    if raw.strip():
                        total = float(raw.strip()) / 1000.0
                except (EOFError, ValueError):
                    pass
            if total is None or v_meas is None:
                say('  run discarded')
                hard_stop()
                continue

            stop_d = total - runup
            say(f'  STOPPING DISTANCE = {total*1000:.0f} - {runup*1000:.0f} '
                f'= {stop_d*1000:.0f} mm')
            if stop_d < 0:
                say('  NEGATIVE -- the run-up estimate exceeds the tape total. Either')
                say('  the calibration is wrong or the car did not travel straight.')
            new.append({
                'timestamp': stamp, 'commanded_speed_m_s': args.speed,
                'cal_id': store.get('active_cal_id'),
                'measured_speed_m_s': v_meas, 'total_tape_m': total,
                'runup_m': runup, 'stop_distance_m': stop_d,
                'odom_braking_m': odo_stop, 'time_to_rest_s': rest_t,
                'odom_scale_k': k, 'governed': not args.direct,
                'imu_delta_v_m_s': abs(dv_imu) if dv_imu is not None else None,
                'imu_distance_m': d_imu,
                'imu_vs_encoder_slip_frac': slip,
                # Raw samples kept so a later analysis can revisit the derivation
                # without needing the robot back.
                'samples': [(round(s[0] - t_start, 4), round(s[1], 4))
                            for s in samples if s[0] >= t_start],
            })
            hard_stop()
    except KeyboardInterrupt:
        say('')
        say('  aborted by operator')
    finally:
        hard_stop()

    store['runs'].extend(new)
    save(store)

    keep, skip, _ = runs_for_active_calibration(store)
    say('')
    say(f'=== fit over {len(keep)} run(s) on the ACTIVE calibration ===')
    if skip:
        say(f'    ({len(skip)} run(s) from other calibrations excluded)')
    report(fit(keep), say)

    os.makedirs(OUT_DIR, exist_ok=True)
    path = os.path.join(OUT_DIR, f'braking-{stamp}.log')
    with open(path, 'w') as f:
        f.write('\n'.join(lines) + '\n')
    say('')
    say(f'log:  {path}')
    say(f'runs: {DATA}')
    return 0


if __name__ == '__main__':
    code = main()
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(code)
