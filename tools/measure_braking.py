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

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT_DIR = os.path.join(REPO, 'MicroROS-assets', 'logs')
DATA = os.path.join(REPO, 'yahboomcar_ws', 'src', 'yahboomcar_safety', 'braking_runs.json')

AT_REST = 0.02
N_REST = 3
MIN_SPEEDS = 3          # distinct speeds required before `a` is identifiable
MIN_SPREAD = 2.5        # max(v)/min(v) below this and the split is untrustworthy
MARGIN_C = 0.10         # m: design margin added to the envelope


def load():
    if os.path.exists(DATA):
        with open(DATA) as f:
            return json.load(f)
    return {'odom_scale_k': None, 'calibrations': [], 'runs': []}


def save(store):
    with open(DATA, 'w') as f:
        json.dump(store, f, indent=2)


def _lstsq(v, d):
    import numpy as np
    A = np.vstack([v, v ** 2]).T
    (t, b), *_ = np.linalg.lstsq(A, d, rcond=None)
    return float(t), float(b)


def fit(runs, n_boot=2000, seed=0):
    """Fit d = T_stop*v + v^2/(2a) with bootstrap CIs and a bias sweep."""
    import numpy as np
    pts = [(r['measured_speed_m_s'], r['stop_distance_m']) for r in runs
           if r.get('stop_distance_m') is not None
           and r.get('measured_speed_m_s')]
    if len(pts) < 2:
        return {'error': f'only {len(pts)} usable run(s); need at least 2'}

    v = np.array([p[0] for p in pts])
    d = np.array([p[1] for p in pts])
    speeds = sorted({round(x, 3) for x in v})
    spread = max(speeds) / min(speeds) if min(speeds) > 0 else 1.0

    t_hat, b_hat = _lstsq(v, d)
    res = d - (t_hat * v + b_hat * v ** 2)

    out = {
        'n_runs': len(pts),
        'distinct_speeds': speeds,
        'speed_spread': spread,
        'T_stop_s': t_hat,
        'residual_rms_m': float(np.sqrt(np.mean(res ** 2))),
        'residual_max_m': float(np.max(np.abs(res))),
        'identifiable': len(speeds) >= MIN_SPEEDS and spread >= MIN_SPREAD,
    }
    out['decel_m_s2'] = (1.0 / (2.0 * b_hat)) if b_hat > 0 else None

    rng = np.random.default_rng(seed)
    boot = []
    for _ in range(n_boot):
        idx = rng.integers(0, len(v), len(v))
        if len({round(x, 3) for x in v[idx]}) < 2:
            continue
        try:
            boot.append(_lstsq(v[idx], d[idx]))
        except Exception:
            continue
    if boot:
        bt = np.array([x[0] for x in boot])
        bb = np.array([x[1] for x in boot])
        out['T_stop_ci'] = [float(np.percentile(bt, 5)), float(np.percentile(bt, 95))]
        good = bb > 0
        if good.sum() > 10:
            a_s = 1.0 / (2.0 * bb[good])
            out['decel_ci'] = [float(np.percentile(a_s, 5)), float(np.percentile(a_s, 95))]
            out['decel_ci_frac_positive'] = float(good.mean())
        out['_boot'] = (bt, bb)

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
    """Conservative predicted stopping distance: bootstrap upper bound + margin."""
    import numpy as np
    if '_boot' not in f:
        return None
    bt, bb = f['_boot']
    preds = bt * speed + bb * speed ** 2
    return float(np.percentile(preds, pct)) + MARGIN_C


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

    if not f['identifiable']:
        say('')
        say(f'  *** `a` IS NOT TRUSTWORTHY FROM THIS DATA ***')
        if len(f['distinct_speeds']) < MIN_SPEEDS:
            say(f'      {len(f["distinct_speeds"])} distinct speed(s); '
                f'{MIN_SPEEDS} needed to separate T_stop from a.')
        if f['speed_spread'] < MIN_SPREAD:
            say(f'      speed spread {f["speed_spread"]:.1f}x is below {MIN_SPREAD}x; '
                'the quadratic term is too small to identify.')

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
    say('  CONSERVATIVE STOPPING ENVELOPE (95th pct of bootstrap + '
        f'{MARGIN_C*1000:.0f} mm margin):')
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
    args = ap.parse_args()

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
            store['odom_scale_k'] = cals[args.use_calibration]['k']
            save(store)
            print(f'\nactive k set to {store["odom_scale_k"]:.4f} '
                  f'(calibration {args.use_calibration})')
        return 0

    if args.fit:
        print('=== fit over recorded runs ===')
        print(f'odometry scale k = {store.get("odom_scale_k")}')
        report(fit(store['runs']), print)
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
    imu = []          # (t, accel_x, gyro_z)
    node.create_subscription(
        Imu, '/imu',
        lambda m: imu.append((time.time(), m.linear_acceleration.x,
                              m.angular_velocity.z)),
        qos_profile_sensor_data)
    topic = '/cmd_vel' if args.direct else '/cmd_vel_raw'
    pub = node.create_publisher(Twist, topic, 10)

    ex = SingleThreadedExecutor()
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()

    stamp = datetime.now().strftime('%Y%m%d-%H%M%S')
    time.sleep(2.0)
    if not samples:
        print('FAIL: no /odom_raw. Car powered? Right domain? Agent up?')
        return 2

    def imu_bias(t_from, t_to):
        """Mean accel-x and gyro-z while stationary: removes static tilt and gyro drift."""
        w = [(a, g) for t, a, g in imu if t_from <= t <= t_to]
        if not w:
            return None, None
        return (sum(x[0] for x in w) / len(w), sum(x[1] for x in w) / len(w))

    def imu_integrate(t_from, t_to, ab, gb):
        """-> (delta_v, distance, delta_yaw) from IMU over a window, biases removed.

        Trapezoidal. /imu runs at 25 Hz, so a 0.3 s event is only ~7 samples and the
        integration itself is coarse; this is a cross-check, not a precision instrument.
        """
        w = [(t, a - ab, g - gb) for t, a, g in imu if t_from <= t <= (t_to or 1e18)]
        if len(w) < 2:
            return None, None, None
        dv = d = dyaw = 0.0
        v = 0.0
        for (t0, a0, g0), (t1, a1, g1) in zip(w, w[1:]):
            dt = t1 - t0
            dv += 0.5 * (a0 + a1) * dt
            d += abs(v) * dt + 0.5 * abs(0.5 * (a0 + a1)) * dt * dt
            v += 0.5 * (a0 + a1) * dt
            dyaw += 0.5 * (g0 + g1) * dt
        return dv, d, dyaw

    def integrate(t_from, t_to=None):
        seg = [s for s in samples if s[0] >= t_from and (t_to is None or s[0] <= t_to)]
        return sum(abs(a[1]) * (b[0] - a[0]) for a, b in zip(seg, seg[1:]))

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
        dyaw = None
        if gb is not None:
            _, _, dyaw = imu_integrate(t0, t_end, ab or 0.0, gb)
        if dyaw is None:
            say('  straightness: NO IMU DATA -- cannot tell whether the push was straight')
        else:
            infl = (dyaw ** 2) / 24.0
            say(f'  straightness: yaw changed {math.degrees(dyaw):+.1f} deg over the push')
            say(f'    implied arc-vs-chord inflation of odometry: {infl*100:.2f}%')
            if abs(math.degrees(dyaw)) > 5.0:
                say('    *** the push CURVED. k is biased low; redo it against a '
                    'straight edge. ***')
        if odo < 1.0:
            say(f'  *** SHORT PUSH ({odo*1000:.0f} mm). A fixed +/-10 mm tape error is '
                f'{10.0/(odo*1000)*100:.1f}% here. Push at least 1.5 m. ***')
        if not 0.5 < k < 2.0:
            say(f'  REFUSED: k={k:.3f} is implausible. Check the push was straight and')
            say('  that the tape figure is in millimetres.')
            return 2
        store.setdefault('calibrations', []).append(
            {'timestamp': stamp, 'tape_m': tape, 'odom_m': odo, 'k': k,
             'yaw_change_rad': dyaw,
             'arc_chord_inflation': ((dyaw ** 2) / 24.0) if dyaw is not None else None})
        store['odom_scale_k'] = k
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
            try:
                input('  Car at the START mark, then press Enter... ')
            except EOFError:
                pass

            # Stationary window first: removes static tilt from accel-x and gyro drift.
            t_bias0 = time.time()
            time.sleep(1.5)
            ab, gb = imu_bias(t_bias0, time.time())

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
                dv_imu, d_imu, _ = imu_integrate(t_zero, t_rest, ab, gb)
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

    say('')
    say(f'=== fit over all {len(store["runs"])} recorded runs ===')
    report(fit(store['runs']), say)

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
