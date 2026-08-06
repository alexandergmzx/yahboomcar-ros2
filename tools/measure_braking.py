#!/usr/bin/env python3
"""Measure how far the car actually travels after being told to stop.

    ./tools/measure_braking.py --speed 0.05 --runs 5
    ./tools/measure_braking.py --fit            # fit the model over all recorded runs

READ docs/first-floor-procedure.md FIRST. This is a FLOOR test, and the first one the
car has had. Clear area, soft perimeter, hand on the power switch.

WHAT IT MEASURES, AND WHY IT IS TWO THINGS
------------------------------------------
Distance from "zero commanded" to "at rest" is not one quantity. It is

    d(v) = v * T_stop  +  v^2 / (2a)

  T_stop  the dead time before deceleration begins -- comms, firmware, and the motors
          actually starting to fight the load. Contributes LINEARLY in v.
  a       the deceleration once braking. Contributes QUADRATICALLY.

Measuring at a single speed cannot separate them: one number, two unknowns, and any
(T_stop, a) pair on a curve fits it. Measuring at several speeds and fitting both terms
does separate them, because they scale differently.

That is why this runs at 0.05 then 0.10 m/s rather than one convenient speed, and it is
also why the floor test is worth more than the elevated one it replaces: T_stop measured
under real load is the number that transfers, and the earlier bench figures explicitly
do not (see docs/safety-case.md).

TAPE IS THE WITNESS, NOT ODOMETRY
---------------------------------
/odom_raw over-reports by roughly 8% against the gyro, and it is encoder-derived, so
during a skid it reports wheel rotation rather than ground travel -- exactly wrong in the
regime being measured. The fit uses the TAPE measurement. Odometry is recorded alongside
purely so the two can be compared, and a growing disagreement is itself a finding.
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


def load_runs():
    if os.path.exists(DATA):
        with open(DATA) as f:
            return json.load(f)
    return {'runs': []}


def fit(runs):
    """Least-squares fit of d = T_stop*v + (1/2a)*v^2 to the tape measurements.

    No constant term. A non-zero intercept would mean the car travels some distance at
    zero speed, which is not a thing; the design margin C in the governor is a separate
    choice and does not belong in a fit to physics.
    """
    import numpy as np
    pts = [(r['speed_m_s'], r['tape_m']) for r in runs if r.get('tape_m') is not None]
    if len(pts) < 2:
        return None
    speeds = sorted({v for v, _ in pts})
    if len(speeds) < 2:
        return {'error': f'all {len(pts)} runs are at {speeds[0]} m/s. '
                         'T_stop and a cannot be separated from a single speed.'}
    v = np.array([p[0] for p in pts])
    d = np.array([p[1] for p in pts])
    A = np.vstack([v, v ** 2]).T
    (t_stop, half_inv_a), *_ = np.linalg.lstsq(A, d, rcond=None)
    if half_inv_a <= 0:
        return {'error': 'fitted quadratic term is non-positive; a is not identifiable '
                         'from this data. More speeds or more runs needed.',
                'T_stop_s': float(t_stop)}
    a = 1.0 / (2.0 * half_inv_a)
    pred = A @ np.array([t_stop, half_inv_a])
    resid = d - pred
    return {
        'T_stop_s': float(t_stop),
        'decel_m_s2': float(a),
        'n_runs': len(pts),
        'speeds': speeds,
        'residual_max_m': float(np.max(np.abs(resid))),
        'residual_rms_m': float(np.sqrt(np.mean(resid ** 2))),
    }


def report_fit(f, say):
    if f is None:
        say('  not enough runs yet (need >= 2 with tape measurements)')
        return
    if 'error' in f:
        say(f'  {f["error"]}')
        return
    say(f'  T_stop = {f["T_stop_s"]*1000:.0f} ms      (dead time before braking)')
    say(f'  a      = {f["decel_m_s2"]:.2f} m/s^2   (deceleration once braking)')
    say(f'  from {f["n_runs"]} runs at {f["speeds"]} m/s')
    say(f'  residuals: rms {f["residual_rms_m"]*1000:.0f} mm, '
        f'max {f["residual_max_m"]*1000:.0f} mm')
    say('')
    say('  predicted stopping distance d = v*T_stop + v^2/(2a):')
    for v in (0.05, 0.10, 0.20, 0.30):
        d = v * f['T_stop_s'] + v * v / (2 * f['decel_m_s2'])
        say(f'    {v:.2f} m/s -> {d*1000:5.0f} mm')
    say('')
    say('  The 0.20 and 0.30 rows are EXTRAPOLATIONS until runs exist at those speeds.')
    say('  Raising the cap is gated on the prediction holding, not on the car surviving.')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--speed', type=float, default=0.05)
    ap.add_argument('--runs', type=int, default=5)
    ap.add_argument('--hold', type=float, default=3.0,
                    help='seconds at speed before commanding the stop')
    ap.add_argument('--max-speed', type=float, default=0.10,
                    help='refuse anything faster without --i-accept-the-risk')
    ap.add_argument('--i-accept-the-risk', action='store_true')
    ap.add_argument('--direct', action='store_true',
                    help='publish /cmd_vel, bypassing the governor. NOT for floor use.')
    ap.add_argument('--domain', type=int, default=20)
    ap.add_argument('--fit', action='store_true',
                    help='fit the model over recorded runs and exit; nothing moves')
    args = ap.parse_args()

    store = load_runs()

    if args.fit:
        print('=== fit over all recorded runs ===')
        report_fit(fit(store['runs']), print)
        return 0

    if args.speed > args.max_speed and not args.i_accept_the_risk:
        print(f'REFUSED: {args.speed} m/s exceeds --max-speed {args.max_speed}.')
        print('The staging table in docs/first-floor-procedure.md exists because a')
        print('model fitted at low speed has to PREDICT the next step before you drive')
        print('it. Pass --i-accept-the-risk if you have read that and still mean it.')
        return 2

    if os.environ.get('ROS_DOMAIN_ID') is None:
        os.environ['ROS_DOMAIN_ID'] = str(args.domain)

    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry

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
    topic = '/cmd_vel' if args.direct else '/cmd_vel_raw'
    pub = node.create_publisher(Twist, topic, 10)

    ex = SingleThreadedExecutor()
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()

    stamp = datetime.now().strftime('%Y%m%d-%H%M%S')
    say(f'braking measurement  {stamp}')
    say(f'publishing to {topic}' + ('  *** GOVERNOR BYPASSED ***' if args.direct else ''))
    say(f'{args.runs} runs at {args.speed} m/s')
    say('')
    say('FLOOR TEST. Hand on the power switch. Ctrl+C aborts, but the switch is faster.')

    time.sleep(2.0)
    if not samples:
        say('FAIL: no /odom_raw. Car powered? Right domain? Agent up?')
        return 2

    def hard_stop():
        for _ in range(15):
            pub.publish(Twist())
            time.sleep(0.04)

    new_runs = []
    try:
        for i in range(args.runs):
            say('')
            say(f'--- run {i+1}/{args.runs} ---')
            try:
                input('  Place the car at the start mark, then press Enter... ')
            except EOFError:
                pass

            t = Twist()
            t.linear.x = float(args.speed)
            end = time.time() + args.hold
            while time.time() < end:
                pub.publish(t)
                time.sleep(0.05)

            # Command zero and STOP PUBLISHING. That is the event being timed, and it is
            # also the realistic case: whatever was driving has said "stop".
            t_zero = time.time()
            for _ in range(3):
                pub.publish(Twist())
                time.sleep(0.02)
            say('  ZERO commanded -- mark where the car comes to rest')
            time.sleep(4.0)

            after = [s for s in samples if s[0] >= t_zero]
            dist = 0.0
            rest_t = None
            for (ta, va), (tb, _vb) in zip(after, after[1:]):
                dist += abs(va) * (tb - ta)
            for j in range(len(after) - N_REST + 1):
                if all(abs(v) <= AT_REST for _, v in after[j:j + N_REST]):
                    rest_t = after[j][0] - t_zero
                    break
            odom_m = dist
            say(f'  odometry: {odom_m*1000:.0f} mm' +
                (f', at rest after {rest_t*1000:.0f} ms' if rest_t else
                 ', NEVER REACHED REST -- abort and check the deadman'))

            tape = None
            try:
                raw = input('  tape-measured distance in mm (blank to discard run): ')
                if raw.strip():
                    tape = float(raw.strip()) / 1000.0
            except (EOFError, ValueError):
                pass
            if tape is None:
                say('  run discarded (no tape measurement)')
                continue
            if odom_m > 0:
                say(f'  odometry/tape = {odom_m/tape:.2f}')
            new_runs.append({
                'timestamp': stamp, 'speed_m_s': args.speed,
                'tape_m': tape, 'odom_m': odom_m, 'time_to_rest_s': rest_t,
                'governed': not args.direct,
            })
            hard_stop()
    except KeyboardInterrupt:
        say('')
        say('  aborted by operator')
    finally:
        hard_stop()

    store['runs'].extend(new_runs)
    with open(DATA, 'w') as f:
        json.dump(store, f, indent=2)

    say('')
    say(f'=== fit over all {len(store["runs"])} recorded runs ===')
    report_fit(fit(store['runs']), say)

    os.makedirs(OUT_DIR, exist_ok=True)
    log_path = os.path.join(OUT_DIR, f'braking-{stamp}.log')
    with open(log_path, 'w') as f:
        f.write('\n'.join(lines) + '\n')
    say('')
    say(f'log:  {log_path}')
    say(f'runs: {DATA}')
    return 0


if __name__ == '__main__':
    code = main()
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(code)
