#!/usr/bin/env python3
"""Measure the system response time T that the safety distance depends on.

    ./tools/measure_latency.py                 # all three measurements
    ./tools/measure_latency.py --repeats 40
    ./tools/measure_latency.py --no-motion     # scan timing only, nothing moves

SAFETY: the command-latency test spins the wheels in short bursts. Run it with the car
ELEVATED. It publishes to /cmd_vel directly, bypassing the governor, because the point
is to measure the uninhibited command path -- and because an obstacle in front would
otherwise make the governor suppress the very step we are timing.

WHY THIS EXISTS
---------------
The governor's stop distance was a round number I chose. The standards give the shape
instead (ISO 13855: S = K*T + C; ISO 3691-4 requires braking distance on top), and for
this robot the reaction term v*T dominates: braking from 0.3 m/s is a couple of
centimetres, while a single scan interval at the rates we have observed (4.1-12.8 Hz)
is 78-243 ms, which at 0.3 m/s is 23-73 mm of travel before anything is even known.

WHAT THIS CAN AND CANNOT RESOLVE
--------------------------------
/odom_raw arrives at about 11 Hz, so a step response cannot be located more precisely
than one inter-message interval, roughly +/-45 ms. Rather than pretend to a point
estimate, each trial is reported as a BRACKET: the last sample still at rest gives a
lower bound, the first sample showing motion an upper bound. Safety numbers should be
taken from the upper bound and the p95, never the mean.

IMPORTANT CAVEAT ON WHAT IS MEASURED
------------------------------------
Run ELEVATED, this measurement is a LOWER BOUND on the on-floor latency, not a
conservative one. An earlier version of this file claimed the opposite; that was wrong.

Unloaded wheels spin up almost instantly, and /odom_raw is encoder-derived, so it
registers motion as soon as the wheels turn. On the floor the motors must first overcome
static friction, rolling resistance and the chassis inertia before the wheels move at
all, so command -> motion takes LONGER there. Sizing a safety distance from the elevated
figure therefore under-sizes it.

Two consequences:
  * T measured here = comms + firmware + (near-zero) spin-up. On the floor the spin-up
    term grows by an unknown amount, so T_floor > T_elevated.
  * What safety actually depends on is command -> motion STOPS (braking response), which
    is a third quantity again, and also load-dependent.

Both must be re-measured on the floor. --thresholds helps separate the load-independent
part: latency to reach a detection threshold is comms + time-to-reach-that-threshold, so
sweeping the threshold and extrapolating toward zero isolates the comms/firmware delay,
which does transfer between elevated and floor.

Firmware and host clocks are not synchronised, so `now - header.stamp` is not a
trustworthy age. Scan timing is therefore measured by host-side inter-arrival, which is
clock-independent; the header-derived figure is reported alongside only so the two can
be compared.
"""
import argparse
import json
import math
import os
import statistics
import sys
import threading
import time
from datetime import datetime

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT_DIR = os.path.join(REPO, 'MicroROS-assets', 'logs')
PARAM_OUT = os.path.join(REPO, 'yahboomcar_ws', 'src', 'yahboomcar_safety',
                         'measured_params.json')


def pct(values, p):
    if not values:
        return float('nan')
    s = sorted(values)
    k = min(len(s) - 1, max(0, int(round((p / 100.0) * (len(s) - 1)))))
    return s[k]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--repeats', type=int, default=25)
    ap.add_argument('--speed', type=float, default=0.15,
                    help='step magnitude for the command-latency test, m/s')
    ap.add_argument('--domain', type=int, default=20)
    ap.add_argument('--no-motion', action='store_true',
                    help='skip the command-latency test; nothing moves')
    ap.add_argument('--scan-seconds', type=float, default=20.0)
    ap.add_argument('--thresholds', default='0.10,0.25,0.50,0.75',
                    help='detection thresholds as fractions of --speed. Sweeping these '
                         'separates the load-independent comms/firmware delay from the '
                         'load-dependent spin-up: extrapolating toward zero threshold '
                         'removes the spin-up term.')
    args = ap.parse_args()
    fracs = sorted(float(x) for x in args.thresholds.split(','))

    if os.environ.get('ROS_DOMAIN_ID') is None:
        os.environ['ROS_DOMAIN_ID'] = str(args.domain)

    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import LaserScan

    lines = []

    def say(m=''):
        print(m, flush=True)
        lines.append(str(m))

    rclpy.init()
    node = Node('measure_latency')

    scan_rx = []          # host receive times
    scan_hdr_age = []     # now - header.stamp, for comparison only
    odom = []             # (host_time, vx, wz)

    def on_scan(msg):
        t = time.time()
        scan_rx.append(t)
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if stamp > 0:
            scan_hdr_age.append(t - stamp)

    def on_odom(msg):
        odom.append((time.time(), msg.twist.twist.linear.x, msg.twist.twist.angular.z))

    node.create_subscription(LaserScan, '/scan', on_scan, qos_profile_sensor_data)
    node.create_subscription(Odometry, '/odom_raw', on_odom, qos_profile_sensor_data)
    cmd = node.create_publisher(Twist, '/cmd_vel', 10)

    # SingleThreadedExecutor, deliberately. A MultiThreadedExecutor runs callbacks on
    # parallel threads, so receive timestamps get appended OUT OF ORDER and the gaps
    # computed from them are fiction -- the first run of this tool reported a 0.6 ms
    # median and a 2.67 s maximum while `ros2 topic hz` independently measured a steady
    # 40-184 ms. Ordering matters more than throughput here; ~23 msg/s is trivial for
    # one thread.
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()

    stamp_name = datetime.now().strftime('%Y%m%d-%H%M%S')
    say(f'latency measurement  {stamp_name}')
    say(f'ROS_DOMAIN_ID={os.environ.get("ROS_DOMAIN_ID")}')
    say('')

    # ---------------------------------------------------------------- scan timing
    say(f'=== scan timing ({args.scan_seconds:.0f}s) ===')
    scan_rx.clear()
    scan_hdr_age.clear()
    time.sleep(args.scan_seconds)
    if len(scan_rx) < 5:
        say(f'FAIL: only {len(scan_rx)} scans received. Is the car on this domain?')
        node.destroy_node()
        rclpy.shutdown()
        return 1

    scan_rx.sort()          # defensive; ordering should already hold single-threaded
    gaps = [b - a for a, b in zip(scan_rx, scan_rx[1:])]
    rate = len(gaps) / (scan_rx[-1] - scan_rx[0])
    say(f'  scans: {len(scan_rx)}   mean rate {rate:.2f} Hz')
    say(f'  inter-arrival  p50 {pct(gaps,50)*1000:6.1f} ms   '
        f'p95 {pct(gaps,95)*1000:6.1f} ms   max {max(gaps)*1000:6.1f} ms')
    say('  (p95 is the honest input to T: a distance sized on the mean is wrong')
    say('   half the time, and this link has been observed between 4 and 13 Hz)')
    if scan_hdr_age:
        say(f'  header-derived age p50 {pct(scan_hdr_age,50)*1000:.0f} ms '
            f'-- NOT trustworthy, firmware and host clocks are unsynchronised')

    results = {
        'timestamp': stamp_name,
        'scan': {'n': len(scan_rx), 'rate_hz': rate,
                 'interarrival_p50_s': pct(gaps, 50),
                 'interarrival_p95_s': pct(gaps, 95),
                 'interarrival_max_s': max(gaps)},
    }

    # ------------------------------------------------------- command latency
    if not args.no_motion:
        say('')
        say(f'=== command latency: {args.repeats} steps at {args.speed} m/s ===')
        say('    (car must be ELEVATED -- the wheels will spin)')
        brackets = []
        trial_samples = []        # per trial: [(dt_since_cmd, vx), ...] for the sweep
        for i in range(args.repeats):
            # settle at rest
            t = Twist()
            for _ in range(6):
                cmd.publish(t)
                time.sleep(0.05)
            time.sleep(0.5)
            odom.clear()
            time.sleep(0.35)          # collect a few at-rest samples

            step = Twist()
            step.linear.x = float(args.speed)
            t_cmd = time.time()
            deadline = t_cmd + 1.5
            while time.time() < deadline:
                cmd.publish(step)
                time.sleep(0.02)
            # stop
            for _ in range(6):
                cmd.publish(Twist())
                time.sleep(0.03)

            samples = [(ts, vx) for ts, vx, _ in odom if ts >= t_cmd - 0.4]
            trial_samples.append([(ts - t_cmd, vx) for ts, vx in samples])

            thresh = args.speed * 0.25
            first = next((ts for ts, vx in samples if ts > t_cmd and abs(vx) > thresh),
                         None)
            if first is None:
                say(f'  trial {i+1:2d}: no motion detected -- skipped')
                continue
            prev = max([ts for ts, vx in samples
                        if ts < first and abs(vx) <= thresh] + [t_cmd])
            lo, hi = max(0.0, prev - t_cmd), first - t_cmd
            brackets.append((lo, hi))
            say(f'  trial {i+1:2d}: latency in [{lo*1000:6.1f}, {hi*1000:6.1f}] ms')

        for _ in range(10):
            cmd.publish(Twist())
            time.sleep(0.03)

        if brackets:
            los = [b[0] for b in brackets]
            his = [b[1] for b in brackets]
            say('')
            say(f'  n = {len(brackets)}')
            say(f'  lower bounds: p50 {pct(los,50)*1000:.0f} ms  p95 {pct(los,95)*1000:.0f} ms')
            say(f'  upper bounds: p50 {pct(his,50)*1000:.0f} ms  p95 {pct(his,95)*1000:.0f} ms')
            say(f'  worst observed upper bound: {max(his)*1000:.0f} ms')
            say('  Use the UPPER bound for safety sizing. The bracket width is set by the')
            say(f'  /odom_raw interval (~{1000/11:.0f} ms), not by uncertainty in the robot.')
            results['command_latency'] = {
                'n': len(brackets),
                'lower_p50_s': pct(los, 50), 'lower_p95_s': pct(los, 95),
                'upper_p50_s': pct(his, 50), 'upper_p95_s': pct(his, 95),
                'upper_max_s': max(his),
                'speed_m_s': args.speed,
                'conditions': 'ELEVATED (unloaded)',
            }

            # --- threshold sweep: separate comms/firmware from spin-up ---------
            # Latency to reach threshold f = t_comms + t_spinup(f). t_comms is load-
            # independent and transfers to the floor; t_spinup does not. If the upper
            # bound is flat across f, spin-up is below the /odom_raw sampling interval
            # and the whole figure is comms + sampling granularity -- which is a useful
            # negative result, not a failure.
            say('')
            say('  threshold sweep (isolating the load-independent part):')
            say('    frac   thresh      upper p95    n')
            sweep = []
            for f in fracs:
                th = args.speed * f
                ups = []
                for s in trial_samples:
                    hit = next((dt for dt, vx in s if dt > 0 and abs(vx) > th), None)
                    if hit is not None:
                        ups.append(hit)
                if ups:
                    u = pct(ups, 95)
                    sweep.append((f, u))
                    say(f'    {f:4.2f}   {th:5.3f} m/s   {u*1000:6.1f} ms   {len(ups):3d}')
                else:
                    say(f'    {f:4.2f}   {th:5.3f} m/s   never reached')

            if len(sweep) >= 2:
                xs = [f for f, _ in sweep]
                ys = [u for _, u in sweep]
                # least-squares line, extrapolated to zero threshold
                mx, my = statistics.fmean(xs), statistics.fmean(ys)
                den = sum((x - mx) ** 2 for x in xs)
                slope = sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / den if den else 0.0
                intercept = my - slope * mx
                spread = max(ys) - min(ys)
                say('')
                say(f'    extrapolated to zero threshold: {intercept*1000:6.1f} ms')
                say(f'    slope {slope*1000:+.1f} ms per unit fraction; '
                    f'spread across thresholds {spread*1000:.1f} ms')
                results['command_latency']['threshold_sweep'] = {
                    'fractions': xs, 'upper_p95_s': ys,
                    'extrapolated_zero_threshold_s': intercept,
                    'spread_s': spread,
                }
                odom_interval = 1.0 / 11.0
                if spread < odom_interval:
                    say(f'    Spread is below the /odom_raw interval (~{odom_interval*1000:.0f} ms),')
                    say('    so spin-up is NOT resolvable here: elevated, the wheels reach every')
                    say('    threshold within one sample. The figure is comms + firmware +')
                    say('    sampling granularity. On the floor spin-up becomes real and adds to it.')
                else:
                    say('    Spread exceeds the sampling interval, so spin-up IS visible: the')
                    say('    intercept is the load-independent part that transfers to the floor.')
        else:
            say('  FAIL: no trial produced detectable motion')

    # ------------------------------------------------------------ total T
    say('')
    say('=== total system response time T ===')
    scan_p95 = results['scan']['interarrival_p95_s']
    cmd_p95 = results.get('command_latency', {}).get('upper_p95_s')
    gov = 1.0 / 20.0
    say(f'  scan interval p95      {scan_p95*1000:6.1f} ms   (detection granularity)')
    say(f'  governor loop          {gov*1000:6.1f} ms   (20 Hz)')
    if cmd_p95 is not None:
        say(f'  command -> motion p95  {cmd_p95*1000:6.1f} ms   (WiFi + agent + firmware)')
        T = scan_p95 + gov + cmd_p95
        say(f'  ------------------------------------')
        say(f'  T (p95, ELEVATED)      {T*1000:6.1f} ms   <-- LOWER BOUND, not conservative')
        say(f'  reaction distance at 0.30 m/s: {0.30*T*1000:.0f} mm')
        say(f'  reaction distance at 0.15 m/s: {0.15*T*1000:.0f} mm')
        results['T_p95_s'] = T
        results['T_conditions'] = 'ELEVATED -- lower bound on the on-floor value'
        results['reaction_distance_at_0.3_m'] = 0.30 * T
        say('')
        say('  DO NOT SIZE A SAFETY DISTANCE FROM THIS YET. Measured with the wheels off')
        say('  the ground, so they spin up unloaded and /odom_raw -- which is encoder-')
        say('  derived -- registers motion almost immediately. On the floor the motors')
        say('  must first overcome static friction, rolling resistance and the chassis')
        say('  inertia, so command -> motion is SLOWER there and T_floor > T_elevated.')
        say('')
        say('  Of the three terms, only the first two transfer to the floor:')
        say(f'    scan interval  {scan_p95*1000:6.1f} ms  transfers (sensor + link, no load)')
        say(f'    governor loop  {gov*1000:6.1f} ms  transfers (software)')
        say(f'    command path   {cmd_p95*1000:6.1f} ms  DOES NOT -- contains unloaded spin-up')
        say('')
        say('  Reaction still dominates braking: at 0.30 m/s the braking term v^2/(2a) is')
        say('  45 mm even at a pessimistic a=1.0 m/s^2 and 11 mm at a=4.0, against')
        say(f'  {0.30*T*1000:.0f} mm of reaction -- and the reaction term only grows on the floor.')
        say('  So `a` remains the minor term, but T must be re-measured under load before')
        say('  either is used.')
        say('')
        say('  Also note: this timed command -> motion STARTS. Safety depends on')
        say('  command -> motion STOPS, a third quantity, also load-dependent.')
    else:
        say('  command latency not measured; T incomplete')

    os.makedirs(OUT_DIR, exist_ok=True)
    log_path = os.path.join(OUT_DIR, f'latency-{stamp_name}.log')
    with open(log_path, 'w') as f:
        f.write('\n'.join(lines) + '\n')
    with open(PARAM_OUT, 'w') as f:
        json.dump(results, f, indent=2)
    say('')
    say(f'log:    {log_path}')
    say(f'params: {PARAM_OUT}')
    say('')
    say('NOTE: deceleration `a` is still unmeasured -- that needs floor space.')
    say('      Without it the braking term of d = v*T + v^2/(2a) + C cannot be computed.')

    # Tear the executor down before the node, or rclpy complains on the way out.
    try:
        ex.shutdown()
    except Exception:
        pass
    node.destroy_node()
    try:
        if rclpy.ok():
            rclpy.shutdown()
    except Exception:
        pass
    return 0


if __name__ == '__main__':
    code = main()
    # rclpy's teardown aborts with "terminate called without an active exception" on the
    # way out, AFTER every result is printed and both files are written. Same workaround
    # as tools/build_arena.py: leave before the destructor can turn a clean run into a
    # core dump and lose the exit status.
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(code)
