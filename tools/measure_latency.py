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
The command-latency test times command -> motion STARTS, which includes motor spin-up.
Safety depends on command -> motion STOPS, i.e. the braking response. These are not the
same quantity. Spin-up must overcome static friction and rotor inertia, so using it as a
proxy is probably conservative, but that is an argument, not a measurement. The braking
response is measured in Phase 2 alongside deceleration, on the floor.

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
    args = ap.parse_args()

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

            thresh = args.speed * 0.25
            samples = [(ts, vx) for ts, vx, _ in odom if ts >= t_cmd - 0.4]
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
            }
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
        say(f'  T (p95, conservative)  {T*1000:6.1f} ms')
        say(f'  reaction distance at 0.30 m/s: {0.30*T*1000:.0f} mm')
        say(f'  reaction distance at 0.15 m/s: {0.15*T*1000:.0f} mm')
        results['T_p95_s'] = T
        results['reaction_distance_at_0.3_m'] = 0.30 * T
        say('')
        say('  Reaction DOMINATES braking for this robot. At 0.30 m/s the braking term')
        say('  v^2/(2a) is 45 mm even for a pessimistic a=1.0 m/s^2, and 11 mm at a=4.0,')
        say('  against 133 mm of reaction. A 4x error in `a` moves the total by ~34 mm.')
        say('  Consequence: the safety distance can be sized NOW from measured latency')
        say('  plus a conservative `a`; the floor measurement confirms rather than gates.')
        say('')
        say('  CAVEAT: this timed command -> motion STARTS (includes spin-up). Braking')
        say('  response is a different quantity and is measured in Phase 2.')
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
    sys.exit(main())
