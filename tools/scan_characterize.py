#!/usr/bin/env python3
"""Characterize /scan (and the other contract topics) passively. Moves nothing.

    ./tools/scan_characterize.py --seconds 150            # bench characterization
    ./tools/scan_characterize.py --seconds 60 --label udp # transport A/B leg

Measures, over the window:
  * mean rate and jitter (host inter-arrival: mean, std, p95, max) for /scan, /imu,
    /odom_raw, /battery -- host-side arrival is the only clock-independent number
    (firmware and host clocks are not synchronised; see measure_latency.py).
  * /scan geometry: points per revolution, angular span, range limits as published.
  * valid-return fraction, overall and PER ANGLE -- the per-angle map is the mount's
    self-hit/occlusion fingerprint: a sector that never returns on the bench will
    never return on the floor either.
  * min/max PLAUSIBLE ranges actually seen (finite returns inside [range_min,
    range_max]).
  * offered QoS of each publisher, from the graph.
  * a RELIABLE-subscriber probe on /scan: the documented trap is that a RELIABLE
    subscriber silently receives nothing; whether that is still true is a fact about
    the current agent, so it is measured, not assumed.

RANGE ACCURACY IS NOT MEASURED. That needs a placed reference target and there is
none; nothing here should be quoted as accuracy evidence.
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

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _layout import LOG_DIR                                     # noqa: E402


def stats(xs):
    if len(xs) < 2:
        return None
    xs = sorted(xs)
    return {'mean': statistics.mean(xs), 'std': statistics.pstdev(xs),
            'p95': xs[int(0.95 * (len(xs) - 1))], 'max': xs[-1], 'n': len(xs)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--seconds', type=float, default=150.0)
    ap.add_argument('--domain', type=int, default=20)
    ap.add_argument('--label', default='default',
                    help='tag for the output file, e.g. the transport profile')
    ap.add_argument('--sector-deg', type=float, default=10.0,
                    help='bin width for the angular coverage map')
    args = ap.parse_args()
    os.environ.setdefault('ROS_DOMAIN_ID', str(args.domain))

    import rclpy
    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import LaserScan, Imu
    from nav_msgs.msg import Odometry
    from std_msgs.msg import UInt16

    lines = []

    def say(m=''):
        print(m, flush=True)
        lines.append(str(m))

    stamp = datetime.now().strftime('%Y%m%d-%H%M%S')
    say(f'scan_characterize {stamp}  domain {os.environ["ROS_DOMAIN_ID"]}  '
        f'window {args.seconds:.0f} s  label={args.label}')
    say(f'  transport profile file: '
        f'{os.environ.get("FASTDDS_DEFAULT_PROFILES_FILE", "(none: defaults)")}')

    rclpy.init()
    node = Node('scan_characterize')

    arrivals = {'/scan': [], '/imu': [], '/odom_raw': [], '/battery': []}
    scans = []                    # every LaserScan message, whole
    reliable_scan_count = [0]

    def on(topic):
        return lambda m: arrivals[topic].append(time.time())

    def on_scan(m):
        arrivals['/scan'].append(time.time())
        scans.append(m)

    node.create_subscription(LaserScan, '/scan', on_scan, qos_profile_sensor_data)
    node.create_subscription(Imu, '/imu', on('/imu'), qos_profile_sensor_data)
    node.create_subscription(Odometry, '/odom_raw', on('/odom_raw'),
                             qos_profile_sensor_data)
    node.create_subscription(UInt16, '/battery', on('/battery'),
                             qos_profile_sensor_data)
    # The documented trap, measured rather than believed:
    node.create_subscription(
        LaserScan, '/scan',
        lambda m: reliable_scan_count.__setitem__(0, reliable_scan_count[0] + 1),
        QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE))

    ex = SingleThreadedExecutor()
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()

    t0 = time.time()
    while time.time() - t0 < args.seconds:
        time.sleep(2.0)
        el = time.time() - t0
        print(f'  {el:5.0f}/{args.seconds:.0f} s   scan {len(scans)}  '
              f'imu {len(arrivals["/imu"])}  odom {len(arrivals["/odom_raw"])}',
              end='\r', flush=True)
    window = time.time() - t0
    print()

    # offered QoS from the graph
    say('')
    say('offered publisher QoS (from the graph):')
    for t, typ in (('/scan', LaserScan), ('/imu', Imu), ('/odom_raw', Odometry),
                   ('/battery', UInt16)):
        infos = node.get_publishers_info_by_topic(t)
        for i in infos:
            q = i.qos_profile
            say(f'  {t:10s} {i.node_name}: reliability={q.reliability.name} '
                f'durability={q.durability.name} depth={q.depth}')

    res = {'stamp': stamp, 'label': args.label, 'window_s': window,
           'profile_file': os.environ.get('FASTDDS_DEFAULT_PROFILES_FILE'),
           'topics': {}}
    say('')
    say(f'rates over {window:.1f} s (host inter-arrival):')
    for t, want in (('/scan', 12.0), ('/imu', 25.0), ('/odom_raw', 11.0),
                    ('/battery', 1.0)):
        ts = arrivals[t]
        rate = (len(ts) - 1) / (ts[-1] - ts[0]) if len(ts) > 2 else 0.0
        gaps = [b - a for a, b in zip(ts, ts[1:])]
        g = stats([x * 1000 for x in gaps])
        res['topics'][t] = {'n': len(ts), 'rate_hz': rate, 'contract_hz': want,
                            'gap_ms': g}
        if g:
            say(f'  {t:10s} {rate:6.2f} Hz (contract {want:.0f})   gap ms: '
                f'mean {g["mean"]:6.1f} std {g["std"]:6.1f} p95 {g["p95"]:6.1f} '
                f'max {g["max"]:7.1f}   n={len(ts)}')
        else:
            say(f'  {t:10s} NO DATA (n={len(ts)})')

    say('')
    say(f'RELIABLE /scan subscriber received: {reliable_scan_count[0]} msgs '
        f'(best-effort one: {len(scans)})')
    res['reliable_scan_msgs'] = reliable_scan_count[0]

    if scans:
        s0 = scans[0]
        n_pts = len(s0.ranges)
        span = math.degrees(s0.angle_max - s0.angle_min)
        say('')
        say(f'/scan geometry: {n_pts} points, span {span:.1f} deg, '
            f'increment {math.degrees(s0.angle_increment):.3f} deg, '
            f'published range [{s0.range_min:.2f}, {s0.range_max:.2f}] m, '
            f'frame {s0.header.frame_id}')
        valid_total = 0
        pts_total = 0
        nbins = max(1, int(round(360.0 / args.sector_deg)))
        bin_valid = [0] * nbins
        bin_count = [0] * nbins
        rmin, rmax = math.inf, 0.0
        for s in scans:
            for i, r in enumerate(s.ranges):
                ang = math.degrees(s.angle_min + i * s.angle_increment) % 360.0
                b = min(nbins - 1, int(ang / args.sector_deg))
                bin_count[b] += 1
                pts_total += 1
                if math.isfinite(r) and s.range_min <= r <= s.range_max:
                    bin_valid[b] += 1
                    valid_total += 1
                    rmin = min(rmin, r)
                    rmax = max(rmax, r)
        say(f'valid-return fraction: {valid_total/pts_total*100:.1f} % over '
            f'{len(scans)} revolutions ({pts_total} points)')
        say(f'plausible ranges actually seen: [{rmin:.3f}, {rmax:.3f}] m '
            '(NOT accuracy -- no reference target placed)')
        say('')
        say(f'angular coverage map ({args.sector_deg:.0f} deg bins, 0 deg = '
            f'angle_min; valid %):')
        row = []
        occluded = []
        for b in range(nbins):
            frac = bin_valid[b] / max(1, bin_count[b]) * 100
            row.append(f'{frac:3.0f}')
            if frac < 50.0:
                occluded.append((b * args.sector_deg, frac))
        for i in range(0, nbins, 12):
            angs = f'{i*args.sector_deg:3.0f}-{min(360, (i+12)*args.sector_deg):3.0f}'
            say(f'  {angs:9s}: ' + ' '.join(row[i:i + 12]))
        if occluded:
            say('  sectors under 50% valid (persistent self-hit/occlusion on this '
                'mount):')
            for a, f in occluded:
                say(f'    {a:5.0f}-{a+args.sector_deg:.0f} deg: {f:.0f} %')
        else:
            say('  no sector below 50% valid')
        res['scan'] = {'points_per_rev': n_pts, 'span_deg': span,
                       'range_limits_published': [s0.range_min, s0.range_max],
                       'valid_fraction': valid_total / pts_total,
                       'range_seen': [rmin, rmax],
                       'revolutions': len(scans),
                       'coverage_bins_deg': args.sector_deg,
                       'coverage_valid_pct': [
                           bin_valid[b] / max(1, bin_count[b]) * 100
                           for b in range(nbins)]}
    else:
        say('NO SCANS RECEIVED -- nothing to characterize')

    os.makedirs(LOG_DIR, exist_ok=True)
    base = os.path.join(LOG_DIR, f'scan-char-{args.label}-{stamp}')
    with open(base + '.log', 'w') as f:
        f.write('\n'.join(lines) + '\n')
    with open(base + '.json', 'w') as f:
        json.dump(res, f, indent=2)
    say('')
    say(f'log: {base}.log')
    return 0 if scans else 1


if __name__ == '__main__':
    code = main()
    sys.stdout.flush()
    os._exit(code)
