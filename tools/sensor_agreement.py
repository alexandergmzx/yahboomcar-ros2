#!/usr/bin/env python3
"""Ask every sensor how far the robot moved, then see whether they agree.

    ./tools/sensor_agreement.py MicroROS-assets/bags/twin_dataset
    ./tools/sensor_agreement.py <bag> --json out.json

Reads a bag. Moves nothing, needs no robot.

WHY
---
Deciding the odometry scale factor `k` meant asking which witness to believe, and the
answer kept being "the tape measure" -- because every estimate the robot makes of its own
motion comes from a wheel or from integrating its own opinion. This tool derives three
estimates that fail in different ways and puts them side by side:

  ENCODERS (/odom_raw)  wheel rotation. Cannot distinguish rotation from slip: on a stand
                        with the wheels in the air it reports a brisk cruise.
  IMU (/imu)            gyro yaw rate is trustworthy over a run; accelerometer needs
                        double integration, so linear DISPLACEMENT from it is only good
                        for fractions of a second (0.5*bias*t^2 -- ~400 mm over 4 s).
                        Reported for short windows only.
  LIDAR (/scan)         scan-to-scan ICP. The only estimate not derived from the robot's
                        own motion, so the only one that can bound drift rather than
                        accumulate it -- and the only one that can be degenerate, which
                        is reported per scan pair rather than hidden.

AGREEMENT IS THE POINT, NOT ANY ONE NUMBER
------------------------------------------
Where they agree, confidence is earned rather than assumed. Where they disagree, the
disagreement localises the fault: encoders reporting travel while the lidar and gyro
report none is wheel slip, and it is exactly what a dirty floor produces.
"""
import argparse
import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _layout import pkg_dir                                     # noqa: E402
sys.path.insert(0, pkg_dir('yahboomcar_localization'))


def read_bag(path, topics):
    """Yield (topic, stamp_seconds, message) in time order."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    # The bags here are mcap; sqlite3 is the rosbag2 default and fails on them with a
    # misleading "file is not a database".
    storage = 'mcap'
    if not any(f.endswith('.mcap') for f in os.listdir(path)):
        storage = 'sqlite3'
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=path, storage_id=storage),
                rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    want = {t for t in topics if t in types}
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        if topic in want:
            yield topic, stamp * 1e-9, deserialize_message(data, get_message(types[topic]))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag')
    ap.add_argument('--json', help='write the full result here')
    ap.add_argument('--max-scans', type=int, default=400,
                    help='cap ICP work on long bags')
    ap.add_argument('--imu-window', type=float, default=0.5,
                    help='seconds; accelerometer displacement is only meaningful over '
                         'short windows')
    args = ap.parse_args()

    import numpy as np
    from yahboomcar_localization.scan_geometry import scan_to_xy
    from yahboomcar_localization.scan_matcher import estimate_motion

    lines = []

    def say(m=''):
        print(m, flush=True)
        lines.append(str(m))

    if not os.path.isdir(args.bag):
        say(f'no such bag: {args.bag}')
        return 2

    say(f'=== sensor agreement: {os.path.basename(args.bag)} ===')

    odom, imu, scans = [], [], []
    for topic, t, m in read_bag(args.bag, ('/odom_raw', '/imu', '/scan')):
        if topic == '/odom_raw':
            odom.append((t, m.twist.twist.linear.x, m.twist.twist.angular.z))
        elif topic == '/imu':
            imu.append((t, m.linear_acceleration.x, m.angular_velocity.z))
        elif topic == '/scan':
            scans.append((t, m))

    say(f'  /odom_raw {len(odom):5d}   /imu {len(imu):5d}   /scan {len(scans):5d}')
    if not odom or not scans:
        say('  FAIL: need /odom_raw and /scan')
        return 2
    t0 = min(odom[0][0], scans[0][0])
    t1 = max(odom[-1][0], scans[-1][0])
    say(f'  duration {t1 - t0:.1f} s')
    say('')

    # ---------------------------------------------------------------- encoders
    enc_dist = sum(abs(a[1]) * (b[0] - a[0]) for a, b in zip(odom, odom[1:]))
    enc_yaw = sum(a[2] * (b[0] - a[0]) for a, b in zip(odom, odom[1:]))
    enc_peak = max(abs(v) for _, v, _ in odom)

    # ------------------------------------------------------- DEAD CHANNEL CHECK
    # A channel stuck at a constant value is not a quiet sensor, it is a broken one --
    # and it is the most dangerous input a filter can receive, because it agrees with
    # everything and never objects. Found the hard way: /imu angular_velocity.z in
    # twin_dataset is EXACTLY zero across all 1256 samples, std 0.000000. Read naively
    # that says "the body did not rotate"; it actually says the gyro reported nothing.
    dead = {}
    if imu:
        gz = np.array([g for _, _, g in imu])
        ax = np.array([a for _, a, _ in imu])
        dead['imu_gyro_z'] = float(np.std(gz)) < 1e-9
        dead['imu_accel_x'] = float(np.std(ax)) < 1e-9
        say(f'  channel check: gyro-z std {np.std(gz):.6f}, '
            f'accel-x std {np.std(ax):.6f} rad/s, m/s^2')
    ov = np.array([v for _, v, _ in odom])
    dead['odom_vx'] = float(np.std(ov)) < 1e-12
    for name, is_dead in dead.items():
        if is_dead:
            say(f'  *** {name} IS DEAD (zero variance). It is not evidence of anything,')
            say('      and a filter fusing it would treat a broken channel as a')
            say('      confident measurement. Excluded from the comparison below. ***')
    say('')

    # -------------------------------------------------------------------- IMU
    # Gyro bias from the quietest second: the interval whose |gyro| mean is smallest.
    gyro_bias = 0.0
    if imu:
        best, span = None, 1.0
        for i, (t, _, _) in enumerate(imu):
            w = [g for tt, _, g in imu if t <= tt < t + span]
            if len(w) > 5:
                m_ = abs(sum(w) / len(w))
                if best is None or m_ < best[0]:
                    best = (m_, sum(w) / len(w))
        gyro_bias = best[1] if best else 0.0
    imu_yaw = sum((a[2] - gyro_bias) * (b[0] - a[0]) for a, b in zip(imu, imu[1:]))

    # ------------------------------------------------------------------ lidar
    step = max(1, len(scans) // args.max_scans)
    used = scans[::step]
    say(f'  ICP over {len(used)} scans (every {step}), '
        f'{used[1][0] - used[0][0]:.3f} s apart')

    lid_dist = lid_yaw = 0.0
    ok = degen = 0
    isotropies = []
    pairs = []          # (gyro_dyaw, lidar_dyaw, turn_rate) per scan pair
    it = np.array([t for t, _, _ in imu]) if imu else np.zeros(0)
    ig = np.array([g for _, _, g in imu]) if imu else np.zeros(0)

    def gyro_between(t0, t1):
        if it.size < 2:
            return None
        sel = (it >= t0) & (it <= t1)
        return float(np.trapezoid(ig[sel] - gyro_bias, it[sel])) if sel.sum() > 1 else None

    prev_xy = None
    prev_t = None
    for t, m in used:
        xy = scan_to_xy(list(m.ranges), m.angle_min, m.angle_increment,
                        range_min=max(m.range_min, 0.05), range_max=m.range_max)
        if prev_xy is not None and len(xy) > 20:
            # estimate_motion, not match: match returns the SCENE transform, which is
            # the inverse of the robot's. See its docstring.
            r = estimate_motion(prev_xy, xy)
            if r.degeneracy:
                isotropies.append(r.degeneracy.isotropy)
                if r.degeneracy.degenerate:
                    degen += 1
            if r.converged:
                ok += 1
                lid_dist += math.hypot(r.dx, r.dy)
                lid_yaw += r.dtheta
                g = gyro_between(prev_t, t)
                if g is not None and t > prev_t:
                    pairs.append((g, r.dtheta, abs(g) / (t - prev_t)))
        prev_xy = xy
        prev_t = t

    say(f'  ICP converged on {ok}/{max(1, len(used) - 1)} pairs; '
        f'{degen} flagged degenerate')
    if isotropies:
        say(f'  geometry isotropy: median {np.median(isotropies):.3f} '
            f'(0 = featureless, 1 = fully constrained)')
    say('')

    # Per-pair regression against the gyro. This is the informative comparison, and the
    # summed totals below are NOT: most scan pairs in a typical run are nearly
    # stationary, and summing a per-pair estimate over hundreds of them accumulates ICP
    # noise as a random walk that can swamp the real signal. Measured on the desk run,
    # 464 of 503 pairs were under 0.05 rad/s: the sum implied a 25% lidar UNDER-read
    # while the regression over genuinely turning pairs implied the opposite.
    if pairs and not dead.get('imu_gyro_z'):
        gg = np.array([q[0] for q in pairs])
        ll = np.array([q[1] for q in pairs])
        rr = np.array([q[2] for q in pairs])
        mv = rr > 0.10
        say('  LIDAR vs GYRO, per scan pair (regression through the origin)')
        say(f'    all {len(pairs)} pairs: slope '
            f'{float(np.sum(gg*ll)/max(np.sum(gg*gg), 1e-12)):.3f}')
        if mv.sum() > 5:
            say(f'    {int(mv.sum())} pairs actually turning (>0.1 rad/s): slope '
                f'{float(np.sum(gg[mv]*ll[mv])/max(np.sum(gg[mv]**2), 1e-12)):.3f}')
        say(f'    {int((~mv).sum())} near-stationary pairs carry no yaw signal and only')
        say('    add ICP noise to any SUM over pairs.')
        say('')
        say('    CAVEAT: the gyro is integrated between BAG ARRIVAL times. Firmware and')
        say('    host clocks are unsynchronised so header stamps are no better, and scan')
        say('    inter-arrival jitter was measured at 113-147 ms p95 -- so the two')
        say('    windows are not the same interval. Indicative, not calibrated.')
        say('')

    # ------------------------------------------------------------- comparison
    say('  PATH LENGTH (metres travelled, not displacement)')
    say(f'    encoders  {enc_dist:8.3f}   peak speed {enc_peak:.3f} m/s')
    say(f'    lidar     {lid_dist:8.3f}')
    say('')
    say('  YAW (radians, summed over pairs -- see the caveat above)')
    say(f'    encoders  {enc_yaw:+8.3f}')
    if dead.get('imu_gyro_z'):
        say('    IMU gyro    DEAD    (channel reported a constant; NOT a measurement)')
    else:
        say(f'    IMU gyro  {imu_yaw:+8.3f}   (bias {gyro_bias:+.5f} rad/s removed)')
    say(f'    lidar     {lid_yaw:+8.3f}')
    say('')

    verdicts = {}

    # Translation: encoders vs lidar. This is the slip test.
    if enc_dist > 0.05:
        ratio = lid_dist / enc_dist
        slip = 1.0 - ratio
        verdicts['translation_ratio_lidar_over_encoder'] = ratio
        verdicts['apparent_slip_fraction'] = slip
        say(f'  TRANSLATION: lidar/encoder = {ratio:.3f}  -> apparent slip {slip*100:+.0f}%')
        if slip > 0.7:
            say('    *** The wheels turned and the world did not move. That is either')
            say('        total slip or the car is on a stand. ***')
        elif slip > 0.25:
            say('    *** Substantial slip: encoders are over-reporting travel. ***')
        else:
            say('    encoders and lidar broadly agree on distance travelled')
    else:
        say('  TRANSLATION: encoders report almost no travel; nothing to compare')

    # Yaw: the sources that are alive should agree if the body really rotated.
    if dead.get('imu_gyro_z'):
        say('')
        say('  YAW AGREEMENT: skipped, the gyro is dead. The lidar is the only surviving')
        say('  independent witness on this run.')
    elif abs(imu_yaw) > 0.05 or abs(enc_yaw) > 0.05:
        say('')
        say('  YAW AGREEMENT (the body cannot rotate by two different amounts)')
        for name, val in (('encoder', enc_yaw), ('lidar', lid_yaw)):
            if abs(imu_yaw) > 1e-6:
                say(f'    {name}/IMU = {val / imu_yaw:+.3f}')
        verdicts['yaw_encoder_over_imu'] = enc_yaw / imu_yaw if imu_yaw else None
        verdicts['yaw_lidar_over_imu'] = lid_yaw / imu_yaw if imu_yaw else None

    verdicts['dead_channels'] = [k for k, v in dead.items() if v]
    if pairs and not dead.get('imu_gyro_z'):
        g = np.array([p_[0] for p_ in pairs])
        lz = np.array([p_[1] for p_ in pairs])
        rate = np.array([p_[2] for p_ in pairs])
        moving = rate > 0.10
        verdicts['lidar_gyro_slope_all'] = float(np.sum(g*lz)/max(np.sum(g*g), 1e-12))
        verdicts['lidar_gyro_slope_moving'] = (
            float(np.sum(g[moving]*lz[moving])/max(np.sum(g[moving]**2), 1e-12))
            if moving.sum() > 5 else None)
        verdicts['n_pairs_moving'] = int(moving.sum())
        verdicts['n_pairs_total'] = len(pairs)
    verdicts.update({
        'encoder_path_m': enc_dist, 'lidar_path_m': lid_dist,
        'encoder_yaw_rad': enc_yaw, 'imu_yaw_rad': imu_yaw, 'lidar_yaw_rad': lid_yaw,
        'gyro_bias_rad_s': gyro_bias,
        'icp_converged': ok, 'icp_pairs': len(used) - 1, 'icp_degenerate': degen,
        'median_isotropy': float(np.median(isotropies)) if isotropies else None,
        'duration_s': t1 - t0,
    })

    if args.json:
        with open(args.json, 'w') as f:
            json.dump(verdicts, f, indent=2)
        say('')
        say(f'  json: {args.json}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
