#!/usr/bin/env python3
"""Are the sensors alive, or just present? Exits NONZERO when a channel is dead.

    ./tools/sensor_health.py                 # 15 s check, all channels
    ./tools/sensor_health.py --seconds 30

Run this BEFORE any floor session. It moves nothing.

WHY A DEAD CHANNEL IS WORSE THAN A MISSING ONE
----------------------------------------------
A topic that stops publishing is obvious: `ros2 topic hz` says nothing arrives, every
consumer times out, and the stack fails loudly. A channel that publishes a CONSTANT is
the dangerous case, because it looks like data and reads as a confident measurement.

A gyro stuck at exactly 0.000000 does not say "I am broken". It says **"the robot is not
rotating"**, and an EKF will fuse that as an observation, shrink its covariance on the
strength of it, and become confident about a rotation it cannot see.

A STATIONARY ROBOT CANNOT ANSWER THIS QUESTION
----------------------------------------------
At rest, a working gyro and a dead one both report approximately zero, and this hardware
quantises to EXACTLY zero. So variance alone cannot tell them apart -- and preflight is
run on a stationary robot, which is precisely when the test is blind.

An earlier version of this file failed a stationary robot outright and would have cried
wolf before every session. Correcting it: at rest the gyro is reported UNDETERMINED, and
--rotate-test is the only thing that settles it.

THE FAULT IS REAL, THOUGH. Cross-checking every recorded bag against whether rotation was
actually COMMANDED separates the two cases:

    bag                gyro std   |wz| commanded   verdict
    selftest-004042    0.305      1.545            live
    selftest-004114    0.000000   0.757            FAULT: told to turn, gyro flat
    selftest-004659    0.000000   0.000            fine -- nothing rotated
    selftest-004728    0.000000   0.914            FAULT
    selftest-004826    0.150      0.693            live
    twin_dataset       0.000000   1.040            FAULT

Three confirmed faults out of five informative runs, with live and faulty runs minutes
apart, so it is intermittent rather than permanent. The accelerometer stays live
throughout -- accel-z reads 9.80 with real variance -- so it is the gyro specifically.
Neither a power cycle nor a serial reset has recovered it (2026-08-06, several attempts).

WHAT IT COSTS
-------------
  * The EKF's IMU input contributes yaw rate. Dead, it contributes a confident zero.
  * imu_filter_madgwick has nothing to integrate, so /imu/data orientation is frozen.
  * tools/measure_braking.py --calibrate needs the gyro to witness that a push was
    STRAIGHT. With it dead, no valid calibration can be produced at all, and the braking
    gate correctly refuses to record runs.
"""
import argparse
import math
import os
import sys
import threading
import time

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--seconds', type=float, default=15.0)
    ap.add_argument('--domain', type=int, default=20)
    ap.add_argument('--rotate-test', action='store_true',
                    help='the only decisive gyro check: you rotate the car by hand and '
                         'the gyro must respond. Needs no motors.')
    ap.add_argument('--rotate-window', type=float, default=0.0,
                    help='non-interactive rotate test: watch for this many seconds and '
                         'use the LIDAR to witness whether a rotation actually happened, '
                         'so you can turn the car whenever you like within the window.')
    args = ap.parse_args()
    os.environ.setdefault('ROS_DOMAIN_ID', str(args.domain))

    import numpy as np
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Imu, LaserScan
    from std_msgs.msg import UInt16

    rclpy.init()
    node = Node('sensor_health')
    odom, scans, imu, batt = [], [], [], []
    node.create_subscription(Odometry, '/odom_raw',
                             lambda m: odom.append((m.twist.twist.linear.x,
                                                    m.twist.twist.angular.z)),
                             qos_profile_sensor_data)
    node.create_subscription(LaserScan, '/scan', lambda m: scans.append(m),
                             qos_profile_sensor_data)
    node.create_subscription(Imu, '/imu',
                             lambda m: imu.append((m.angular_velocity.x,
                                                   m.angular_velocity.y,
                                                   m.angular_velocity.z,
                                                   m.linear_acceleration.x,
                                                   m.linear_acceleration.z)),
                             qos_profile_sensor_data)
    node.create_subscription(UInt16, '/battery', lambda m: batt.append(m.data),
                             qos_profile_sensor_data)
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()

    print(f'listening {args.seconds:.0f} s on ROS_DOMAIN_ID='
          f'{os.environ["ROS_DOMAIN_ID"]}...', flush=True)
    time.sleep(args.seconds)

    fails, warns = [], []

    def rate(n):
        return n / args.seconds

    # ---- is anything arriving at all? ----
    print()
    print('  channel      rate      status')
    for name, data, want in (('/odom_raw', odom, 11.0), ('/scan', scans, 12.0),
                             ('/imu', imu, 25.0), ('/battery', batt, 1.0)):
        r = rate(len(data))
        ok = r >= want * 0.5
        print(f'  {name:12s} {r:5.1f} Hz  {"ok" if ok else "SILENT / TOO SLOW"}')
        if not ok:
            fails.append(f'{name} at {r:.1f} Hz, expected ~{want:.0f}')
    if not imu or not scans or not odom:
        print('\nFAIL: core topics are silent. Is the board powered and registered?')
        return 2

    # ---- present, but is it MEASURING? ----
    print()
    print('  STUCK-CHANNEL CHECK (a constant reads as a confident measurement)')
    a = np.array(imu)
    checks = [
        ('imu gyro x', a[:, 0]), ('imu gyro y', a[:, 1]), ('imu gyro z', a[:, 2]),
        ('imu accel x', a[:, 3]), ('imu accel z', a[:, 4]),
    ]
    for label, series in checks:
        s = float(np.std(series))
        flat = s < 1e-9
        if 'gyro' in label:
            # At rest a working gyro reads ~0 and this hardware quantises to exactly 0,
            # so a flat reading here is UNDETERMINED, not dead. Failing it would cry wolf
            # before every session, since preflight runs on a stationary robot.
            verdict = 'flat (undetermined at rest)' if flat else 'live'
        elif label == 'imu accel z':
            # Gravity is a signal that is always present, so this axis CAN be judged at
            # rest: it must read ~9.8 and it must vary.
            mag = float(np.mean(np.abs(series)))
            verdict = 'live' if (not flat and 8.0 < mag < 11.5) else 'SUSPECT'
            if verdict == 'SUSPECT':
                fails.append(f'accel z reads {mag:.2f} m/s^2 with std {s:.6f}; '
                             'gravity should be ~9.8 and should vary')
        else:
            # accel x/y are legitimately ~0 on a level, still robot, and quantise to
            # exactly 0. Not judgeable here either.
            verdict = 'flat (level and still, expected)' if flat else 'live'
        print(f'    {label:12s} std {s:.6f}  {verdict}')

    gyro_flat = all(float(np.std(a[:, i])) < 1e-9 for i in (0, 1, 2))
    if gyro_flat:
        warns.append('gyro is flat on all three axes. At rest that proves nothing -- '
                     'run --rotate-test to settle it.')

    # Odometry at rest is legitimately constant, so a zero-variance odom is only
    # suspicious if it never moves across a run where motion was commanded. Not
    # something this tool can judge, so it is reported rather than failed.
    ov = np.array([o[0] for o in odom])
    if float(np.std(ov)) < 1e-12:
        print(f'    odom vx      std 0.000000  constant at {ov[0]:.3f} '
              f'({"at rest, expected" if abs(ov[0]) < 0.02 else "STUCK WHILE NONZERO"})')
        if abs(ov[0]) >= 0.02:
            fails.append('odom vx stuck at a non-zero value')

    # ---- lidar quality ----
    print()
    m = scans[-1]
    r = np.array(m.ranges)
    in_range = ((r >= m.range_min) & (r <= m.range_max)).sum()
    print(f'  LIDAR: {in_range}/{len(r)} returns in range '
          f'[{m.range_min:.2f}, {m.range_max:.1f}] m')
    if in_range < len(r) * 0.5:
        fails.append(f'only {in_range}/{len(r)} lidar returns are usable')
    elif in_range < len(r) * 0.8:
        warns.append(f'{len(r) - in_range} lidar returns invalid; check for obstruction')

    # ---- battery ----
    if batt:
        v = batt[-1] / 10.0
        gate = 7.4
        print(f'  BATTERY: {v:.1f} V  (session gate {gate} V)')
        if v < gate:
            fails.append(f'battery {v:.1f} V below the {gate} V session gate')

    # ---- lidar-witnessed rotate test, no timing coordination needed ----
    if args.rotate_window > 0:
        print()
        print(f'  LIDAR-WITNESSED ROTATE TEST -- {args.rotate_window:.0f} s window.')
        print('  Turn the car by hand about the vertical axis at ANY point during it.')
        print('  The lidar independently establishes whether a rotation happened, so the')
        print('  gyro is judged against physical evidence rather than against your word')
        print('  or my timing.')
        sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
        from _layout import pkg_dir
        sys.path.insert(0, pkg_dir('yahboomcar_localization'))
        from yahboomcar_localization.scan_geometry import scan_to_xy
        from yahboomcar_localization.scan_matcher import estimate_motion

        imu.clear()
        scans.clear()
        t0 = time.time()
        while time.time() - t0 < args.rotate_window:
            time.sleep(1.0)
            print(f'    {time.time()-t0:4.0f} s / {args.rotate_window:.0f} s  '
                  f'({len(scans)} scans, {len(imu)} imu)', end='\r', flush=True)
        print()

        b = np.array(imu) if imu else np.zeros((0, 5))
        gyro_peak = float(np.max(np.abs(b[:, 2]))) if len(b) else 0.0

        # What the LIDAR says happened, independent of the IMU entirely.
        lidar_yaw = 0.0
        prev = None
        step = max(1, len(scans) // 120)
        for sc in scans[::step]:
            xy = scan_to_xy(list(sc.ranges), sc.angle_min, sc.angle_increment,
                            range_min=max(sc.range_min, 0.05), range_max=sc.range_max)
            if prev is not None and len(xy) > 20:
                r = estimate_motion(prev, xy)
                if r.converged:
                    lidar_yaw += abs(r.dtheta)
            prev = xy

        print(f'    lidar saw {math.degrees(lidar_yaw):.0f} deg of total rotation')
        print(f'    gyro peak {gyro_peak:.4f} rad/s')
        if lidar_yaw < math.radians(30):
            warns.append(f'the lidar only saw {math.degrees(lidar_yaw):.0f} deg of '
                         'rotation, so the car was probably not turned enough. '
                         'INCONCLUSIVE -- rerun and turn it further.')
            print('    -> INCONCLUSIVE: not enough rotation to judge the gyro.')
        elif gyro_peak < 0.05:
            fails.append(f'the lidar witnessed {math.degrees(lidar_yaw):.0f} deg of '
                         f'rotation while the gyro peaked at {gyro_peak:.4f} rad/s -- '
                         'the gyro did not see a rotation that demonstrably happened')
            print('    -> DEAD. The room moved and the gyro did not notice.')
        else:
            print('    -> LIVE. It responded to a rotation the lidar confirms happened.')
            warns[:] = [w for w in warns if 'gyro is flat' not in w]

    # ---- the only decisive gyro test: make it rotate ----
    if args.rotate_test:
        print()
        print('  ROTATE TEST -- the gyro cannot be judged at rest, so rotate it.')
        print('  Pick the car up and turn it briskly left and right about the VERTICAL')
        print('  axis for a few seconds. Motors stay off; this is your hands only.')
        try:
            input('  Press Enter, then rotate for ~8 s... ')
        except EOFError:
            print('  (no console; skipping)')
        else:
            imu.clear()
            time.sleep(8.0)
            if len(imu) < 20:
                fails.append('no IMU data during the rotate test')
            else:
                b = np.array(imu)
                peak = float(np.max(np.abs(b[:, 2])))
                spread = float(np.std(b[:, 2]))
                print(f'    gyro z: peak {peak:.4f} rad/s, std {spread:.6f}')
                if peak < 0.05:
                    fails.append(f'gyro z peaked at only {peak:.4f} rad/s while the car '
                                 'was rotated by hand -- the gyro is NOT responding')
                    print('    -> DEAD. It did not see a rotation you performed.')
                else:
                    print('    -> LIVE. It responded to real rotation.')
                    warns[:] = [w for w in warns if 'gyro is flat' not in w]

    # ---- verdict ----
    print()
    for w in warns:
        print(f'  WARN: {w}')
    if fails:
        print('  SENSOR HEALTH: FAIL')
        for f in fails:
            print(f'    - {f}')
        print()
        if any('gyro' in f for f in fails):
            print('  The gyro on this robot is intermittently faulty -- confirmed in 3 of')
            print('  5 informative bags, where rotation was COMMANDED and the gyro stayed')
            print('  flat. Neither a power cycle nor a serial reset has recovered it.')
            print('  A filter will fuse a flat gyro as "not rotating", and')
            print('  measure_braking.py --calibrate needs it to witness that a push was')
            print('  straight, so calibration is blocked while it is out.')
        return 1

    if warns and not args.rotate_test:
        print('  SENSOR HEALTH: PASS with caveats -- rerun with --rotate-test to settle')
        print('  the gyro, which cannot be judged on a stationary robot.')
        return 0
    print('  SENSOR HEALTH: PASS -- every channel is alive and measuring')
    return 0


if __name__ == '__main__':
    code = main()
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(code)
