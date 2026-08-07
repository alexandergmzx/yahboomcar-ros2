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

THIS IS NOT HYPOTHETICAL ON THIS ROBOT. The gyro is intermittently dead. Across every bag
recorded on 2026-08-06:

    selftest-004042    LIVE   (std 0.305)
    selftest-004114    DEAD   (std 0.000000)
    selftest-004659    DEAD
    selftest-004728    DEAD
    selftest-004826    LIVE   (std 0.150)
    twin_dataset       DEAD

Four of six, with live and dead runs minutes apart, so it is not a permanent fault and
not something a single check at the start of the day settles. The accelerometer stays
live throughout (gravity reads correctly on z), so it is the gyro specifically.

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


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--seconds', type=float, default=15.0)
    ap.add_argument('--domain', type=int, default=20)
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
        dead = s < 1e-9
        print(f'    {label:12s} std {s:.6f}  {"DEAD" if dead else "live"}')
        if dead:
            fails.append(f'{label} is stuck at {series[0]:.6f}')

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
            print('  The gyro is intermittently dead on this robot -- 4 of 6 recorded')
            print('  bags. A power cycle has brought it back before. It is NOT safe to')
            print('  treat a stuck gyro as "no rotation": a filter will fuse that as a')
            print('  measurement. tools/measure_braking.py --calibrate also needs it to')
            print('  witness that a push was straight, so calibration is blocked.')
        return 1

    print('  SENSOR HEALTH: PASS -- every channel is alive and measuring')
    return 0


if __name__ == '__main__':
    code = main()
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(code)
