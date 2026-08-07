#!/usr/bin/env python3
"""Check the firmware contract against whatever SIMULATOR is running.

    ./tools/check_isaac_contract.py --seconds 30

SIMULATOR ONLY. It refuses to run if YB_Car_Node is on the domain, because it DRIVES:
it commands 0.12 m/s so that /cmd_vel is proven to actually move the robot, and a
subscriber that only listens could never establish that.

That refusal was missing when this file was first committed. It published 0.12 m/s and
0.3 rad/s with no hardware guard, no zeros on exit and no domain pin, so running it on
ROS_DOMAIN_ID=20 would have driven the car ungoverned, and a Ctrl+C in its first half
would have left that command latched forever on a firmware with no watchdog. Found by
audit. The guards now come from tools/_cmd_vel_safety.py, which every driving tool here
shares.

Runs in SYSTEM python (3.12) on purpose: rclpy cannot be imported into Isaac's
interpreter at all -- Isaac is 3.11 and Jazzy builds rclpy for 3.12, an ABI mismatch
rather than a path problem -- so this has to be a separate process from the simulator
it is measuring.

It caught two things a backend cannot see about itself: sim_runner counted exactly 12
renders per second while /scan was actually arriving at 14.4 Hz, and the 2D simulator
published a perfectly constant 9.81 accelerometer, which is what a DEAD sensor looks
like. Both were invisible from inside the publisher.

Compares what a backend publishes against CLAUDE.md's measured contract:
  pub /scan 12 Hz, /odom_raw 11 Hz, /imu 25 Hz, /battery 1 Hz
  sub /cmd_vel

WHAT IT DOES NOT CHECK, since the name once claimed "EVERY field": QoS profiles, frame
ids beyond /scan's, message timestamps, /beep and the two servo topics, and the sign
conventions of anything. It checks rates, scan geometry, IMU liveness and battery scale.
"""
import argparse
import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _cmd_vel_safety import SafeCmdVel, require_simulator      # noqa: E402

import rclpy                                                    # noqa: E402
from nav_msgs.msg import Odometry                               # noqa: E402
from rclpy.node import Node                                     # noqa: E402
from rclpy.qos import qos_profile_sensor_data                   # noqa: E402
from sensor_msgs.msg import Imu, LaserScan                      # noqa: E402
from std_msgs.msg import UInt16

WANT_HZ = {'scan': 12.0, 'odom_raw': 11.0, 'imu': 25.0, 'battery': 1.0}

ap = argparse.ArgumentParser()
ap.add_argument('--seconds', type=float, default=30.0)
ap.add_argument('--speed', type=float, default=0.12)
ap.add_argument('--turn', type=float, default=0.3)
ap.add_argument('--domain', type=int, default=66,
                help='simulator domain; the car is on 20 and is refused')
args = ap.parse_args()
os.environ.setdefault('ROS_DOMAIN_ID', str(args.domain))

rclpy.init()
n = Node('contract_check')
got = {k: [] for k in WANT_HZ}


def rec(k):
    return lambda m: got[k].append((time.time(), m))


n.create_subscription(LaserScan, '/scan', rec('scan'), qos_profile_sensor_data)
n.create_subscription(Odometry, '/odom_raw', rec('odom_raw'), qos_profile_sensor_data)
n.create_subscription(Imu, '/imu', rec('imu'), qos_profile_sensor_data)
n.create_subscription(UInt16, '/battery', rec('battery'), qos_profile_sensor_data)

# THE GUARD. This tool commands motion, so it must never find the real robot.
require_simulator(n, what='check_isaac_contract.py')

with SafeCmdVel(n, ['/cmd_vel']) as safe:
    t_end = time.time() + args.seconds
    drive_until = time.time() + args.seconds * 0.5
    while time.time() < t_end:
        # Drive for the first half, so /cmd_vel is proven to actually move the robot.
        if time.time() < drive_until:
            safe.publish(vx=args.speed, wz=args.turn)
        else:
            safe.publish()
        rclpy.spin_once(n, timeout_sec=0.05)
# Leaving the block published zeros. So does Ctrl+C, SIGTERM, or any exception above.

fails = []
print(f'{"topic":12s} {"msgs":>6s} {"Hz":>7s} {"want":>6s}')
for k, want in WANT_HZ.items():
    ms = got[k]
    hz = (len(ms) - 1) / (ms[-1][0] - ms[0][0]) if len(ms) > 1 else 0.0
    # 10%, not 20%: a 20% band exactly masked /scan at 14.4 Hz against 12.
    flag = '' if abs(hz - want) <= max(0.3, want * 0.10) else '  <-- OFF'
    if not ms:
        flag = '  <-- SILENT'
        fails.append(f'{k} never published')
    elif flag:
        fails.append(f'{k} at {hz:.1f} Hz, want ~{want}')
    print(f'{k:12s} {len(ms):6d} {hz:7.1f} {want:6.1f}{flag}')

print()
if got['scan']:
    m = got['scan'][-1][1]
    inc = math.degrees(m.angle_increment)
    print(f'scan: {len(m.ranges)} beams, {inc:.4f} deg, '
          f'{math.degrees(m.angle_min):.1f}..{math.degrees(m.angle_max):.1f} deg, '
          f'{m.range_min:.3f}-{m.range_max:.3f} m, frame {m.header.frame_id!r}')
    if len(m.ranges) != 360:
        fails.append(f'scan has {len(m.ranges)} beams, want 360')
    if abs(inc - 1.0) > 0.01:
        fails.append(f'scan increment {inc:.4f} deg, want 1.0')

if got['imu']:
    m = got['imu'][-1][1]
    az = [x[1].linear_acceleration.z for x in got['imu']]
    mean = sum(az) / len(az)
    var = sum((v - mean) ** 2 for v in az) / len(az)
    print(f'imu:  orientation_covariance[0]={m.orientation_covariance[0]} '
          f'(-1 means "not provided"), accel_z mean {mean:.3f} std {var**0.5:.4f}')
    if m.orientation_covariance[0] != -1.0:
        fails.append('imu claims an orientation; this IMU is 6-axis with no magnetometer')
    if var == 0.0:
        fails.append('imu accel_z has ZERO variance -- that is what a dead sensor looks like')

if got['odom_raw']:
    xs = [math.hypot(x[1].pose.pose.position.x, x[1].pose.pose.position.y)
          for x in got['odom_raw']]
    print(f'odom: travelled {max(xs) - min(xs):.3f} m during the run')
    if max(xs) - min(xs) < 0.02:
        fails.append('robot never moved -- /cmd_vel is not reaching the wheels')

if got['battery']:
    v = got['battery'][-1][1].data
    print(f'batt: raw {v} -> {v/10:.1f} V')
    if not (60 <= v <= 90):
        fails.append(f'battery raw {v} is not a plausible tenths-of-a-volt reading')

print()
if fails:
    print(f'=== {len(fails)} CONTRACT FAILURES ===')
    for f in fails:
        print(f'  - {f}')
    sys.exit(1)
print('CONTRACT SATISFIED')
