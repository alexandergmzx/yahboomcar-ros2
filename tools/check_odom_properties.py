#!/usr/bin/env python3
"""Does /odom_raw behave like ENCODERS, or like a cheat? SIMULATOR ONLY.

    ./tools/check_odom_properties.py

Two properties, both of which a simulator can get wrong in ways that look fine:

  1. REVERSE READS NEGATIVE. Deriving speed with hypot() makes every direction positive,
     so a robot backing up reports that it is going forwards. Found by audit in the Isaac
     backend.

  2. SLIP IS VISIBLE. /odom_raw on the real robot is ENCODER-derived: it measures wheel
     rotation, not displacement. On this robot's stand the two differ by 92% -- the wheels
     claimed 1.602 m while the lidar saw 0.128 m. A backend that derives odometry from the
     physics engine's ground-truth pose deletes that disagreement entirely, and every EKF,
     scan matcher and slip detector tested against it then passes for the wrong reason.

It DRIVES, so it refuses to run against YB_Car_Node and always leaves the robot stopped.
"""
import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _cmd_vel_safety import SafeCmdVel, require_simulator          # noqa: E402

import rclpy                                                        # noqa: E402
from nav_msgs.msg import Odometry                                   # noqa: E402
from rclpy.node import Node                                         # noqa: E402
from rclpy.qos import qos_profile_sensor_data                       # noqa: E402

# 66 is simctl's; the Isaac backend is usually started on whatever the
# shell already has, so an explicit ROS_DOMAIN_ID always wins.
os.environ.setdefault('ROS_DOMAIN_ID', '66')
rclpy.init()
n = Node('odom_props')
odom, truth = [], []
n.create_subscription(Odometry, '/odom_raw',
                      lambda m: odom.append(m), qos_profile_sensor_data)
n.create_subscription(Odometry, '/sim/ground_truth', lambda m: truth.append(m), 10)
require_simulator(n, what='check_odom_properties.py')
safe = SafeCmdVel(n, ['/cmd_vel'])
safe.__enter__()
import atexit; atexit.register(safe.stop)


def drive(vx, wz, secs):
    end = time.time() + secs
    while time.time() < end:
        safe.publish(vx=vx, wz=wz)
        rclpy.spin_once(n, timeout_sec=0.02)


def settle(secs=2.0):
    end = time.time() + secs
    while time.time() < end:
        safe.publish()
        rclpy.spin_once(n, timeout_sec=0.02)


fails = []
settle(3.0)

# ---- 1. reverse ------------------------------------------------------------
odom.clear()
drive(-0.10, 0.0, 6.0)
rev = [m.twist.twist.linear.x for m in odom]
settle()
if not rev:
    fails.append('no /odom_raw during reverse')
else:
    mean_rev = sum(rev) / len(rev)
    print(f'REVERSE  commanded -0.10 m/s  ->  mean reported {mean_rev:+.4f} m/s '
          f'(min {min(rev):+.4f}, max {max(rev):+.4f})')
    if mean_rev >= 0:
        fails.append(f'reverse reported {mean_rev:+.4f} m/s -- not negative. '
                     'hypot() strikes again.')

# ---- 2. forward, for comparison -------------------------------------------
odom.clear()
drive(0.10, 0.0, 6.0)
fwd = [m.twist.twist.linear.x for m in odom]
settle()
if fwd:
    print(f'FORWARD  commanded +0.10 m/s  ->  mean reported '
          f'{sum(fwd)/len(fwd):+.4f} m/s')

# ---- 3. does odometry track the wheels rather than the world? -------------
# The robot is against a wall or free; either way, compare the DISTANCE odometry
# claims against the ground-truth displacement over the same window.
odom.clear()
truth.clear()
# Wait for BOTH. Reading truth[-1] right after clear() gave None and silently skipped
# the whole slip comparison -- a checker bug that hid the thing being checked.
deadline = time.time() + 10
while time.time() < deadline and not (odom and truth):
    rclpy.spin_once(n, timeout_sec=0.1)
p0 = odom[-1].pose.pose.position
t0 = truth[-1].pose.pose.position if truth else None
drive(0.15, 0.0, 8.0)
settle()
p1 = odom[-1].pose.pose.position
t1 = truth[-1].pose.pose.position if truth else None
odo_d = math.hypot(p1.x - p0.x, p1.y - p0.y)
print(f'ODOMETRY claims {odo_d:.3f} m of travel')
if t0 is not None and t1 is not None:
    true_d = math.hypot(t1.x - t0.x, t1.y - t0.y)
    slip = (1 - true_d / odo_d) * 100 if odo_d > 1e-6 else 0.0
    print(f'GROUND TRUTH moved {true_d:.3f} m  ->  slip {slip:+.1f}%')
    print('  (nonzero slip means odometry is measuring WHEELS, not the world --')
    print('   which is the property the real robot has and the point of the fix)')
else:
    print('  (no /sim/ground_truth on this backend; slip not comparable here)')

print()
if fails:
    print(f'=== {len(fails)} FAILURES ===')
    for f in fails:
        print(f'  - {f}')
    sys.exit(1)
print('ODOMETRY PROPERTIES OK')
