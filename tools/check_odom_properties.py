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
# ENFORCED, not merely printed. This used to calculate slip, print it, and pass
# regardless -- an audit reproduced a run with 0.0% odometry/truth divergence that
# still said OK, which is exactly the signature of an implementation that copies
# ground truth into odometry. The property either gets demonstrated or the verdict
# is UNPROVEN (nonzero exit); it is never assumed.
#
# The scenario matters per backend:
#   * a COLLISION backend (Isaac): driving at the arena wall guarantees slip --
#     wheels turn, body pinned. That divergence is the property.
#   * the 2D backend with slip:=0 genuinely HAS no slip -- odometry and truth agree
#     by construction, correctly. Agreement there proves nothing either way, so the
#     verdict is UNPROVEN with instructions, not OK and not FAIL.
odom.clear()
truth.clear()
# Wait for BOTH. Reading truth[-1] right after clear() gave None and silently skipped
# the whole slip comparison -- a checker bug that hid the thing being checked.
deadline = time.time() + 10
while time.time() < deadline and not (odom and truth):
    rclpy.spin_once(n, timeout_sec=0.1)
if not truth:
    fails.append('no /sim/ground_truth -- without truth the wheels-vs-world property '
                 'cannot be witnessed at all, and unwitnessed is not passed')
if not odom:
    fails.append('no /odom_raw before the slip phase')

unproven = None
if odom and truth:
    p0 = odom[-1].pose.pose.position
    t0 = truth[-1].pose.pose.position
    # Long enough that any collision backend reaches a wall from anywhere in the
    # 4x4 m arena and spends time pinned against it.
    drive(0.15, 0.0, 20.0)
    settle()
    p1 = odom[-1].pose.pose.position
    t1 = truth[-1].pose.pose.position
    odo_d = math.hypot(p1.x - p0.x, p1.y - p0.y)
    true_d = math.hypot(t1.x - t0.x, t1.y - t0.y)
    slip = (1 - true_d / odo_d) * 100 if odo_d > 1e-6 else 0.0
    print(f'ODOMETRY claims {odo_d:.3f} m of travel')
    print(f'GROUND TRUTH moved {true_d:.3f} m  ->  slip {slip:+.1f}%')
    if odo_d < 0.1:
        fails.append(f'odometry advanced only {odo_d*1000:.0f} mm while commanded '
                     'forward -- the wheels themselves are not being measured')
    elif slip >= 3.0:
        print('  slip DEMONSTRATED: odometry is measuring wheels, not the world')
    else:
        # Odometry and truth agree. On a collision backend after 20 s at a wall that
        # would be the cheat signature; on the 2D sim with slip:=0 it is correct
        # behaviour that simply cannot demonstrate the property.
        is_2d = any(name == 'fake_robot'
                    for name, _ in n.get_node_names_and_namespaces())
        if is_2d:
            unproven = ('2D backend with no slip configured: odometry equals truth by '
                        'construction, so the wheels-vs-world property CANNOT be '
                        'witnessed here. Restart with `./tools/simctl start --slip 0.3` '
                        'and rerun.')
        else:
            fails.append(f'slip only {slip:+.1f}% after 20 s of commanded driving on a '
                         'collision backend -- odometry is tracking ground truth, '
                         'which is the cheat this checker exists to catch')

print()
if fails:
    print(f'=== {len(fails)} FAILURES ===')
    for f in fails:
        print(f'  - {f}')
    sys.exit(1)
if unproven:
    print('=== UNPROVEN ===')
    print(f'  {unproven}')
    sys.exit(1)
print('ODOMETRY PROPERTIES OK -- reverse signed, and slip demonstrated')
