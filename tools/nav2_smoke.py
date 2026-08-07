#!/usr/bin/env python3
"""Send Nav2 a goal and assert it is actually reached. Exits NONZERO when it is not.

    ./tools/nav2_smoke.py --x 1.0 --y 0.5
    ./tools/nav2_smoke.py --check-only        # report readiness, command nothing

Needs a robot (real or simulated) plus SLAM or AMCL plus Nav2 already running. The
simulated route needs no floor:

    ros2 launch yahboomcar_sim  sim_bringup_launch.py
    ros2 launch yahboomcar_nav  slam_toolbox_launch.py
    ros2 launch yahboomcar_nav  navigation_dwb_launch.py

WHY A SMOKE TEST AND NOT "IT LOOKED FINE IN RVIZ"
-------------------------------------------------
Nav2's parameters have been ported and every node configures -- four Jazzy breakages were
found and fixed to get that far -- but configuring is not navigating. A stack can load
every plugin, accept a goal, publish a plan and still never move, and watching RViz is
not a test because nothing fails.

So this asserts: the action server exists, the goal is accepted, the robot ENDS UP within
tolerance of where it was sent, and it did so within a timeout. Anything else exits
nonzero.

IT DELIBERATELY DOES NOT RUN THE SAFETY GOVERNOR
------------------------------------------------
The governor caps speed and stops for obstacles, which would fight a controller trying to
plan through a gap -- the robot would stall and this would report a Nav2 failure that is
really a governor success. The two need reconciling deliberately rather than discovering
mid-drive, so this reports the peak speed Nav2 commanded and leaves the decision visible.

ON THE REAL ROBOT that means Nav2 drives UNGOVERNED. Do not run it on the floor until the
stopping envelope is measured. In simulation there is nothing to hit.
"""
import argparse
import math
import os
import sys
import threading
import time


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--x', type=float, default=1.0)
    ap.add_argument('--y', type=float, default=0.0)
    ap.add_argument('--yaw', type=float, default=0.0)
    ap.add_argument('--tolerance', type=float, default=0.30, help='metres')
    ap.add_argument('--timeout', type=float, default=90.0)
    ap.add_argument('--domain', type=int, default=55)
    ap.add_argument('--check-only', action='store_true')
    args = ap.parse_args()

    os.environ.setdefault('ROS_DOMAIN_ID', str(args.domain))

    import rclpy
    from rclpy.action import ActionClient
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from geometry_msgs.msg import PoseStamped, Twist
    from nav2_msgs.action import NavigateToPose
    from nav_msgs.msg import Odometry

    lines = []

    def say(m=''):
        print(m, flush=True)
        lines.append(str(m))

    rclpy.init()
    node = Node('nav2_smoke')
    poses, cmds = [], []
    node.create_subscription(
        Odometry, '/odom',
        lambda m: poses.append((m.pose.pose.position.x, m.pose.pose.position.y)), 10)
    node.create_subscription(
        Twist, '/cmd_vel',
        lambda m: cmds.append((m.linear.x, m.angular.z)), 10)
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()
    time.sleep(3.0)

    say('=== Nav2 smoke test ===')

    # ---- readiness, reported one line per prerequisite so a failure names itself ----
    ok = True
    if not poses:
        say('  FAIL: no /odom. Is the robot (or the simulator) running?')
        ok = False
    else:
        say(f'  /odom present, robot at ({poses[-1][0]:+.2f}, {poses[-1][1]:+.2f})')

    client = ActionClient(node, NavigateToPose, 'navigate_to_pose')
    if not client.wait_for_server(timeout_sec=10.0):
        say('  FAIL: navigate_to_pose action server absent. Nav2 is not up, or its '
            'lifecycle nodes were never activated.')
        ok = False
    else:
        say('  navigate_to_pose action server present')

    if not ok:
        return 2
    if args.check_only:
        say('  check-only: prerequisites met, commanding nothing')
        return 0

    start = poses[-1]
    goal = NavigateToPose.Goal()
    goal.pose = PoseStamped()
    goal.pose.header.frame_id = 'map'
    goal.pose.header.stamp = node.get_clock().now().to_msg()
    goal.pose.pose.position.x = args.x
    goal.pose.pose.position.y = args.y
    goal.pose.pose.orientation.z = math.sin(args.yaw / 2.0)
    goal.pose.pose.orientation.w = math.cos(args.yaw / 2.0)

    say(f'  goal: ({args.x:+.2f}, {args.y:+.2f}) in map, tolerance '
        f'{args.tolerance*1000:.0f} mm, timeout {args.timeout:.0f} s')
    cmds.clear()

    send = client.send_goal_async(goal)
    t0 = time.time()
    while not send.done() and time.time() - t0 < 10.0:
        time.sleep(0.1)
    if not send.done():
        say('  FAIL: goal was never accepted or rejected')
        return 1
    handle = send.result()
    if not handle.accepted:
        say('  FAIL: goal REJECTED by the action server')
        return 1
    say('  goal accepted')

    result_future = handle.get_result_async()
    while not result_future.done() and time.time() - t0 < args.timeout:
        time.sleep(0.2)

    end = poses[-1] if poses else start
    err = math.hypot(end[0] - args.x, end[1] - args.y)
    travelled = math.hypot(end[0] - start[0], end[1] - start[1])
    peak_v = max((abs(c[0]) for c in cmds), default=0.0)
    peak_w = max((abs(c[1]) for c in cmds), default=0.0)

    say('')
    say(f'  start   ({start[0]:+.2f}, {start[1]:+.2f})')
    say(f'  end     ({end[0]:+.2f}, {end[1]:+.2f})')
    say(f'  travelled {travelled:.2f} m, final error {err*1000:.0f} mm')
    say(f'  Nav2 commanded up to {peak_v:.2f} m/s and {peak_w:.2f} rad/s '
        f'({len(cmds)} /cmd_vel messages)')

    if peak_v > 0.35:
        say(f'  NOTE: {peak_v:.2f} m/s exceeds the governor cap of 0.35 m/s. On the '
            'floor the governor would clamp this, so Nav2 would not get the speed it '
            'planned for.')

    if not result_future.done():
        say('')
        say(f'  FAIL: timed out after {args.timeout:.0f} s without a result')
        return 1
    if err > args.tolerance:
        say('')
        say(f'  FAIL: finished {err*1000:.0f} mm from the goal, tolerance is '
            f'{args.tolerance*1000:.0f} mm')
        return 1
    if travelled < 0.05:
        say('')
        say('  FAIL: the robot never moved. A goal that is accepted and completed '
            'without motion usually means it was already inside tolerance, or the '
            'controller published to a topic nothing is listening to.')
        return 1

    say('')
    say(f'  PASS: reached the goal within {err*1000:.0f} mm')
    return 0


if __name__ == '__main__':
    code = main()
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(code)
