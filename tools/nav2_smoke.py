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

So this asserts: the action server exists, the goal is accepted, the action reports
SUCCEEDED, and the robot ENDS UP within tolerance of where it was sent, measured in the
frame the goal was expressed in. Anything else exits nonzero.

TWO THINGS THIS USED TO GET WRONG, both found by audit, and the reason the previously
reported "177 mm" figure has been retracted rather than restated:

  * IT COMPARED FRAMES THAT ARE NOT THE SAME FRAME. The goal is in `map`; the position
    came from /odom, which is in `odom`. They differ by exactly the map->odom transform
    -- SLAM's running correction for accumulated odometry error, measured in this repo
    jumping by up to 125 mm (docs/rviz-guide.md). Near the origin of a fresh session the
    two nearly coincide, which is why the mistake produced a plausible number instead of
    an obviously wrong one. Position now comes from TF map->base_link.

  * IT NEVER CHECKED THE ACTION STATUS. It waited for the result future to complete and
    then judged distance alone -- but ABORTED and CANCELED goals also complete. A Nav2
    that gave up one metre from the goal, having drifted close enough by luck, would have
    been recorded as a pass.

IT DELIBERATELY DOES NOT RUN THE SAFETY GOVERNOR
------------------------------------------------
The governor caps speed and stops for obstacles, which would fight a controller trying to
plan through a gap -- the robot would stall and this would report a Nav2 failure that is
really a governor success. The two need reconciling deliberately rather than discovering
mid-drive, so this reports the peak speed Nav2 commanded and leaves the decision visible.

ON THE REAL ROBOT that means Nav2 drives UNGOVERNED, and this tool REFUSES to run there
unless the refusal is explicitly overridden. Nav2 publishes straight to /cmd_vel; the
navigation launch does not start cmd_vel_deadman; and the firmware has no command
watchdog. The chain of things that would stop a runaway is, on the floor, empty.

In simulation there is nothing to hit, so it runs freely.
"""
import argparse
import math
import os
import signal
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _cmd_vel_safety import target_of                           # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--x', type=float, default=1.0)
    ap.add_argument('--y', type=float, default=0.0)
    ap.add_argument('--yaw', type=float, default=0.0)
    ap.add_argument('--tolerance', type=float, default=0.30, help='metres')
    ap.add_argument('--timeout', type=float, default=90.0)
    # 66 is what tools/simctl uses; this defaulted to 55, so the documented
    # `./tools/nav2_smoke.py --x 0.8` against a simctl simulation found nothing.
    ap.add_argument('--domain', type=int, default=66)
    ap.add_argument('--goal-frame', default='map')
    ap.add_argument('--check-only', action='store_true')
    ap.add_argument('--i-accept-driving-unprotected', action='store_true',
                    help='allow this to command a REAL robot. Nav2 bypasses the safety '
                         'governor, the nav launch starts no deadman, and the firmware '
                         'has no watchdog.')
    args = ap.parse_args()

    os.environ.setdefault('ROS_DOMAIN_ID', str(args.domain))

    import rclpy
    from action_msgs.msg import GoalStatus
    from rclpy.action import ActionClient
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from geometry_msgs.msg import PoseStamped, Twist
    from nav2_msgs.action import NavigateToPose
    from nav_msgs.msg import Odometry
    import tf2_ros

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
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer, node)      # noqa: F841
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()
    time.sleep(3.0)

    def pose_in_goal_frame():
        """Robot position in the SAME frame the goal is expressed in, or None.

        /odom is in `odom`; the goal is in `map`. Comparing them is comparing two
        different frames, and they differ by SLAM's live map->odom correction.
        """
        try:
            tr = tf_buffer.lookup_transform(
                args.goal_frame, 'base_link', rclpy.time.Time())
            return (tr.transform.translation.x, tr.transform.translation.y)
        except Exception:
            return None

    say('=== Nav2 smoke test ===')

    # ---- readiness, reported one line per prerequisite so a failure names itself ----
    ok = True
    if not poses:
        say('  FAIL: no /odom. Is the robot (or the simulator) running?')
        ok = False
    else:
        say(f'  /odom present, robot at ({poses[-1][0]:+.2f}, {poses[-1][1]:+.2f}) '
            f'in odom')

    here = pose_in_goal_frame()
    if here is None:
        say(f'  FAIL: no TF {args.goal_frame} -> base_link. SLAM or AMCL is not '
            f'publishing it, so a goal in {args.goal_frame} cannot be verified. '
            '(AMCL will not publish map->odom without an initial pose.)')
        ok = False
    else:
        say(f'  TF {args.goal_frame}->base_link present, robot at '
            f'({here[0]:+.2f}, {here[1]:+.2f}) in {args.goal_frame}')

    # WHICH ROBOT? Nav2 on the real car drives with nothing between it and the motors.
    # target_of polls with the asymmetric dwell (hardware ends the search, a simulator
    # must survive a confirmation window) and recognises the Isaac backend by
    # /sim/ground_truth -- a single node-name look here called Isaac "unknown" and
    # PROCEEDED, which is fail-open twice over. Audit finding.
    target = target_of(node)
    say(f'  target: {target.upper()}')
    if target in ('hardware', 'both') and not args.i_accept_driving_unprotected:
        say('')
        say('  REFUSED: this would drive the REAL robot, and Nav2 drives unprotected.')
        say('    * Nav2 publishes to /cmd_vel, bypassing cmd_vel_governor entirely')
        say('    * navigation_dwb_launch.py does not start cmd_vel_deadman')
        say('    * the firmware has NO command watchdog: a crash leaves it driving')
        say('  Nothing in that chain would stop a runaway. Measure the stopping')
        say('  envelope first (docs/first-floor-procedure.md), keep a hand on the power')
        say('  switch, then pass --i-accept-driving-unprotected if you still mean it.')
        return 2
    if target == 'nothing' and not args.i_accept_driving_unprotected:
        say('')
        say('  REFUSED: no positive evidence of a simulator on this domain, and "I did')
        say('  not find the car" is not evidence the car is not there -- discovery may')
        say('  just be slow. An unidentified domain gets no autonomous goals. Start a')
        say('  simulator (./tools/simctl start), check ROS_DOMAIN_ID, or pass')
        say('  --i-accept-driving-unprotected to command it anyway.')
        return 2
    if target in ('hardware', 'both', 'nothing'):
        say('  *** OVERRIDDEN: commanding an unprotected target with no governor and '
            'no deadman. Hand on the power switch. ***')

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

    start = pose_in_goal_frame()
    goal = NavigateToPose.Goal()
    goal.pose = PoseStamped()
    goal.pose.header.frame_id = args.goal_frame
    goal.pose.header.stamp = node.get_clock().now().to_msg()
    goal.pose.pose.position.x = args.x
    goal.pose.pose.position.y = args.y
    goal.pose.pose.orientation.z = math.sin(args.yaw / 2.0)
    goal.pose.pose.orientation.w = math.cos(args.yaw / 2.0)

    say(f'  goal: ({args.x:+.2f}, {args.y:+.2f}) in map, tolerance '
        f'{args.tolerance*1000:.0f} mm, timeout {args.timeout:.0f} s')
    cmds.clear()

    zero_pub = node.create_publisher(Twist, '/cmd_vel', 10)
    odom_twist = []
    node.create_subscription(
        Odometry, '/odom_raw',
        lambda m: odom_twist.append((time.time(), m.twist.twist.linear.x,
                                     m.twist.twist.angular.z)), 10)

    def cancel_and_verify(handle, why):
        """Cancel, CHECK the response, publish EXPLICIT ZEROS, then OBSERVE at rest.

        Two audit findings live here, and the second is the one this whole repo exists
        to remember:

        * A completed cancel future is not an accepted cancellation -- the CancelGoal
          response carries `goals_canceling`, and an empty list is a rejection.
        * "/cmd_vel went quiet" is NOT a stop on this firmware. THE ESP32 RETAINS THE
          LAST NONZERO COMMAND INDEFINITELY, so silence after a cancel means the car
          keeps driving on whatever Nav2 said last. A previous version of this function
          declared success on exactly that silence.

        So the sequence is: cancel -> wait for Nav2 to stop publishing (otherwise it
        republishes over the zeros) -> publish an explicit zero burst -> and only call
        it stopped when /odom_raw is OBSERVED near zero. No observation, no verdict.
        """
        say(f'  {why} -- CANCELLING the goal')
        acked = False
        if handle is not None:
            try:
                cancel = handle.cancel_goal_async()
                tc = time.time()
                while not cancel.done() and time.time() - tc < 10.0:
                    time.sleep(0.1)
                if cancel.done():
                    resp = cancel.result()
                    acked = bool(resp and resp.goals_canceling)
                    say('  cancel ACCEPTED by the server' if acked else
                        '  WARNING: server DECLINED the cancel (no goals canceling)')
                else:
                    say('  WARNING: cancel response never arrived')
            except Exception as e:
                say(f'  WARNING: cancel failed ({e})')
        else:
            # The send future timed out, so there is no handle -- but the goal may
            # still be accepted late and start driving. A zero goal ID cancels ALL
            # goals on the server.
            say('  no goal handle (send timed out) -- cancelling ALL goals')
            try:
                from action_msgs.srv import CancelGoal
                cli = node.create_client(CancelGoal,
                                         '/navigate_to_pose/_action/cancel_goal')
                if cli.wait_for_service(timeout_sec=5.0):
                    fut = cli.call_async(CancelGoal.Request())   # zero UUID = all
                    tc = time.time()
                    while not fut.done() and time.time() - tc < 10.0:
                        time.sleep(0.1)
                    acked = fut.done()
                    say('  cancel-all sent' if acked else
                        '  WARNING: cancel-all response never arrived')
                else:
                    say('  WARNING: cancel service absent')
            except Exception as e:
                say(f'  WARNING: cancel-all failed ({e})')

        # 1. Wait for Nav2 to stop COMMANDING -- zeros published under a live
        #    controller just get overridden at its next cycle.
        quiet_since = time.time()
        deadline = time.time() + 8.0
        n_before = len(cmds)
        nav2_quiet = False
        while time.time() < deadline:
            time.sleep(0.2)
            recent = cmds[n_before:]
            n_before = len(cmds)
            if any(abs(v) > 0.01 or abs(w) > 0.01 for v, w in recent):
                quiet_since = time.time()
            elif time.time() - quiet_since >= 2.0:
                nav2_quiet = True
                break
        if not nav2_quiet:
            say('  *** Nav2 IS STILL COMMANDING MOTION after the cancel. Kill the nav')
            say('  launch, and on the real robot use the power switch. ***')

        # 2. EXPLICIT ZEROS. Silence is not a stop on a firmware that latches the last
        #    command; only a zero is.
        z = Twist()
        for _ in range(20):
            zero_pub.publish(z)
            time.sleep(0.05)
        say('  explicit zeros published (silence is not a stop on this firmware)')

        # 3. OBSERVE the stop. /odom_raw is the witness; no data means UNKNOWN.
        odom_twist.clear()
        t_obs = time.time()
        while time.time() - t_obs < 4.0:
            time.sleep(0.2)
        recent = [s for s in odom_twist if s[0] > time.time() - 2.0]
        if not recent:
            say('  *** NO /odom_raw DATA: the stop is UNVERIFIED. Do not walk away')
            say('  from this robot. ***')
            return False
        peak = max(max(abs(v), abs(w)) for _, v, w in recent)
        if peak <= 0.03:
            say(f'  VERIFIED AT REST: /odom_raw peak {peak:.3f} over the last 2 s')
            return acked and nav2_quiet
        say(f'  *** STILL MOVING: /odom_raw peak {peak:.3f}. Power switch. ***')
        return False

    send = client.send_goal_async(goal)
    t0 = time.time()
    while not send.done() and time.time() - t0 < 10.0:
        time.sleep(0.1)
    if not send.done():
        say('  FAIL: goal was never accepted or rejected within 10 s')
        # The goal may still be accepted LATE, with no handle here to cancel it --
        # a driving goal owned by nobody. Cancel everything and verify the stop.
        cancel_and_verify(None, 'send-goal response timed out')
        return 1
    handle = send.result()
    if not handle.accepted:
        say('  FAIL: goal REJECTED by the action server')
        return 1
    say('  goal accepted')

    # From here a goal is LIVE. Ctrl+C or SIGTERM used to exit this process outright,
    # leaving Nav2 navigating with no governor and no deadman -- walking away from a
    # moving robot. Both now cancel first. Audit finding.
    def on_signal(signum, _frame):
        say(f'\n  signal {signum} while a goal is live')
        cancel_and_verify(handle, f'interrupted by signal {signum}')
        sys.stdout.flush()
        os._exit(143 if signum == signal.SIGTERM else 130)

    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, on_signal)

    result_future = handle.get_result_async()
    try:
        while not result_future.done() and time.time() - t0 < args.timeout:
            time.sleep(0.2)
    except Exception:
        cancel_and_verify(handle, 'exception while waiting for the result')
        raise
    finally:
        # The goal is settled or being handled; stop intercepting signals.
        signal.signal(signal.SIGINT, signal.SIG_DFL)
        signal.signal(signal.SIGTERM, signal.SIG_DFL)

    timed_out = not result_future.done()
    if timed_out:
        # CANCEL, do not merely return. Returning left Nav2 still driving -- on the real
        # robot, with no governor and no deadman in that launch, walking away from a
        # moving robot is the worst possible response to a timeout.
        say('')
        cancel_and_verify(handle, f'timed out after {args.timeout:.0f} s')

    end = pose_in_goal_frame() or start
    err = math.hypot(end[0] - args.x, end[1] - args.y)
    travelled = math.hypot(end[0] - start[0], end[1] - start[1])
    peak_v = max((abs(c[0]) for c in cmds), default=0.0)
    peak_w = max((abs(c[1]) for c in cmds), default=0.0)

    status = result_future.result().status if not timed_out else None
    status_name = {
        GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
        GoalStatus.STATUS_ABORTED: 'ABORTED',
        GoalStatus.STATUS_CANCELED: 'CANCELED',
        GoalStatus.STATUS_EXECUTING: 'EXECUTING',
    }.get(status, f'status {status}')

    say('')
    say(f'  start   ({start[0]:+.2f}, {start[1]:+.2f})  in {args.goal_frame}')
    say(f'  end     ({end[0]:+.2f}, {end[1]:+.2f})  in {args.goal_frame}')
    say(f'  travelled {travelled:.2f} m, final error {err*1000:.0f} mm')
    say(f'  action status: {status_name}')
    say(f'  Nav2 commanded up to {peak_v:.2f} m/s and {peak_w:.2f} rad/s '
        f'({len(cmds)} /cmd_vel messages)')

    if peak_v > 0.35:
        say(f'  NOTE: {peak_v:.2f} m/s exceeds the governor cap of 0.35 m/s. On the '
            'floor the governor would clamp this, so Nav2 would not get the speed it '
            'planned for.')

    if timed_out:
        say('')
        say(f'  FAIL: timed out after {args.timeout:.0f} s without a result')
        return 1
    # A completed future is NOT success: ABORTED and CANCELED complete too, and a Nav2
    # that gave up while happening to be near the goal would otherwise have passed.
    if status != GoalStatus.STATUS_SUCCEEDED:
        say('')
        say(f'  FAIL: Nav2 reported {status_name}, not SUCCEEDED. Distance to the goal '
            f'is {err*1000:.0f} mm, but the navigator did not consider it reached.')
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
