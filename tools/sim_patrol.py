#!/usr/bin/env python3
"""Drive the simulated robot around its arena, so there is something to watch.

    ./tools/sim_patrol.py                    # loop until stopped
    ./tools/sim_patrol.py --laps 2 --speed 0.15

For demonstrations and for exercising SLAM. It publishes /cmd_vel_raw by default, so the
safety governor stays in the loop exactly as it would on the floor -- pass --direct only
if no governor is running.

REFUSES TO DRIVE REAL HARDWARE. It looks for `fake_robot` and stops if it finds
`YB_Car_Node` instead: an unattended patrol loop is the last thing that should ever reach
a real robot whose firmware has no command watchdog.

IT HANDLES SIGTERM, AND THAT IS NOT A DETAIL
--------------------------------------------
An earlier version caught only KeyboardInterrupt. `pkill` sends SIGTERM, whose default
action terminates the process immediately -- so the `finally` block never ran, no zero was
ever published, and the last command stayed latched. The simulated robot kept driving and
was 12 m outside a 4 m room before anyone stopped it.

That is not a simulator quirk. It is the real firmware's behaviour, faithfully reproduced:
no command watchdog, so whatever was last commanded runs forever. The demonstration was
accidental and completely convincing.

The lesson is not "handle SIGTERM" -- it is that **no publisher can be relied on to clean
up after itself**, because SIGKILL cannot be caught at all. That is exactly why
cmd_vel_deadman exists as a separate process, and why it should be running whenever
anything is driving.
"""
import argparse
import math
import os
import signal
import sys
import threading
import time


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--speed', type=float, default=0.18)
    ap.add_argument('--turn', type=float, default=0.6, help='rad/s')
    ap.add_argument('--side', type=float, default=1.6, help='metres per leg')
    ap.add_argument('--laps', type=int, default=0, help='0 = forever')
    ap.add_argument('--domain', type=int, default=66)
    ap.add_argument('--direct', action='store_true',
                    help='publish /cmd_vel, bypassing the governor')
    args = ap.parse_args()
    os.environ.setdefault('ROS_DOMAIN_ID', str(args.domain))

    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from geometry_msgs.msg import Twist

    rclpy.init()
    node = Node('sim_patrol')
    topic = '/cmd_vel' if args.direct else '/cmd_vel_raw'
    pub = node.create_publisher(Twist, topic, 10)
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()
    # Discovery is not instant, and a single look 2 s in reported "no simulator" while
    # the simulator was demonstrably running. Poll instead of guessing a sleep.
    names = []
    deadline = time.time() + 15.0
    while time.time() < deadline:
        time.sleep(1.0)
        names = [n for n, _ in node.get_node_names_and_namespaces()]
        if 'fake_robot' in names or 'YB_Car_Node' in names:
            break

    if 'YB_Car_Node' in names and 'fake_robot' not in names:
        print('REFUSED: this is the REAL robot. A patrol loop must never drive hardware '
              'unattended -- the firmware has no command watchdog, so if this process '
              'dies mid-leg the car keeps going.')
        return 2
    if 'fake_robot' not in names:
        print('No simulator found on this domain. Start it with:')
        print('  ros2 launch yahboomcar_sim sim_bringup_launch.py')
        return 2

    print(f'patrolling: {args.side} m legs at {args.speed} m/s, publishing {topic}')
    print('Ctrl+C to stop; the robot is left stationary.')

    def drive(vx, wz, seconds):
        t = Twist()
        t.linear.x, t.angular.z = vx, wz
        end = time.time() + seconds
        while time.time() < end and not stopping['now']:
            pub.publish(t)
            time.sleep(0.05)

    stopping = {'now': False}

    def on_signal(signum, _frame):
        # SIGTERM (pkill) would otherwise kill this process outright, leaving the last
        # command latched forever on a robot with no watchdog.
        print(f'\n  signal {signum}: stopping the robot before exiting', flush=True)
        stopping['now'] = True

    signal.signal(signal.SIGTERM, on_signal)
    signal.signal(signal.SIGINT, on_signal)

    lap = 0
    try:
        while (args.laps == 0 or lap < args.laps) and not stopping['now']:
            lap += 1
            for leg in range(4):
                if stopping['now']:
                    break
                print(f'  lap {lap}, leg {leg + 1}/4', flush=True)
                drive(args.speed, 0.0, args.side / args.speed)
                drive(0.0, args.turn, (math.pi / 2) / args.turn)
    except KeyboardInterrupt:
        print('\n  stopping')
    finally:
        # The firmware retains the last command forever, and so does the simulator,
        # because it models that. Leaving without zeroing would leave it driving.
        for _ in range(20):
            pub.publish(Twist())
            time.sleep(0.04)
        print('  stopped')
    return 0


if __name__ == '__main__':
    code = main()
    sys.stdout.flush()
    os._exit(code)
