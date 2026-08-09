#!/usr/bin/env python3
"""Drive the simulated robot around its arena, so there is something to watch.

    ./tools/sim_patrol.py                    # loop until stopped
    ./tools/sim_patrol.py --laps 2 --speed 0.15

For demonstrations and for exercising SLAM. It publishes /cmd_vel_raw by default, so the
safety governor stays in the loop exactly as it would on the floor -- pass --direct only
if no governor is running.

REFUSES TO DRIVE REAL HARDWARE, via the shared guard in tools/_cmd_vel_safety.py: an
unattended patrol loop is the last thing that should ever reach a real robot whose
firmware has no command watchdog.

IT IS OPEN LOOP, AND ON ISAAC THAT SHOWS
----------------------------------------
The legs are timed, not navigated -- this drives a square by dead reckoning and has no
idea where it is. On the 2D simulator that is harmless because there are NO COLLISIONS
at all. Isaac has real contact physics, so a dead-reckoned leg can run into a wall or
one of the corner boxes (the shared layout in yahboomcar_sim.arena -- four 0.3 m boxes
by the corners since 2026-08-08) and pin the robot: wheels turning, body still, the
obstacle closer than the lidar's 0.12 m range_min so it cannot even be seen.

So there is a stuck detector, on GROUND TRUTH rather than odometry. Odometry is
encoder-derived and reports a robot wedged against a wall as travelling normally --
detecting "stuck" from the wheels is precisely what wheels cannot do.

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

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _cmd_vel_safety import require_simulator                   # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--speed', type=float, default=0.18)
    ap.add_argument('--turn', type=float, default=0.6, help='rad/s')
    # 1.0, not 1.6. The arena is 4x4 m with boxes at (1.55, 0), (0, 1.55) and the four
    # corners, so the clear region is roughly |x| < 1.3, |y| < 1.2. A 1.6 m leg from the
    # origin drives straight into the box at (1.55, 0) on the FIRST leg -- which the 2D
    # simulator hid completely, having no collisions, and Isaac does not.
    ap.add_argument('--side', type=float, default=1.0, help='metres per leg')
    ap.add_argument('--laps', type=int, default=0, help='0 = forever')
    ap.add_argument('--domain', type=int, default=66)
    ap.add_argument('--direct', action='store_true',
                    help='publish /cmd_vel, bypassing the governor')
    ap.add_argument('--stuck-seconds', type=float, default=3.0,
                    help='how long to allow no progress before calling it blocked')
    ap.add_argument('--stuck-metres', type=float, default=0.03,
                    help='movement below this over --stuck-seconds counts as blocked')
    args = ap.parse_args()
    os.environ.setdefault('ROS_DOMAIN_ID', str(args.domain))

    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import LaserScan
    from rclpy.qos import qos_profile_sensor_data

    rclpy.init()
    node = Node('sim_patrol')
    topic = '/cmd_vel' if args.direct else '/cmd_vel_raw'
    pub = node.create_publisher(Twist, topic, 10)
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()
    # THE SHARED GUARD, not a private copy. This file is where the pattern was
    # extracted from -- and then it kept its own version, which went stale: it looked
    # for a node named `fake_robot`, and the ISAAC backend advertises NO ROS NODES AT
    # ALL (isaacsim.ros2.bridge's OmniGraph publishes without creating discoverable
    # named nodes). So `simctl start --backend isaac` came up with the patrol silently
    # refused -- "No simulator found on this domain" -- and the robot sat at the origin
    # while everything else looked healthy.
    #
    # require_simulator() recognises a simulator two ways, including /sim/ground_truth,
    # which the firmware cannot publish. One implementation, so a fix reaches every
    # caller.
    require_simulator(node, what='sim_patrol.py')

    print(f'patrolling: {args.side} m legs at {args.speed} m/s, publishing {topic}')
    print('Ctrl+C to stop; the robot is left stationary.')

    # STUCK DETECTION, and the reason it is needed at all.
    #
    # This patrol is OPEN LOOP: it drives timed legs and has no idea where it is. That
    # was harmless on the 2D simulator, which has NO COLLISIONS -- the robot simply
    # passes through walls and boxes (which is how one ended up 12 m outside a 4 m
    # room). Isaac has real contact physics, so the same open-loop path pins the robot
    # against the first thing it meets and the demo appears frozen.
    #
    # Ground truth is used deliberately. This is a SIMULATOR-ONLY tool -- it refuses
    # hardware -- and /sim/ground_truth is what identifies a simulator in the first
    # place. Odometry could not do this job: it is encoder-derived, so a robot with its
    # wheels spinning against a wall reports that it is travelling happily. Detecting
    # "stuck" from the wheels is exactly the thing wheels cannot do.
    truth = {'xy': None, 'seen': False}

    def _on_truth(m):
        truth['xy'] = (m.pose.pose.position.x, m.pose.pose.position.y)
        truth['seen'] = True

    node.create_subscription(Odometry, '/sim/ground_truth', _on_truth, 10)

    scan = {'msg': None}
    node.create_subscription(LaserScan, '/scan',
                             lambda m: scan.__setitem__('msg', m),
                             qos_profile_sensor_data)

    def _clearest_heading():
        """-> relative bearing (rad) of the widest open direction, or None.

        Averages range over coarse sectors and picks the best. Crude on purpose: this
        only has to beat 'turn 90 degrees into the same wall'.
        """
        m = scan['msg']
        if m is None:
            return None
        sectors = {}
        for i, r in enumerate(m.ranges):
            if not math.isfinite(r) or r <= 0:
                continue
            bearing = m.angle_min + i * m.angle_increment
            if abs(bearing) > math.pi * 0.75:
                continue                      # ignore straight behind
            key = round(math.degrees(bearing) / 30.0)
            sectors.setdefault(key, []).append(r)
        if not sectors:
            return None
        best = max(sectors, key=lambda k: sum(sectors[k]) / len(sectors[k]))
        return math.radians(best * 30.0)

    def drive(vx, wz, seconds):
        """-> True if it ran to completion, False if it gave up because nothing moved."""
        t = Twist()
        t.linear.x, t.angular.z = vx, wz
        end = time.time() + seconds
        mark, mark_t = truth['xy'], time.time()
        while time.time() < end and not stopping['now']:
            pub.publish(t)
            time.sleep(0.05)
            # Only forward legs are checked; a rotation legitimately holds position.
            if vx == 0 or not truth['seen'] or truth['xy'] is None:
                continue
            if mark is None:
                # Ground truth had not arrived when this leg began, so there is no
                # reference to measure against yet. Seed it now and start the clock
                # from here rather than subscripting a None -- which is exactly what
                # this did, crashing the patrol on its first leg.
                mark, mark_t = truth['xy'], time.time()
                continue
            if time.time() - mark_t >= args.stuck_seconds:
                moved = math.hypot(truth['xy'][0] - mark[0], truth['xy'][1] - mark[1])
                if moved < args.stuck_metres:
                    print(f'  BLOCKED: commanded {vx:.2f} m/s but moved {moved*1000:.0f} '
                          f'mm in {args.stuck_seconds:.0f} s', flush=True)
                    return False
                mark, mark_t = truth['xy'], time.time()
        return True

    def unstick():
        """Back off, then turn toward whichever direction the lidar says is clearest.

        A FIXED 90 degree turn is not enough, and that is not a guess -- it was measured.
        Backing off ~0.15 m (the governor caps reverse at 0.10 m/s) and turning a fixed
        quarter turn put the robot straight back into the same wall: it oscillated inside
        a 0.14 x 0.24 m box for a full minute, blocked-recover-blocked, never escaping the
        corner. Open-loop recovery from an open-loop failure just repeats it.

        So the scan picks the heading. This is still not navigation -- it is a demo
        driver -- but it uses the one sensor that knows where the space is.
        """
        print('  recovering: reversing, then turning toward open space', flush=True)
        drive(-args.speed, 0.0, 3.0)
        best = _clearest_heading()
        if best is None:
            drive(0.0, args.turn, (math.pi / 2) / args.turn)     # no scan; fall back
            return
        # Turn in the cheaper direction, at most a half turn either way.
        secs = abs(best) / args.turn
        print(f'    clearest heading {math.degrees(best):+.0f} deg; turning {secs:.1f} s',
              flush=True)
        drive(0.0, math.copysign(args.turn, best), secs)

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
                if not drive(args.speed, 0.0, args.side / args.speed):
                    unstick()
                    continue
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
