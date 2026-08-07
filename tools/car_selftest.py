#!/usr/bin/env python3
"""One command that exercises the car and reports what it measured.

    ./tools/car_selftest.py                  # sensors + gentle wheel/servo motion
    ./tools/car_selftest.py --sensors-only   # no motion at all
    ./tools/car_selftest.py --handspin       # odometry-truth test, see below

Prints a PASS/FAIL table of MEASURED values against the firmware contract, records
everything to a timestamped bag, writes the same table to a log, and exits non-zero on
failure so it can gate other work.

It reports numbers rather than verdicts alone, deliberately: an earlier claim in this
project ("the motors move") was an inference from /odom_raw that was never checked
against the physical robot. You should be able to disagree with the interpretation
without re-running anything.

SAFETY: with --motion (the default) this commands real wheel movement. Run it with the
car ELEVATED unless the safety governor is active and you have floor space.

--handspin settles a question this project has not answered: whether /odom_raw reflects
encoders or is computed open-loop from cmd_vel. It commands nothing and asks you to turn
a wheel by hand. If /odom_raw responds, feedback is real; if it stays flat, then odometry
only echoes intent, and calibration, mapping and the twin's wheel animation all rest on a
number that never observed the world.
"""
import argparse
import math
import os
import subprocess
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _cmd_vel_safety import install_stop_handlers            # noqa: E402
from datetime import datetime

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
BAG_DIR = os.path.join(REPO, 'MicroROS-assets', 'bags')
LOG_DIR = os.path.join(REPO, 'MicroROS-assets', 'logs')

# The firmware's published contract (ROS node topic information.pdf), with the
# tolerance we accept. Rates are Hz.
CONTRACT = [
    ('/scan',     'sensor_msgs/msg/LaserScan', 12.0, 3.0),
    ('/imu',      'sensor_msgs/msg/Imu',       25.0, 6.0),
    ('/odom_raw', 'nav_msgs/msg/Odometry',     11.0, 3.0),
    ('/battery',  'std_msgs/msg/UInt16',        1.0, 0.5),
]
BAG_TOPICS = ['/scan', '/imu', '/imu/data', '/odom_raw', '/odom', '/battery',
              '/cmd_vel', '/servo_s1', '/servo_s2', '/tf', '/tf_static', '/joint_states']

MIN_BATTERY_V = 7.0     # below this, refuse to command motion


class SystemExit_NoData(Exception):
    """Raised when nothing was received, so no conclusion may be drawn."""


class Results:
    def __init__(self):
        self.rows = []
        self.failed = False

    def add(self, name, measured, expected, ok, note=''):
        self.rows.append((name, str(measured), str(expected), bool(ok), note))
        if not ok:
            self.failed = True

    def render(self):
        w = max(len(r[0]) for r in self.rows) + 2
        out = [f'{"check":<{w}}{"measured":>14}{"expected":>14}   {"":4} notes',
               '-' * (w + 40)]
        for name, meas, exp, ok, note in self.rows:
            out.append(f'{name:<{w}}{meas:>14}{exp:>14}   '
                       f'{"PASS" if ok else "FAIL"} {note}')
        out.append('')
        out.append('RESULT: ' + ('FAIL' if self.failed else 'PASS'))
        return '\n'.join(out)


def agent_running():
    try:
        out = subprocess.run(['docker', 'ps', '--filter',
                              'ancestor=microros/micro-ros-agent:jazzy',
                              '--format', '{{.Names}}'],
                             capture_output=True, text=True, timeout=15)
        return [n for n in out.stdout.split() if n]
    except Exception:
        return []


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--sensors-only', action='store_true', help='no motion')
    ap.add_argument('--handspin', action='store_true',
                    help='odometry-truth test: you turn a wheel, nothing is commanded')
    ap.add_argument('--speed', type=float, default=0.12, help='m/s for the motion test')
    ap.add_argument('--duration', type=float, default=8.0,
                    help='seconds to sample each topic set')
    ap.add_argument('--no-bag', action='store_true')
    ap.add_argument('--direct', action='store_true',
                    help='publish straight to /cmd_vel, bypassing the safety governor. '
                         'Only for when no governor is running and the car is elevated.')
    ap.add_argument('--domain', type=int, default=20,
                    help='ROS_DOMAIN_ID the board is configured for (default 20). '
                         'Must match, or nothing is received and every check reads 0 Hz.')
    ap.add_argument('--servos', action='store_true',
                    help='also command the 2-DOF gimbal. Off by default: the Standard '
                         'chassis has no servos, and commanding absent hardware proves '
                         'nothing.')
    args = ap.parse_args()

    # Set the domain BEFORE rclpy.init, or we silently join the wrong graph. An earlier
    # run with ROS_DOMAIN_ID unset received zero messages and still printed a confident
    # conclusion about the encoders -- see the no-data gate below.
    env_domain = os.environ.get('ROS_DOMAIN_ID')
    if env_domain is None:
        os.environ['ROS_DOMAIN_ID'] = str(args.domain)
    elif int(env_domain) != args.domain:
        print(f'NOTE: ROS_DOMAIN_ID={env_domain} in the environment, but --domain='
              f'{args.domain}. Using {env_domain}; pass --domain to override.')
        args.domain = int(env_domain)

    import threading

    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Imu, LaserScan
    from std_msgs.msg import Int32, UInt16

    res = Results()
    stamp = datetime.now().strftime('%Y%m%d-%H%M%S')
    os.makedirs(LOG_DIR, exist_ok=True)
    log_path = os.path.join(LOG_DIR, f'selftest-{stamp}.log')
    transcript = []

    def say(m=''):
        print(m, flush=True)
        transcript.append(str(m))

    say(f'car self-test  {stamp}')
    say(f'ROS_DOMAIN_ID={os.environ.get("ROS_DOMAIN_ID")} (board must match)')
    say(f'command topic: {"/cmd_vel (DIRECT, governor bypassed)" if args.direct else "/cmd_vel_raw (via governor)"}')

    # --- agent -------------------------------------------------------------
    names = agent_running()
    if names:
        say(f'micro-ROS agent already running: {names[0]} (reusing, not starting a second)')
        res.add('agent', 'running', 'running', True, names[0])
    else:
        say('no micro-ROS agent running. Start one, then re-run:')
        say('  docker run -it --rm -v /dev:/dev -v /dev/shm:/dev/shm --privileged \\')
        say('    --net=host microros/micro-ros-agent:jazzy udp4 --port 8090 -v4')
        res.add('agent', 'absent', 'running', False, 'start it first')
        say(res.render())
        with open(log_path, 'w') as f:
            f.write('\n'.join(transcript) + '\n')
        return 1

    rclpy.init()
    node = Node('car_selftest')
    # Spin on a dedicated thread. A main-loop rclpy.spin_once() handles ONE callback
    # per call, so sampling four topics totalling ~49 msg/s from a ~50 Hz loop silently
    # drops messages and under-reports every rate by roughly the same factor -- which
    # looks exactly like a robot fault. Measured rates must not depend on our loop.
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    counts = {t: 0 for t, _, _, _ in CONTRACT}
    last = {}
    scan_meta = {}

    def mk(topic, msgtype):
        def cb(msg):
            counts[topic] += 1
            last[topic] = msg
            if topic == '/scan' and not scan_meta:
                scan_meta['n'] = len(msg.ranges)
                scan_meta['frame'] = msg.header.frame_id
                scan_meta['span_deg'] = round(math.degrees(msg.angle_max - msg.angle_min))
        # Sensor-data QoS (BEST_EFFORT). The firmware publishes /imu -- and the other
        # sensor streams -- BEST_EFFORT, which is incompatible with a default RELIABLE
        # subscription: messages are dropped silently and every rate reads low, which
        # looks like a degraded robot rather than a subscriber misconfiguration.
        return node.create_subscription(msgtype, topic, cb, qos_profile_sensor_data)

    mk('/scan', LaserScan); mk('/imu', Imu)
    mk('/odom_raw', Odometry); mk('/battery', UInt16)

    # --- optional recording ------------------------------------------------
    bag_proc = None
    bag_path = os.path.join(BAG_DIR, f'selftest-{stamp}')
    if not args.no_bag:
        os.makedirs(BAG_DIR, exist_ok=True)
        bag_proc = subprocess.Popen(
            ['ros2', 'bag', 'record', '-o', bag_path] + BAG_TOPICS,
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        say(f'recording -> {bag_path}')

    # Drive through the governor by default. This tool commands real wheel motion, so
    # it is exactly the path that should be limited by the lidar rather than exempt
    # from it. --direct exists for the bench, where no governor may be running.
    cmd_topic = '/cmd_vel' if args.direct else '/cmd_vel_raw'
    cmd_pub = node.create_publisher(Twist, cmd_topic, 10)
    # `finally` alone does not survive SIGTERM (pkill).
    install_stop_handlers(cmd_pub)
    s1 = node.create_publisher(Int32, '/servo_s1', 10)
    s2 = node.create_publisher(Int32, '/servo_s2', 10)

    spin_thread.start()

    def spin(seconds):
        time.sleep(seconds)

    def stop_wheels():
        t = Twist()
        for _ in range(12):
            cmd_pub.publish(t)
            spin(0.03)

    try:
        # --- sensors -------------------------------------------------------
        say(f'\nsampling sensors for {args.duration:.0f}s ...')
        for k in counts:
            counts[k] = 0
        spin(args.duration)
        for topic, _type, expected, tol in CONTRACT:
            hz = counts[topic] / args.duration
            ok = abs(hz - expected) <= tol and counts[topic] > 0
            res.add(topic, f'{hz:.2f} Hz', f'{expected:.0f}+/-{tol:.0f}', ok)

        # HARD GATE. Every conclusion below assumes we are actually receiving from the
        # robot. A previous version reported "odometry is probably open-loop" after
        # receiving zero messages of any kind, because the domain id did not match.
        # No data means no verdict.
        total = sum(counts.values())
        if total == 0:
            say('\n*** RECEIVED NOTHING FROM THE ROBOT ***')
            say(f'  Zero messages on all {len(counts)} topics over {args.duration:.0f}s.')
            say('  This says nothing about the robot. Likely causes, in order:')
            say(f'    1. domain mismatch  - trying {os.environ.get("ROS_DOMAIN_ID")}; '
                'check the board with tools/provision_board.py --dry-run')
            say('    2. car powered off, or needs a reset press to rejoin the agent')
            say('    3. agent not bridging - docker logs uros-udp')
            say('  No further checks will run, and no conclusions drawn.')
            res.add('data received', '0 messages', '> 0', False, 'aborting; see above')
            raise SystemExit_NoData()

        if scan_meta:
            say(f'\nlidar: {scan_meta.get("n")} points, {scan_meta.get("span_deg")} deg span, '
                f'frame={scan_meta.get("frame")}')
            res.add('scan points', scan_meta.get('n'), '360', scan_meta.get('n', 0) >= 180)

        volts = None
        if '/battery' in last:
            volts = last['/battery'].data / 10.0
            ok = volts >= MIN_BATTERY_V
            res.add('battery', f'{volts:.1f} V', f'>= {MIN_BATTERY_V}', ok,
                    '' if ok else 'too low for motion')

        # --- odometry truth ------------------------------------------------
        if args.handspin:
            say('\n=== HAND-SPIN TEST (nothing is commanded) ===')
            say('Turn ONE wheel by hand, steadily, for the next 12 seconds.')
            say('Watching /odom_raw ...')
            base = last.get('/odom_raw')
            base_x = base.pose.pose.position.x if base else 0.0
            peak = 0.0
            t0 = time.time()
            while time.time() - t0 < 12.0:
                time.sleep(0.02)
                m = last.get('/odom_raw')
                if m:
                    peak = max(peak, abs(m.twist.twist.linear.x),
                               abs(m.twist.twist.angular.z))
            end = last.get('/odom_raw')
            moved_pose = abs((end.pose.pose.position.x if end else 0.0) - base_x)
            say(f'  peak |twist| while you turned it : {peak:.4f}')
            say(f'  pose x change                    : {moved_pose:.4f} m')
            real = peak > 0.01 or moved_pose > 0.005
            res.add('encoders real', 'yes' if real else 'NO RESPONSE',
                    'responds to hand motion', real,
                    '' if real else 'odometry may be open-loop from cmd_vel')
            say('  => encoder feedback looks REAL' if real else
                '  => NO response: odometry is probably open-loop. Calibration, mapping\n'
                '     and the twin all rest on this; treat their results as unproven.')

        # --- motion --------------------------------------------------------
        if not args.sensors_only and not args.handspin:
            if volts is not None and volts < MIN_BATTERY_V:
                say(f'\nskipping motion: battery {volts:.1f} V below {MIN_BATTERY_V} V')
                res.add('motion', 'skipped', 'run', False, 'battery too low')
            else:
                say(f'\n=== MOTION (car should be ELEVATED) speed={args.speed} m/s ===')
                for label, vx, wz in (('forward', args.speed, 0.0),
                                      ('backward', -args.speed, 0.0),
                                      ('rotate', 0.0, 0.6)):
                    before = last.get('/odom_raw')
                    b = abs(before.twist.twist.linear.x) if before else 0.0
                    say(f'  {label} ...')
                    t = Twist(); t.linear.x = float(vx); t.angular.z = float(wz)
                    peak_v = peak_w = 0.0
                    t0 = time.time()
                    while time.time() - t0 < 3.0:
                        cmd_pub.publish(t)
                        time.sleep(0.02)
                        m = last.get('/odom_raw')
                        if m:
                            peak_v = max(peak_v, abs(m.twist.twist.linear.x))
                            peak_w = max(peak_w, abs(m.twist.twist.angular.z))
                    stop_wheels(); spin(0.8)
                    if wz:
                        ok = peak_w > 0.1
                        res.add(f'motion {label}', f'wz={peak_w:.3f}', 'wz > 0.1', ok)
                    else:
                        ok = peak_v > abs(vx) * 0.4
                        res.add(f'motion {label}', f'vx={peak_v:.3f}',
                                f'vx > {abs(vx)*0.4:.2f}', ok)

                if args.servos:
                    say('  servos ...')
                    for ang in (-40, 0, 40, 0):
                        s1.publish(Int32(data=ang)); spin(0.5)
                    for ang in (-40, 0, 20, 0):
                        s2.publish(Int32(data=ang)); spin(0.5)
                    res.add('servos', 'commanded', 'commanded', True,
                            'watch the gimbal yourself; not observable from ROS')
                else:
                    say('  servos: skipped (--servos to command them)')

                say('\nNOTE: these motion rows say the ROBOT REPORTED motion on /odom_raw.')
                say('      They do NOT prove the wheels physically turned. Use --handspin,')
                say('      and watch the wheels yourself.')
    except SystemExit_NoData:
        pass
    finally:
        # Order matters: stop the robot while the node is still usable, then tear the
        # executor down, or rclpy raises "cannot use Destroyable" on the way out.
        try:
            stop_wheels()
        except Exception:
            pass
        try:
            executor.remove_node(node)
            executor.shutdown()
        except Exception:
            pass
        if bag_proc:
            bag_proc.send_signal(2)
            try:
                bag_proc.wait(timeout=12)
            except Exception:
                bag_proc.kill()
            say(f'\nbag written: {bag_path}')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    say()
    say(res.render())
    with open(log_path, 'w') as f:
        f.write('\n'.join(transcript) + '\n')
    print(f'\nlog: {log_path}')
    return 1 if res.failed else 0


if __name__ == '__main__':
    sys.exit(main())
