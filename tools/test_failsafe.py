#!/usr/bin/env python3
"""Does the car stop when its command path DIES, rather than shuts down cleanly?

    ./tools/test_failsafe.py                    # cases 1 and 2 -- safe, repeatable
    ./tools/test_failsafe.py --test-agent-loss  # case 3 -- WEDGES THE BOARD, see below
    ./tools/test_failsafe.py --negative-test    # prove this tool can FAIL

SAFETY: the wheels spin. Run with the car ELEVATED.

WHY THIS IS THE GATE
--------------------
The governor publishes a zero twist from its `finally` block, which covers Ctrl+C and
nothing else. SIGKILL, a segfault, an OOM kill, the agent dying, or the Wi-Fi dropping
all skip that path entirely. geometry_msgs/Twist carries no timestamp and no expiry, so
nothing in the message says "this command is stale" -- the FIRMWARE has to expire
retained commands on its own.

Yahboom ships no firmware source. So this has never been demonstrated, and until it is,
every floor test is riding on an assumption. If the firmware latches the last command,
any crash of any component is a runaway car and no amount of governor work fixes it.

WHAT EACH CASE ACTUALLY PROVES
------------------------------
  1. commands stop arriving   -- the fundamental question. Agent stays up, so /odom_raw
                                 keeps flowing and the stop is directly observable.
  2. governor SIGKILLed       -- same thing from the firmware's view, but proves the
                                 whole chain, including that no zero escapes on the way
                                 down. Killed by process GROUP: `ros2 run` spawns a
                                 child, so killing the parent alone proves nothing.
  3. agent frozen             -- OPT-IN, and destructive. Freezing the agent mid-stream
                                 corrupts the XRCE session: the agent logs
                                 "deserialization error processing WRITE_DATA
                                 submessage", the board's client wedges and does NOT
                                 re-register, and recovery needs the agent restarted AND
                                 the CAR POWER-CYCLED BY HAND. Learned the hard way.
                                 The tool restarts the agent for you; it cannot power-
                                 cycle the car. The finding is already decisive from the
                                 2026-08-06 run, so there is little reason to repeat it.

                                 Awkward for a second reason: the observation channel
                                 dies with the control channel. Handled by a reconnect
                                 probe -- freeze, wait, thaw, sample immediately.
                                 NON-ZERO on reconnect is decisive proof of latching.
                                 ZERO is AMBIGUOUS (watchdog, or a reset on session
                                 re-establishment), so that branch asks you to watch the
                                 wheels and records the answer as human-observed rather
                                 than claiming it automatically.
  4. Wi-Fi loss               -- the same board-side event as 3 (the XRCE session drops),
                                 so 3 covers it. Not separately faked.

RESOLUTION
----------
/odom_raw arrives at ~11 Hz, so a stop cannot be located more precisely than one
interval. Times are reported as BRACKETS, same as tools/measure_latency.py: last sample
still moving = lower bound, first sample at rest = upper bound. Take the upper bound.
"""
import argparse
import json
import os
import signal
import subprocess
import sys
import threading
import time
from datetime import datetime

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT_DIR = os.path.join(REPO, 'MicroROS-assets', 'logs')
REPORT = os.path.join(REPO, 'yahboomcar_ws', 'src', 'yahboomcar_safety',
                      'failsafe_report.json')
WS = os.path.join(REPO, 'yahboomcar_ws')

MOVING = 0.05      # m/s: above this the wheels are definitely turning
AT_REST = 0.02     # m/s: below this, sustained, they are stopped
N_REST = 3         # consecutive at-rest samples before believing it


class Monitor:
    """Collects /odom_raw with a SingleThreadedExecutor.

    Single-threaded deliberately: a MultiThreadedExecutor appends receive timestamps out
    of order across threads, which silently corrupts every interval computed from them.
    That bug produced a 0.6 ms median and a 2.67 s maximum in tools/measure_latency.py
    before `ros2 topic hz` caught it.
    """

    def __init__(self, node, Odometry, qos):
        self.samples = []          # (host_time, vx, wz)
        node.create_subscription(Odometry, '/odom_raw', self._cb, qos)

    def _cb(self, msg):
        self.samples.append((time.time(), msg.twist.twist.linear.x,
                             msg.twist.twist.angular.z))

    def since(self, t):
        return [s for s in self.samples if s[0] >= t]

    def is_moving(self, window=1.0):
        recent = self.since(time.time() - window)
        return any(abs(vx) > MOVING or abs(wz) > MOVING for _, vx, wz in recent)


def find_rest(samples, t_ref, timeout):
    """First sustained at-rest run after t_ref. Returns (lo, hi) bracket or None.

    Requires N_REST consecutive at-rest samples so a single dropout or a zero-crossing
    during deceleration is not mistaken for a stop.
    """
    after = [s for s in samples if s[0] >= t_ref]
    for i in range(len(after) - N_REST + 1):
        run = after[i:i + N_REST]
        if all(abs(vx) <= AT_REST and abs(wz) <= AT_REST for _, vx, wz in run):
            hi = run[0][0] - t_ref
            if hi > timeout:
                return None
            prev = [s[0] for s in after[:i] if abs(s[1]) > AT_REST or abs(s[2]) > AT_REST]
            lo = (max(prev) - t_ref) if prev else 0.0
            return (max(0.0, lo), hi)
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--speed', type=float, default=0.15)
    ap.add_argument('--drive-seconds', type=float, default=2.0)
    ap.add_argument('--stop-timeout', type=float, default=3.0,
                    help='a stop slower than this is treated as a failure')
    ap.add_argument('--outage-seconds', type=float, default=6.0,
                    help='how long the agent stays down in case 3')
    ap.add_argument('--domain', type=int, default=20)
    ap.add_argument('--container', default='uros-udp')
    ap.add_argument('--test-agent-loss', action='store_true',
                    help='run case 3. DESTRUCTIVE: wedges the board\'s XRCE client. '
                         'You will have to power-cycle the car afterwards.')
    ap.add_argument('--negative-test', action='store_true',
                    help='keep commanding motion and assert this tool reports FAILURE. '
                         'Proves the detector cannot false-positive a stop.')
    args = ap.parse_args()

    if os.environ.get('ROS_DOMAIN_ID') is None:
        os.environ['ROS_DOMAIN_ID'] = str(args.domain)

    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry

    lines = []

    def say(m=''):
        print(m, flush=True)
        lines.append(str(m))

    rclpy.init()
    node = Node('test_failsafe')
    mon = Monitor(node, Odometry, qos_profile_sensor_data)
    cmd = node.create_publisher(Twist, '/cmd_vel', 10)
    cmd_raw = node.create_publisher(Twist, '/cmd_vel_raw', 10)

    ex = SingleThreadedExecutor()
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()

    def hard_stop(reps=15):
        """Always leave the car stopped, whatever the test proved."""
        for _ in range(reps):
            cmd.publish(Twist())
            time.sleep(0.04)

    def drive(pub, seconds):
        t = Twist()
        t.linear.x = float(args.speed)
        end = time.time() + seconds
        while time.time() < end:
            pub.publish(t)
            time.sleep(0.05)

    stamp = datetime.now().strftime('%Y%m%d-%H%M%S')
    say(f'fail-safe test  {stamp}')
    say(f'ROS_DOMAIN_ID={os.environ["ROS_DOMAIN_ID"]}   speed={args.speed} m/s')
    say('CAR MUST BE ELEVATED -- the wheels will spin.')
    say('')

    time.sleep(2.0)
    if not mon.samples:
        say('FAIL: no /odom_raw. Is the car powered and on this domain?')
        return 2

    results = {'timestamp': stamp, 'speed_m_s': args.speed,
               'stop_timeout_s': args.stop_timeout, 'cases': {}}

    def record(key, title, stopped, bracket, detail='', observed='automatic'):
        results['cases'][key] = {
            'title': title, 'stopped': stopped,
            'time_to_stop_lo_s': bracket[0] if bracket else None,
            'time_to_stop_hi_s': bracket[1] if bracket else None,
            'observation': observed, 'detail': detail,
        }
        if stopped:
            say(f'  PASS: stopped in [{bracket[0]*1000:.0f}, {bracket[1]*1000:.0f}] ms')
        else:
            say(f'  FAIL: still moving after {args.stop_timeout:.1f} s -- {detail}')

    # ------------------------------------------------- 1. commands stop arriving
    say('=== case 1: commands stop arriving (no zeros sent) ===')
    drive(cmd, args.drive_seconds)
    if not mon.is_moving():
        say('  INVALID: wheels never started. Cannot prove a stop that never began.')
        hard_stop()
        results['cases']['command_cessation'] = {'stopped': None,
                                                 'detail': 'motion never started'}
    else:
        if args.negative_test:
            # Negative control: keep commanding. A correct detector must report FAILURE.
            say('  [negative test] still commanding -- expecting this to FAIL')
            t_ref = time.time()
            end = t_ref + args.stop_timeout + 1.0
            while time.time() < end:
                t = Twist(); t.linear.x = float(args.speed)
                cmd.publish(t); time.sleep(0.05)
            br = find_rest(mon.samples, t_ref, args.stop_timeout)
            hard_stop()
            if br is None:
                say('  NEGATIVE TEST PASSED: detector correctly reported no stop.')
                results['negative_test'] = 'passed'
            else:
                say(f'  NEGATIVE TEST FAILED: detector claimed a stop at {br} '
                    'while the car was still being commanded. Do not trust this tool.')
                results['negative_test'] = 'FAILED'
            with open(REPORT, 'w') as f:
                json.dump(results, f, indent=2)
            return 0 if results['negative_test'] == 'passed' else 1

        t_ref = time.time()          # last command published; now we go silent
        say(f'  commands ceased at t=0; watching /odom_raw for {args.stop_timeout} s')
        time.sleep(args.stop_timeout + 0.5)
        br = find_rest(mon.samples, t_ref, args.stop_timeout)
        record('command_cessation', 'commands stop arriving', br is not None, br,
               '' if br else 'firmware appears to LATCH the last command')
        hard_stop()

    # ------------------------------------------------------ 2. governor SIGKILL
    say('')
    say('=== case 2: governor SIGKILLed mid-motion ===')
    gov_env = dict(os.environ)
    # Permissive distances on purpose: this case tests the CRASH path, not the obstacle
    # logic, and the car is elevated under a desk where the lidar sees walls at < 0.35 m.
    setup = os.path.join(WS, 'install', 'setup.bash')
    gov_cmd = ('source /opt/ros/jazzy/setup.bash && '
               f'source {setup} && '
               'exec ros2 run yahboomcar_safety cmd_vel_governor --ros-args '
               '-p stop_distance:=0.01 -p slow_distance:=0.02 -p max_speed:=0.5')
    proc = subprocess.Popen(['bash', '-c', gov_cmd], env=gov_env,
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                            start_new_session=True)      # own process group
    time.sleep(4.0)
    if proc.poll() is not None:
        say('  INVALID: governor exited before the test started.')
        results['cases']['governor_sigkill'] = {'stopped': None,
                                                'detail': 'governor failed to start'}
    else:
        drive(cmd_raw, args.drive_seconds)
        if not mon.is_moving():
            say('  INVALID: governor passed no motion through (obstacle? stale scan?).')
            results['cases']['governor_sigkill'] = {'stopped': None,
                                                    'detail': 'no motion through governor'}
        else:
            t_ref = time.time()
            # Kill the GROUP. `ros2 run` spawns the node as a child, so killing only the
            # parent would leave the real publisher alive and prove nothing.
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            say('  SIGKILL sent to the governor process group at t=0')
            time.sleep(args.stop_timeout + 0.5)
            br = find_rest(mon.samples, t_ref, args.stop_timeout)
            record('governor_sigkill', 'governor SIGKILLed', br is not None, br,
                   '' if br else 'no zero escaped, and the firmware did not expire it')
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        except (ProcessLookupError, PermissionError):
            pass
    hard_stop()

    # -------------------------------------------------------- 3. agent stopped
    if args.test_agent_loss:
        say('')
        say('=== case 3: micro-ROS agent stopped (also covers Wi-Fi loss) ===')
        say(f'  WATCH THE WHEELS. The agent goes down for {args.outage_seconds:.0f} s;')
        say('  you are the only witness while it is down.')
        try:
            input('  Press Enter when you are watching... ')
        except EOFError:
            pass
        drive(cmd, args.drive_seconds)
        moving_before = mon.is_moving()
        if not moving_before:
            say('  INVALID: wheels never started.')
            results['cases']['agent_loss'] = {'stopped': None,
                                              'detail': 'motion never started'}
        else:
            # PAUSE, not stop. The agent runs with --rm (AutoRemove=true), so
            # `docker stop` DELETES the container and `docker start` then fails --
            # it would destroy the user's agent to run a test. Pause freezes the
            # process via the cgroup freezer and unpause resumes it.
            #
            # It is also the better simulation: paused, the UDP socket stays open and
            # packets are silently dropped, which is what Wi-Fi loss looks like to the
            # board. `stop` closes the socket, producing ICMP port-unreachable -- an
            # explicit signal the board would never get from a dead radio.
            subprocess.run(['docker', 'pause', args.container],
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            say(f'  agent PAUSED at t=0; frozen for {args.outage_seconds:.0f} s')
            time.sleep(args.outage_seconds)
            n_before = len(mon.samples)
            subprocess.run(['docker', 'unpause', args.container],
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            say('  agent restarted; waiting for the session to come back')
            deadline = time.time() + 25.0
            while time.time() < deadline and len(mon.samples) <= n_before:
                time.sleep(0.2)
            fresh = mon.samples[n_before:]
            if not fresh:
                say('  INCONCLUSIVE: session did not return within 25 s.')
                verdict, detail = None, 'no reconnect'
            else:
                peak = max(max(abs(vx), abs(wz)) for _, vx, wz in fresh[:10])
                say(f'  first samples after reconnect: peak |v| = {peak:.3f}')
                if peak > MOVING:
                    verdict = False
                    detail = ('DECISIVE: still moving on reconnect -- the firmware '
                              'latched the command through the entire outage')
                    say(f'  FAIL: {detail}')
                else:
                    verdict = None
                    detail = ('AMBIGUOUS: at rest on reconnect. Could be a firmware '
                              'watchdog, or a reset on session re-establishment.')
                    say(f'  {detail}')
            hard_stop()
            answer = None
            try:
                answer = input('  Did the wheels keep spinning while the agent was '
                               'paused? [y/N] ').strip().lower()
            except EOFError:
                pass
            if answer is None:
                # No witness. An unanswered question is NOT a pass -- defaulting a
                # missing observation to "it stopped" would manufacture evidence.
                human = 'not observed'
            else:
                human = 'kept spinning' if answer.startswith('y') else 'stopped'
            say(f'  human observation: wheels {human}')
            if verdict is None and answer is not None:
                verdict = not answer.startswith('y')
            # The session is corrupt now whatever the outcome. Replace the container
            # rather than leaving a wedged agent behind: it runs with --rm, so `stop`
            # deletes it and only a fresh `run` brings it back.
            say('  restarting the agent (the paused session is corrupt)')
            subprocess.run(['docker', 'stop', args.container],
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            subprocess.run(
                ['docker', 'run', '-d', '--rm', '--name', args.container,
                 '-v', '/dev:/dev', '-v', '/dev/shm:/dev/shm',
                 '--privileged', '--net=host',
                 'microros/micro-ros-agent:jazzy', 'udp4', '--port', '8090', '-v4'],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            say('  >>> POWER-CYCLE THE CAR NOW. The board will not re-register '
                'on its own. <<<')
            results['cases']['agent_loss'] = {
                'title': 'micro-ROS agent frozen',
                'stopped': verdict, 'observation': 'human + reconnect probe',
                'human_observation': human, 'detail': detail,
                'outage_s': args.outage_seconds,
            }
    hard_stop(25)

    # ------------------------------------------------------------- verdict
    say('')
    say('=== verdict ===')
    cases = results['cases']
    failed = [k for k, v in cases.items() if v.get('stopped') is False]
    unknown = [k for k, v in cases.items() if v.get('stopped') is None]
    passed = [k for k, v in cases.items() if v.get('stopped') is True]
    for k in passed:
        hi = cases[k].get('time_to_stop_hi_s')
        say(f'  PASS     {k}' + (f'  ({hi*1000:.0f} ms)' if hi else ''))
    for k in unknown:
        say(f'  UNKNOWN  {k}  -- {cases[k].get("detail", "")}')
    for k in failed:
        say(f'  FAIL     {k}  -- {cases[k].get("detail", "")}')

    if failed:
        say('')
        say('  NO FIRMWARE WATCHDOG. This is not fixable in software: any crash of any')
        say('  component leaves the car driving. Floor testing is gated on a hand on the')
        say('  physical power switch, at a speed slow enough to catch on foot.')
        code = 1
    elif unknown:
        say('')
        say('  INCONCLUSIVE. Do not treat the command path as fail-safe.')
        code = 1
    else:
        bound = max(c['time_to_stop_hi_s'] for c in cases.values()
                    if c.get('time_to_stop_hi_s'))
        say('')
        say(f'  Command loss stops the car within {bound*1000:.0f} ms in every case')
        say(f'  tested. At 0.30 m/s that is {0.30*bound*1000:.0f} mm of coast, which')
        say('  must be added to the stopping envelope as a crash-case term.')
        results['watchdog_bound_s'] = bound
        code = 0

    os.makedirs(OUT_DIR, exist_ok=True)
    log_path = os.path.join(OUT_DIR, f'failsafe-{stamp}.log')
    with open(log_path, 'w') as f:
        f.write('\n'.join(lines) + '\n')
    with open(REPORT, 'w') as f:
        json.dump(results, f, indent=2)
    say('')
    say(f'log:    {log_path}')
    say(f'report: {REPORT}')
    return code


if __name__ == '__main__':
    code = main()
    # rclpy's teardown aborts with "terminate called without an active exception" after
    # everything is written, turning a clean run into a core dump and losing the status.
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(code)
