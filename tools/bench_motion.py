#!/usr/bin/env python3
"""Bench characterization of the REAL car's drivetrain, elevated, wheels free.

    ./tools/bench_motion.py matrix   --elevated    # direction/sign matrix + IMU signs
    ./tools/bench_motion.py step     --elevated    # command -> motion / steady state
    ./tools/bench_motion.py latch    --elevated    # publisher killed; motion persists
    ./tools/bench_motion.py estop    --elevated    # latched motion, zeroed by estop
    ./tools/bench_motion.py deadman  --elevated    # kill driver under safety chain, x5

EVERY NUMBER THIS PRINTS IS FREE-SPIN (car elevated, wheels unloaded). Free-spin
velocities say nothing about floor speeds; what transfers is signs, symmetry, latency
brackets and the latching behaviour itself.

THE GUARDRAILS, all structural:
  * hardware REQUIRED: refuses to run unless YB_Car_Node is discovered, and refuses
    if any simulator evidence shares the domain (a bench result must be the car's).
  * caps: |vx| <= 0.15 m/s, |wz| <= 1.0 rad/s, clamped at the only publish site.
  * every phase runs inside SafeCmdVel (zeros on exit, exception, SIGINT, SIGTERM)
    and ends with an OBSERVED-zero check on /odom_raw -- publishing zeros is not the
    same as the car having stopped, and the exit code says which one happened.
  * one publisher at a time: phases that drive /cmd_vel directly refuse to start
    while a governor or deadman is alive on the domain (their stale-command zeros
    would interleave with the drive -- measured on the simulator as stuttering);
    the deadman phase, whose subject IS the safety chain, starts its own.
  * battery is read first and motion refused below MIN_BATTERY_V.

The latch phases exist because the firmware retains commands indefinitely (measured
three ways, 2026-08-06). `latch` kills an ungoverned driver with SIGKILL -- no zero can
escape -- and REQUIRES the motion to persist: this car latching is the documented
behaviour, and a bench that cannot reproduce it is not measuring the car.
"""
import argparse
import json
import math
import os
import signal
import statistics
import subprocess
import sys
import threading
import time
from datetime import datetime

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _cmd_vel_safety import SafeCmdVel, REAL_ROBOT_NODE, SIMULATOR_NODE, \
    SIMULATOR_TOPIC                                             # noqa: E402
from _layout import LOG_DIR, BAG_DIR, WS_SETUP, pkg_dir         # noqa: E402

MAX_VX = 0.15        # m/s  -- session authorization ceiling, structural
MAX_WZ = 1.0         # rad/s
MIN_BATTERY_V = 7.0
MOVING = 0.05        # m/s or rad/s: definitely moving (matches test_failsafe)
AT_REST = 0.02       # sustained below this = stopped
N_REST = 3

RESULTS = os.path.join(pkg_dir('yahboomcar_safety'), 'bench_motion.json')


class Tap:
    """Collect /odom_raw and /imu with receive timestamps."""

    def __init__(self, node):
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import Imu
        from rclpy.qos import qos_profile_sensor_data
        self.odom = []            # (t, vx, wz)
        self.imu = []             # (t, gyro_z, accel_x)
        node.create_subscription(Odometry, '/odom_raw', self._on_odom,
                                 qos_profile_sensor_data)
        node.create_subscription(Imu, '/imu', self._on_imu,
                                 qos_profile_sensor_data)

    def _on_odom(self, m):
        self.odom.append((time.time(), m.twist.twist.linear.x,
                          m.twist.twist.angular.z))

    def _on_imu(self, m):
        self.imu.append((time.time(), m.angular_velocity.z,
                         m.linear_acceleration.x))

    def odom_since(self, t):
        return [s for s in self.odom if s[0] >= t]

    def imu_since(self, t):
        return [s for s in self.imu if s[0] >= t]


def find_rest(samples, t_ref, timeout):
    """First sustained at-rest run after t_ref -> (lo, hi) bracket or None.
    Same bracket semantics as test_failsafe.py / measure_latency.py."""
    after = [s for s in samples if s[0] >= t_ref]
    for i in range(len(after) - N_REST + 1):
        run = after[i:i + N_REST]
        if all(abs(vx) <= AT_REST and abs(wz) <= AT_REST for _, vx, wz in run):
            hi = run[0][0] - t_ref
            if hi > timeout:
                return None
            prev = [s[0] for s in after[:i]
                    if abs(s[1]) > AT_REST or abs(s[2]) > AT_REST]
            lo = (max(prev) - t_ref) if prev else 0.0
            return (max(0.0, lo), hi)
    return None


def observed_zero(tap, say, seconds=3.0):
    """After zeros were published: is the car OBSERVED at rest on /odom_raw?"""
    t0 = time.time()
    time.sleep(seconds)
    recent = tap.odom_since(t0)
    if not recent:
        say('  ZERO NOT VERIFIED: no /odom_raw samples arrived -- nobody observed '
            'the car. Treat as NOT stopped; check the link, then the wheels.')
        return False
    peak = max(max(abs(vx), abs(wz)) for _, vx, wz in recent)
    ok = peak <= AT_REST
    say(f'  zero verified on /odom_raw: peak |v| = {peak:.3f} over {seconds:.0f} s '
        f'({len(recent)} samples) -> {"AT REST" if ok else "STILL MOVING"}')
    return ok


def graph_nodes(node):
    return [n for n, _ in node.get_node_names_and_namespaces()]


def require_bench_hardware(node, say, seconds=15.0):
    """The car, and ONLY the car. Positive evidence, polled; simulator presence or
    absence-of-everything both refuse."""
    deadline = time.time() + seconds
    while time.time() < deadline:
        time.sleep(1.0)
        names = graph_nodes(node)
        topics = [t for t, _ in node.get_topic_names_and_types()]
        if SIMULATOR_NODE in names or SIMULATOR_TOPIC in topics:
            say('REFUSED: simulator evidence on this domain. A bench number recorded '
                'against a simulator would poison the hardware record.')
            sys.exit(2)
        if REAL_ROBOT_NODE in names:
            return
    say(f'REFUSED: {REAL_ROBOT_NODE} not discovered within {seconds:.0f} s. No car, '
        'no bench test.')
    sys.exit(2)


def battery_volts(node, say, seconds=4.0):
    from std_msgs.msg import UInt16
    got = []
    node.create_subscription(UInt16, '/battery', lambda m: got.append(m.data), 10)
    t0 = time.time()
    while time.time() - t0 < seconds and not got:
        time.sleep(0.2)
    if not got:
        say('  battery: NO DATA in 4 s (topic silent?) -- refusing motion')
        sys.exit(2)
    v = got[-1] / 10.0
    say(f'  battery: {v:.1f} V')
    if v < MIN_BATTERY_V:
        say(f'  REFUSED: below {MIN_BATTERY_V} V; a sagging pack corrupts every '
            'velocity number and the car browns out mid-test.')
        sys.exit(2)
    return v


def no_rival_publishers(node, say):
    """Phases driving /cmd_vel directly must own it: refuse under governor/deadman."""
    names = graph_nodes(node)
    rivals = [n for n in names if n in ('cmd_vel_governor', 'cmd_vel_deadman')]
    if rivals:
        say(f'REFUSED: {rivals} alive on this domain. Their stale-command zeros '
            'interleave with a direct drive (measured as stuttering on the sim). '
            'Stop the safety launch, or use the deadman phase, whose subject it is.')
        sys.exit(2)


def spawn_driver(vx, wz, topic, domain):
    """A SEPARATE process publishing at 20 Hz, so a kill really is a dead publisher.
    Caps enforced here too -- this is a second publish site. The child is
    tools/_bench_driver.py (a file, not a -c snippet: an earlier -c version went
    through json.dumps and bash, arrived with literal backslash-n, and died on a
    SyntaxError before publishing anything)."""
    vx = max(-MAX_VX, min(MAX_VX, vx))
    wz = max(-MAX_WZ, min(MAX_WZ, wz))
    driver = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                          '_bench_driver.py')
    cmd = (f'source /opt/ros/jazzy/setup.bash && source {WS_SETUP} && '
           f'export ROS_DOMAIN_ID={domain} && '
           f'exec python3 {driver} {vx} {wz} {topic}')
    return subprocess.Popen(['bash', '-c', cmd], start_new_session=True,
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def kill_group(proc, sig=signal.SIGKILL):
    try:
        os.killpg(os.getpgid(proc.pid), sig)
    except (ProcessLookupError, PermissionError):
        pass


def steady(vals):
    if not vals:
        return float('nan'), float('nan')
    return statistics.mean(vals), (statistics.pstdev(vals) if len(vals) > 1 else 0.0)


# --------------------------------------------------------------------- phases
def phase_matrix(node, tap, safe, say, res):
    """Small steps on each axis; signs and magnitudes vs REP-103, L/R symmetry,
    IMU yaw-rate sign against commanded rotation. FREE-SPIN."""
    say('=== direction/sign matrix (FREE-SPIN) ===')
    steps = [('+x', 0.12, 0.0), ('-x', -0.12, 0.0),
             ('+yaw', 0.0, 0.8), ('-yaw', 0.0, -0.8)]
    out = {}
    for label, vx, wz in steps:
        say(f'  step {label}: cmd vx={vx:+.2f} m/s wz={wz:+.2f} rad/s, 2.5 s')
        t0 = time.time()
        end = t0 + 2.5
        while time.time() < end:
            safe.publish(vx=vx, wz=wz)
            time.sleep(0.05)
        # steady window = the last 1.5 s of the drive
        odom = [s for s in tap.odom_since(t0 + 1.0) if s[0] <= end]
        imu = [s for s in tap.imu_since(t0 + 1.0) if s[0] <= end]
        m_vx, sd_vx = steady([s[1] for s in odom])
        m_wz, sd_wz = steady([s[2] for s in odom])
        g_wz, g_sd = steady([s[1] for s in imu])
        say(f'    /odom_raw: vx {m_vx:+.4f}±{sd_vx:.4f}  wz {m_wz:+.4f}±{sd_wz:.4f} '
            f'({len(odom)} samples)')
        say(f'    /imu gyro_z: {g_wz:+.4f}±{g_sd:.4f} rad/s ({len(imu)} samples)')
        out[label] = {'cmd_vx': vx, 'cmd_wz': wz,
                      'odom_vx_mean': m_vx, 'odom_vx_sd': sd_vx,
                      'odom_wz_mean': m_wz, 'odom_wz_sd': sd_wz,
                      'imu_gyro_z_mean': g_wz, 'imu_gyro_z_sd': g_sd,
                      'odom_n': len(odom), 'imu_n': len(imu)}
        safe.stop()
        if not observed_zero(tap, say, 2.0):
            say('  aborting matrix: could not verify rest between steps')
            return False
    # verdicts
    ok = True
    for label, d in out.items():
        cmd = d['cmd_vx'] if 'x' in label else d['cmd_wz']
        got = d['odom_vx_mean'] if 'x' in label else d['odom_wz_mean']
        sign_ok = math.copysign(1, got) == math.copysign(1, cmd) and abs(got) > MOVING
        say(f'  {label}: sign {"MATCHES" if sign_ok else "WRONG/ABSENT"} '
            f'(cmd {cmd:+.2f} -> odom {got:+.4f})')
        d['sign_ok'] = sign_ok
        ok &= sign_ok
    sym_x = abs(out['+x']['odom_vx_mean']) / max(1e-9, abs(out['-x']['odom_vx_mean']))
    sym_y = abs(out['+yaw']['odom_wz_mean']) / max(
        1e-9, abs(out['-yaw']['odom_wz_mean']))
    say(f'  symmetry |+x|/|-x| = {sym_x:.3f}   |+yaw|/|-yaw| = {sym_y:.3f}')
    # IMU sign vs commanded rotation (the gyro-fault watch lives here too)
    for label in ('+yaw', '-yaw'):
        d = out[label]
        if abs(d['odom_wz_mean']) > MOVING and d['imu_gyro_z_sd'] == 0.0 \
                and d['imu_gyro_z_mean'] == 0.0:
            say(f'  *** GYRO FAULT SIGNATURE during {label}: odom shows rotation, '
                'gyro flat 0.000000. Bag it (see --bag) and do not fuse yaw today.')
            d['gyro_fault'] = True
        else:
            d['gyro_fault'] = False
            g_sign_ok = (math.copysign(1, d['imu_gyro_z_mean'])
                         == math.copysign(1, d['cmd_wz']))
            say(f'  {label}: imu gyro_z sign {"MATCHES" if g_sign_ok else "WRONG"} '
                f'({d["imu_gyro_z_mean"]:+.4f} vs cmd {d["cmd_wz"]:+.2f})')
            d['imu_sign_ok'] = g_sign_ok
    res['matrix'] = {'steps': out, 'sym_x': sym_x, 'sym_yaw': sym_y}
    return ok


def phase_step(node, tap, safe, say, res, reps=3):
    """Command -> first /odom_raw motion (bracket), -> steady state, steady vs
    commanded. FREE-SPIN: floor values WILL be slower (spin-up under load)."""
    say('=== step response (FREE-SPIN; brackets are one 11 Hz interval wide) ===')
    trials = []
    for axis, vx, wz in (('x', 0.12, 0.0), ('yaw', 0.0, 0.8)):
        for i in range(reps):
            time.sleep(1.5)
            t_cmd = time.time()
            end = t_cmd + 3.0
            while time.time() < end:
                safe.publish(vx=vx, wz=wz)
                time.sleep(0.05)
            odom = tap.odom_since(t_cmd)
            key = 1 if axis == 'x' else 2
            first = next((s for s in odom if abs(s[key]) > AT_REST), None)
            if first is None:
                say(f'  {axis} #{i+1}: NO MOTION observed in 3 s -- invalid trial')
                trials.append({'axis': axis, 'valid': False})
                safe.stop()
                observed_zero(tap, say, 2.0)
                continue
            hi = first[0] - t_cmd
            before = [s[0] for s in odom if s[0] < first[0]]
            lo = (max(before) - t_cmd) if before else 0.0
            plateau_vals = [abs(s[key]) for s in odom if s[0] >= end - 1.0]
            plateau, psd = steady(plateau_vals)
            t_90 = next((s[0] - t_cmd for s in odom
                         if abs(s[key]) >= 0.9 * plateau), None)
            cmd = abs(vx) if axis == 'x' else abs(wz)
            say(f'  {axis} #{i+1}: first motion [{lo*1000:.0f}, {hi*1000:.0f}] ms, '
                f'90% of plateau at {t_90*1000:.0f} ms, '
                f'plateau {plateau:.4f}±{psd:.4f} vs cmd {cmd:.2f} '
                f'(ratio {plateau/cmd:.3f})')
            trials.append({'axis': axis, 'valid': True, 'lo_s': lo, 'hi_s': hi,
                           't90_s': t_90, 'plateau': plateau, 'plateau_sd': psd,
                           'cmd': cmd, 'ratio': plateau / cmd})
            safe.stop()
            if not observed_zero(tap, say, 2.0):
                return False
    good = [t for t in trials if t['valid']]
    if good:
        his = [t['hi_s'] * 1000 for t in good]
        say(f'  first-motion upper bounds: {sorted(f"{h:.0f}" for h in his)} ms '
            f'(max {max(his):.0f} ms)')
    res['step'] = trials
    return bool(good)


def phase_latch(node, tap, safe, say, res, min_persist=3.0):
    """SIGKILL the (separate) driver; motion must PERSIST -- that is the firmware
    behaviour this whole safety architecture exists for. Zeroed by our finalizer."""
    say('=== latching confirmation (driver SIGKILLed, no deadman present) ===')
    no_rival_publishers(node, say)
    say(f'  driver: separate process, 0.12 m/s on /cmd_vel; kill at t=0; '
        f'watch {min_persist:.0f} s')
    drv = spawn_driver(0.12, 0.0, '/cmd_vel', os.environ['ROS_DOMAIN_ID'])
    try:
        t0 = time.time()
        while time.time() - t0 < 14.0:
            time.sleep(0.2)
            if any(abs(s[1]) > MOVING for s in tap.odom_since(time.time() - 0.6)):
                break
        else:
            say("  INVALID: driver produced no motion in 14 s")
            kill_group(drv)
            return False
        t_kill = time.time()
        kill_group(drv, signal.SIGKILL)
        say('  SIGKILL sent at t=0 (kill -9: no cleanup zero can escape)')
        time.sleep(min_persist + 0.5)
        window = [s for s in tap.odom_since(t_kill)
                  if s[0] <= t_kill + min_persist]
        still = [s for s in window if abs(s[1]) > MOVING]
        frac = len(still) / max(1, len(window))
        last_moving = max((s[0] - t_kill for s in still), default=0.0)
        say(f'  {len(window)} samples in the window; {frac*100:.0f}% still moving; '
            f'last moving sample at +{last_moving:.2f} s')
        persisted = last_moving >= min_persist - 0.2
        say(f'  {"CONFIRMED" if persisted else "NOT CONFIRMED"}: command '
            f'{"latched through" if persisted else "did NOT persist"} '
            f'{min_persist:.0f} s with the publisher dead')
        res['latch'] = {'persisted': persisted, 'window_frac_moving': frac,
                        'last_moving_s': last_moving, 'min_persist_s': min_persist}
    finally:
        kill_group(drv)
        say('  finalizer: zeroing')
        safe.stop()
    return observed_zero(tap, say) and res.get('latch', {}).get('persisted', False)


def phase_estop(node, tap, safe, say, res):
    """Latch motion, then zero it via `simctl estop --domain <car>` -- the
    documented operator path -- and verify on /odom_raw."""
    say('=== estop path (latched motion, zeroed by simctl estop) ===')
    no_rival_publishers(node, say)
    drv = spawn_driver(0.12, 0.0, '/cmd_vel', os.environ['ROS_DOMAIN_ID'])
    try:
        t0 = time.time()
        while time.time() - t0 < 14.0:
            time.sleep(0.2)
            if any(abs(s[1]) > MOVING for s in tap.odom_since(time.time() - 0.6)):
                break
        else:
            say("  INVALID: no motion to estop (14 s)")
            return False
        kill_group(drv, signal.SIGKILL)
        time.sleep(1.0)          # latched, publisher dead: the estop scenario
        tools = os.path.dirname(os.path.abspath(__file__))
        t_estop = time.time()
        p = subprocess.run([os.path.join(tools, 'simctl'), 'estop',
                            '--domain', os.environ['ROS_DOMAIN_ID']],
                           capture_output=True, text=True, timeout=60)
        say('  simctl estop output: '
            + ' / '.join(p.stdout.strip().splitlines()[-2:]))
        br = find_rest(tap.odom, t_estop, 15.0)
        if br:
            say(f'  at rest [{br[0]*1000:.0f}, {br[1]*1000:.0f}] ms after estop '
                'started (includes estop process startup, not just publish->stop)')
        res['estop'] = {'stopped': br is not None,
                        'lo_s': br[0] if br else None, 'hi_s': br[1] if br else None}
    finally:
        kill_group(drv)
        safe.stop()
    return observed_zero(tap, say) and bool(res.get('estop', {}).get('stopped'))


def phase_deadman(node, tap, safe, say, res, reps=5):
    """The floor safety chain from the FLEET install: governor + deadman up, driver
    on /cmd_vel_raw SIGKILLed mid-motion. Stop latency x reps, vs 693-762 ms prior."""
    say('=== safety chain: driver killed under governor + deadman ===')
    names = graph_nodes(node)
    if 'cmd_vel_governor' in names or 'cmd_vel_deadman' in names:
        say('REFUSED: a safety chain is already up; this phase must own it or the '
            'measurement is of an unknown mix.')
        return False
    say('  starting safety_launch from the fleet install (permissive distances: the '
        'bench lidar sees near walls, and the subject is the CRASH path)')
    launch = subprocess.Popen(
        ['bash', '-c',
         f'source /opt/ros/jazzy/setup.bash && source {WS_SETUP} && '
         f'export ROS_DOMAIN_ID={os.environ["ROS_DOMAIN_ID"]} && '
         'exec ros2 launch yahboomcar_safety safety_launch.py '
         'stop_distance:=0.01 slow_distance:=0.02 max_speed:=0.2'],
        start_new_session=True, stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL)
    brackets = []
    try:
        deadline = time.time() + 20.0
        while time.time() < deadline:
            time.sleep(1.0)
            names = graph_nodes(node)
            if 'cmd_vel_governor' in names and 'cmd_vel_deadman' in names:
                break
        else:
            say('  FAILED: governor+deadman not both discovered in 20 s')
            return False
        say('  governor + deadman up')
        for i in range(reps):
            drv = spawn_driver(0.12, 0.0, '/cmd_vel_raw',
                               os.environ['ROS_DOMAIN_ID'])
            try:
                t0 = time.time()
                while time.time() - t0 < 8.0:
                    time.sleep(0.2)
                    if any(abs(s[1]) > MOVING
                           for s in tap.odom_since(time.time() - 0.6)):
                        break
                else:
                    say(f'  run {i+1}: INVALID -- governor passed no motion '
                        '(obstacle rule? scan stale?)')
                    continue
                time.sleep(0.8)      # settle at speed
                t_kill = time.time()
                kill_group(drv, signal.SIGKILL)
                time.sleep(3.5)
                br = find_rest(tap.odom, t_kill, 3.0)
                if br:
                    say(f'  run {i+1}: stopped in [{br[0]*1000:.0f}, '
                        f'{br[1]*1000:.0f}] ms')
                    brackets.append(br)
                else:
                    say(f'  run {i+1}: NOT STOPPED within 3 s -- chain failed to '
                        'catch the dead driver')
            finally:
                kill_group(drv)
            safe.stop()
            if not observed_zero(tap, say, 2.0):
                say('  aborting: rest not verified between runs')
                return False
    finally:
        say('  stopping the safety launch (car first: zeros, then the kill)')
        safe.stop()
        kill_group(launch, signal.SIGTERM)
        time.sleep(2.0)
        kill_group(launch, signal.SIGKILL)
    if brackets:
        his = sorted(b[1] * 1000 for b in brackets)
        say(f'  stop-latency upper bounds across {len(brackets)} runs: '
            f'{", ".join(f"{h:.0f}" for h in his)} ms '
            f'(spread {his[0]:.0f}-{his[-1]:.0f}; prior 693-762 ms)')
    res['deadman'] = {'reps_valid': len(brackets),
                      'brackets_ms': [[b[0] * 1000, b[1] * 1000]
                                      for b in brackets]}
    return len(brackets) >= max(3, reps - 1) and observed_zero(tap, say)


PHASES = {'matrix': phase_matrix, 'step': phase_step, 'latch': phase_latch,
          'estop': phase_estop, 'deadman': phase_deadman}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('phase', choices=list(PHASES))
    ap.add_argument('--domain', type=int, default=20)
    ap.add_argument('--elevated', action='store_true',
                    help='REQUIRED assertion that the car is elevated with wheels '
                         'free. This tool spins real wheels.')
    ap.add_argument('--reps', type=int, default=None)
    ap.add_argument('--bag', action='store_true',
                    help='record scan/imu/odom_raw/cmd_vel to a bag for the run')
    args = ap.parse_args()

    if not args.elevated:
        print('REFUSED: pass --elevated only if the car is on the stand, wheels '
              'free. This tool commands real motion.')
        return 2
    os.environ['ROS_DOMAIN_ID'] = str(args.domain)

    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node

    lines = []

    def say(m=''):
        print(m, flush=True)
        lines.append(str(m))

    stamp = datetime.now().strftime('%Y%m%d-%H%M%S')
    say(f'bench_motion {args.phase}  {stamp}  ROS_DOMAIN_ID={args.domain}  '
        'ALL VELOCITIES FREE-SPIN')

    rclpy.init()
    node = Node('bench_motion')
    tap = Tap(node)
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()

    require_bench_hardware(node, say)
    say(f'  target: {REAL_ROBOT_NODE} (hardware)')
    battery_volts(node, say)
    if args.phase in ('matrix', 'step'):
        no_rival_publishers(node, say)

    bag = None
    if args.bag:
        os.makedirs(BAG_DIR, exist_ok=True)
        bagpath = os.path.join(BAG_DIR, f'bench-{args.phase}-{stamp}')
        bag = subprocess.Popen(
            ['bash', '-c',
             f'source /opt/ros/jazzy/setup.bash && source {WS_SETUP} && '
             f'export ROS_DOMAIN_ID={args.domain} && '
             f'exec ros2 bag record -o {bagpath} /scan /imu /odom_raw /cmd_vel '
             '/cmd_vel_raw /battery'],
            start_new_session=True, stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL)
        say(f'  bagging to {bagpath}')

    res = {'stamp': stamp, 'phase': args.phase, 'free_spin': True,
           'domain': args.domain}
    ok = False
    try:
        with SafeCmdVel(node, ['/cmd_vel']) as safe:
            kw = {}
            if args.reps and args.phase in ('step', 'deadman'):
                kw['reps'] = args.reps
            ok = PHASES[args.phase](node, tap, safe, say, res, **kw)
    finally:
        if bag is not None:
            kill_group(bag, signal.SIGTERM)
        # SafeCmdVel.__exit__ has already zeroed; verify ONCE more, out loud.
        final_ok = observed_zero(tap, say, 2.5)
        say('')
        say(f'RESULT: {"PASS" if ok else "FAIL"}   '
            f'final rest {"VERIFIED" if final_ok else "NOT VERIFIED"}')
        os.makedirs(LOG_DIR, exist_ok=True)
        log = os.path.join(LOG_DIR, f'bench-{args.phase}-{stamp}.log')
        with open(log, 'w') as f:
            f.write('\n'.join(lines) + '\n')
        allres = {}
        if os.path.exists(RESULTS):
            try:
                with open(RESULTS) as f:
                    allres = json.load(f)
            except ValueError:
                allres = {}
        allres[f'{args.phase}-{stamp}'] = res
        with open(RESULTS, 'w') as f:
            json.dump(allres, f, indent=2)
        say(f'log: {log}')
        say(f'results: {RESULTS}')
    return 0 if (ok and final_ok) else 1


if __name__ == '__main__':
    code = main()
    # rclpy teardown aborts ("terminate called without an active exception") after
    # everything is written; same exit pattern as test_failsafe.py to keep a clean
    # run from ending in a core dump.
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(code)
