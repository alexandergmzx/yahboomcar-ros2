#!/usr/bin/env python3
"""Do the two EKF configurations behave differently when the wheels lie?

    ./tools/ekf_ab_test.py                  # both configs, back to back
    ./tools/ekf_ab_test.py --config vendor  # just one
    ./tools/ekf_ab_test.py --seconds 20

SAFETY: the wheels spin. Car ELEVATED, body held still.

THE EXPERIMENT
--------------
A car on a stand is the cleanest possible test of a state estimator, because the right
answer is known exactly and it is ZERO. The wheels turn, the body does not move, and
anything the filter reports as displacement is error.

That is not a contrived case. It is wheel slip taken to its limit, and slip is the one
failure the encoders cannot see: to them, a spinning wheel on a stand and a wheel driving
across a floor are the same measurement.

  VENDOR ekf.yaml     fuses wheel POSE and wheel TWIST from /odom_raw, plus IMU yaw and
                      yaw rate. Nothing in it observes the world, so nothing can
                      contradict the wheels.

  CORRECTED           fuses wheel TWIST only, IMU YAW RATE only, and adds /odom_laser --
                      a pose derived from the room, which is not moving.

PREDICTION, RECORDED BEFORE THE RUN so it can be wrong:

    vendor     reports a LARGE displacement, roughly the integrated wheel distance
    corrected  reports NEAR ZERO, because the lidar contradicts the wheels

If the corrected config does not win, that is the result and it gets written down as it
came out. The point of stating this first is that a prediction made afterwards is not a
prediction.

WHAT THIS DOES NOT SHOW
-----------------------
That the corrected config is better at driving. A stand tests one specific claim -- that
adding a world-referenced input stops the filter believing a slipping wheel -- and says
nothing about behaviour in real motion, where the lidar has its own errors. This is
necessary, not sufficient.
"""
import argparse
import json
import math
import os
import shutil
import signal
import subprocess
import sys
import threading
import time
from datetime import datetime

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _cmd_vel_safety import install_stop_handlers            # noqa: E402

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WS = os.path.join(REPO, 'yahboomcar_ws')
PARAM_DIR = os.path.join(WS, 'src', 'yahboomcar_bringup', 'param')
OUT_DIR = os.path.join(REPO, 'MicroROS-assets', 'logs')
BAG_DIR = os.path.join(REPO, 'MicroROS-assets', 'bags')
RESULT = os.path.join(WS, 'src', 'yahboomcar_bringup', 'param', 'ekf_ab_result.json')

CONFIGS = {
    'vendor': ('ekf.yaml', 'wheel pose + twist, IMU yaw + rate; nothing observes the world'),
    'corrected': ('ekf_corrected.yaml', 'wheel twist only, IMU rate only, + /odom_laser pose'),
}


def sh(cmd, **kw):
    """Run a command under a sourced ROS environment, in its own process group."""
    full = (f'source /opt/ros/jazzy/setup.bash && source {WS}/install/setup.bash && '
            f'{cmd}')
    return subprocess.Popen(['bash', '-c', full], start_new_session=True,
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, **kw)


def kill_group(proc):
    if proc is None:
        return
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
    except (ProcessLookupError, PermissionError):
        pass


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--config', choices=list(CONFIGS) + ['both'], default='both')
    ap.add_argument('--speed', type=float, default=0.15)
    ap.add_argument('--seconds', type=float, default=15.0)
    ap.add_argument('--domain', type=int, default=20)
    ap.add_argument('--no-bag', action='store_true')
    args = ap.parse_args()

    os.environ.setdefault('ROS_DOMAIN_ID', str(args.domain))
    stamp = datetime.now().strftime('%Y%m%d-%H%M%S')
    lines = []

    def say(m=''):
        print(m, flush=True)
        lines.append(str(m))

    import rclpy
    from ament_index_python.packages import get_package_share_directory
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry

    share_bringup = get_package_share_directory('yahboomcar_bringup')

    say(f'=== EKF A/B under pure slip  {stamp} ===')
    say('CAR MUST BE ELEVATED, body held still. Ground truth displacement = 0.')
    say('')
    say('PREDICTION (recorded before the run):')
    say('  vendor     -> LARGE displacement; nothing contradicts the wheels')
    say('  corrected  -> NEAR ZERO; /odom_laser reports a stationary world')
    say('')

    results = {'timestamp': stamp, 'speed_m_s': args.speed,
               'seconds': args.seconds,
               'ground_truth_displacement_m': 0.0,
               'prediction': {'vendor': 'large', 'corrected': 'near zero'},
               'conditions': 'car elevated on centre stand, body held still',
               'runs': {}}

    which = list(CONFIGS) if args.config == 'both' else [args.config]

    for name in which:
        fname, desc = CONFIGS[name]
        path = os.path.join(PARAM_DIR, fname)
        if not os.path.exists(path):
            say(f'SKIP {name}: {path} missing')
            continue

        say(f'--- {name}: {fname} ---')
        say(f'    {desc}')

        rclpy.init()
        node = Node(f'ekf_ab_{name}')
        odom_raw, odom_fused, laser = [], [], []
        node.create_subscription(
            Odometry, '/odom_raw',
            lambda m: odom_raw.append((m.twist.twist.linear.x,)), qos_profile_sensor_data)
        node.create_subscription(
            Odometry, '/odom',
            lambda m: odom_fused.append((m.pose.pose.position.x,
                                         m.pose.pose.position.y)), 10)
        node.create_subscription(
            Odometry, '/odom_laser',
            lambda m: laser.append((m.pose.pose.position.x,
                                    m.pose.pose.position.y)), 10)
        cmd = node.create_publisher(Twist, '/cmd_vel', 10)
        # `finally` alone does not survive SIGTERM (pkill).
        install_stop_handlers(cmd)
        ex = SingleThreadedExecutor()
        ex.add_node(node)
        threading.Thread(target=ex.spin, daemon=True).start()

        procs = []
        try:
            # The firmware publishes raw /imu; BOTH configs subscribe to /imu/data, which
            # only exists once imu_filter_madgwick fuses the 6-axis IMU into an
            # orientation. Started for both runs so neither is handicapped by a missing
            # input. Remaps and node names copied from yahboomcar_bringup_launch.py --
            # ROS 2 binds parameters by node name, so the name is load-bearing.
            imu_param = os.path.join(share_bringup, 'param', 'imu_filter_param.yaml')
            procs.append(sh(f'ros2 run imu_filter_madgwick imu_filter_madgwick_node '
                            f'--ros-args --params-file {imu_param} '
                            f'-r __node:=imu_filter_madgwick -r imu/data_raw:=/imu'))

            # The filter under test. Started fresh each time so neither run inherits the
            # other's state. The remap is essential: robot_localization publishes
            # `odometry/filtered` by default, and everything here -- and Nav2, and
            # cartographer -- expects /odom.
            procs.append(sh(f'ros2 run robot_localization ekf_node '
                            f'--ros-args --params-file {path} '
                            f'-r __node:=ekf_filter_node '
                            f'-r odometry/filtered:=/odom'))
            if name == 'corrected':
                # /odom_laser is the whole point of this configuration; without it the
                # corrected filter is just the vendor one with fewer inputs.
                procs.append(sh('ros2 run yahboomcar_localization laser_odometry'))
            bag_path = None
            if not args.no_bag:
                bag_path = os.path.join(BAG_DIR, f'ekf-ab-{name}-{stamp}')
                procs.append(sh(f'ros2 bag record -o {bag_path} '
                                f'/odom /odom_raw /odom_laser /imu/data /scan /cmd_vel'))
            time.sleep(8.0)

            if not odom_fused:
                say('    WARNING: no /odom yet; the filter may not have started')

            odom_fused.clear()
            laser.clear()
            start_fused = None
            t = Twist()
            t.linear.x = float(args.speed)
            end = time.time() + args.seconds
            while time.time() < end:
                cmd.publish(t)
                if start_fused is None and odom_fused:
                    start_fused = odom_fused[0]
                time.sleep(0.05)
            for _ in range(20):
                cmd.publish(Twist())
                time.sleep(0.04)
            time.sleep(2.0)

            wheel_dist = args.speed * args.seconds      # what the wheels 'travelled'
            if odom_fused and start_fused:
                dx = odom_fused[-1][0] - start_fused[0]
                dy = odom_fused[-1][1] - start_fused[1]
                fused = math.hypot(dx, dy)
            else:
                fused = None
            lz = (math.hypot(laser[-1][0] - laser[0][0], laser[-1][1] - laser[0][1])
                  if len(laser) > 1 else None)

            say(f'    wheels commanded    : {wheel_dist*1000:7.0f} mm '
                f'({args.speed} m/s x {args.seconds:.0f} s)')
            if fused is None:
                say('    EKF /odom           : NO DATA')
            else:
                say(f'    EKF /odom moved     : {fused*1000:7.0f} mm   '
                    f'(truth 0 mm)   [{len(odom_fused)} msgs]')
            if lz is not None:
                say(f'    /odom_laser moved   : {lz*1000:7.0f} mm')
            results['runs'][name] = {
                'config_file': fname, 'description': desc,
                'wheel_commanded_m': wheel_dist,
                'ekf_displacement_m': fused,
                'laser_displacement_m': lz,
                'n_odom_msgs': len(odom_fused),
                'bag': bag_path,
            }
        finally:
            for _ in range(15):
                cmd.publish(Twist())
                time.sleep(0.03)
            for p in procs:
                kill_group(p)
            node.destroy_node()
            try:
                if rclpy.ok():
                    rclpy.shutdown()
            except Exception:
                pass
            time.sleep(4.0)      # let the filter and recorder actually exit
        say('')

    # ------------------------------------------------------------------ verdict
    say('=== verdict ===')
    v = results['runs'].get('vendor', {}).get('ekf_displacement_m')
    c = results['runs'].get('corrected', {}).get('ekf_displacement_m')
    if v is not None and c is not None:
        say(f'  vendor    {v*1000:7.0f} mm')
        say(f'  corrected {c*1000:7.0f} mm')
        say(f'  truth           0 mm')
        say('')
        if c < v * 0.5:
            say('  PREDICTION HELD: the corrected config is substantially closer to')
            say('  ground truth. Adding a world-referenced input stopped the filter')
            say('  believing a wheel that was lying to it.')
            results['verdict'] = 'prediction held'
        elif c > v:
            say('  PREDICTION FAILED, and in the worst direction: the corrected config')
            say('  is WORSE. Recorded as-is. Do not adopt it on the strength of the')
            say('  analysis alone.')
            results['verdict'] = 'prediction failed - corrected is worse'
        else:
            say('  INCONCLUSIVE: the two are comparable. The corrected config is not')
            say('  obviously better here, whatever the reasoning behind it says.')
            results['verdict'] = 'inconclusive'
    else:
        say('  incomplete: one or both runs produced no /odom')
        results['verdict'] = 'incomplete'

    os.makedirs(OUT_DIR, exist_ok=True)
    log = os.path.join(OUT_DIR, f'ekf-ab-{stamp}.log')
    with open(log, 'w') as f:
        f.write('\n'.join(lines) + '\n')
    with open(RESULT, 'w') as f:
        json.dump(results, f, indent=2)
    say('')
    say(f'log:    {log}')
    say(f'result: {RESULT}')
    return 0


if __name__ == '__main__':
    code = main()
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(code)
