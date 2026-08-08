#!/usr/bin/env python3
"""Does the simulated chassis actually follow the real one? Exits NONZERO when it does not.

    ISAAC=~/isaac/env_isaaclab/bin/python
    $ISAAC tools/verify_twin.py                 # drive from a synthetic TF fixture
    $ISAAC tools/verify_twin.py --live          # drive from the real robot's /tf
    $ISAAC tools/verify_twin.py --gui

  negative controls -- these MUST fail, and the tool is untrustworthy if they pass:
    $ISAAC tools/verify_twin.py --no-tf         # nothing publishes; expect "no pose"
    $ISAAC tools/verify_twin.py --drop-test     # shove the base underground; expect "fallen"

WHY THIS EXISTS
---------------
The previous Isaac scene subscribed only to derived joint states. It animated wheels and
a gimbal and never subscribed to /odom or /tf at all, so the simulated chassis did not
follow the real one in any sense -- and the old verifier reported success regardless,
because it returned no status and swallowed exceptions. An audit caught both.

So the bar here is not "it ran". It is that the verifier FAILS when the twin is broken,
which is why the two negative controls above are part of the interface rather than a
footnote. A verifier that cannot fail is decoration.

WHY THE ROS PATH IS AN OMNIGRAPH NODE AND NOT rclpy
---------------------------------------------------
Isaac's interpreter is Python 3.11; ROS 2 Jazzy builds rclpy for 3.12. The C extension
does not load across that gap -- not a path problem, an ABI one. So the twin cannot
subscribe to anything from Python directly, and the test fixture has to run as a separate
system-python process. isaacsim.ros2.bridge is C++ and carries its own ROS 2; a probe
confirmed it shares a DDS graph with system ROS 2 (a clock published from Isaac appeared
in `ros2 topic list`).

WHY THE GENERIC SUBSCRIBER AND NOT ROS2SubscribeTransformTree
-------------------------------------------------------------
The TF-tree node was the obvious choice and does not work here. Five configurations were
tried -- kinematic and dynamic base, articulation root vs base_link, explicit frame map
vs name matching, empty roots -- and the chassis did not move in any of them. Part of the
reason is visible in the stage: the URDF produces no `base_footprint` prim at all, only
`base_link`, which is itself the articulation root, so there is nothing for the published
`odom -> base_footprint` frame to bind to by name.

Rather than keep guessing at an undocumented node, the pose comes through the generic
ROS2Subscriber decoding nav_msgs/msg/Odometry. It exposes every message field as a
readable output attribute, so Python reads the pose each frame and authors the transform
itself. Nothing is hidden inside a node whose semantics are not written down anywhere,
and when it breaks the failure is in code that can be read.

/odom rather than /odom_raw, deliberately: the base pose should follow the EKF's fused
estimate. The WHEELS are still driven from /odom_raw, because on a stand the EKF
correctly reports ~0 yaw while the wheels spin, and using /odom there would freeze them.

WHY GRAVITY IS DISABLED RATHER THAN THE BASE MADE KINEMATIC
-----------------------------------------------------------
A twin follows a pose decided elsewhere, so PhysX must not fight the transform it is told
to adopt. The obvious way to say that is kinematicEnabled -- and it does not work here:
a kinematic body leaves the dynamic set, so the physics tensor view stops matching it and
SingleArticulation fails outright with "Pattern /World/Robot/base_link did not match any
rigid bodies". The articulation API and kinematic flags are mutually exclusive.

What works is keeping the articulation dynamic, disabling gravity on every link, and
authoring the world pose each frame. Gravity is the only force acting on a free-floating
robot here, so removing it leaves the pose entirely determined by what odometry says.

That is the opposite of what the ARENA wants: tools/sim_runner.py deliberately keeps
gravity and contacts real so friction and slip are simulated, because that is what makes
it a test rig. Two modes, and conflating them is how you get a twin that lies.
"""
import argparse
import json
import math
import os
import subprocess
import sys
import time
from datetime import datetime

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _layout import REPO, USD_DIR, LOG_DIR                      # noqa: E402
ARENA_USD = os.path.join(USD_DIR, 'arena.usd')
ROBOT_PRIM = '/World/Robot'

# Must match tools/_tf_source.py.
WAYPOINTS = [
    (0.0, 0.0, 0.0),
    (0.5, 0.0, 0.0),
    (0.5, 0.5, math.pi / 2),
    (0.0, 0.5, math.pi),
    (0.0, 0.0, -math.pi / 2),
]
FLOOR_Z = -0.25          # below this the base has fallen through the world


def ang_err(a, b):
    return abs(math.atan2(math.sin(a - b), math.cos(a - b)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--gui', action='store_true')
    ap.add_argument('--hold', type=float, default=2.5)
    ap.add_argument('--tol-xy', type=float, default=0.05, help='metres')
    ap.add_argument('--tol-yaw', type=float, default=0.15, help='radians')
    ap.add_argument('--domain', type=int, default=20)
    ap.add_argument('--topic', default='odom', help='Odometry topic to follow')
    ap.add_argument('--max-gap', type=float, default=0.5,
                    help='live mode: worst tolerated gap between messages, seconds')
    ap.add_argument('--live', action='store_true',
                    help='use the real robot /tf instead of the fixture')
    ap.add_argument('--no-tf', action='store_true',
                    help='NEGATIVE CONTROL: publish nothing; this must FAIL')
    ap.add_argument('--drop-test', action='store_true',
                    help='NEGATIVE CONTROL: shove the base underground; must FAIL')
    args = ap.parse_args()

    os.environ.setdefault('ROS_DOMAIN_ID', str(args.domain))
    stamp = datetime.now().strftime('%Y%m%d-%H%M%S')
    lines = []

    def say(m=''):
        # Isaac's carb logger swallows stdout once SimulationApp starts, so a failed run
        # otherwise looks like a silent success. Everything goes to a file too.
        print(m, flush=True)
        lines.append(str(m))

    def finish(code, reason=''):
        os.makedirs(LOG_DIR, exist_ok=True)
        path = os.path.join(LOG_DIR, f'verify-twin-{stamp}.log')
        with open(path, 'w') as f:
            f.write('\n'.join(lines) + '\n')
        print(f'\nreport: {path}', flush=True)
        print(f'VERDICT: {"PASS" if code == 0 else "FAIL"}  {reason}', flush=True)
        sys.stdout.flush()
        sys.stderr.flush()
        # Before app.close(): it swallows the exit status and can turn a clean failure
        # into an exit 0, which is precisely the defect this tool exists to avoid.
        os._exit(code)

    say(f'verify_twin  {stamp}')
    if not os.path.exists(ARENA_USD):
        say(f'FAIL: no arena at {ARENA_USD}. Run tools/build_arena.py first.')
        finish(2, 'arena USD missing')

    from isaacsim import SimulationApp
    app = SimulationApp({'headless': not args.gui})

    from isaacsim.core.utils.extensions import enable_extension
    # Without this the bridge nodes do not exist and the graph silently does nothing --
    # the same failure mode that made the URDF importer look like it worked.
    enable_extension('isaacsim.ros2.bridge')
    for _ in range(10):
        app.update()

    import omni.usd
    import omni.graph.core as og
    from isaacsim.core.api import SimulationContext
    from pxr import Usd, UsdGeom, UsdPhysics

    omni.usd.get_context().open_stage(ARENA_USD)
    app.update()
    stage = omni.usd.get_context().get_stage()

    base = None
    for prim in Usd.PrimRange(stage.GetPrimAtPath(ROBOT_PRIM)):
        if prim.GetName() == 'base_link' and prim.HasAPI(UsdPhysics.RigidBodyAPI):
            base = prim
            break
    if base is None:
        say('FAIL: no base_link rigid body under the robot')
        finish(2, 'no base_link')
    base_path = str(base.GetPath())
    say(f'base prim: {base_path}')

    try:
        og.Controller.edit(
            {'graph_path': '/TwinGraph', 'evaluator_name': 'execution'},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ('Tick', 'omni.graph.action.OnPlaybackTick'),
                    ('Odom', 'isaacsim.ros2.bridge.ROS2Subscriber'),
                ],
                og.Controller.Keys.CONNECT: [
                    ('Tick.outputs:tick', 'Odom.inputs:execIn'),
                ],
                og.Controller.Keys.SET_VALUES: [
                    ('Odom.inputs:topicName', args.topic),
                    ('Odom.inputs:messagePackage', 'nav_msgs'),
                    ('Odom.inputs:messageSubfolder', 'msg'),
                    ('Odom.inputs:messageName', 'Odometry'),
                ],
            })
        say(f'graph built: OnPlaybackTick -> ROS2Subscriber '
            f'nav_msgs/msg/Odometry on /{args.topic}')
    except Exception as e:
        say(f'FAIL: could not build the odom graph: {e}')
        finish(2, 'graph construction failed')

    import numpy as np
    from isaacsim.core.prims import SingleArticulation

    sim = SimulationContext()
    sim.initialize_physics()
    sim.play()
    for _ in range(30):
        sim.step(render=True)
        app.update()

    art = SingleArticulation(base_path)
    art.initialize()

    # Let it SETTLE under gravity before freezing, then read the ride height off the
    # settled pose. Freezing first left the robot floating 26 cm above the floor at its
    # authored import height -- it tracked poses perfectly and looked absurd.
    for _ in range(90):
        sim.step(render=False)
        app.update()

    spawn_pos, _ = art.get_world_pose()

    # Freeze AFTER reading the pose, and globally rather than per-prim. Applying
    # PhysxRigidBodyAPI to the links at runtime is a schema change that invalidates the
    # physics tensor view, and the next get_world_pose() dies inside
    # get_root_transforms(). Setting scene gravity touches no schemas.
    #
    # kinematicEnabled would also work in principle and does not work here: a kinematic
    # body leaves the dynamic set, so SingleArticulation cannot bind to it at all.
    sim.get_physics_context().set_gravity(0.0)
    say('settled, then scene gravity set to 0 (twin mode)')
    # The robot is planar and /odom carries z = 0, so adopting the message's z verbatim
    # would sink the chassis into the floor by its ride height. Only x, y and yaw come
    # from odometry; z stays where the model spawned.
    spawn_z = float(spawn_pos[2])
    say(f'spawn z = {spawn_z:.3f} m (held constant; odom is planar)')

    def attr(name):
        return og.Controller.attribute(f'/TwinGraph/Odom.outputs:{name}')

    A = {k: attr(k) for k in (
        'header:frame_id', 'header:stamp:sec', 'header:stamp:nanosec',
        'pose:pose:position:x', 'pose:pose:position:y',
        'pose:pose:orientation:x', 'pose:pose:orientation:y',
        'pose:pose:orientation:z', 'pose:pose:orientation:w')}

    xf = UsdGeom.XformCache()

    def pose():
        xf.Clear()
        m = xf.GetLocalToWorldTransform(base)
        t = m.ExtractTranslation()
        r = m.ExtractRotationMatrix()
        return (float(t[0]), float(t[1]), float(t[2]),
                math.atan2(r[0][1], r[0][0]))

    src = None
    if args.no_tf:
        say('NEGATIVE CONTROL: nothing will publish. This run must FAIL.')
    elif args.live:
        say('LIVE: expecting the real robot to publish /odom.')
        say('      (needs yahboomcar_bringup running -- the EKF publishes it)')
    else:
        src = subprocess.Popen(
            ['/usr/bin/python3', os.path.join(REPO, 'tools', '_tf_source.py'),
             '--hold', str(args.hold), '--domain', str(args.domain)],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            start_new_session=True)
        say(f'fixture started (pid {src.pid}), {args.hold}s per waypoint')

    duration = args.hold * len(WAYPOINTS) + 2.0
    say(f'sampling for {duration:.0f} s')
    samples = []
    n_frames_with_msg = 0
    msg_stamps = []          # (source stamp, wall time applied) per DISTINCT message
    pose_errors = []         # |commanded pose - achieved sim pose|, metres
    last_cmd_xy = None       # compared one frame late, see below
    last_stamp = None
    t0 = time.time()
    dropped = False
    while time.time() - t0 < duration:
        sim.step(render=True)
        app.update()

        # An empty frame_id means the subscriber has never decoded a message. Using it
        # as the gate avoids mistaking the all-zero default pose for a real one at the
        # origin -- which is exactly where waypoint 0 sits.
        try:
            frame = og.Controller.get(A['header:frame_id'])
        except Exception:
            frame = ''
        if frame and not (args.drop_test and dropped):
            n_frames_with_msg += 1
            # A LATCHED message is re-read every render frame. Counting frames therefore
            # measures the renderer, not the link. Distinct source stamps measure the
            # link. Audit finding.
            stamp_s = (int(og.Controller.get(A['header:stamp:sec']))
                       + int(og.Controller.get(A['header:stamp:nanosec'])) * 1e-9)
            if stamp_s != last_stamp:
                msg_stamps.append((stamp_s, time.time()))
                last_stamp = stamp_s
            px = float(og.Controller.get(A['pose:pose:position:x']))
            py = float(og.Controller.get(A['pose:pose:position:y']))
            qx = float(og.Controller.get(A['pose:pose:orientation:x']))
            qy = float(og.Controller.get(A['pose:pose:orientation:y']))
            qz = float(og.Controller.get(A['pose:pose:orientation:z']))
            qw = float(og.Controller.get(A['pose:pose:orientation:w']))
            art.set_world_pose(position=np.array([px, py, spawn_z]),
                               orientation=np.array([qw, qx, qy, qz]))
            # Compare against the pose commanded on the PREVIOUS frame. set_world_pose
            # does not take effect until the next physics step, so comparing within the
            # same frame charges the twin for a delay it has not had a chance to serve --
            # it reported a 500 mm "error" on each waypoint step, which is exactly the
            # step size. In live mode that would have been a false failure.
            achieved = pose()
            if last_cmd_xy is not None:
                pose_errors.append(math.hypot(achieved[0] - last_cmd_xy[0],
                                              achieved[1] - last_cmd_xy[1]))
            last_cmd_xy = (px, py)

        samples.append(pose())

        if args.drop_test and not dropped and time.time() - t0 > 3.0:
            # NEGATIVE CONTROL: author the base underground and confirm the fallen check
            # actually fires. A "fallen" test that has never seen a fall proves nothing.
            # Pose application stops here, otherwise the next frame would lift it back.
            art.set_world_pose(position=np.array([0.0, 0.0, -1.0]),
                               orientation=np.array([1.0, 0.0, 0.0, 0.0]))
            dropped = True
            say('  drop-test: base authored to z = -1.0, pose updates suspended')

    if src is not None:
        try:
            os.killpg(os.getpgid(src.pid), 15)
        except (ProcessLookupError, PermissionError):
            pass

    say('')
    say(f'{len(samples)} pose samples')
    if not samples:
        say('FAIL: no samples collected at all')
        finish(1, 'no samples')

    # ---- check 1: did the base fall through the world? ----
    min_z = min(s[2] for s in samples)
    say(f'min z = {min_z:.3f} (floor threshold {FLOOR_Z})')
    if min_z < FLOOR_Z:
        say('FAIL: base fell below the floor')
        finish(1, f'base fell to z={min_z:.3f}')

    # ---- check 2: did the pose move at all? ----
    span_xy = max(math.hypot(s[0] - samples[0][0], s[1] - samples[0][1])
                  for s in samples)
    say(f'max displacement from start = {span_xy:.3f} m')
    if span_xy < 0.05:
        say('FAIL: the base never moved. No pose is reaching the twin -- the graph is')
        say('      not receiving /tf, or the frame name does not match.')
        finish(1, 'no pose received')

    # ---- link quality: distinct messages, dropouts, lag, tracking error ----
    say('')
    say(f'distinct messages: {len(msg_stamps)}  '
        f'(vs {n_frames_with_msg} render frames that re-read a latched message)')
    if len(msg_stamps) < 2:
        say('FAIL: fewer than two distinct messages. The pose is latched, not live.')
        finish(1, 'no live message stream')

    gaps = [b[1] - a[1] for a, b in zip(msg_stamps, msg_stamps[1:])]
    gaps.sort()
    p95 = gaps[min(len(gaps) - 1, int(0.95 * (len(gaps) - 1)))]
    dropouts = [g for g in gaps if g > args.max_gap]
    rate = len(gaps) / (msg_stamps[-1][1] - msg_stamps[0][1])
    say(f'  message rate {rate:.1f} Hz   inter-arrival p95 {p95*1000:.0f} ms   '
        f'max {gaps[-1]*1000:.0f} ms')
    say(f'  dropouts over {args.max_gap*1000:.0f} ms: {len(dropouts)}')
    if pose_errors:
        pe = sorted(pose_errors)
        say(f'  sim-vs-commanded pose error: p95 {pe[int(0.95*(len(pe)-1))]*1000:.2f} mm, '
            f'max {pe[-1]*1000:.2f} mm')

    results_extra = {
        'distinct_messages': len(msg_stamps),
        'render_frames_with_message': n_frames_with_msg,
        'message_rate_hz': rate,
        'interarrival_p95_s': p95,
        'interarrival_max_s': gaps[-1],
        'dropouts': len(dropouts),
        'pose_error_max_m': max(pose_errors) if pose_errors else None,
    }

    if args.live:
        # The synthetic waypoints are meaningless against a real robot, which goes
        # wherever it was driven. Checking them in live mode -- as an earlier version
        # did -- tested nothing. What matters live is SYNCHRONISATION.
        say('')
        say('live mode: waypoint checks skipped (they describe the fixture, not a robot)')
        bad = []
        if gaps[-1] > args.max_gap:
            bad.append(f'worst message gap {gaps[-1]*1000:.0f} ms exceeds '
                       f'{args.max_gap*1000:.0f} ms')
        if pose_errors and max(pose_errors) > args.tol_xy:
            bad.append(f'pose error {max(pose_errors)*1000:.0f} mm exceeds tolerance')
        say('')
        if bad:
            for b in bad:
                say(f'FAIL: {b}')
            finish(1, '; '.join(bad))
        say(f'PASS: twin tracked live /{args.topic} at {rate:.1f} Hz, worst gap '
            f'{gaps[-1]*1000:.0f} ms, pose error under {args.tol_xy*1000:.0f} mm')
        say('NOTE: this bounds Isaac-side application lag against message arrival. It')
        say('      does NOT measure robot-to-Isaac end-to-end latency -- that needs a')
        say('      synchronised clock the firmware does not provide.')
        finish(0, 'live twin synchronised within bounds')

    # ---- check 3: was every waypoint actually visited? ----
    say('')
    say('waypoint tracking (best sample per waypoint):')
    worst = 0.0
    missed = []
    for i, (wx, wy, wyaw) in enumerate(WAYPOINTS):
        # Score on POSITION AND YAW together, each normalised by its own tolerance.
        # Matching on position alone is wrong whenever two waypoints share a position
        # and differ only in heading -- wp0 and wp4 both sit at the origin, so an
        # xy-only match handed wp4 a wp0 sample and reported a 90 deg yaw error against
        # a twin that was tracking correctly.
        best, best_s = None, None
        for smp in samples:
            e = (math.hypot(smp[0] - wx, smp[1] - wy) / args.tol_xy
                 + ang_err(smp[3], wyaw) / args.tol_yaw)
            if best is None or e < best:
                best, best_s = e, smp
        xy_e = math.hypot(best_s[0] - wx, best_s[1] - wy)
        yaw_e = ang_err(best_s[3], wyaw)
        best = xy_e
        ok = xy_e <= args.tol_xy and yaw_e <= args.tol_yaw
        worst = max(worst, best)
        say(f'  wp{i} ({wx:+.2f},{wy:+.2f},{math.degrees(wyaw):+4.0f}deg): '
            f'xy err {best*1000:5.0f} mm, yaw err {math.degrees(yaw_e):5.1f} deg  '
            f'{"ok" if ok else "MISS"}')
        if not ok:
            missed.append(i)

    if missed:
        say('')
        say(f'FAIL: {len(missed)} waypoint(s) never reached: {missed}')
        finish(1, f'waypoints missed: {missed}')

    say('')
    say(f'PASS: every waypoint tracked within {args.tol_xy*1000:.0f} mm and '
        f'{math.degrees(args.tol_yaw):.0f} deg (worst xy {worst*1000:.0f} mm)')
    finish(0, 'twin follows the commanded chassis pose')


if __name__ == '__main__':
    main()
