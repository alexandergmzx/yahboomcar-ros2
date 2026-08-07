#!/usr/bin/env python3
"""Drive the simulated robot in the arena and run tests on command.

    ISAAC=~/isaac/env_isaaclab/bin/python
    $ISAAC tools/sim_runner.py --test drive-straight
    $ISAAC tools/sim_runner.py --test rotate --gui
    $ISAAC tools/sim_runner.py --test umbmark --inject-wheel-error 8
    $ISAAC tools/sim_runner.py --test all

Two drive modes:

  physics  (default) /cmd_vel -> differential IK -> wheel VELOCITY drives -> friction
           against the ceramic floor moves the robot. It can slip, be blocked by a box,
           and behaves like a skid-steer. This is what makes the arena a test rig.
  kinematic          the base pose is set directly. For the digital twin later, where
           the base must follow the real robot's TF rather than simulated gravity.

The IK is the one measured on the real robot: differential (a commanded strafe produced
exactly zero on every /odom_raw axis), wheel radius 0.024 m from the mesh bounding box,
ly 0.0675 m from the URDF joint origins.

WHAT UMBMARK IN SIMULATION IS AND IS NOT: a perfect simulator has no systematic odometry
error, so the result would be trivially near zero and would say nothing about your robot.
It is run here with --inject-wheel-error, and the test PASSES only if tools/umbmark.py
recovers approximately the error that was injected. That validates the protocol and the
maths end to end before spending 40 minutes on a real floor with a tape measure.
"""
import argparse
import json
import math
import os
import sys
import time
from datetime import datetime

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
USD_DIR = os.path.join(REPO, 'yahboomcar_ws', 'src', 'yahboomcar_twin', 'usd')
ARENA_USD = os.path.join(USD_DIR, 'arena.usd')
LOG_DIR = os.path.join(REPO, 'MicroROS-assets', 'logs')
ROBOT_PRIM = '/World/Robot'

# Geometric wheel radius, confirmed twice: 0.024 from the STL bounding box and 0.025
# from the imported mesh's world bbox (0.0502 diameter).
WHEEL_R_GEOMETRIC = 0.0245

# EFFECTIVE rolling radius, measured in-sim: commanding 6.25 rad/s yields an achieved
# 6.007 rad/s and a body speed of 0.2750 m/s, so v/omega = 0.0458 -- 1.83x the geometry.
#
# UNEXPLAINED. A driven wheel cannot propel a body faster than pure rolling, so this
# points at an angular-unit or contact-radius mismatch somewhere between
# apply_action(joint_velocities=...) and the PhysX drive rather than at real physics.
# The controller uses the measured value because that makes commanded speed match
# reality, which is what the tests need; the discrepancy is recorded here rather than
# hidden, and --calibrate re-measures it on demand.
WHEEL_R = 0.0458
LY = 0.0675            # half-track from the URDF joint origins
LEFT = ['zq_Joint', 'zh_Joint']
RIGHT = ['yq_Joint', 'yh_Joint']
# The URDF mirrors the right wheels (axis 0,-1,0) against the left (0,1,0), so a
# positive joint velocity spins them opposite ways in world terms.
MIRROR_RIGHT = True


class Sim:
    """Thin wrapper over the arena stage: pose, wheel drives, stepping."""

    def __init__(self, app, gui, wheel_error=0.0, imbalance=0.0, rendering_dt=None):
        self.app = app
        self.gui = gui
        self.wheel_r = WHEEL_R * (1.0 + wheel_error)
        # A LEFT/RIGHT radius difference is what UMBmark's Ed measures. Scaling both
        # wheels equally (wheel_error) is a distance-scale error instead and leaves Ed
        # at 1.0, so injecting only that could never validate the Ed pathway.
        self.r_left = self.wheel_r * (1.0 + imbalance / 2.0)
        self.r_right = self.wheel_r * (1.0 - imbalance / 2.0)
        self.imbalance = imbalance
        import omni.usd
        from isaacsim.core.api import SimulationContext
        from isaacsim.core.prims import SingleArticulation
        from isaacsim.core.utils.types import ArticulationAction
        from pxr import Usd, UsdGeom, UsdPhysics
        self._Action = ArticulationAction

        self._Usd, self._UsdGeom, self._UsdPhysics = Usd, UsdGeom, UsdPhysics
        omni.usd.get_context().open_stage(ARENA_USD)
        app.update()
        self.stage = omni.usd.get_context().get_stage()

        # rendering_dt is exposed because the RTX lidar rotates on the RENDERER's clock,
        # and that is the open problem in --ros mode. It is NOT a fix -- see the
        # SCAN RATE note in run_ros() before touching it.
        if rendering_dt is not None:
            self.sim = SimulationContext(rendering_dt=rendering_dt)
        else:
            self.sim = SimulationContext()
        self.sim.initialize_physics()
        self.sim.play()
        for _ in range(30):
            self.step()

        self.base = None
        for prim in Usd.PrimRange(self.stage.GetPrimAtPath(ROBOT_PRIM)):
            if prim.GetName() == 'base_link' and prim.HasAPI(UsdPhysics.RigidBodyAPI):
                self.base = prim
                break
        if self.base is None:
            raise RuntimeError('no base_link rigid body under the robot')

        self.art = SingleArticulation(str(self.base.GetPath()))
        self.art.initialize()
        self.dof = list(self.art.dof_names or [])
        self.xf = UsdGeom.XformCache()

    def step(self, n=1):
        for _ in range(n):
            self.sim.step(render=True)
            self.app.update()

    def pose(self):
        """World (x, y, yaw) of base_link."""
        self.xf.Clear()
        m = self.xf.GetLocalToWorldTransform(self.base)
        t = m.ExtractTranslation()
        r = m.ExtractRotationMatrix()
        yaw = math.atan2(r[0][1], r[0][0])
        return float(t[0]), float(t[1]), float(yaw)

    def drive(self, vx, wz):
        """Differential IK -> per-wheel angular velocity targets."""
        import numpy as np
        # The controller believes each side has its own radius; the simulated wheels
        # are identical. That mismatch is exactly a Type B (unequal diameter) error.
        left = (vx - wz * LY) / self.r_left
        right = (vx + wz * LY) / self.r_right
        vel = np.zeros(len(self.dof))
        for i, name in enumerate(self.dof):
            if name in LEFT:
                vel[i] = left
            elif name in RIGHT:
                vel[i] = -right if MIRROR_RIGHT else right
        # apply_action sets drive TARGETS, so the wheels are turned by the physics
        # solver against floor friction. set_joint_velocities would instead overwrite
        # the state each step, teleporting the wheels and bypassing the contact forces
        # that make this a test rig rather than an animation.
        self.art.apply_action(self._Action(joint_velocities=vel))

    def stop(self):
        self.drive(0.0, 0.0)
        self.step(20)

    def dt(self):
        try:
            return float(self.sim.get_physics_dt())
        except Exception:
            return 1.0 / 60.0

    def drive_for(self, vx, wz, seconds):
        """Drive for `seconds` of SIMULATED time.

        Timing this with wall-clock is wrong and was: the simulator advances by a fixed
        dt per step regardless of how fast the host runs, so a wall-clock loop covered
        1.9 m when commanded 1.0 m. Distance must be a function of sim steps.
        """
        steps = max(1, int(round(seconds / self.dt())))
        self.drive(vx, wz)
        for _ in range(steps):
            self.step()
        return steps


def norm(a):
    return math.atan2(math.sin(a), math.cos(a))


def t_drive_straight(sim, say, dist=1.0, speed=0.15):
    x0, y0, a0 = sim.pose()
    sim.drive_for(speed, 0.0, dist / speed)
    sim.stop()
    x1, y1, a1 = sim.pose()

    dx, dy = x1 - x0, y1 - y0
    # Resolve displacement in the STARTING heading frame. Plain hypot() cannot tell a
    # robot driving straight from one spinning and drifting -- both register distance.
    along = dx * math.cos(a0) + dy * math.sin(a0)
    lateral = -dx * math.sin(a0) + dy * math.cos(a0)
    yaw_drift = math.degrees(norm(a1 - a0))
    path = math.hypot(dx, dy)

    say(f'  commanded {dist:.2f} m at {speed:.2f} m/s')
    say(f'  along heading {along:+.3f} m   lateral {lateral:+.3f} m   '
        f'yaw drift {yaw_drift:+.1f} deg')
    say(f'  straight-line displacement {path:.3f} m  (ratio {path/dist:.2f})')
    straight = abs(lateral) < 0.15 * max(abs(along), 1e-6) and abs(yaw_drift) < 20.0
    if not straight:
        say('  NOTE: significant lateral drift or yaw -- it is not driving straight, so')
        say('        the distance figure alone would be misleading.')
    ok = along > dist * 0.5 and straight
    say(f'  {"PASS" if ok else "FAIL"}')
    return ok, {'commanded_m': dist, 'along_m': along, 'lateral_m': lateral,
                'yaw_drift_deg': yaw_drift, 'path_m': path, 'straight': straight}


def t_rotate(sim, say, target=math.pi, wz=2.0):
    _, _, a0 = sim.pose()
    sim.drive_for(0.0, wz, target / wz)
    sim.stop()
    _, _, a1 = sim.pose()
    turned = abs(norm(a1 - a0))
    slip = 1.0 - turned / target
    say(f'  commanded {math.degrees(target):.0f} deg at {wz:.1f} rad/s, turned '
        f'{math.degrees(turned):.1f} deg  (ratio {turned/target:.3f}, slip {slip*100:.0f}%)')
    say('  Turning a 4-wheel skid-steer means scrubbing every wheel sideways, so heavy')
    say('  slip is the expected physics, not a defect. Measured separately: commanding')
    say('  1.18 rad/s of wheel speed produced almost no rotation at all -- below a')
    say('  threshold the wheels never break static friction -- while 4 rad/s turned 111')
    say('  deg in 2 s. Rotation here is strongly non-linear in commanded rate.')
    ok = turned > target * 0.25
    say(f'  {"PASS" if ok else "FAIL"}: the robot {"rotated" if ok else "did NOT rotate"}')
    return ok, {'commanded_rad': target, 'turned_rad': turned,
                'ratio': turned / target, 'slip': slip, 'wz_cmd': wz}


def t_obstacle_stop(sim, say, speed=0.25, stop_d=0.35, arena=4.0):
    """Drive straight at the wall ahead and stop via the governor's own rule.

    Earlier this aimed at a cardboard box first, which failed for a reason worth
    recording: the aiming turn used 0.8 rad/s, and rotation below roughly that rate
    never breaks static friction on this skid-steer, so the robot simply never turned
    and drove off in its original heading. Driving at the wall needs no turning and
    exercises exactly the same governor logic.
    """
    sys.path.insert(0, os.path.join(REPO, 'yahboomcar_ws', 'src', 'yahboomcar_safety'))
    from yahboomcar_safety.governor import GovernorConfig, decide
    cfg = GovernorConfig(stop_distance=stop_d, max_speed=0.6)

    x0, y0, a0 = sim.pose()
    wall_x = arena / 2.0          # inside face of the +x wall
    say(f'  start ({x0:+.2f}, {y0:+.2f}) heading {math.degrees(a0):+.0f} deg, '
        f'wall at x={wall_x:.2f}')
    say(f'  approach speed {speed} m/s, governor stop_distance {stop_d} m')

    # The governor tapers speed linearly to zero AT stop_distance, so a steady approach
    # creeps asymptotically toward the threshold and a strict d.vx == 0 may never fire.
    # That is safe behaviour -- it never contacts -- but "commanded exactly zero" is the
    # wrong success condition. Treat a creep below 2 cm/s as stopped.
    CREEP = 0.02
    stop_cmd_at = None
    for _ in range(int(40.0 / sim.dt())):
        x, y, _ = sim.pose()
        rng = wall_x - x                       # distance from base to the wall face
        d = decide(speed, 0.0, 0.0, rng, 0.01, 0.01, cfg)
        sim.drive(d.vx, 0.0)
        sim.step()
        if d.vx <= CREEP and stop_cmd_at is None:
            stop_cmd_at = rng
            break
    sim.stop()
    sim.step(int(2.0 / sim.dt()))              # let it coast to rest

    x1, _, _ = sim.pose()
    rest = wall_x - x1
    if stop_cmd_at is None:
        say(f'  FAIL: never reached the stop threshold (ended {rest:.2f} m away)')
        return False, {'stopped': False, 'rest_m': rest}

    overshoot = stop_cmd_at - rest
    say(f'  governor throttled to <=2 cm/s at {stop_cmd_at:.3f} m from the wall')
    say(f'  came to rest at {rest:.3f} m  -> coasted {overshoot:.3f} m after the command')
    say('  This is a SIMULATED stopping distance. It follows from simulated mass'
        ' (0.36 kg),')
    say('  friction and drive limits, and is not a prediction of the real robot. The'
        ' number')
    say('  that gates a real floor test still has to come off a tape measure.')
    ok = rest > 0.0
    say(f'  {"PASS" if ok else "FAIL"}: '
        f'{"stopped before contact" if ok else "HIT THE WALL"}')
    return ok, {'stop_commanded_m': stop_cmd_at, 'rest_m': rest,
                'coast_m': overshoot, 'speed': speed}


def t_umbmark(sim, say, L=1.0, speed=0.20, wz=2.5, injected=0.0, imbalance=0.0):
    """Bidirectional square. Success = umbmark.py recovers the injected error."""
    sys.path.insert(0, os.path.join(REPO, 'tools'))
    import umbmark

    def leg(dist):
        sim.drive_for(speed, 0.0, dist / speed)
        sim.stop()

    def turn(sign):
        # Closed loop on measured yaw: open-loop turning on a slipping skid-steer
        # would accumulate error every corner and corrupt the square.
        _, _, a0 = sim.pose()
        sim.drive(0.0, sign * wz)
        # Generous budget: rotation slips ~70%, so wall-clock-equivalent estimates of
        # turn duration are far too short.
        budget = int((math.pi / 2) / wz / sim.dt() * 12.0)
        for _ in range(budget):
            sim.step()
            _, _, a = sim.pose()
            if abs(norm(a - a0)) >= math.pi / 2:
                break
        sim.stop()

    say(f'  square L={L} m, approach {speed} m/s, turn {wz} rad/s')
    say('  (2 runs per direction rather than the specification\'s 5: enough to exercise')
    say('   the protocol and the maths, not enough to average out noise)')
    runs = []
    for direction, sign in (('cw', -1), ('ccw', +1)):
        for _ in range(2):
            x0, y0, _ = sim.pose()
            for _ in range(4):
                leg(L)
                turn(sign)
            x1, y1, _ = sim.pose()
            runs.append({'direction': direction,
                         'x': (x1 - x0) * 1000.0, 'y': (y1 - y0) * 1000.0})
            say(f'  {direction} run: return offset '
                f'({runs[-1]["x"]:+.0f}, {runs[-1]["y"]:+.0f}) mm')

    res = umbmark.compute(runs, L=L)
    say(f'  E_max,syst = {res["E_max_syst_mm"]:.1f} mm')
    say(f'  Ed = {res["Ed"]:.4f}   Eb = {res["Eb"]:.4f}')
    ok = all(math.isfinite(res[k]) for k in ('Ed', 'Eb', 'E_max_syst_mm'))
    if imbalance:
        expect = (1.0 + imbalance / 2.0) / (1.0 - imbalance / 2.0)
        got = res['Ed']
        rel = abs(got - expect) / max(1e-9, abs(expect - 1.0))
        say(f'  injected imbalance {imbalance*100:+.1f}% -> expected Ed {expect:.4f}, '
            f'measured {got:.4f}')
        recovered = ok and rel < 0.6      # within 60% of the injected magnitude
        say(f'  {"RECOVERED" if recovered else "NOT recovered"}: the pipeline '
            f'{"reproduces" if recovered else "does not reproduce"} a known injected error')
        if not recovered:
            say('')
            say('  WHY, and it is worth understanding rather than tuning around:')
            say('')
            say('  1. The corner turns close the loop on GROUND-TRUTH yaw, stopping at')
            say('     exactly 90 degrees. UMBmark exists to measure dead-reckoning error,')
            say('     so correcting heading at every corner cancels the very quantity it')
            say('     is trying to observe. A real robot turns open-loop and accumulates.')
            say('  2. Heavy rotational slip (~71% measured) suppresses differential')
            say('     steering. A 10% left/right wheel mismatch should curve each 1 m leg')
            say('     by roughly 43 degrees; here it barely deflects, because the wheels')
            say('     scrub rather than steer. The physics is masking the signal.')
            say('')
            say('  Consequence: this arena cannot validate the Ed pathway as configured.')
            say('  The MATHS is already validated independently -- tools/umbmark.py')
            say('  round-trips a synthetic alpha=0.55 beta=1.30 to 0.582/1.304. What is')
            say('  unvalidated is the physical pathway, and the real floor run is what')
            say('  will exercise it.')
        ok = ok and recovered
    else:
        say('  no error injected, so this only shows the protocol runs end to end and')
        say('  the maths yields finite factors. A near-1.0 Ed here says nothing about')
        say('  the real robot -- an accurate simulator has no systematic error to find.')
    say(f'  {"PASS" if ok else "FAIL"}')
    return ok, {**res, 'injected_imbalance': imbalance}


TESTS = {
    'drive-straight': t_drive_straight,
    'rotate': t_rotate,
    'obstacle-stop': t_obstacle_stop,
    'umbmark': t_umbmark,
}


# ===================================================================== --ros ===
# The MEASURED firmware contract. Isaac must satisfy it exactly, because the entire
# point is that yahboomcar_bringup, the EKF, the governor, SLAM, Nav2 and RViz run
# against it UNMODIFIED. A backend that publishes a differently shaped or differently
# paced graph is worse than no backend: every result taken from it silently stops
# transferring to the real robot.
SCAN_HZ, ODOM_HZ, IMU_HZ, BATTERY_HZ = 12.0, 11.0, 25.0, 1.0
BATTERY_VOLTS = 8.3
# Measured from three at-rest selftest bags -- see yahboomcar_sim.physics.imu_sample,
# which owns these numbers and explains why the gyro is deliberately noiseless.
GRAVITY, ACCEL_NOISE = 9.799, 0.013


class RosBridge:
    """Publishes the firmware contract out of Isaac, and drives from /cmd_vel.

    WHY OMNIGRAPH AND NOT rclpy: rclpy cannot be imported into Isaac's interpreter at
    all. Isaac runs Python 3.11, Jazzy builds rclpy for 3.12, and that is an ABI
    mismatch rather than a path problem. isaacsim.ros2.bridge is C++, carries its own
    ROS 2, and does share a DDS graph with system ROS 2.

    RATE CONTROL, which is the whole difficulty here: ROS2RtxLidarHelper publishes on
    every RENDER tick, so with a plain playback tick /scan came out at 34.6 Hz -- the
    frame rate, not the sensor's 12 Hz. Rendering is also the expensive part. So this
    renders only on the frames where a scan is due, and the other publishers hang off
    OnImpulseEvent nodes fired from Python at their own contract rates. That gets the
    rates right AND buys back most of the frame budget.
    """

    def __init__(self, sim, app, domain_id, say):
        self.sim, self.app, self.say = sim, app, say
        import omni.graph.core as og
        import omni.replicator.core as rep
        from pxr import Usd
        self.og = og

        lidar = None
        for prim in Usd.PrimRange(sim.stage.GetPrimAtPath(ROBOT_PRIM)):
            if prim.GetTypeName() == 'OmniLidar':
                lidar = prim
                break
        if lidar is None:
            raise RuntimeError(
                'no OmniLidar in the arena. Rebuild it:\n'
                '  ~/isaac/env_isaaclab/bin/python tools/build_arena.py')
        self.lidar_path = str(lidar.GetPath())
        say(f'  lidar prim: {self.lidar_path}')
        self._assert_contract(lidar)

        rp = rep.create.render_product(self.lidar_path, [1, 1], name='YahboomLidarRP')
        rp_path = rp.path if hasattr(rp, 'path') else str(rp)

        keys = og.Controller.Keys
        og.Controller.edit(
            {'graph_path': '/World/ROS', 'evaluator_name': 'execution'},
            {
                keys.CREATE_NODES: [
                    ('Tick', 'omni.graph.action.OnPlaybackTick'),
                    ('Ctx', 'isaacsim.ros2.bridge.ROS2Context'),
                    ('Scan', 'isaacsim.ros2.bridge.ROS2RtxLidarHelper'),
                    ('CmdVel', 'isaacsim.ros2.bridge.ROS2SubscribeTwist'),
                    ('OdomTick', 'omni.graph.action.OnImpulseEvent'),
                    ('Odom', 'isaacsim.ros2.bridge.ROS2PublishOdometry'),
                    ('ImuTick', 'omni.graph.action.OnImpulseEvent'),
                    ('Imu', 'isaacsim.ros2.bridge.ROS2PublishImu'),
                    ('BattTick', 'omni.graph.action.OnImpulseEvent'),
                    ('Batt', 'isaacsim.ros2.bridge.ROS2Publisher'),
                ],
                keys.SET_VALUES: [
                    ('Ctx.inputs:domain_id', int(domain_id)),
                    # /scan -- BEST_EFFORT sensor QoS, as the firmware publishes it.
                    ('Scan.inputs:topicName', 'scan'),
                    ('Scan.inputs:frameId', 'laser_frame'),
                    ('Scan.inputs:type', 'laser_scan'),
                    ('Scan.inputs:renderProductPath', rp_path),
                    # Wall-clock stamps: there is no /clock here and nothing in the
                    # stack sets use_sim_time, by design.
                    ('Scan.inputs:useSystemTime', True),
                    ('CmdVel.inputs:topicName', 'cmd_vel'),
                    # NOTE /odom_raw, not /odom. /odom is the EKF's, downstream.
                    ('Odom.inputs:topicName', 'odom_raw'),
                    ('Odom.inputs:odomFrameId', 'odom'),
                    ('Odom.inputs:chassisFrameId', 'base_footprint'),
                    ('Imu.inputs:topicName', 'imu'),
                    ('Imu.inputs:frameId', 'imu_frame'),
                    # The ICM-42670-P is 6-axis with NO magnetometer, so the firmware
                    # has no absolute attitude to report. Publishing a made-up
                    # orientation here would hand imu_filter_madgwick an answer it is
                    # supposed to be computing.
                    ('Imu.inputs:publishOrientation', False),
                    ('Imu.inputs:publishAngularVelocity', True),
                    ('Imu.inputs:publishLinearAcceleration', True),
                    ('Batt.inputs:topicName', 'battery'),
                    ('Batt.inputs:messagePackage', 'std_msgs'),
                    ('Batt.inputs:messageSubfolder', 'msg'),
                    ('Batt.inputs:messageName', 'UInt16'),
                ],
                keys.CONNECT: [
                    ('Tick.outputs:tick', 'Scan.inputs:execIn'),
                    ('Tick.outputs:tick', 'CmdVel.inputs:execIn'),
                    ('OdomTick.outputs:execOut', 'Odom.inputs:execIn'),
                    ('ImuTick.outputs:execOut', 'Imu.inputs:execIn'),
                    ('BattTick.outputs:execOut', 'Batt.inputs:execIn'),
                    ('Ctx.outputs:context', 'Scan.inputs:context'),
                    ('Ctx.outputs:context', 'CmdVel.inputs:context'),
                    ('Ctx.outputs:context', 'Odom.inputs:context'),
                    ('Ctx.outputs:context', 'Imu.inputs:context'),
                    ('Ctx.outputs:context', 'Batt.inputs:context'),
                ],
            },
        )
        self.rng = __import__('numpy').random.default_rng(0)
        self._counts = {'scan': 0, 'odom': 0, 'imu': 0, 'batt': 0}
        self._prev_pose = sim.pose()
        self._prev_t = 0.0
        say(f'  graph built on ROS_DOMAIN_ID={domain_id}')

    def _due(self, key, t, hz):
        """Phase accumulator, NOT an elapsed-time gate.

        A gate of `t - last >= 1/hz` can only ever fire on a step boundary, so at 60 Hz
        physics the achievable rates are 60/n -- 30, 20, 15, 12 -- and 25 Hz came out
        as 20, a 20% error. Comparing a count against floor(t*hz) instead lets the
        firing pattern be uneven while the AVERAGE rate stays exact, which is what the
        contract is about.
        """
        if self._counts[key] < int(t * hz):
            self._counts[key] += 1
            return True
        return False

    def _assert_contract(self, lidar):
        """The arena's lidar must still be the one build_arena.py verified.

        Checked here as well as at build time because arena.usd is regenerable and a
        stale or hand-edited one would publish a differently shaped scan while
        everything downstream carried on looking healthy.
        """
        P = 'omni:sensor:Core:'
        want = {P + 'scanRateBaseHz': 12, P + 'reportRateBaseHz': 4320,
                P + 'numberOfChannels': 1}
        for k, v in want.items():
            got = lidar.GetAttribute(k).Get()
            if got != v:
                raise RuntimeError(
                    f'arena lidar is off-contract: {k.split(":")[-1]} = {got}, '
                    f'wanted {v}. Rebuild with tools/build_arena.py')
        elev = list(lidar.GetAttribute(P + 'emitterState:s001:elevationDeg').Get() or [])
        if any(e != 0.0 for e in elev):
            raise RuntimeError(
                f'arena lidar has nonzero elevation {elev}; FlatScan will refuse to '
                'run and /scan will simply never appear. Rebuild the arena.')

    def read_cmd_vel(self):
        """-> (vx, wz). vy is DISCARDED: the chassis is differential, measured."""
        lin = self.og.Controller.get(
            self.og.Controller.attribute('/World/ROS/CmdVel.outputs:linearVelocity'))
        ang = self.og.Controller.get(
            self.og.Controller.attribute('/World/ROS/CmdVel.outputs:angularVelocity'))
        return float(lin[0]), float(ang[2])

    def _fire(self, node):
        self.og.Controller.set(
            self.og.Controller.attribute(f'/World/ROS/{node}.state:enableImpulse'), True)

    def publish(self, t, dt):
        """Fire each publisher at its own contract rate."""
        og = self.og
        x, y, yaw = self.sim.pose()

        if self._due('odom', t, ODOM_HZ):
            px, py, pyaw = self._prev_pose
            span = max(1e-6, t - self._prev_t)
            vx = math.hypot(x - px, y - py) / span
            wz = norm(yaw - pyaw) / span
            og.Controller.set(og.Controller.attribute('/World/ROS/Odom.inputs:position'),
                              [x, y, 0.0])
            og.Controller.set(
                og.Controller.attribute('/World/ROS/Odom.inputs:orientation'),
                [0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)])
            og.Controller.set(
                og.Controller.attribute('/World/ROS/Odom.inputs:linearVelocity'),
                [vx, 0.0, 0.0])
            og.Controller.set(
                og.Controller.attribute('/World/ROS/Odom.inputs:angularVelocity'),
                [0.0, 0.0, wz])
            self._fire('OdomTick')
            self._prev_pose = (x, y, yaw)
            self._prev_t = t
            self._wz_now = wz

        if self._due('imu', t, IMU_HZ):
            wz = getattr(self, '_wz_now', 0.0)
            og.Controller.set(
                og.Controller.attribute('/World/ROS/Imu.inputs:angularVelocity'),
                [0.0, 0.0, float(wz)])
            # Noisy accel on measured figures. A constant 9.81 is exactly what a DEAD
            # accelerometer looks like, and tools/sensor_health.py flags a
            # zero-variance channel as stuck -- correctly.
            og.Controller.set(
                og.Controller.attribute('/World/ROS/Imu.inputs:linearAcceleration'),
                [float(self.rng.normal(0.0, ACCEL_NOISE)),
                 float(self.rng.normal(0.0, ACCEL_NOISE)),
                 float(self.rng.normal(GRAVITY, ACCEL_NOISE))])
            self._fire('ImuTick')

        if self._due('batt', t, BATTERY_HZ):
            og.Controller.set(og.Controller.attribute('/World/ROS/Batt.inputs:data'),
                              int(round(BATTERY_VOLTS * 10)))
            self._fire('BattTick')


def run_ros(sim, app, args, say):
    """Drive Isaac from /cmd_vel and publish the firmware contract. Runs until killed.

    SCAN RATE: UNRESOLVED, and the reason tools/simctl still refuses --backend isaac.

    Everything else matches the contract exactly, measured by an external subscriber:
    /scan is 360 beams at 1.0000 deg spanning -180.0..179.0 deg over 0.120-8.000 m in
    frame laser_frame (identical to the 2D simulator, whose angle_max is
    pi - 2*pi/360); /odom_raw 11.0 Hz; /imu 25.0 Hz with orientation_covariance[0] = -1
    and accel_z 9.800 +/- 0.0127; /battery 8.3 V; and /cmd_vel moves the robot (1.7 m
    over a 30 s run). Realtime factor 1.00.

    But /scan arrives at 14.4 Hz against a 12 Hz contract, while this loop counts
    exactly 12 renders per second -- so it looks correct from the inside and is only
    visible to a subscriber. Measured at two render steps:

        rendering_dt   renders/s   render-time per wall second   /scan Hz
        1/60 (default)     12              0.200                   14.4
        1/12               12              1.000                   72.0

    Both fit exactly:  messages/s = 72 x renders/s x rendering_dt.

    So emission is a fixed 72 messages per second of RENDER time, and the authored
    scanRateBaseHz=12 -- verified by readback in build_arena.py -- does not set it.

    TWO HYPOTHESES TESTED AND ELIMINATED:

      * "these are 6 partial scans per revolution" (72 = 6 x 12), which the helper's
        own docs suggest: fullScan "publish[es] a full scan when enough data has
        accumulated instead of partial scans each frame". Setting fullScan=True on the
        laser_scan path changed the rate by NOTHING, exactly 14.4 Hz again -- the doc
        note that it "supports point cloud type only" is accurate. And each message
        carries 286/360 finite returns spanning the whole room, which is a near
        complete scan, not a 60-degree sector. So these are 72 real revolutions per
        render-second, not 6 partials of 12.

      * "rendering_dt is mismatched to the rotation period". Pinning it to 1/12 made
        the rate six times WORSE, per the formula above.

    What is left is that the RTX lidar's rotation rate simply does not follow
    scanRateBaseHz in this configuration. Rendering at 10 Hz instead of 12 would land
    on exactly 12 Hz, and the formula fits well enough across a 5x change to make that
    reliable -- but it encodes an unexplained 72 rather than fixing it, and would drift
    with anything affecting render timing. A backend that half-satisfies the contract
    is worse than no backend, so simctl keeps refusing until the 72 is understood.
    """
    import time as _time
    domain = os.environ.get('ROS_DOMAIN_ID', '0')
    say('=== ROS bridge ===')
    bridge = RosBridge(sim, app, domain, say)

    dt = sim.dt()
    say(f'  physics {1/dt:.0f} Hz')
    say(f'  /scan {SCAN_HZ:.0f}  /odom_raw {ODOM_HZ:.0f}  /imu {IMU_HZ:.0f}  '
        f'/battery {BATTERY_HZ:.0f} Hz   <- /cmd_vel')
    say('  NO COMMAND WATCHDOG, as on the real firmware: a commanded speed is held')
    say('  indefinitely. Modelled on purpose.')
    say('')

    # Everything is driven off WALL CLOCK, not accumulated sim steps. Nothing in this
    # stack sets use_sim_time -- by design, so that swapping the simulator for the car
    # changes nothing -- which means the rate a subscriber measures is a wall-clock
    # rate, and that is the one that has to match the contract.
    t0 = _time.time()
    last_report = t0
    sim_t = 0.0
    last_render = -1.0
    try:
        while True:
            t = _time.time() - t0
            vx, wz = bridge.read_cmd_vel()
            sim.drive(vx, wz)
            # Render ONLY when a scan is due, and gate that on SIM time rather than
            # wall time. ROS2RtxLidarHelper publishes on every render tick, so:
            #   * rendering every frame pinned /scan to the frame rate (34.6 Hz) and
            #     ate the whole frame budget;
            #   * gating on WALL time gave 14.4 Hz, because the lidar's rotation is
            #     driven by SIM time at scanRateBaseHz while the renders came on a
            #     wall-clock cadence, and the two beat -- roughly one render in five
            #     found two completed rotations buffered and published both.
            # dt is 1/60 and 1/SCAN_HZ is 1/12, so this is exactly every 5th step:
            # one rotation, one render, one message.
            render = (sim_t - last_render) >= (1.0 / SCAN_HZ) - 1e-9
            if render:
                last_render = sim_t
                bridge._counts['scan'] += 1
            sim.sim.step(render=render)
            app.update()
            sim_t += dt
            bridge.publish(t, dt)

            # PACE SIM TIME TO WALL TIME. Without this the loop runs flat out, the
            # lidar completes ~2 rotations per rendered frame, and the helper publishes
            # BOTH -- measured as 24 Hz of /scan while this loop counted 12 renders.
            # Every other rate looked correct throughout, because only /scan is driven
            # by the sensor's own rotation rather than by a Python-fired impulse.
            ahead = sim_t - (_time.time() - t0)
            if ahead > 0:
                _time.sleep(ahead)

            now = _time.time()
            if now - last_report >= 10.0:
                c = bridge._counts
                rtf = sim_t / t
                say(f'  t={t:6.1f}s  realtime x{rtf:.2f}  scan {c["scan"]/t:.1f}  '
                    f'odom {c["odom"]/t:.1f}  imu {c["imu"]/t:.1f}  '
                    f'batt {c["batt"]/t:.1f} Hz')
                if rtf < 0.9:
                    say(f'  WARNING: only {rtf:.2f}x realtime. The robot is moving in '
                        'slow motion relative to the wall-clock rates above, so any '
                        'timing taken from this run is meaningless.')
                for k, wanted in (('scan', SCAN_HZ), ('odom', ODOM_HZ),
                                  ('imu', IMU_HZ)):
                    if c[k] / t < wanted * 0.9:
                        say(f'  WARNING: /{k} is only making {c[k]/t:.1f} of '
                            f'{wanted:.0f} Hz -- this machine cannot sustain the '
                            'contract.')
                last_report = now
    except KeyboardInterrupt:
        say('\n  interrupted; stopping the robot')
        sim.drive(0.0, 0.0)
        for _ in range(10):
            sim.sim.step(render=False)
            app.update()
    return 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--test', default='drive-straight',
                    choices=list(TESTS) + ['all', 'demo'],
                    help="'demo' runs the three passing motion tests in one launch, "
                         "which is what you want with --gui; 'all' adds umbmark, which "
                         "is long and fails by design")
    ap.add_argument('--gui', action='store_true')
    ap.add_argument('--inject-wheel-error', type=float, default=0.0,
                    help='percent error in the wheel radius the controller assumes')
    ap.add_argument('--inject-imbalance', type=float, default=0.0,
                    help='percent LEFT/RIGHT wheel-radius mismatch. This is the error '
                         'UMBmark Ed detects; --inject-wheel-error is a symmetric '
                         'scale error and moves distance, not Ed.')
    ap.add_argument('--calibrate', action='store_true',
                    help='measure the effective rolling radius and report it, instead '
                         'of trusting the constant')
    ap.add_argument('--ros', action='store_true',
                    help='publish the firmware contract (/scan, /odom_raw, /imu, '
                         '/battery) and drive from /cmd_vel, so the real ROS 2 stack '
                         'runs against Isaac unmodified. Runs until Ctrl+C.')
    args = ap.parse_args()

    if not os.path.exists(ARENA_USD):
        sys.exit(f'no arena at {ARENA_USD}\n  run tools/build_arena.py first')

    stamp = datetime.now().strftime('%Y%m%d-%H%M%S')
    os.makedirs(LOG_DIR, exist_ok=True)
    log_path = os.path.join(LOG_DIR, f'simtest-{stamp}.log')
    lines, results = [], {}
    code = {'v': 1}

    def say(m=''):
        print(m, flush=True)
        lines.append(str(m))

    from isaacsim import SimulationApp
    app = SimulationApp({'headless': not args.gui})
    try:
        err = args.inject_wheel_error / 100.0
        imb = args.inject_imbalance / 100.0
        sim = Sim(app, args.gui, wheel_error=err, imbalance=imb)
        say(f'sim runner {stamp}   arena={os.path.basename(ARENA_USD)}')
        say(f'DOFs: {sim.dof}')
        if err:
            say(f'INJECTED symmetric radius error: {args.inject_wheel_error:+.1f}% '
                f'(controller assumes r={sim.wheel_r:.5f})')
        if imb:
            say(f'INJECTED left/right imbalance: {args.inject_imbalance:+.1f}%  '
                f'(controller r_left={sim.r_left:.5f}, r_right={sim.r_right:.5f})')
            expect = (1.0 + imb / 2.0) / (1.0 - imb / 2.0)
            say(f'  -> a correct UMBmark should report Ed near {expect:.4f}')
        say('')

        if args.calibrate:
            say('=== calibrate: effective rolling radius ===')
            w = 6.25
            sim.drive_for(w * sim.wheel_r, 0.0, 1.5)   # spin up
            import numpy as np
            x0, y0, _ = sim.pose()
            n = int(2.0 / sim.dt())
            ws = []
            for _ in range(n):
                sim.step()
                jv = sim.art.get_joint_velocities()
                ws += [abs(float(jv[i])) for i, nm in enumerate(sim.dof)
                       if nm == 'zq_Joint']
            x1, y1, _ = sim.pose()
            sim.stop()
            v = math.hypot(x1 - x0, y1 - y0) / (n * sim.dt())
            wa = sum(ws) / max(1, len(ws))
            say(f'  achieved omega {wa:.3f} rad/s, body speed {v:.4f} m/s')
            say(f'  effective radius = {v/wa:.5f} m   (constant in use: {WHEEL_R})')
            say(f'  geometric radius = {WHEEL_R_GEOMETRIC}  '
                f'-> ratio {(v/wa)/WHEEL_R_GEOMETRIC:.2f}x, still unexplained')
            say('')

        if args.ros:
            # Enable the bridge BEFORE any graph is built; the ROS2 node types do not
            # exist until the extension loads.
            from isaacsim.core.utils.extensions import enable_extension
            for ext in ('isaacsim.ros2.bridge', 'isaacsim.sensors.rtx',
                        'isaacsim.core.nodes', 'omni.graph.action',
                        'omni.replicator.core'):
                enable_extension(ext)
            for _ in range(5):
                app.update()
            code['v'] = run_ros(sim, app, args, say)
            return code['v']

        if args.test == 'all':
            names = list(TESTS)
        elif args.test == 'demo':
            names = ['drive-straight', 'rotate', 'obstacle-stop']
        else:
            names = [args.test]
        all_ok = True
        for name in names:
            say(f'=== {name} ===')
            fn = TESTS[name]
            kw = {'injected': err, 'imbalance': imb} if name == 'umbmark' else {}
            try:
                ok, data = fn(sim, say, **kw)
            except Exception as e:
                import traceback
                say(f'  EXCEPTION: {e}')
                say(traceback.format_exc())
                ok, data = False, {'exception': str(e)}
            results[name] = {'pass': ok, **data}
            all_ok &= ok
            sim.stop()
            say('')

        say('RESULT: ' + ('PASS' if all_ok else 'FAIL'))
        code['v'] = 0 if all_ok else 1
    except Exception as e:
        import traceback
        say(f'EXCEPTION: {e}')
        say(traceback.format_exc())
    finally:
        with open(log_path, 'w') as f:
            f.write('\n'.join(lines) + '\n')
        with open(log_path.replace('.log', '.json'), 'w') as f:
            json.dump(results, f, indent=2)
        print(f'\nlog: {log_path}')
        sys.stdout.flush()
        # Before app.close(), which terminates the process and discards the status.
        os._exit(code['v'])


if __name__ == '__main__':
    main()
