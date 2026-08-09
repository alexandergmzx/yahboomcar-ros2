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

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
# Layout-aware paths (fleet D-12): the old REPO-relative constants here survived the
# extraction unnoticed and killed the isaac backend -- this file exited at the arena
# check while simctl polled topics for its full 420 s budget. See _layout.py.
from _layout import REPO, USD_DIR, LOG_DIR, pkg_dir            # noqa: E402
# YAHBOOM_ARENA_USD selects a VARIANT (absolute path or basename under USD_DIR) --
# simctl --fun points here at arena_fun.usd (feather boxes). The canonical
# arena.usd stays the default and the only one calibration results may come from.
_arena_env = os.environ.get('YAHBOOM_ARENA_USD', '')
ARENA_USD = (_arena_env if os.path.isabs(_arena_env)
             else os.path.join(USD_DIR, _arena_env or 'arena.usd'))
ROBOT_PRIM = '/World/Robot'

# Geometric wheel radius, confirmed twice: 0.024 from the STL bounding box and 0.025
# from the imported mesh's world bbox (0.0502 diameter).
WHEEL_R_GEOMETRIC = 0.0245

# EFFECTIVE rolling radius as seen through the joint-velocity API: 0.0458, i.e. 1.9x
# the geometry. SHARPENED 2026-08-08: the anomaly SURVIVED replacing the faceted hull
# colliders with analytic cylinders of EXACTLY r=0.024 -- commanded 0.2 m/s through
# this constant yields a measured 0.210 m/s ground-truth speed on wheels whose true
# radius is beyond doubt. A real contact cannot roll at twice its radius, so this is a
# UNITS/CONVENTION factor of ~2.0 between apply_action/get_joint_velocities and the
# physical angular velocity, not contact geometry. Where the 2 comes from is still
# unexplained; the constant compensates it exactly, and --calibrate re-measures.
#
# CORROBORATED 2026-08-08 on a SECOND robot: the RaspTank twin (fresh URDF,
# analytic cylinders authored at exactly r=0.025) measured 0.399 m/s of ground
# truth against 0.2 m/s commanded through the geometric radius — the same ~2.0,
# on hardware-independent geometry. That rules out anything specific to this
# robot's import; it is a property of the Isaac joint-velocity pathway itself.
# Evidence: rasptank-ros2/tools/build_rasptank_arena.py forward gate;
# fleet docs/research-log.md session-5 table.
WHEEL_R = 0.0458
LY = 0.0675            # half-track from the URDF joint origins
LEFT = ['zq_Joint', 'zh_Joint']
RIGHT = ['yq_Joint', 'yh_Joint']
# The URDF mirrors the right wheels (axis 0,-1,0) against the left (0,1,0), so a
# positive joint velocity spins them opposite ways in world terms.
MIRROR_RIGHT = True

# SKID-STEER SLIP COMPENSATION, measured on the cylinder-wheel arena (2026-08-08).
# Even with clean cylinder colliders and low-friction rear wheels, achieved yaw tracks
# commanded yaw as roughly
#       achieved = YAW_GAIN * commanded - YAW_LOSS        (zero below the breakaway)
# because turning a skid-steer IS controlled slip -- the lateral friction the front
# wheels need for drive also resists the turn. Real skid-steer firmware compensates
# with exactly this kind of feedforward. Measured points: wz 2.0 -> 0.645, 3.0 -> 1.23;
# below ~1.5 rad/s of wheel-speed target the wheels do not break static friction at
# all, so small commanded yaws are lifted to the working region rather than dropped.
YAW_GAIN = 0.585
YAW_LOSS = 0.52
# The compensated command is CAPPED at the largest value measured stable. Uncapped
# extrapolation collapsed: a commanded 3.0 rad/s became a 6.0 rad/s wheel-differential
# request and yaw fell to 0.075 rad/s (wild slip), while 4.31 -- the compensated form
# of a commanded 2.0 -- still yielded 1.225. So the achievable body yaw tops out around
# 1.2 rad/s; the vendor keyboard's default turn is 1.0 and the governor caps at 1.5,
# both inside the working range.
YAW_CMD_CAP = 4.31


def compensate_yaw(wz):
    """Measured slip-compensation feedforward: desired body yaw -> wheel-differential
    command. Pure function so the constants are pytest-testable outside Isaac."""
    if wz == 0.0:
        return 0.0
    return math.copysign(min(YAW_CMD_CAP, (abs(wz) + YAW_LOSS) / YAW_GAIN), wz)


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
        """Differential IK -> per-wheel angular velocity targets.

        wz passes through the measured slip-compensation feedforward: the wheel
        DIFFERENTIAL needed for a desired body yaw is larger than kinematics says,
        because a skid-steer turns by slipping. See YAW_GAIN/YAW_LOSS.
        """
        import numpy as np
        wz = compensate_yaw(wz)
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


def t_rotate(sim, say, target=math.pi, wz=1.0):
    # wz=1.0 (was 2.0): the teleop/governor working range, where tracking is the
    # claim under test -- measured 98% after the cylinder/rear-slide/feedforward fix.
    # Yaw is integrated STEPWISE (unwrapped); the old endpoint-wrap arithmetic could
    # not tell 350 degrees from -10 and only asserted >25% rotation, which an audit
    # correctly called unable to validate anything. See t_rotate_body below.
    _, _, a0 = sim.pose()
    # UNWRAPPED stepwise integration: sum small per-step deltas instead of wrapping
    # the endpoint difference, so 350 degrees is not mistaken for -10.
    steps = max(1, int(round((target / wz) / sim.dt())))
    sim.drive(0.0, wz)
    turned = 0.0
    prev = a0
    for _ in range(steps):
        sim.step()
        _, _, a = sim.pose()
        turned += abs(norm(a - prev))
        prev = a
    sim.stop()
    ratio = turned / target
    say(f'  commanded {math.degrees(target):.0f} deg at {wz:.1f} rad/s, turned '
        f'{math.degrees(turned):.1f} deg (unwrapped)  tracking {100*ratio:.0f}%')
    # The claim under test is the post-fix behaviour: cylinder wheels + rear-slide +
    # slip feedforward track ~98% at wz=1.0. 85% is the regression floor -- the
    # pre-fix behaviour was 0%, and the old ">25% of target" bar could not tell the
    # fixed simulator from a broken one.
    ok = ratio > 0.85
    say(f'  {"PASS" if ok else "FAIL"}: tracking {"holds" if ok else "REGRESSED"} '
        f'(floor 85%)')
    return ok, {'commanded_rad': target, 'turned_rad': turned,
                'ratio': ratio, 'wz_cmd': wz}


def t_obstacle_stop(sim, say, speed=0.25, stop_d=0.35, arena=4.0):
    """Drive straight at the wall ahead and stop via the governor's own rule.

    Earlier this aimed at a cardboard box first, which failed for a reason worth
    recording: the aiming turn used 0.8 rad/s, and rotation below roughly that rate
    never breaks static friction on this skid-steer, so the robot simply never turned
    and drove off in its original heading. Driving at the wall needs no turning and
    exercises exactly the same governor logic.
    """
    sys.path.insert(0, pkg_dir('yahboomcar_safety'))
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
# Same firmware-acceptance assumption as the 2D backend (vendor keyboard
# defaults; the real cap is unmeasured). Duplicated as literals because rclpy
# -- and thus yahboomcar_sim -- cannot be imported into Isaac's 3.11.
FIRMWARE_MAX_SPEED = 1.0
FIRMWARE_MAX_YAW = 5.0
BATTERY_VOLTS = 8.3

# MEASURED, AND UNEXPLAINED. ROS2RtxLidarHelper emits a fixed 72 LaserScan messages per
# second of RENDER time, and the scanRateBaseHz=12 authored on the prim -- verified by
# readback in build_arena.py -- does not set it. Two render steps, both fitting
# `messages/s = 72 x renders/s x rendering_dt` exactly:
#
#     rendering_dt   renders/s   render-time per wall second   /scan Hz
#     1/60 (default)     12              0.200                   14.4
#     1/12               12              1.000                   72.0
#
# Two explanations were tested and eliminated: fullScan=True changed the rate by nothing
# (its "point cloud type only" note is accurate, and each message carries 286/360 finite
# returns, so these are whole revolutions and not partials), and pinning rendering_dt to
# the scan period made it six times worse.
#
# So the render cadence is CALIBRATED against this constant rather than derived from the
# sensor's own configured rate. That is a compromise, taken deliberately: it lands /scan
# on the contract, but it encodes a number nobody has explained, and it will drift if
# anything changes render timing. run_ros() prints that caveat at startup, every run.
LIDAR_MSGS_PER_RENDER_SECOND = 72.0
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
                    # GROUND TRUTH, on a topic the firmware does not have. It exists so
                    # a test can score odometry against what actually happened -- the
                    # 2D simulator publishes the same topic for the same reason. It is
                    # NOT part of the contract and nothing in the stack subscribes.
                    ('TruthTick', 'omni.graph.action.OnImpulseEvent'),
                    ('Truth', 'isaacsim.ros2.bridge.ROS2PublishOdometry'),
                ],
                keys.SET_VALUES: [
                    ('Ctx.inputs:domain_id', int(domain_id)),
                    # NOT /scan: a large minority of RTX helper messages arrive with
                    # their content circularly rotated ~+/-85 deg (revolutions
                    # assembled across the 1.2-messages-per-render phase seam)
                    # [measured 2026-08-09; histograms in _scan_frame_relay.py]. The
                    # raw output goes to a private topic and _scan_frame_relay.py
                    # (spawned by run_ros) validates each scan against the shared
                    # arena geometry and publishes only clean ones as /scan.
                    ('Scan.inputs:topicName', 'scan_isaac_raw'),
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
                    ('Truth.inputs:topicName', 'sim/ground_truth'),
                    ('Truth.inputs:odomFrameId', 'world'),
                    ('Truth.inputs:chassisFrameId', 'base_link_truth'),
                ],
                keys.CONNECT: [
                    ('Tick.outputs:tick', 'Scan.inputs:execIn'),
                    ('Tick.outputs:tick', 'CmdVel.inputs:execIn'),
                    ('OdomTick.outputs:execOut', 'Odom.inputs:execIn'),
                    ('ImuTick.outputs:execOut', 'Imu.inputs:execIn'),
                    ('BattTick.outputs:execOut', 'Batt.inputs:execIn'),
                    ('TruthTick.outputs:execOut', 'Truth.inputs:execIn'),
                    ('Ctx.outputs:context', 'Scan.inputs:context'),
                    ('Ctx.outputs:context', 'CmdVel.inputs:context'),
                    ('Ctx.outputs:context', 'Odom.inputs:context'),
                    ('Ctx.outputs:context', 'Imu.inputs:context'),
                    ('Ctx.outputs:context', 'Batt.inputs:context'),
                    ('Ctx.outputs:context', 'Truth.inputs:context'),
                ],
            },
        )
        self.rng = __import__('numpy').random.default_rng(0)
        self._counts = {'scan': 0, 'odom': 0, 'imu': 0, 'batt': 0, 'truth': 0}
        # Encoder-integrated pose, starting at the origin exactly as the firmware's does.
        # It is deliberately NOT seeded from ground truth: the whole point is that it
        # drifts away from the world.
        self._odom = (0.0, 0.0, 0.0)
        self._prev_true_yaw = sim.pose()[2]
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
        """-> (vx, wz). vy is DISCARDED: the chassis is differential, measured.

        CLAMPED to the same firmware-acceptance caps the 2D backend applies. Isaac used
        to pass commands through UNCAPPED, so the two backends disagreed about the very
        firmware they both claim to model -- a 2.0 m/s command was a 1.0 m/s robot on
        one and a 2.0 m/s robot on the other. The caps are assumptions (vendor keyboard
        defaults); see yahboomcar_sim.physics.FIRMWARE_MAX_SPEED. Both backends share
        the assumption so they can at least be wrong identically.
        """
        lin = self.og.Controller.get(
            self.og.Controller.attribute('/World/ROS/CmdVel.outputs:linearVelocity'))
        ang = self.og.Controller.get(
            self.og.Controller.attribute('/World/ROS/CmdVel.outputs:angularVelocity'))
        vx = max(-FIRMWARE_MAX_SPEED, min(FIRMWARE_MAX_SPEED, float(lin[0])))
        wz = max(-FIRMWARE_MAX_YAW, min(FIRMWARE_MAX_YAW, float(ang[2])))
        return vx, wz

    def wheel_odometry(self, dt):
        """Integrate WHEEL ROTATION into a pose, the way encoders do. -> (x, y, yaw, v, w)

        THIS IS THE POINT, and it is not a detail. /odom_raw on the real robot is
        ENCODER-DERIVED: it measures how far the wheels turned, not how far the robot
        went. Those differ whenever the wheels slip, and on this robot's stand they
        differ by 92% -- the wheels claimed 1.602 m while the lidar saw 0.128 m. That
        disagreement between wheels and world is the single thing the whole localisation
        and fusion effort here exists to study.

        This used to derive /odom_raw from the physics engine's GROUND TRUTH chassis
        displacement, which silently deleted exactly that: perfect odometry, no slip
        anywhere, so an EKF or a scan matcher tested against it never saw the error it
        was built to detect. It also used hypot(), so REVERSE motion was published as a
        POSITIVE velocity. Audit finding.

        Reading the articulation's joint velocities instead means slip appears for free:
        PhysX can spin a wheel against a floor it cannot grip, and the encoders here
        report that spin exactly as the real ones would.
        """
        import numpy as np
        jv = self.sim.art.get_joint_velocities()
        left = [float(jv[i]) for i, n in enumerate(self.sim.dof) if n in LEFT]
        right = [float(jv[i]) for i, n in enumerate(self.sim.dof) if n in RIGHT]
        if not left or not right:
            return self._odom + (0.0, 0.0)
        wl = float(np.mean(left))
        # The URDF mirrors the right wheels, so a positive joint velocity spins them
        # the opposite way in world terms -- the same MIRROR_RIGHT convention drive() uses.
        wr = -float(np.mean(right)) if MIRROR_RIGHT else float(np.mean(right))

        # Differential kinematics, the exact inverse of drive(). WHEEL_R is the measured
        # effective rolling radius, not the geometric one; see the constant's note.
        v = (wl + wr) * 0.5 * WHEEL_R
        w = (wr - wl) * WHEEL_R / (2.0 * LY)

        x, y, yaw = self._odom
        yaw = norm(yaw + w * dt)
        x += v * math.cos(yaw) * dt
        y += v * math.sin(yaw) * dt
        self._odom = (x, y, yaw)
        return x, y, yaw, v, w

    def _stamp(self, node):
        """Set inputs:timeStamp before firing. It DEFAULTS TO 0.0.

        A zero stamp is not cosmetic. imu_filter_madgwick refuses to update orientation
        at all -- "The IMU message time stamp is zero, and the parameter constant_dt is
        not set" -- so the EKF gets no fused attitude, and TF ends up at time 0. The 2D
        simulator has always stamped its messages; the Isaac backend did not, and it
        showed up only in the bringup log.

        WALL CLOCK, because nothing in this stack sets use_sim_time -- by design, so
        that swapping the simulator for the car changes nothing.
        """
        import time as _t
        self.og.Controller.set(
            self.og.Controller.attribute(f'/World/ROS/{node}.inputs:timeStamp'),
            float(_t.time()))

    def _fire(self, node):
        self.og.Controller.set(
            self.og.Controller.attribute(f'/World/ROS/{node}.state:enableImpulse'), True)

    def publish(self, t, dt):
        """Fire each publisher at its own contract rate."""
        og = self.og
        # Encoder-derived pose, integrated every step so slip accumulates properly
        # rather than being sampled at the publish rate.
        ox, oy, oyaw, ov, ow = self.wheel_odometry(dt)
        # The BODY's true yaw rate, for the IMU. The IMU is bolted to the chassis, so it
        # measures the body -- which is why it disagrees with the wheels under slip, and
        # why publishing the wheel-derived rate here would have hidden that too.
        _, _, tyaw = self.sim.pose()
        body_w = norm(tyaw - self._prev_true_yaw) / max(1e-6, t - self._prev_t)
        self._prev_true_yaw = tyaw
        self._prev_t = t

        if self._due('odom', t, ODOM_HZ):
            og.Controller.set(og.Controller.attribute('/World/ROS/Odom.inputs:position'),
                              [ox, oy, 0.0])
            og.Controller.set(
                og.Controller.attribute('/World/ROS/Odom.inputs:orientation'),
                [0.0, 0.0, math.sin(oyaw / 2.0), math.cos(oyaw / 2.0)])
            # SIGNED. hypot() made reverse motion read as positive.
            og.Controller.set(
                og.Controller.attribute('/World/ROS/Odom.inputs:linearVelocity'),
                [ov, 0.0, 0.0])
            og.Controller.set(
                og.Controller.attribute('/World/ROS/Odom.inputs:angularVelocity'),
                [0.0, 0.0, ow])
            self._stamp('Odom')
            self._fire('OdomTick')

        if self._due('imu', t, IMU_HZ):
            wz = body_w
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
            self._stamp('Imu')
            self._fire('ImuTick')

        # Ground truth at the odometry rate, so the two are directly comparable.
        if self._due('truth', t, ODOM_HZ):
            tx, ty, tyw = self.sim.pose()
            og.Controller.set(og.Controller.attribute('/World/ROS/Truth.inputs:position'),
                              [tx, ty, 0.0])
            og.Controller.set(
                og.Controller.attribute('/World/ROS/Truth.inputs:orientation'),
                [0.0, 0.0, math.sin(tyw / 2.0), math.cos(tyw / 2.0)])
            self._stamp('Truth')
            self._fire('TruthTick')

        if self._due('batt', t, BATTERY_HZ):
            og.Controller.set(og.Controller.attribute('/World/ROS/Batt.inputs:data'),
                              int(round(BATTERY_VOLTS * 10)))
            self._fire('BattTick')


def run_ros(sim, app, args, say):
    """Drive Isaac from /cmd_vel and publish the firmware contract. Runs until killed.

    SCAN RATE: UNRESOLVED, and the reason tools/simctl still refuses --backend isaac.

    /odom_raw is ENCODER-DERIVED (see wheel_odometry), so it slips and drifts like the
    real thing and reverse reads negative. It was ground-truth-derived via hypot() until
    an audit caught it: perfect odometry with no slip, and reverse reported positive.

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
    scanRateBaseHz in this configuration.

    SO THE RENDER CADENCE IS CALIBRATED against the measured 72, rather than derived
    from the sensor's configured rate: render_hz = SCAN_HZ / (72 * rendering_dt), which
    at the default rendering_dt of 1/60 means rendering 10 times a second to get 12
    scans. That is a COMPROMISE and is announced at startup on every run. It lands
    /scan on the contract; it does not explain the 72, and it will drift if anything
    changes render timing. The alternative was leaving the backend unusable over a 20%
    rate error, which costs the physics, collisions and appearance Isaac exists for.
    """
    import time as _time
    domain = os.environ.get('ROS_DOMAIN_ID', '0')
    say('=== ROS bridge ===')
    bridge = RosBridge(sim, app, domain, say)

    dt = sim.dt()
    try:
        rdt = float(sim.sim.get_rendering_dt())
    except Exception:
        rdt = dt
    if rdt <= 0:
        rdt = dt
    # Initial guess from the last-measured emission constant; the LOOP below replaces
    # it with feedback from a real subscriber, because the constant has now drifted
    # twice (72 -> ~84 msgs per render-second between arena builds). A constant is
    # testimony; only a subscriber is a measurement.
    render_hz = SCAN_HZ / (LIDAR_MSGS_PER_RENDER_SECOND * rdt)

    # FEEDBACK SENSOR: rclpy cannot exist in this interpreter (Isaac is 3.11, Jazzy
    # builds rclpy for 3.12), so a system-python child measures the real /scan rate
    # and reports through a file. Dies with this process group.
    import subprocess as _sp
    import tempfile as _tf
    rate_file = os.path.join(_tf.gettempdir(), f'scan_rate_{os.getpid()}.txt')
    probe = _sp.Popen(
        ['bash', '-c',
         'source /opt/ros/jazzy/setup.bash 2>/dev/null && '
         f'exec python3 {REPO}/tools/_scan_rate_probe.py --out {rate_file}'],
        env=dict(os.environ),
        stdout=_sp.DEVNULL, stderr=_sp.DEVNULL)
    say(f'  scan-rate feedback probe: pid {probe.pid} (system python; the only real '
        'measurement)')
    # SCAN FILTER: a minority of RTX helper messages carry phase-corrupted content
    # (see the graph comment at Scan.inputs:topicName); this child validates each
    # scan_isaac_raw message against the shared arena geometry and republishes only
    # the clean ones as /scan. Dropping lowers the delivered rate; the rate trim
    # below measures the FILTERED /scan and renders faster to hold the contract.
    # System python for the same rclpy/3.11 reason as the probe above.
    # stdout/stderr inherited on purpose: the filter's drop counts land in this
    # process's log (simctl-isaac.log under simctl), where a corrupted session is
    # visible instead of silently filtered.
    relay = _sp.Popen(
        ['bash', '-c',
         'source /opt/ros/jazzy/setup.bash 2>/dev/null && '
         f'exec python3 {REPO}/tools/_scan_frame_relay.py'],
        env=dict(os.environ))
    say(f'  scan filter: pid {relay.pid} (scan_isaac_raw -> /scan, phase-corrupted '
        'revolutions dropped, validated against the shared arena)')
    say(f'  physics {1/dt:.0f} Hz, rendering_dt {rdt:.5f} s')
    say(f'  /scan {SCAN_HZ:.0f}  /odom_raw {ODOM_HZ:.0f}  /imu {IMU_HZ:.0f}  '
        f'/battery {BATTERY_HZ:.0f} Hz   <- /cmd_vel')
    say('')
    say('  *** /scan RATE IS CALIBRATED, NOT DERIVED ***')
    say(f'  The RTX lidar emits a measured {LIDAR_MSGS_PER_RENDER_SECOND:.0f} messages '
        'per second of RENDER time and')
    say('  ignores the scanRateBaseHz=12 authored on its prim. Nobody has explained')
    say(f'  that number. Rendering {render_hz:.1f} times a second is what lands /scan on')
    say(f'  {SCAN_HZ:.0f} Hz given it -- so the rate is right, for a reason that is not')
    say('  understood, and it will drift if anything changes render timing.')
    say('  Verify with:  ./tools/check_isaac_contract.py')
    say('')
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
    renders_done = 0
    last_trim = _time.time()
    last_render_wall = 0.0
    scan_measured = None
    try:
        while True:
            t = _time.time() - t0
            vx, wz = bridge.read_cmd_vel()
            sim.drive(vx, wz)
            # Render ONLY when a scan is due, gated on SIM time and paced at render_hz,
            # which is CALIBRATED against the measured 72-per-render-second emission
            # rate rather than derived from the sensor's configured rate. Rendering
            # every frame instead pinned /scan to the frame rate (34.6 Hz measured) and
            # ate the whole frame budget.
            # CLOSED LOOP ON WALL TIME, not an interval in sim time. The interval
            # version drifted with loop speed -- an external audit measured /scan at
            # 14.1 Hz while the sim-time gate believed itself exact. Driving the
            # CUMULATIVE render count toward render_hz x elapsed-wall-seconds pins the
            # long-run average against the only clock a subscriber sees.
            wall = _time.time()
            render = (wall - last_render_wall) >= (1.0 / render_hz)
            if render:
                last_render_wall = wall
                renders_done += 1
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
            # CLOSED LOOP on the measured rate: every ~10 s, trim render pacing by the
            # ratio of target to measured. Converges regardless of what the emission
            # constant is this week. Guarded: ignore stale or absurd readings.
            # Faster, undamped trims for the first 30 s so a fresh session converges
            # before anyone measures it -- an external 45 s check caught the slow
            # transient (13.2 mean) around a healthy 12.5 steady state.
            trim_period = 5.0 if t < 30.0 else 10.0
            trim_blend = 1.0 if t < 30.0 else 0.5
            if now - last_trim >= trim_period and t > 5.0:
                last_trim = now
                try:
                    with open(rate_file) as f:
                        hz_s, n_s, ts_s = f.read().split()
                    measured, ts = float(hz_s), float(ts_s)
                    if now - ts < 12.0 and 2.0 < measured < 100.0:
                        scan_measured = measured
                        new_hz = render_hz * (SCAN_HZ / measured)
                        render_hz = max(2.0, min(
                            40.0, (1 - trim_blend) * render_hz + trim_blend * new_hz))
                        if abs(measured - SCAN_HZ) > 0.06 * SCAN_HZ:
                            say(f'  scan rate measured {measured:.1f} Hz -> render '
                                f'pacing trimmed to {render_hz:.2f}/s')
                except (OSError, ValueError):
                    pass          # probe not up yet; keep the current pacing
            if now - last_report >= 10.0:
                c = bridge._counts
                rtf = sim_t / t
                # scan~ is an ESTIMATE (renders x 1.2), not a count of published
                # messages -- this process cannot subscribe to its own bridge (no
                # rclpy in Isaac's python). The old line printed the render count AS
                # the scan rate and disagreed with an external subscriber by 2 Hz
                # while looking exact. Only ./tools/check_isaac_contract.py, run
                # OUTSIDE this process, measures the real rate.
                scan_s = (f'scan {scan_measured:.1f} (probe-measured)'
                          if scan_measured is not None else
                          'scan ?.? (probe not reporting yet)')
                say(f'  t={t:6.1f}s  realtime x{rtf:.2f}  {scan_s}  '
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
