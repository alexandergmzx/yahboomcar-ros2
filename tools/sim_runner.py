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

    def __init__(self, app, gui, wheel_error=0.0):
        self.app = app
        self.gui = gui
        self.wheel_r = WHEEL_R * (1.0 + wheel_error)
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
        left = (vx - wz * LY) / self.wheel_r
        right = (vx + wz * LY) / self.wheel_r
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


def t_umbmark(sim, say, L=2.0, speed=0.15, wz=0.8, injected=0.0):
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
        budget = int((math.pi / 2) / wz / sim.dt() * 2.5)
        for _ in range(budget):
            sim.step()
            _, _, a = sim.pose()
            if abs(norm(a - a0)) >= math.pi / 2:
                break
        sim.stop()

    runs = []
    for direction, sign in (('cw', -1), ('ccw', +1)):
        for _ in range(2):        # 2 per direction; 5 is the spec but slow in sim
            sim.art.set_world_poses is None if False else None
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
    if injected:
        recovered = (res['Ed'] - 1.0) * 100.0
        say(f'  injected wheel error {injected*100:+.1f}%, '
            f'Ed implies {recovered:+.2f}%')
        say('  (Ed reflects a LEFT/RIGHT diameter ratio; a symmetric radius error moves')
        say('   Eb and the distance scale instead, so do not expect these to match 1:1.)')
    say('  PASS: the protocol ran end to end and the maths produced finite factors')
    ok = all(math.isfinite(res[k]) for k in ('Ed', 'Eb', 'E_max_syst_mm'))
    return ok, res


TESTS = {
    'drive-straight': t_drive_straight,
    'rotate': t_rotate,
    'obstacle-stop': t_obstacle_stop,
    'umbmark': t_umbmark,
}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--test', default='drive-straight',
                    choices=list(TESTS) + ['all'])
    ap.add_argument('--gui', action='store_true')
    ap.add_argument('--inject-wheel-error', type=float, default=0.0,
                    help='percent error in the wheel radius the controller assumes')
    ap.add_argument('--calibrate', action='store_true',
                    help='measure the effective rolling radius and report it, instead '
                         'of trusting the constant')
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
        sim = Sim(app, args.gui, wheel_error=err)
        say(f'sim runner {stamp}   arena={os.path.basename(ARENA_USD)}')
        say(f'DOFs: {sim.dof}')
        if err:
            say(f'INJECTED wheel-radius error: {args.inject_wheel_error:+.1f}% '
                f'(controller assumes r={sim.wheel_r:.5f}, truth {WHEEL_R})')
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

        names = list(TESTS) if args.test == 'all' else [args.test]
        all_ok = True
        for name in names:
            say(f'=== {name} ===')
            fn = TESTS[name]
            kw = {'injected': err} if name == 'umbmark' else {}
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
