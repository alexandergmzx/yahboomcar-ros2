#!/usr/bin/env python3
"""Build the Isaac Sim test arena: ceramic floor, white walls, movable boxes, robot.

    ~/isaac/env_isaaclab/bin/python tools/build_arena.py            # build + verify
    ~/isaac/env_isaaclab/bin/python tools/build_arena.py --gui      # watch it
    ~/isaac/env_isaaclab/bin/python tools/build_arena.py --no-verify

Output: yahboomcar_ws/src/yahboomcar_twin/usd/arena.usd

Sized from the UMBmark research in docs/odometry-calibration.md: a 2x2 m square path
needs ~3x3 m of clear floor, so the default arena is 4x4 m.

WHY THE ROBOT USED TO FALL, since this file exists to prevent it recurring:
  1. the ground was a UsdGeom.Plane -- visual geometry with no collider at all;
  2. the robot's wheels had no collision geometry either, because the URDF declares
     only <visual> and the importer's collision_from_visuals defaults to False.
Either alone is enough to drop the robot through the world. Every collidable surface
here gets an explicit UsdPhysics.CollisionAPI, and --verify runs physics and fails if
the base descends.

Friction is a real parameter, not decoration: this is a skid-steer, every turn is slip,
and the floor's friction coefficient dominates how it behaves. 0.6/0.5 is a plausible
dry ceramic-on-rubber starting point, NOT a measurement of your floor.
"""
import argparse
import math
import os
import sys

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
USD_DIR = os.path.join(REPO, 'yahboomcar_ws', 'src', 'yahboomcar_twin', 'usd')
ROBOT_USD = os.path.join(USD_DIR, 'micro4', 'micro4.usd')
ARENA_USD = os.path.join(USD_DIR, 'arena.usd')
ROBOT_PRIM = '/World/Robot'

# Contact patch sits ~0.045 m below base_link: wheel centre 0.02125 below, radius 0.024.
WHEEL_DROP = 0.045
SPAWN_CLEARANCE = 0.01


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--size', type=float, default=4.0, help='floor side, metres')
    ap.add_argument('--wall-height', type=float, default=0.4)
    ap.add_argument('--boxes', type=int, default=4)
    ap.add_argument('--friction', type=float, default=0.6)
    ap.add_argument('--gui', action='store_true')
    ap.add_argument('--no-verify', dest='verify', action='store_false', default=True)
    ap.add_argument('--verify-seconds', type=float, default=10.0)
    ap.add_argument('--break-floor', action='store_true',
                    help='NEGATIVE TEST: author the floor with no collider, to prove '
                         'the fall detector actually detects a fall. A check that has '
                         'never failed is not known to work.')
    args = ap.parse_args()

    if not os.path.exists(ROBOT_USD):
        sys.exit(f'robot USD missing: {ROBOT_USD}\n'
                 f'  run: ~/isaac/env_isaaclab/bin/python tools/urdf_to_usd.py')

    report = os.path.join(USD_DIR, 'arena_report.txt')
    lines = []
    result = {'code': 1}   # pessimistic: only success sets it to 0

    def say(m=''):
        print(m, flush=True)
        lines.append(str(m))

    from isaacsim import SimulationApp
    app = SimulationApp({'headless': not args.gui})
    try:
        import omni.usd
        from pxr import (Gf, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics, UsdShade)

        omni.usd.get_context().new_stage()
        stage = omni.usd.get_context().get_stage()
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(stage, 1.0)
        world = UsdGeom.Xform.Define(stage, '/World')
        stage.SetDefaultPrim(world.GetPrim())

        scene = UsdPhysics.Scene.Define(stage, '/World/PhysicsScene')
        scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
        scene.CreateGravityMagnitudeAttr().Set(9.81)

        def visual_material(name, rgb, rough=0.5):
            mat = UsdShade.Material.Define(stage, f'/World/Looks/{name}')
            sh = UsdShade.Shader.Define(stage, f'/World/Looks/{name}/Shader')
            sh.CreateIdAttr('UsdPreviewSurface')
            sh.CreateInput('diffuseColor', Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*rgb))
            sh.CreateInput('roughness', Sdf.ValueTypeNames.Float).Set(rough)
            mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), 'surface')
            return mat

        # Physics material. Bound with purpose="physics" so it governs contact, not looks.
        phys_mat = UsdShade.Material.Define(stage, '/World/PhysicsMaterials/Ceramic')
        pm = UsdPhysics.MaterialAPI.Apply(phys_mat.GetPrim())
        pm.CreateStaticFrictionAttr().Set(args.friction)
        pm.CreateDynamicFrictionAttr().Set(max(0.0, args.friction - 0.1))
        pm.CreateRestitutionAttr().Set(0.0)

        ceramic = visual_material('Ceramic', (0.86, 0.86, 0.83), rough=0.25)
        white = visual_material('WallWhite', (0.93, 0.93, 0.95), rough=0.6)
        cardboard = visual_material('Cardboard', (0.72, 0.55, 0.35), rough=0.9)

        def box(path, size, translate, material, collider=True,
                rigid=False, mass=None):
            """Axis-aligned box. Cube is unit-sized, so scale carries the dimensions."""
            cube = UsdGeom.Cube.Define(stage, path)
            cube.CreateSizeAttr(1.0)
            x = UsdGeom.Xformable(cube.GetPrim())
            x.AddTranslateOp().Set(Gf.Vec3d(*translate))
            x.AddScaleOp().Set(Gf.Vec3f(*size))
            UsdShade.MaterialBindingAPI.Apply(cube.GetPrim()).Bind(material)
            if collider:
                UsdPhysics.CollisionAPI.Apply(cube.GetPrim()).CreateCollisionEnabledAttr(True)
                UsdShade.MaterialBindingAPI.Apply(cube.GetPrim()).Bind(
                    phys_mat, materialPurpose='physics')
            if rigid:
                UsdPhysics.RigidBodyAPI.Apply(cube.GetPrim())
                if mass is not None:
                    UsdPhysics.MassAPI.Apply(cube.GetPrim()).CreateMassAttr(mass)
            return cube

        s = args.size
        half = s / 2.0
        t = 0.05                      # floor thickness

        # Floor: top surface at z = 0, so the robot spawns just above zero.
        box('/World/Floor', (s, s, t), (0.0, 0.0, -t / 2.0), ceramic,
            collider=not args.break_floor)
        if args.break_floor:
            say('  !! --break-floor: floor authored WITHOUT a collider (negative test)')

        # Perimeter walls, inside faces flush with +/-half.
        wt, wh = 0.05, args.wall_height
        for name, size, pos in (
            ('N', (s + 2 * wt, wt, wh), (0.0, half + wt / 2, wh / 2)),
            ('S', (s + 2 * wt, wt, wh), (0.0, -half - wt / 2, wh / 2)),
            ('E', (wt, s, wh), (half + wt / 2, 0.0, wh / 2)),
            ('W', (wt, s, wh), (-half - wt / 2, 0.0, wh / 2)),
        ):
            box(f'/World/Wall_{name}', size, pos, white)

        # Movable cardboard boxes, kept clear of the 2x2 m UMBmark square so they do
        # not silently corrupt a calibration run.
        b = 0.3
        spots = [(1.45, 1.45), (-1.45, 1.30), (1.35, -1.40), (-1.30, -1.45),
                 (0.0, 1.55), (1.55, 0.0)]
        for i in range(min(args.boxes, len(spots))):
            bx, by = spots[i]
            box(f'/World/Box_{i}', (b, b, b), (bx, by, b / 2 + 0.001),
                cardboard, rigid=True, mass=0.4)

        # Lighting: dome for fill, distant for shape and shadows.
        dome = UsdLux.DomeLight.Define(stage, '/World/Lights/Dome')
        dome.CreateIntensityAttr(700.0)
        key = UsdLux.DistantLight.Define(stage, '/World/Lights/Key')
        key.CreateIntensityAttr(2200.0)
        key.CreateAngleAttr(0.8)
        UsdGeom.Xformable(key.GetPrim()).AddRotateXYZOp().Set(Gf.Vec3f(-40.0, 0.0, 35.0))

        # Camera: elevated 3/4 view framing the whole floor.
        cam = UsdGeom.Camera.Define(stage, '/World/ArenaCam')
        cx = UsdGeom.Xformable(cam.GetPrim())
        cx.AddTranslateOp().Set(Gf.Vec3d(s * 0.85, -s * 0.85, s * 0.75))
        cx.AddRotateXYZOp().Set(Gf.Vec3f(62.0, 0.0, 45.0))
        cam.CreateFocalLengthAttr(18.0)
        cam.CreateClippingRangeAttr(Gf.Vec2f(0.05, 200.0))

        # The robot, spawned clear of the floor rather than interpenetrating it.
        robot = UsdGeom.Xform.Define(stage, ROBOT_PRIM)
        robot.GetPrim().GetReferences().AddReference(ROBOT_USD)
        spawn_z = WHEEL_DROP + SPAWN_CLEARANCE
        # The referenced robot already carries xformOps from the import, so AddTranslateOp
        # would collide with the existing stack. XformCommonAPI edits in place instead.
        UsdGeom.XformCommonAPI(robot.GetPrim()).SetTranslate(
            Gf.Vec3d(0.0, 0.0, spawn_z))

        os.makedirs(USD_DIR, exist_ok=True)
        stage.Export(ARENA_USD)
        say(f'arena written: {ARENA_USD}')
        say(f'  floor {s} x {s} m, walls {wh} m, {min(args.boxes, len(spots))} boxes, '
            f'friction {args.friction}')
        say(f'  robot spawned at z={spawn_z:.3f} m')

        colliders = sum(1 for p in stage.Traverse(Usd.TraverseInstanceProxies())
                        if p.HasAPI(UsdPhysics.CollisionAPI))
        say(f'  collider prims in stage: {colliders}')

        if not args.verify:
            say('\n(skipping physics verification)')
            result['code'] = 0
            return 0

        # ---- the check whose absence let a falling robot pass as working ----
        say(f'\nverifying the robot does not fall ({args.verify_seconds:.0f}s of physics)')
        omni.usd.get_context().open_stage(ARENA_USD)
        app.update()

        from isaacsim.core.api import SimulationContext
        from pxr import UsdGeom as _UG
        sim = SimulationContext()
        sim.initialize_physics()
        sim.play()

        stage2 = omni.usd.get_context().get_stage()

        # Track the base_link RIGID BODY, not the wrapper Xform. Physics moves the
        # bodies underneath /World/Robot; the wrapper's own transform never changes,
        # so reading it yields a constant that passes whether the robot rests or falls.
        from pxr import UsdPhysics as _UP
        base = None
        for prim in Usd.PrimRange(stage2.GetPrimAtPath(ROBOT_PRIM)):
            if prim.GetName() == 'base_link' and prim.HasAPI(_UP.RigidBodyAPI):
                base = prim
                break
        if base is None:
            for prim in Usd.PrimRange(stage2.GetPrimAtPath(ROBOT_PRIM)):
                if prim.HasAPI(_UP.RigidBodyAPI):
                    base = prim
                    break
        if base is None:
            say('  FAIL: no rigid body found under the robot; nothing to track')
            say('\nRESULT: FAIL')
            return 1
        say(f'  tracking rigid body: {base.GetPath()}')
        xf_cache = _UG.XformCache()

        def base_z():
            xf_cache.Clear()
            m = xf_cache.GetLocalToWorldTransform(base)
            return m.ExtractTranslation()[2]

        for _ in range(30):
            sim.step(render=args.gui)
            app.update()

        z0 = base_z()
        zmin = zmax = z0
        import time
        t0 = time.time()
        while time.time() - t0 < args.verify_seconds:
            sim.step(render=args.gui)
            app.update()
            z = base_z()
            zmin, zmax = min(zmin, z), max(zmax, z)
        z1 = base_z()
        sim.stop()

        say(f'  base z: start {z0:+.4f}  end {z1:+.4f}  min {zmin:+.4f}  max {zmax:+.4f}')
        drop = z0 - zmin
        fell = zmin < -0.05 or drop > 0.10
        if fell:
            say(f'  FAIL: base fell {drop:.3f} m (min z {zmin:.3f}). '
                'Check colliders on both floor and wheels.')
        else:
            say(f'  PASS: base held within {drop*1000:.1f} mm of spawn; it is resting '
                'on the floor, not falling through it.')
        say('\nRESULT: ' + ('FAIL' if fell else 'PASS'))
        result['code'] = 1 if fell else 0
        return result['code']
    except Exception as e:
        import traceback
        say(f'EXCEPTION: {type(e).__name__}: {e}')
        say(traceback.format_exc())
        raise
    finally:
        os.makedirs(USD_DIR, exist_ok=True)
        with open(report, 'w') as f:
            f.write('\n'.join(lines) + '\n')
        # os._exit BEFORE app.close(): SimulationApp.close() terminates the process on
        # its own terms and discards our status, so every run -- including a robot
        # falling 4.7 km -- reported success to the caller. Skipping Isaac's teardown is
        # acceptable for a short-lived build tool; a wrong exit code in a gate is not.
        code = result['code']
        sys.stdout.flush()
        sys.stderr.flush()
        os._exit(code)


if __name__ == '__main__':
    sys.exit(main())
