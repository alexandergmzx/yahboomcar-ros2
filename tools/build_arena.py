#!/usr/bin/env python3
"""Build the Isaac Sim test arena: ceramic floor, white walls, movable boxes, robot.

    ~/isaac/env_isaaclab/bin/python tools/build_arena.py            # build + verify
    ~/isaac/env_isaaclab/bin/python tools/build_arena.py --gui      # watch it
    ~/isaac/env_isaaclab/bin/python tools/build_arena.py --no-verify

Output: arena.usd in the twin USD dir (_layout.USD_DIR; never extracted, R-05)

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

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _layout import REPO, USD_DIR                              # noqa: E402
ROBOT_USD = os.path.join(USD_DIR, 'micro4', 'micro4.usd')
ARENA_USD = os.path.join(USD_DIR, 'arena.usd')
ROBOT_PRIM = '/World/Robot'

# Contact patch sits ~0.045 m below base_link: wheel centre 0.02125 below, radius 0.024.
WHEEL_DROP = 0.045
SPAWN_CLEARANCE = 0.01

# ----------------------------------------------------------------- the lidar
# The firmware contract from CLAUDE.md, which the Isaac backend must satisfy exactly:
# /scan is 360 beams at 1.000 deg over 0.12-8.0 m at ~12 Hz. A backend publishing a
# differently shaped scan is worse than no backend, because every result taken from it
# silently stops transferring to the real robot.
LIDAR_BEAMS = 360
LIDAR_HZ = 12
LIDAR_RANGE_MIN = 0.12
LIDAR_RANGE_MAX = 8.0
# Offset of laser_frame from base_link.
LIDAR_XYZ = (-0.0046, 0.0, 0.094)


def author_lidar(stage, base_path, Gf, Vt, say, break_it=False,
                 beams=None, hz=None, range_min=None, range_max=None,
                 xyz=None, name='laser_frame_lidar'):
    # Parameterized for fleet reuse (session 5, rasptank twin): keyword
    # args default to this module's MS200 constants, so every existing
    # caller is unchanged; the C1 caller passes its own contract.
    beams = LIDAR_BEAMS if beams is None else beams
    hz = LIDAR_HZ if hz is None else hz
    range_min = LIDAR_RANGE_MIN if range_min is None else range_min
    range_max = LIDAR_RANGE_MAX if range_max is None else range_max
    xyz = LIDAR_XYZ if xyz is None else xyz
    """Create the RTX lidar and author the contract onto it. Returns its prim path.

    THREE TRAPS, each of which looks like success:

    1. `IsaacSensorCreateRtxLidar(config='...')` with an unknown config name returns
       ok=True and a valid prim. It only carb.log_warn's -- and Isaac's logger swallows
       output -- then silently gives you a GENERIC 128-channel 3D lidar. So the config
       name is never trusted here; every parameter is authored explicitly and READ BACK.

    2. The JSON profiles under `profileBaseFolder` are the deprecated camera-based path
       and are ignored by Isaac 5.1, which drives RTX lidars from `omni:sensor:Core:*`
       PRIM ATTRIBUTES instead. A JSON config file was written, verified to have no
       effect whatsoever, and deleted.

    3. `channelId` is 1-BASED. A 0 gives "Malformed model parameter update: channelId 0
       is either less than 1 or greater than numberOfChannels 1", after which the plugin
       keeps the previous profile -- so the sensor silently stays 3D.

    And the reason all of that matters: IsaacComputeRTXLidarFlatScan refuses to run at
    all unless every elevation is zero ("Lidar prim is not a 2D Lidar, and node will not
    execute"), so a 3D default publishes nothing and looks like a dead topic.
    """
    import omni.kit.commands
    ok, sensor = omni.kit.commands.execute(
        'IsaacSensorCreateRtxLidar',
        path=name,
        parent=base_path,
        translation=Gf.Vec3d(*xyz),
        orientation=Gf.Quatd(1.0, 0.0, 0.0, 0.0),
    )
    if not sensor:
        say('  FAIL: could not create the lidar prim')
        return None
    lidar = stage.GetPrimAtPath(str(sensor.GetPath()))

    P = 'omni:sensor:Core:'
    E = P + 'emitterState:s001:'
    wanted = {
        E + 'azimuthDeg': Vt.FloatArray([0.0]),
        E + 'elevationDeg': Vt.FloatArray([0.0]),   # zero, or FlatScan refuses to run
        E + 'channelId': Vt.UIntArray([1]),         # 1-based
        E + 'fireTimeNs': Vt.UIntArray([0]),
        P + 'numberOfChannels': 1,
        P + 'numberOfEmitters': 1,
        P + 'scanRateBaseHz': hz,
        # Firings per second / rotations per second = beams per revolution. This, not
        # any explicit resolution field, is what sets the 1.000 deg increment.
        P + 'reportRateBaseHz': beams * hz,
        P + 'nearRangeM': range_min,
        P + 'farRangeM': range_max,
        P + 'maxReturns': 1,
        # ROS LaserScan sweeps upward from angle_min, i.e. counter-clockwise.
        P + 'rotationDirection': 'CCW',
        P + 'scanType': 'ROTARY',
        P + 'rayType': 'IDEALIZED',
    }
    for name, val in wanted.items():
        a = lidar.GetAttribute(name)
        if not a or not a.IsValid():
            say(f'  FAIL: lidar attribute missing: {name}')
            return None
        a.Set(val)

    if break_it:
        # NEGATIVE TEST: leave the sensor 3D, which is exactly the state the default
        # config lands in and the state that makes FlatScan refuse to run. The readback
        # below must catch it; a check that has never failed is not known to work.
        say('  !! --break-lidar: restoring a nonzero elevation (negative test)')
        lidar.GetAttribute(E + 'elevationDeg').Set(Vt.FloatArray([-15.0]))

    bad = []
    for name, val in wanted.items():
        got = lidar.GetAttribute(name).Get()
        g = list(got) if hasattr(got, '__len__') and not isinstance(got, str) else got
        w = list(val) if hasattr(val, '__len__') and not isinstance(val, str) else val
        # float32 storage: 0.12 reads back as 0.11999999731779099.
        if isinstance(w, float) and isinstance(g, float):
            same = abs(g - w) < 1e-5
        else:
            same = (g == w)
        if not same:
            bad.append(f'{name.split(":")[-1]} = {g!r}, wanted {w!r}')
    if bad:
        say('  FAIL: lidar parameters did not stick:')
        for b in bad:
            say(f'    {b}')
        return None

    say(f'  lidar: {lidar.GetPath()}')
    say(f'    {beams} beams at {360/beams:.3f} deg, '
        f'{range_min}-{range_max} m, {hz} Hz  '
        f'(all {len(wanted)} parameters read back)')
    return str(lidar.GetPath())


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--size', type=float, default=4.0, help='floor side, metres')
    ap.add_argument('--wall-height', type=float, default=0.4)
    ap.add_argument('--boxes', type=int, default=4)
    ap.add_argument('--friction', type=float, default=0.6)
    ap.add_argument('--gui', action='store_true')
    ap.add_argument('--no-verify', dest='verify', action='store_false', default=True)
    ap.add_argument('--verify-seconds', type=float, default=10.0)
    ap.add_argument('--box-mass', type=float, default=0.12,
                    help='kg per cardboard box. An empty ~30 cm box is ~0.1-0.15 kg; '
                         'the old 0.4 outweighed the robot.')
    ap.add_argument('--rear-wheel-friction', type=float, default=0.1,
                    help='mu for the REAR wheel cylinders. Low on purpose: PhysX has '
                         'no anisotropic friction, and rear-slide approximates a '
                         'differential+caster so the skid-steer can actually turn. '
                         'A modelling choice, measured, not physics of the real car.')
    ap.add_argument('--robot-mass', type=float, default=1.0,
                    help='kg, TOTAL for the robot. Estimate from holding the real car '
                         '("like a kilogram"); the URDF inertials total only 0.355 kg. '
                         'Put the car on a scale and update this.')
    ap.add_argument('--no-lidar', dest='lidar', action='store_false', default=True,
                    help='omit the RTX lidar. Without it the arena cannot feed SLAM, '
                         'Nav2 or the governor -- they all wait on /scan, silently.')
    ap.add_argument('--break-lidar', action='store_true',
                    help='NEGATIVE TEST: author the lidar with a nonzero elevation, to '
                         'prove the parameter readback actually catches a sensor that '
                         'is not on the contract.')
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
        #
        # MASS: 0.12 kg default, was 0.4. At 0.4 each box OUTWEIGHED the robot (whose
        # URDF inertials total 0.355 kg), which is why the car "could barely move"
        # boxes that are supposed to be light cardboard -- field report. An empty
        # ~30 cm cardboard box is roughly 0.1-0.15 kg.
        b = 0.3
        spots = [(1.45, 1.45), (-1.45, 1.30), (1.35, -1.40), (-1.30, -1.45),
                 (0.0, 1.55), (1.55, 0.0)]
        for i in range(min(args.boxes, len(spots))):
            bx, by = spots[i]
            box(f'/World/Box_{i}', (b, b, b), (bx, by, b / 2 + 0.001),
                cardboard, rigid=True, mass=args.box_mass)

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

        # ROBOT MASS. The URDF inertials total 0.355 kg -- recorded as a known defect
        # since the second audit -- while the real car is about 1 kg in the hand. With
        # the old 0.4 kg boxes the ROBOT WAS THE LIGHTER PARTY in every collision, which
        # is why it could barely shift them. The difference is added to base_link (the
        # chassis is where the battery and boards live); the wheels keep their URDF
        # inertials so the contact patches behave the same. ESTIMATE until the car
        # meets a kitchen scale; --robot-mass replaces it, and the report records it.
        # Both figures read from the URDF's <mass> elements, not remembered.
        urdf_total = 0.355482
        if args.robot_mass > urdf_total:
            base_prim = None
            for prim in Usd.PrimRange(stage.GetPrimAtPath(ROBOT_PRIM)):
                if prim.GetName() == 'base_link':
                    base_prim = prim
                    break
            if base_prim is None:
                say('  FAIL: no base_link to set the robot mass on')
                return 1
            extra = args.robot_mass - urdf_total
            # The URDF gives base_link its own share of the 0.355; MassAPI's mass attr
            # OVERRIDES that link's mass outright, so the new value is its URDF share
            # plus everything the URDF undercounts.
            base_urdf_share = 0.222555   # base_link's own <mass> in MicroROS.urdf
            UsdPhysics.MassAPI.Apply(base_prim).CreateMassAttr(
                base_urdf_share + extra)
            say(f'  robot mass: base_link set to {base_urdf_share + extra:.3f} kg so '
                f'the articulation totals ~{args.robot_mass:.2f} kg')
            say(f'    (URDF total {urdf_total} kg is a known defect; '
                f'{args.robot_mass:.2f} kg is an in-hand ESTIMATE -- weigh the car)')
        else:
            say(f'  robot mass left at URDF inertials (~{urdf_total} kg): '
                f'--robot-mass {args.robot_mass} does not exceed them')

        # ---- WHEEL CONTACT, measured into shape ---------------------------------
        # The importer gave every wheel a FACETED CONVEX HULL of its visual mesh -- a
        # polygon prism that pressed flats into the floor (0-4 mm penetration under
        # 1 kg). Twisting a loaded flat is a parking brake: measured, the wheels could
        # not even reach commanded speed during turns (66% of target) and the body
        # turned at 2-3% of command, which in the field read as "barely rotates".
        #
        # Two authored changes, each MEASURED against a turn ladder:
        #   1. Smooth analytic CYLINDER colliders (r=0.024, w=0.0215, from the STL)
        #      replace the hulls: wheels 66% -> 105-109% of target, linear 83-89% ->
        #      93-105%.
        #   2. LOW-FRICTION REAR wheels (mu 0.1 vs 0.6 front). A uniform-mu skid-steer
        #      cannot be helped by friction tuning -- drive moment and lateral
        #      resistance both scale with mu -- and PhysX has no anisotropic friction,
        #      which is what real rubber has. Rear-slide approximates a differential
        #      drive with casters: turn response doubled again (15% -> 32-41%).
        #      MODELLING CHOICE, stated: the real car is 4WD skid-steer.
        #
        # Residual, also measured: below ~1.5 rad/s of commanded wheel speed the
        # wheels do not break static friction at all (0%), and above it the achieved
        # yaw is ~0.585*wz - 0.52. sim_runner.drive() carries the measured inverse as
        # slip-compensation feedforward -- which is what real skid-steer firmware does.
        for w in ('zq_Link', 'yq_Link', 'yh_Link', 'zh_Link'):
            link = stage.GetPrimAtPath(f'{ROBOT_PRIM}/{w}')
            if not link or not link.IsValid():
                say(f'  FAIL: wheel link {w} missing')
                return 1
            for prim in Usd.PrimRange(link, Usd.TraverseInstanceProxies()):
                if prim.IsInstanceable():
                    prim.SetInstanceable(False)
            disabled = 0
            for prim in Usd.PrimRange(link):
                if prim.HasAPI(UsdPhysics.CollisionAPI):
                    UsdPhysics.CollisionAPI(prim).CreateCollisionEnabledAttr(False)
                    disabled += 1
            cyl = UsdGeom.Cylinder.Define(stage, f'{ROBOT_PRIM}/{w}/wheel_cyl')
            cyl.CreateAxisAttr('Y')                 # wheel axis is local Y (URDF)
            cyl.CreateRadiusAttr(0.024)
            cyl.CreateHeightAttr(0.0215)
            cyl.CreatePurposeAttr(UsdGeom.Tokens.guide)   # collider, not a visual
            UsdPhysics.CollisionAPI.Apply(cyl.GetPrim())
            is_rear = w in ('yh_Link', 'zh_Link')
            mu = args.rear_wheel_friction if is_rear else args.friction
            wmat = UsdShade.Material.Define(stage, f'/World/PhysicsMaterials/Wheel_{w}')
            wapi = UsdPhysics.MaterialAPI.Apply(wmat.GetPrim())
            wapi.CreateStaticFrictionAttr().Set(mu)
            wapi.CreateDynamicFrictionAttr().Set(max(0.05, mu - 0.1))
            wapi.CreateRestitutionAttr().Set(0.0)
            UsdShade.MaterialBindingAPI.Apply(cyl.GetPrim()).Bind(
                wmat, materialPurpose='physics')
            say(f'  {w}: {disabled} hull collider(s) off, cylinder on, mu={mu}'
                f'{" (rear/slide)" if is_rear else ""}')

        # The lidar, parented under base_link so it rides with the chassis. It is
        # authored into the arena rather than at run time so `arena.usd` is complete on
        # its own and anything opening it gets a sensor already on the contract.
        if args.lidar:
            from pxr import Vt
            from isaacsim.core.utils.extensions import enable_extension
            # Without this the OmniLidar prim type is unknown and creation fails.
            enable_extension('isaacsim.sensors.rtx')
            app.update()
            base_for_lidar = None
            for prim in Usd.PrimRange(stage.GetPrimAtPath(ROBOT_PRIM)):
                if prim.GetName() == 'base_link' and prim.HasAPI(UsdPhysics.RigidBodyAPI):
                    base_for_lidar = prim
                    break
            if base_for_lidar is None:
                say('  FAIL: no base_link to attach the lidar to')
                return 1
            if author_lidar(stage, str(base_for_lidar.GetPath()), Gf, Vt, say,
                            break_it=args.break_lidar) is None:
                say('\nRESULT: FAIL (lidar)')
                return 1
        else:
            say('  no lidar (--no-lidar): SLAM, Nav2 and the governor will all '
                'wait on /scan forever')

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
