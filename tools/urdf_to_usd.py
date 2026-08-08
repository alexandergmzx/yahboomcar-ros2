#!/usr/bin/env python3
"""Convert the car's URDF to a USD articulation for Isaac Sim 5.1.0, headless.

    ~/isaac/env_isaaclab/bin/python tools/urdf_to_usd.py

Output: micro4/micro4.usd in the twin USD dir (_layout.USD_DIR; never extracted, R-05)

Two hard-won details, both of which previously caused the simulated robot to fall
through the world:

  * `collision_from_visuals` MUST be True. This URDF has no <collision> elements at
    all, so without it the wheels are imported with no collision geometry and pass
    through any floor, however solid that floor is.
  * `fix_base` MUST be False for a mobile robot, but that is only safe once the scene
    actually has a ground collider. A free base over visual-only ground falls forever.

The importer API differs between Isaac versions and the docs lag it. 5.1.0 uses the
`_urdf` binding with the `URDFParseAndImportFile` command; 6.0.1 replaced both with a
class-based `URDFImporter`. This targets 5.1.0, which is what is installed, and logs the
config it actually applied so a mismatch is visible rather than silent.
"""
import argparse
import os
import re
import shutil
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _layout import USD_DIR, vendor_pkg_dir                     # noqa: E402
DESC = vendor_pkg_dir('yahboomcar_description')
DEFAULT_URDF = os.path.join(DESC, 'urdf', 'MicroROS.urdf')
DEFAULT_OUT = os.path.join(USD_DIR, 'micro4', 'micro4.usd')

EXPECTED_JOINTS = {'zq_Joint', 'yq_Joint', 'yh_Joint', 'zh_Joint',
                   'jq1_Joint', 'jq2_Joint'}


def resolve_package_paths(urdf_path):
    """Rewrite package:// mesh refs to absolute paths in a temp copy."""
    with open(urdf_path) as f:
        text = f.read()
    missing = []

    def sub(m):
        rel = m.group(2)
        abs_path = os.path.join(DESC, rel)
        if not os.path.exists(abs_path):
            missing.append(rel)
        return f'{m.group(1)}{abs_path}{m.group(3)}'

    new, n = re.subn(r'(filename=")package://yahboomcar_description/([^"]+)(")',
                     sub, text)
    tmp_dir = tempfile.mkdtemp(prefix='urdf_abs_')
    tmp_urdf = os.path.join(tmp_dir, os.path.basename(urdf_path))
    with open(tmp_urdf, 'w') as f:
        f.write(new)
    return tmp_urdf, tmp_dir, n, missing


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--urdf', default=DEFAULT_URDF)
    ap.add_argument('--out', default=DEFAULT_OUT)
    # Fleet reuse (session 5): the verify section was yahboom-hardcoded.
    # Defaults unchanged; other robots pass their own expectations.
    ap.add_argument('--expect-joints', default=None,
                    help='comma-separated movable joints (default: yahboom set)')
    ap.add_argument('--min-geoms', type=int, default=9,
                    help='minimum geometry prims (Mesh+Cube+Cylinder+Capsule+Sphere)')
    ap.add_argument('--velocity-drives', action='store_true', default=True,
                    help='wheels get velocity drives (physics drive mode)')
    ap.add_argument('--position-drives', dest='velocity_drives',
                    action='store_false', help='wheels get position drives (twin mode)')
    args = ap.parse_args()

    if not os.path.exists(args.urdf):
        sys.exit(f'error: no URDF at {args.urdf}')

    tmp_urdf, tmp_dir, n_meshes, missing = resolve_package_paths(args.urdf)
    os.makedirs(os.path.dirname(args.out), exist_ok=True)

    report_path = os.path.splitext(args.out)[0] + '_import_report.txt'
    lines = []

    def say(m=''):
        print(m, flush=True)
        lines.append(str(m))

    def flush():
        os.makedirs(os.path.dirname(report_path), exist_ok=True)
        with open(report_path, 'w') as f:
            f.write('\n'.join(lines) + '\n')

    say(f'resolved {n_meshes} package:// mesh references')
    if missing:
        say(f'WARNING: {len(missing)} mesh files missing: {sorted(set(missing))}')

    from isaacsim import SimulationApp
    app = SimulationApp({'headless': True})
    try:
        from isaacsim.core.utils.extensions import enable_extension
        enable_extension('isaacsim.asset.importer.urdf')
        app.update()

        import omni.kit.commands
        from isaacsim.asset.importer.urdf import _urdf

        cfg = _urdf.ImportConfig()
        # Without this the wheels have NO collision geometry -- the URDF declares only
        # <visual>. This is what made the robot fall through a solid floor.
        cfg.collision_from_visuals = True
        cfg.fix_base = False            # mobile robot; requires real ground beneath it
        cfg.merge_fixed_joints = False  # keep imu_Link / radar_Link as usable frames
        cfg.make_default_prim = True
        cfg.self_collision = False
        cfg.distance_scale = 1.0
        cfg.create_physics_scene = False   # the arena owns the physics scene
        cfg.import_inertia_tensor = True
        if args.velocity_drives:
            cfg.default_drive_type = _urdf.UrdfJointTargetType.JOINT_DRIVE_VELOCITY

        say('\nimport config applied:')
        for a in sorted(dir(cfg)):
            if a.startswith('_'):
                continue
            try:
                v = getattr(cfg, a)
            except Exception:
                continue
            if not callable(v):
                say(f'    {a} = {v!r}')

        say(f'\nimporting {tmp_urdf}')
        status, prim_path = omni.kit.commands.execute(
            'URDFParseAndImportFile', urdf_path=tmp_urdf,
            import_config=cfg, dest_path=args.out)
        say(f'status={status}  prim={prim_path}')

        from pxr import Usd, UsdPhysics
        stage_path = args.out if os.path.exists(args.out) else None
        if stage_path is None:
            say('ERROR: no USD written')
            flush()
            sys.exit(1)

        # LoadAll + instance proxies: the importer emits instanced geometry, and a plain
        # traversal reports zero meshes on a perfectly good asset.
        stage = Usd.Stage.Open(stage_path, Usd.Stage.LoadAll)
        meshes, joints, colliders = 0, [], 0
        for prim in stage.Traverse(Usd.TraverseInstanceProxies()):
            t = str(prim.GetTypeName())
            if t in ('Mesh', 'Cube', 'Cylinder', 'Capsule', 'Sphere'):
                meshes += 1
            if 'Joint' in t:
                joints.append((prim.GetName(), t))
            if prim.HasAPI(UsdPhysics.CollisionAPI):
                colliders += 1

        say(f'\nUSD: {stage_path}')
        say(f'geom prims    : {meshes}   (expect >= {args.min_geoms})')
        say(f'joints        : {len(joints)}')
        say(f'collider prims: {colliders}   (0 here means it WILL fall through the floor)')
        for nme, t in sorted(joints):
            say(f'    {nme:22s} {t}')

        movable = {n for n, t in joints if 'Revolute' in t or 'Prismatic' in t}
        say(f'\nmovable joints: {len(movable)} -> {sorted(movable)}')
        expected = (set(args.expect_joints.split(','))
                    if args.expect_joints else EXPECTED_JOINTS)
        ok = expected <= movable and meshes >= args.min_geoms and colliders > 0
        if not expected <= movable:
            say(f'MISSING joints: {sorted(expected - movable)}')
        if colliders == 0:
            say('FAIL: no collision geometry -- check collision_from_visuals')
        say('\nRESULT: ' + ('PASS' if ok else 'FAIL'))
        flush()
        return 0 if ok else 1
    except Exception as e:
        import traceback
        say(f'EXCEPTION: {type(e).__name__}: {e}')
        say(traceback.format_exc())
        flush()
        raise
    finally:
        app.close()
        shutil.rmtree(tmp_dir, ignore_errors=True)


if __name__ == '__main__':
    sys.exit(main())
