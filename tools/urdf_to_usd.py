#!/usr/bin/env python3
"""Convert the car's URDF to a USD articulation for Isaac Sim, headless.

Run with Isaac Sim's own interpreter, not the system one:

    ~/isaacsim/python.sh tools/urdf_to_usd.py

The first run is slow (shader compilation and extension loading), several minutes is
normal. Output goes to yahboomcar_ws/src/yahboomcar_twin/usd/micro4.usd.

Two details that matter:

  * The URDF references meshes as package://yahboomcar_description/..., which Isaac
    cannot resolve without a ROS environment. This script rewrites them to absolute
    paths in a temporary copy, leaving the tracked URDF untouched.
  * fix_base is False. The base must be free to move, or the twin cannot follow the
    real robot's pose.
"""
import argparse
import os
import re
import shutil
import sys
import tempfile

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DESC = os.path.join(REPO, 'yahboomcar_ws', 'src', 'yahboomcar_description')
DEFAULT_URDF = os.path.join(DESC, 'urdf', 'MicroROS.urdf')
DEFAULT_OUT = os.path.join(REPO, 'yahboomcar_ws', 'src', 'yahboomcar_twin', 'usd',
                           'micro4.usd')


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
    print(f'resolved {n} package:// mesh references')
    if missing:
        print(f'WARNING: {len(missing)} mesh files do not exist: {sorted(set(missing))}')

    tmp_dir = tempfile.mkdtemp(prefix='urdf_abs_')
    tmp_urdf = os.path.join(tmp_dir, os.path.basename(urdf_path))
    with open(tmp_urdf, 'w') as f:
        f.write(new)
    return tmp_urdf, tmp_dir


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--urdf', default=DEFAULT_URDF)
    ap.add_argument('--out', default=DEFAULT_OUT)
    args = ap.parse_args()

    if not os.path.exists(args.urdf):
        sys.exit(f'error: no URDF at {args.urdf}')

    tmp_urdf, tmp_dir = resolve_package_paths(args.urdf)
    os.makedirs(os.path.dirname(args.out), exist_ok=True)

    # Isaac's carb logger swallows plain stdout once SimulationApp starts, so the
    # report is written to a file as well -- otherwise a failed run looks like a
    # silent success (exit 0, no output, no USD).
    report_path = os.path.splitext(args.out)[0] + '_import_report.txt'
    report_lines = []

    def say(msg):
        print(msg, flush=True)
        report_lines.append(str(msg))

    def flush_report():
        os.makedirs(os.path.dirname(report_path), exist_ok=True)
        with open(report_path, 'w') as f:
            f.write('\n'.join(report_lines) + '\n')

    from isaacsim import SimulationApp
    app = SimulationApp({'headless': True})

    try:
        # The URDF importer is not enabled in the default headless experience.
        from isaacsim.core.utils.extensions import enable_extension
        enable_extension('isaacsim.asset.importer.urdf')
        app.update()

        # Isaac Sim 6.0.1 replaced the old _urdf binding and the
        # "URDFParseAndImportFile" kit command with this class-based API.
        from isaacsim.asset.importer.urdf import URDFImporter, URDFImporterConfig

        out_dir = os.path.dirname(args.out)
        cfg = URDFImporterConfig(
            urdf_path=tmp_urdf,
            usd_path=out_dir,          # a DIRECTORY, not a file
            merge_fixed_joints=False,  # keep imu_Link / radar_Link as real frames
            fix_base=False,            # the twin must be free to move
            allow_self_collision=False,
            collision_from_visuals=True,   # the URDF ships no <collision> geometry
            # Position targets so /joint_states can drive the articulation.
            joint_target_type='position',
        )

        say(f'importing {tmp_urdf}')
        produced = URDFImporter(cfg).import_urdf()
        say(f'importer returned: {produced}')

        # Report the articulation so the result can be checked without opening the GUI.
        from pxr import Usd
        stage_path = produced if produced and os.path.exists(produced) else args.out
        stage = Usd.Stage.Open(stage_path)
        joints, meshes = [], 0
        for prim in stage.Traverse():
            t = str(prim.GetTypeName())
            if 'Joint' in t:
                joints.append((prim.GetName(), t))
            if t == 'Mesh':
                meshes += 1

        say(f'USD: {stage_path}  exists={os.path.exists(stage_path)}')
        say(f'meshes: {meshes}')
        say(f'joints: {len(joints)}')
        for n, t in sorted(joints):
            say(f'    {n:22s} {t}')

        movable = [n for n, t in joints if 'Revolute' in t or 'Prismatic' in t]
        say(f'movable joints: {len(movable)} -> {sorted(movable)}')
        expected = {'zq_Joint', 'yq_Joint', 'yh_Joint', 'zh_Joint',
                    'jq1_Joint', 'jq2_Joint'}
        if expected <= set(movable):
            say('OK   all six expected movable joints present')
        else:
            say(f'MISSING: {sorted(expected - set(movable))}')
    except Exception as e:
        import traceback
        say(f'EXCEPTION: {type(e).__name__}: {e}')
        say(traceback.format_exc())
        raise
    finally:
        flush_report()
        app.close()
        shutil.rmtree(tmp_dir, ignore_errors=True)


if __name__ == '__main__':
    main()
