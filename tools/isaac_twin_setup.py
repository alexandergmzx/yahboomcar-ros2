#!/usr/bin/env python3
"""!! KNOWN BROKEN -- DEFERRED. Do not trust a passing run. !!

Superseded in practice by tools/build_arena.py + tools/sim_runner.py, which are working
and verified. This file is kept only because the live-twin design will build on it.

Three defects, all confirmed by external audit:

  1. It references usd/MicroROS/MicroROS.usda, which tools/urdf_to_usd.py no longer
     produces -- the converter now writes usd/micro4/micro4.usd. This script cannot
     find its asset on a clean checkout.
  2. The ground is a UsdGeom.Plane with no collider, so the robot falls through the
     world. build_arena.py fixed this; that fix was never carried over here.
  3. It reports failure without failing: main() returns no status and swallows
     exceptions, so it is unusable as a gate.

It also never subscribed to /odom or /tf, which is why the twin animates joints but
does not follow the real chassis.

Build the Isaac Sim digital-twin scene: robot + ROS 2 OmniGraph, saved to USD.

    ~/isaacsim/python.sh tools/isaac_twin_setup.py            # build and save
    ~/isaacsim/python.sh tools/isaac_twin_setup.py --run      # build, then simulate

Produces yahboomcar_ws/src/yahboomcar_twin/usd/twin_scene.usd: a stage that *references*
the imported robot, adds ground and light, and carries an OmniGraph subscribing to
/joint_states and driving the articulation. Referencing rather than editing means
tools/urdf_to_usd.py can be re-run without clobbering this.

Graph:

    OnPlaybackTick ──execOut──▶ ROS2SubscribeJointState ──execOut──▶ IsaacArticulationController
                                        │ jointNames ─────────────────▶ jointNames
                                        └ positionCommand ────────────▶ positionCommand

Node type names were read out of the installed extensions rather than the docs, because
the URDF importer API already moved under us once in 6.0.1:
  isaacsim.ros2.bridge.ROS2SubscribeJointState  (outputs execOut/jointNames/positionCommand)
  isaacsim.core.nodes.IsaacArticulationController (inputs execIn/targetPrim/jointNames/...)

ROS_DOMAIN_ID must match the robot (20) and must be set BEFORE Isaac starts, since the
bridge reads it at context creation.
"""
import argparse
import os
import sys

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
USD_DIR = os.path.join(REPO, 'yahboomcar_ws', 'src', 'yahboomcar_twin', 'usd')
ROBOT_USD = os.path.join(USD_DIR, 'MicroROS', 'MicroROS.usda')
SCENE_USD = os.path.join(USD_DIR, 'twin_scene.usd')
ROBOT_PRIM = '/World/micro4'
GRAPH_PATH = '/World/TwinGraph'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--run', action='store_true',
                    help='simulate after building (Ctrl-C to stop)')
    ap.add_argument('--seconds', type=float, default=0.0,
                    help='with --run, stop after N seconds (0 = forever)')
    ap.add_argument('--stiffness', type=float, default=1e5,
                    help='angular drive stiffness; 0 (the import default) means the '
                         'joints never follow the command')
    ap.add_argument('--damping', type=float, default=1e4)
    ap.add_argument('--max-force', type=float, default=1e6)
    ap.add_argument('--headless', action='store_true', default=True)
    ap.add_argument('--gui', dest='headless', action='store_false',
                    help='open the Isaac Sim window')
    args = ap.parse_args()

    if not os.path.exists(ROBOT_USD):
        sys.exit(f'error: robot USD missing at {ROBOT_USD}\n'
                 f'       run: ~/isaacsim/python.sh tools/urdf_to_usd.py')

    domain = os.environ.get('ROS_DOMAIN_ID', '<unset>')
    report_path = os.path.join(USD_DIR, 'twin_scene_report.txt')
    lines = []

    def say(m):
        print(m, flush=True)
        lines.append(str(m))

    from isaacsim import SimulationApp
    app = SimulationApp({'headless': args.headless})

    try:
        from isaacsim.core.utils.extensions import enable_extension
        enable_extension('isaacsim.ros2.bridge')
        app.update()

        import omni.graph.core as og
        import omni.usd
        from pxr import Usd, UsdGeom, UsdLux, Sdf

        say(f'ROS_DOMAIN_ID={domain} (must match the robot, and be set before launch)')

        # Fresh stage referencing the imported robot, so re-importing never clobbers it.
        omni.usd.get_context().new_stage()
        stage = omni.usd.get_context().get_stage()
        UsdGeom.Xform.Define(stage, '/World')
        stage.SetDefaultPrim(stage.GetPrimAtPath('/World'))

        robot = UsdGeom.Xform.Define(stage, ROBOT_PRIM)
        robot.GetPrim().GetReferences().AddReference(ROBOT_USD)
        say(f'referenced robot: {ROBOT_USD} -> {ROBOT_PRIM}')

        # The importer applies PhysicsDriveAPI to all six joints but writes no gain
        # values, so stiffness defaults to ZERO -- a position drive with no stiffness
        # exerts no force, and the joints only twitch instead of tracking the command.
        # Gains are set here, in the scene layer, so re-running urdf_to_usd.py does not
        # clobber them. Values are for a *visual* twin: stiff enough to follow the
        # commanded angle closely, damped enough not to ring.
        from pxr import UsdPhysics
        JOINTS = ['zq_Joint', 'yq_Joint', 'yh_Joint', 'zh_Joint', 'jq1_Joint', 'jq2_Joint']
        driven = []
        for prim in Usd.PrimRange(stage.GetPrimAtPath(ROBOT_PRIM)):
            if prim.GetName() in JOINTS and 'Joint' in str(prim.GetTypeName()):
                drive = UsdPhysics.DriveAPI.Apply(prim, 'angular')
                drive.CreateTypeAttr().Set('force')
                drive.CreateStiffnessAttr().Set(args.stiffness)
                drive.CreateDampingAttr().Set(args.damping)
                drive.CreateMaxForceAttr().Set(args.max_force)
                driven.append(prim.GetName())
        say(f'drive gains set on {len(driven)} joints '
            f'(stiffness={args.stiffness}, damping={args.damping}): {sorted(driven)}')
        if len(driven) != len(JOINTS):
            say(f'WARNING: expected {len(JOINTS)} driven joints, got {len(driven)}')

        UsdGeom.Plane.Define(stage, '/World/GroundPlane').CreateAxisAttr('Z')
        light = UsdLux.DistantLight.Define(stage, '/World/Light')
        light.CreateIntensityAttr(1500.0)

        # The imported articulation root sits under the referenced prim; find it so the
        # controller targets the right thing rather than a guessed path.
        art_root = None
        for prim in Usd.PrimRange(stage.GetPrimAtPath(ROBOT_PRIM)):
            if prim.HasAPI(Usd.SchemaRegistry.GetAPITypeFromSchemaTypeName(
                    'PhysicsArticulationRootAPI')):
                art_root = prim.GetPath()
                break
        if art_root is None:
            art_root = Sdf.Path(ROBOT_PRIM)
            say('WARNING: no PhysicsArticulationRootAPI found; targeting the reference '
                'prim. If joints do not move, this is the first thing to check.')
        say(f'articulation root: {art_root}')

        keys = og.Controller.Keys
        og.Controller.edit(
            {'graph_path': GRAPH_PATH, 'evaluator_name': 'execution'},
            {
                keys.CREATE_NODES: [
                    ('OnTick', 'omni.graph.action.OnPlaybackTick'),
                    ('SubJointState', 'isaacsim.ros2.bridge.ROS2SubscribeJointState'),
                    ('ArtController', 'isaacsim.core.nodes.IsaacArticulationController'),
                ],
                keys.SET_VALUES: [
                    ('SubJointState.inputs:topicName', 'joint_states'),
                    ('ArtController.inputs:robotPath', str(art_root)),
                ],
                keys.CONNECT: [
                    ('OnTick.outputs:tick', 'SubJointState.inputs:execIn'),
                    ('SubJointState.outputs:execOut', 'ArtController.inputs:execIn'),
                    ('SubJointState.outputs:jointNames', 'ArtController.inputs:jointNames'),
                    ('SubJointState.outputs:positionCommand',
                     'ArtController.inputs:positionCommand'),
                ],
            },
        )
        say(f'graph built at {GRAPH_PATH}')

        # Point the controller at the articulation prim as well as the path string.
        ctrl = stage.GetPrimAtPath(f'{GRAPH_PATH}/ArtController')
        if ctrl and ctrl.IsValid():
            rel = ctrl.GetRelationship('inputs:targetPrim')
            if rel:
                rel.SetTargets([art_root])
                say('targetPrim relationship set')

        os.makedirs(USD_DIR, exist_ok=True)
        stage.Export(SCENE_USD)
        say(f'scene saved: {SCENE_USD}')

        nodes = [p.GetPath() for p in Usd.PrimRange(stage.GetPrimAtPath(GRAPH_PATH))
                 if p.GetTypeName() == 'OmniGraphNode']
        say(f'graph nodes: {len(nodes)}')
        for n in nodes:
            say(f'    {n}')

        if args.run:
            from isaacsim.core.api import SimulationContext
            sim = SimulationContext()
            sim.initialize_physics()
            sim.play()
            say('')
            say('simulating -- publish /joint_states to drive the twin, e.g.:')
            say('    ros2 bag play --loop MicroROS-assets/bags/twin_dataset')
            say('    ros2 launch yahboomcar_twin twin_launch.py')
            import time
            t0 = time.time()
            try:
                while True:
                    sim.step(render=True)
            # OnPlaybackTick -- and therefore the whole ROS graph -- is driven by the
            # APPLICATION update loop, not by physics stepping. Without app.update()
            # here the subscriber silently receives nothing and the twin never moves,
            # which looks exactly like a wiring or domain-id problem.
                    app.update()
                    if args.seconds and (time.time() - t0) > args.seconds:
                        break
            except KeyboardInterrupt:
                say('interrupted')
            sim.stop()
    except Exception as e:
        import traceback
        say(f'EXCEPTION: {type(e).__name__}: {e}')
        say(traceback.format_exc())
        raise
    finally:
        os.makedirs(USD_DIR, exist_ok=True)
        with open(report_path, 'w') as f:
            f.write('\n'.join(lines) + '\n')
        app.close()


if __name__ == '__main__':
    main()
