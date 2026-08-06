#!/usr/bin/env python3
"""Verify the Isaac twin actually articulates from ROS /joint_states.

    export ROS_DOMAIN_ID=20
    ~/isaacsim/python.sh tools/isaac_twin_verify.py --seconds 40
    # meanwhile, in another shell:
    ros2 bag play --loop MicroROS-assets/bags/twin_dataset
    ros2 launch yahboomcar_twin twin_launch.py use_description:=false

Loads twin_scene.usd, runs physics, and samples the articulation's joint positions over
time. Reports per-joint travel, so "the graph exists" is distinguished from "the joints
actually moved" -- the whole point, since a correctly-built graph with a topic mismatch
looks identical to a working one until you measure.
"""
import argparse
import os
import sys
import time

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
USD_DIR = os.path.join(REPO, 'yahboomcar_ws', 'src', 'yahboomcar_twin', 'usd')
SCENE_USD = os.path.join(USD_DIR, 'twin_scene.usd')
REPORT = os.path.join(USD_DIR, 'twin_verify_report.txt')
EXPECTED = ['zq_Joint', 'yq_Joint', 'yh_Joint', 'zh_Joint', 'jq1_Joint', 'jq2_Joint']


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--seconds', type=float, default=30.0)
    ap.add_argument('--gui', action='store_true')
    args = ap.parse_args()

    if not os.path.exists(SCENE_USD):
        sys.exit(f'no scene at {SCENE_USD}; run tools/isaac_twin_setup.py first')

    lines = []

    def say(m):
        print(m, flush=True)
        lines.append(str(m))

    from isaacsim import SimulationApp
    app = SimulationApp({'headless': not args.gui})
    try:
        from isaacsim.core.utils.extensions import enable_extension
        enable_extension('isaacsim.ros2.bridge')
        app.update()

        import omni.usd
        omni.usd.get_context().open_stage(SCENE_USD)
        app.update()
        say(f'ROS_DOMAIN_ID={os.environ.get("ROS_DOMAIN_ID", "<unset>")}')
        say(f'opened {SCENE_USD}')

        from isaacsim.core.api import SimulationContext
        from isaacsim.core.prims import SingleArticulation

        sim = SimulationContext()
        sim.initialize_physics()
        sim.play()
        # app.update() is essential: OnPlaybackTick, and so the entire ROS graph, is
        # driven by the application update loop rather than by physics stepping. Without
        # it the subscriber receives nothing and every joint reads exactly 0.000.
        for _ in range(20):
            sim.step(render=True)
            app.update()

        art = SingleArticulation('/World/micro4/Geometry/base_link')
        art.initialize()
        names = list(art.dof_names or [])
        say(f'articulation DOFs: {len(names)} -> {names}')

        missing = [j for j in EXPECTED if j not in names]
        if missing:
            say(f'WARNING: expected joints absent from the articulation: {missing}')

        lo = {n: float('inf') for n in names}
        hi = {n: float('-inf') for n in names}
        samples = 0
        t0 = time.time()
        while time.time() - t0 < args.seconds:
            sim.step(render=True)
            app.update()
            pos = art.get_joint_positions()
            if pos is not None:
                for i, n in enumerate(names):
                    v = float(pos[i])
                    lo[n] = min(lo[n], v)
                    hi[n] = max(hi[n], v)
                samples += 1

        sim.stop()
        say(f'samples: {samples} over {args.seconds:.0f}s')
        say('')
        say(f'{"joint":<14}{"min":>10}{"max":>10}{"range":>10}')
        moved = []
        for n in names:
            if lo[n] == float('inf'):
                continue
            rng = hi[n] - lo[n]
            say(f'{n:<14}{lo[n]:>10.3f}{hi[n]:>10.3f}{rng:>10.3f}')
            if rng > 0.05:
                moved.append(n)
        say('')
        say(f'joints that moved: {len(moved)} -> {sorted(moved)}')

        # "Moved at all" is too weak a bar. The first run passed that test while the
        # joints were barely twitching, because the imported drives had zero stiffness.
        # Compare against what the recording actually commands: /servo_s1 sweeps +/-60
        # deg (2.09 rad peak-to-peak) and the wheels wind continuously, so a working
        # twin should show wheel travel of at least a couple of radians.
        gimbal_ok = (hi.get('jq1_Joint', 0) - lo.get('jq1_Joint', 0)) > 1.5
        wheels = [hi[n] - lo[n] for n in ('zq_Joint', 'yq_Joint', 'yh_Joint', 'zh_Joint')
                  if n in hi and lo[n] != float('inf')]
        wheels_ok = wheels and min(wheels) > 2.0
        say(f'  jq1 range {hi.get("jq1_Joint",0)-lo.get("jq1_Joint",0):.3f} rad '
            f'(expect >1.5 for a +/-60 deg sweep) -> {"ok" if gimbal_ok else "TOO SMALL"}')
        say(f'  min wheel range {min(wheels) if wheels else 0:.3f} rad '
            f'(expect >2.0) -> {"ok" if wheels_ok else "TOO SMALL"}')

        if len(moved) >= 4 and gimbal_ok and wheels_ok:
            say('VERDICT: twin articulates AND tracks the commanded angles')
        elif len(moved) >= 4:
            say('VERDICT: joints move but do NOT track. Almost always zero/low drive '
                'stiffness -- the URDF importer applies DriveAPI without gain values. '
                'Re-run isaac_twin_setup.py (it sets them) or raise --stiffness.')
        elif moved:
            say('VERDICT: PARTIAL -- some joints moved, some did not')
        else:
            say('VERDICT: NO MOVEMENT. Check, in order: is anything publishing '
                '/joint_states; does ROS_DOMAIN_ID match; is the topic name in the '
                'graph right; is targetPrim the articulation root.')
    except Exception as e:
        import traceback
        say(f'EXCEPTION: {type(e).__name__}: {e}')
        say(traceback.format_exc())
    finally:
        with open(REPORT, 'w') as f:
            f.write('\n'.join(lines) + '\n')
        app.close()


if __name__ == '__main__':
    main()
