#!/usr/bin/env python3
"""Replay ONE physical bag through slam_toolbox under a given parameter file.

    ./tools/replay_slam_bag.py --bag <bag-dir> --params <slam_toolbox.yaml> --out run1_huber
    ./tools/replay_slam_bag.py --bag <bag-dir> --inspect        # topics only, runs nothing

Exists so parameter comparisons are FAIR. The previous SLAM pass compared configurations
across separate live sessions, which means the motion, the scan noise and the room
occupancy all differed between arms -- so any difference in the maps was unattributable.
Replaying the same recorded bag makes the input bit-identical for every arm, and the
only thing that varies is the parameter under test.

    Phase 4 of docs/slam-delivery-plan.md: >= 2 runs per configuration, same bag
    checksum in every row, winner chosen by tools/score_slam_map.py, and an honest
    "no demonstrated improvement" when the arms sit inside repeatability noise.

SIM TIME IS MANDATORY HERE, and it is the one place in this repo that uses it. Live
sessions deliberately never set `use_sim_time` (so that swapping the simulator for the
car changes nothing), but a replay must run on the bag's clock or the transforms are
compared against wall time and every lookup fails. `ros2 bag play --clock` publishes
/clock and slam_toolbox is launched with `use_sim_time: true` to match.

WHAT A BAG MUST CONTAIN, and why an incomplete one is refused rather than scored:
/scan alone is not enough. slam_toolbox needs `odom -> base_footprint` at each scan's
timestamp, which comes from /tf (or from a live EKF, which a replay does not have). A
bag without /tf produces an empty map that looks like a SLAM failure and is actually a
recording failure -- exactly the confusion the missing-/odom_raw A/B bag caused in the
previous pass. --inspect reports what is present; the runner refuses to replay without
the required set unless --force is given.

DOMAIN: replays run on a scratch domain (default 68) and the car's domain is refused
structurally, before anything starts. A replay publishes /tf and /map; putting that on
the live robot's graph would fight the real transforms.
"""
import argparse
import glob
import hashlib
import json
import os
import signal
import subprocess
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _layout import REPO, WS_SETUP, LOG_DIR                    # noqa: E402
from _cmd_vel_safety import CAR_DOMAIN                          # noqa: E402
# Shared, NOT a private glob: this tool's first draft unlinked every
# /dev/shm/fastrtps* on teardown, and the machine was running a parallel fleet
# session at the time. See tools/_dds_shm.py.
from _dds_shm import stale_segments                             # noqa: E402

REQUIRED_TOPICS = ('/scan', '/tf')
RECOMMENDED_TOPICS = ('/tf_static', '/odom', '/odom_raw', '/imu')
DEFAULT_DOMAIN = 68


def sh(cmd, domain, log=None, background=True):
    """Run with ROS sourced, in its own process group so teardown can kill the tree."""
    full = (f'source /opt/ros/jazzy/setup.bash && source {WS_SETUP} && '
            f'export ROS_DOMAIN_ID={domain} && {cmd}')
    out = open(log, 'w') if log else subprocess.DEVNULL
    p = subprocess.Popen(['bash', '-c', full], stdout=out, stderr=subprocess.STDOUT,
                         start_new_session=True)
    if background:
        return p
    p.wait()
    return p


def kill_tree(p):
    if p is None or p.poll() is not None:
        return
    for sig in (signal.SIGTERM, signal.SIGKILL):
        try:
            os.killpg(os.getpgid(p.pid), sig)
        except (ProcessLookupError, PermissionError):
            return
        for _ in range(20):
            if p.poll() is not None:
                return
            time.sleep(0.15)


def bag_checksum(uri):
    """SHA-256 over the bag's data files, so a comparison table can prove every arm
    consumed the same input."""
    h = hashlib.sha256()
    files = sorted(glob.glob(os.path.join(uri, '*.mcap'))
                   + glob.glob(os.path.join(uri, '*.db3')))
    for path in files:
        with open(path, 'rb') as f:
            for chunk in iter(lambda: f.read(1 << 20), b''):
                h.update(chunk)
    return h.hexdigest() if files else ''


def bag_topics(uri):
    """-> {topic: count}. Uses `ros2 bag info`, which handles storage-id detection."""
    cmd = (f'source /opt/ros/jazzy/setup.bash && source {WS_SETUP} && '
           f'ros2 bag info {uri}')
    p = subprocess.run(['bash', '-c', cmd], capture_output=True, text=True, timeout=120)
    topics = {}
    for line in p.stdout.splitlines():
        line = line.strip()
        if not line.startswith('Topic:'):
            continue
        parts = [seg.strip() for seg in line.split('|')]
        name = parts[0].split('Topic:', 1)[1].strip()
        count = 0
        for seg in parts:
            if seg.startswith('Count:'):
                count = int(seg.split(':', 1)[1])
        topics[name] = count
    return topics


def inspect(uri):
    """Report what the bag holds and whether it can drive a replay. -> (topics, ok)."""
    topics = bag_topics(uri)
    print(f'=== bag: {uri}')
    if not topics:
        print('  no topics found -- is this a bag directory?')
        return topics, False
    for name in sorted(topics):
        print(f'    {name:<34} {topics[name]:6d} msgs')
    missing = [t for t in REQUIRED_TOPICS if t not in topics]
    absent_rec = [t for t in RECOMMENDED_TOPICS if t not in topics]
    print()
    if missing:
        print(f'  MISSING REQUIRED: {", ".join(missing)}')
        print('  A replay without these cannot produce a map, and the empty map would')
        print('  look like a SLAM failure rather than the recording gap it is.')
    if absent_rec:
        print(f'  missing (recommended): {", ".join(absent_rec)}')
    if not missing:
        print('  replayable: required topics present')
    return topics, not missing


def replay(args):
    domain = args.domain
    if domain == CAR_DOMAIN:
        print(f'REFUSED: domain {domain} is the CAR\'S domain. A replay publishes /tf '
              'and /map;')
        print('  putting those on the live robot\'s graph would fight the real '
              'transforms.')
        return 2

    topics, ok = inspect(args.bag)
    if args.inspect:
        return 0 if ok else 1
    if not ok and not args.force:
        print('\nrefusing to replay an incomplete bag (--force to override, and say so '
              'in the report)')
        return 1

    os.makedirs(LOG_DIR, exist_ok=True)
    stamp = time.strftime('%Y%m%d-%H%M%S')
    slam_log = os.path.join(LOG_DIR, f'replay-slam-{stamp}.log')
    play_log = os.path.join(LOG_DIR, f'replay-play-{stamp}.log')
    checksum = bag_checksum(args.bag)
    print(f'\n=== replay  domain {domain}  params {os.path.basename(args.params)}')
    print(f'  bag sha256: {checksum[:16] or "(no data files found)"}')

    p_slam = p_play = None
    result = {'bag': os.path.abspath(args.bag), 'bag_sha256': checksum,
              'params': os.path.abspath(args.params), 'domain': domain,
              'topics': topics, 'started': stamp}
    try:
        print('  [1/4] slam_toolbox (use_sim_time)...')
        p_slam = sh('ros2 launch yahboomcar_config slam_launch.py '
                    f'params_file:={os.path.abspath(args.params)} use_sim_time:=true',
                    domain, log=slam_log)
        time.sleep(8)
        if p_slam.poll() is not None:
            print(f'  FAILED: slam_toolbox exited immediately. See {slam_log}')
            result['error'] = 'slam_toolbox exited during startup'
            return 1

        print(f'  [2/4] bag play --clock (rate {args.rate})...')
        p_play = sh(f'ros2 bag play --clock --rate {args.rate} {args.bag}',
                    domain, log=play_log)
        t0 = time.time()
        while p_play.poll() is None and time.time() - t0 < args.timeout:
            time.sleep(2.0)
        played = time.time() - t0
        if p_play.poll() is None:
            print(f'  playback still running after {args.timeout:.0f} s budget; '
                  'stopping it')
            kill_tree(p_play)
        print(f'        playback finished in {played:.0f} s')
        result['playback_seconds'] = round(played, 1)

        print('  [3/4] settling, then saving the map...')
        time.sleep(args.settle)
        out_base = os.path.abspath(args.out)
        os.makedirs(os.path.dirname(out_base) or '.', exist_ok=True)
        saver = sh(f'ros2 run nav2_map_server map_saver_cli -f {out_base}',
                   domain, log=os.path.join(LOG_DIR, f'replay-save-{stamp}.log'),
                   background=False)
        saved = (saver.returncode == 0 and os.path.exists(out_base + '.yaml')
                 and os.path.exists(out_base + '.pgm'))
        result['map_saved'] = saved
        result['map_yaml'] = out_base + '.yaml' if saved else ''
        print(f'        map saved: {saved}'
              + ('' if saved else '  (no /map was ever published)'))

        # The pose graph is a required Phase-2/5 artifact; ask for it, report honestly
        # if the service is not there rather than pretending it is optional.
        if saved and args.posegraph:
            sh(f'ros2 service call /slam_toolbox/serialize_map '
               f'slam_toolbox/srv/SerializePoseGraph "{{filename: \'{out_base}\'}}"',
               domain, log=os.path.join(LOG_DIR, f'replay-graph-{stamp}.log'),
               background=False)
            result['posegraph'] = os.path.exists(out_base + '.posegraph')
            print(f'        pose graph: {result.get("posegraph")}')
    finally:
        print('  [4/4] teardown...')
        kill_tree(p_play)
        kill_tree(p_slam)
        time.sleep(1.0)
        leftovers = subprocess.run(
            ['bash', '-c', "pgrep -f 'async_slam_toolbox_node|bag pla[y]' | wc -l"],
            capture_output=True, text=True).stdout.strip()
        result['processes_left'] = int(leftovers or 0)
        print(f'        processes left: {leftovers}')
        # STALE ONLY: segments no live process has mapped. A parallel fleet session's
        # transport is not this tool's to delete.
        segs = stale_segments()
        for s in segs:
            try:
                os.unlink(s)
            except OSError:
                pass
        result['shm_segments_cleared'] = len(segs)
        print(f'        stale DDS segments cleared: {len(segs)}')

    if args.json:
        with open(args.json, 'w') as f:
            json.dump(result, f, indent=2)
        print(f'  json: {args.json}')
    print(f'  slam log: {slam_log}')
    return 0 if result.get('map_saved') else 1


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--bag', required=True, help='bag DIRECTORY to replay')
    ap.add_argument('--params',
                    default=os.path.join(REPO, 'yahboomcar_config', 'param',
                                         'slam_toolbox.yaml'))
    ap.add_argument('--out', default='replay_map', help='map basename to save')
    ap.add_argument('--domain', type=int, default=DEFAULT_DOMAIN)
    ap.add_argument('--rate', type=float, default=1.0)
    ap.add_argument('--timeout', type=float, default=600.0)
    ap.add_argument('--settle', type=float, default=8.0,
                    help='seconds after playback before saving the map')
    ap.add_argument('--posegraph', action='store_true',
                    help='also serialize the pose graph')
    ap.add_argument('--inspect', action='store_true',
                    help='report the bag contents and exit, running nothing')
    ap.add_argument('--force', action='store_true',
                    help='replay even if required topics are missing (say so in the '
                         'report)')
    ap.add_argument('--json', default='')
    args = ap.parse_args()
    return replay(args)


if __name__ == '__main__':
    sys.exit(main())
