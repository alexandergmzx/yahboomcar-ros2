#!/usr/bin/env python3
"""SLAM lens: a live browser view of map + scan + poses, ALIGNED, with the
four numbers that convict a bad mapping session while it is still running.

    ./tools/slam_lens.py                     # sim default (domain 66), port 8765
    ./tools/slam_lens.py --domain 68 --sim-time   # watching a replay
    then open  http://localhost:8765/

WHAT IT SHOWS (all in the SLAM map frame, one canvas):
  * the occupancy grid as slam_toolbox publishes it
  * the latest scan's endpoints at their TF-resolved pose, colored hit/miss
    against the map — misalignment is directly visible, not inferred
  * the TF pose (map->base: SLAM's opinion), a ground-truth ghost (aligned
    once at start), and a pure-odometry ghost (odom->base under the
    map->odom captured at start, i.e. the pose if SLAM never corrected)
  * live metrics with history: scan-to-map fit, pose divergence vs truth,
    /odom_raw-vs-truth yaw-rate ratio, scan staleness, TF resolve health

WHY IT EXISTS: the 2026-08-09 isaac --fun session produced a smeared map
with FIVE contributing failure layers (render-pacing runaway, the encoder
yaw lie, queue-full TF drops, shoved fun boxes, and an RViz frame artifact)
and not one of them was visible in RViz while it happened. Each of the four
metrics here corresponds to one measured failure; the metric module
(tools/_slam_lens_core.py) documents which.

DESIGN NOTES, deliberately copied from fleet-console (the console's
"SLAM map view + lidar overlay" milestone is where this ports to later):
  * subscriptions are BEST_EFFORT sensor QoS regardless of the publisher's
    offer (fleet OI-20: BEST_EFFORT matches any offer; the reverse starves)
    — except /map, which is latched TRANSIENT_LOCAL/RELIABLE by slam_toolbox
    and needs the matching subscription.
  * SingleThreadedExecutor, and callbacks only store-and-stamp; all metric
    computation happens in the 5 Hz snapshot loop. fleet-console measured
    MultiThreaded at 102.5% CPU vs 8.8% on this exact workload class, and a
    per-callback reduction starving the send loop through the GIL.
  * the browser gets COALESCED SNAPSHOTS at a fixed rate over one WebSocket,
    never per-message forwarding. The map ships only when its seq changes.

READ-ONLY BY CONSTRUCTION: subscribes and looks up TF, publishes nothing,
so it can watch any session — sim, replay, or (from the bench, domain given
explicitly) the real car — without being able to disturb it.

Dependencies: rclpy + tf2_ros from the sourced fleet env, numpy, and the
`websockets` package (present system-wide; FastAPI/aiohttp are not, and a
diagnostic tool should not grow a venv). The page is plain canvas + JS
served by this same process: one port, no build step.
"""
import argparse
import asyncio
import hashlib
import http
import json
import math
import os
import sys
import threading
import time
from collections import deque

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _slam_lens_core import (                                   # noqa: E402
    PoseAligner, StalenessTracker, TruthHistory, YawRatioWindow, content_lag,
    divergence, occupied_mask_dilated, rle_encode, scan_endpoints,
    scan_map_fit, transform_points)
from _layout import REPO                                        # noqa: E402

# The content-lag metric needs the shared walls model (the same single source
# the Isaac USD is built from). Sim-only by nature; the lens still runs
# without it (tile reads em-dash), e.g. pointed at hardware.
try:
    sys.path.insert(0, os.path.join(REPO, 'yahboomcar_sim'))
    from yahboomcar_sim.arena import raycast, segments_room     # noqa: E402
    WALL_SEGS = segments_room()
except ImportError:                                             # pragma: no cover
    WALL_SEGS = None

PAGE = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'slam_lens.html')
SNAPSHOT_HZ = 5.0
HISTORY_LEN = 1500              # 5 min at SNAPSHOT_HZ
TF_WINDOW = 100                 # snapshots in the TF-health ratio window


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _r3(x):
    return None if x is None else round(float(x), 3)


class LensNode:
    """Owns the rclpy node, subscriptions and TF buffer. Thread-safe state."""

    def __init__(self, args):
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import (QoSDurabilityPolicy, QoSProfile,
                               QoSReliabilityPolicy, qos_profile_sensor_data)
        from nav_msgs.msg import OccupancyGrid, Odometry
        from sensor_msgs.msg import LaserScan
        import tf2_ros

        self.args = args
        self.lock = threading.Lock()
        self.node = Node('slam_lens', parameter_overrides=[
            rclpy.parameter.Parameter('use_sim_time', value=bool(args.sim_time))])

        # ---- stored state (written by callbacks under lock) ----
        self.scans = deque(maxlen=4)          # (msg, rx wall time), newest last
        self.tf_pending = deque(maxlen=64)    # (stamp msg, rx) awaiting their verdict
        self.map_msg = None
        self.map_seq = 0
        self.truth = None
        self.odom = None
        self.counts = {'scan': 0, 'map': 0, 'truth': 0, 'odom': 0, 'odom_raw': 0}
        self.t0 = time.time()
        self.stale = StalenessTracker()
        self.yaw_win = YawRatioWindow()
        self.truth_hist = TruthHistory()
        self._lag = None              # (offset_s, rms_m) of the last sweep
        self._lag_tick = 0
        self.truth_align = PoseAligner()      # truth frame -> map frame
        self.odom_align = PoseAligner()       # odom frame  -> map frame (frozen at t0)
        self.tf_results = deque(maxlen=TF_WINDOW)
        self.tf_fail_streak = 0
        self._mask_cache = (None, None)       # (map_seq, dilated mask)

        self.tf_buffer = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=30))
        self.tf_listener = tf2_ros.TransformListener(
            self.tf_buffer, self.node, spin_thread=False)

        latched = QoSProfile(
            depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        s = qos_profile_sensor_data

        self.node.create_subscription(OccupancyGrid, args.map_topic, self._on_map, latched)
        self.node.create_subscription(LaserScan, args.scan_topic, self._on_scan, s)
        self.node.create_subscription(Odometry, args.truth_topic, self._on_truth, s)
        self.node.create_subscription(Odometry, '/odom', self._on_odom, s)
        self.node.create_subscription(Odometry, '/odom_raw', self._on_odom_raw, s)

        self._rclpy = rclpy

    # ---- callbacks: store and stamp, nothing else -------------------------
    def _on_map(self, msg):
        with self.lock:
            self.map_msg = msg
            self.map_seq += 1
            self.counts['map'] += 1

    def _on_scan(self, msg):
        digest = hashlib.blake2b(
            np.asarray(msg.ranges, dtype=np.float32).tobytes(), digest_size=16).digest()
        now = time.time()
        with self.lock:
            self.scans.append((msg, now))
            self.tf_pending.append((msg.header.stamp, now))
            self.counts['scan'] += 1
            self.stale.feed(digest)

    def _on_truth(self, msg):
        p = msg.pose.pose
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        with self.lock:
            self.truth = (p.position.x, p.position.y, yaw_of(p.orientation))
            self.counts['truth'] += 1
            self.yaw_win.feed_truth_yaw(time.time(), self.truth[2])
            self.truth_hist.feed(stamp, self.truth)

    def _on_odom(self, msg):
        p = msg.pose.pose
        with self.lock:
            self.odom = (p.position.x, p.position.y, yaw_of(p.orientation))
            self.counts['odom'] += 1

    def _on_odom_raw(self, msg):
        with self.lock:
            self.counts['odom_raw'] += 1
            self.yaw_win.feed_odom(time.time(), msg.twist.twist.angular.z)

    # ---- TF ----------------------------------------------------------------
    def _lookup(self, target, source, stamp=None):
        """-> (x, y, yaw) or None. Zero timeout: the snapshot loop never blocks."""
        try:
            t = self.tf_buffer.lookup_transform(
                target, source,
                stamp if stamp is not None else self._rclpy.time.Time())
        except Exception:
            return None
        tr, q = t.transform.translation, t.transform.rotation
        return (tr.x, tr.y, yaw_of(q))

    # ---- the snapshot ------------------------------------------------------
    def build_state(self):
        """Compute everything for one snapshot. Called from the asyncio loop."""
        with self.lock:
            scans = list(self.scans)
            map_msg = self.map_msg
            map_seq = self.map_seq
            truth = self.truth
            odom = self.odom
            counts = dict(self.counts)
            stale_run = self.stale.current_run
            stale_max = self.stale.max_run
            stale_frac = self.stale.duplicate_fraction
            yaw_ratio, yaw_n = self.yaw_win.ratio()

        now = time.time()
        base = self.args.base_frame

        pose = self._lookup(self.args.map_frame, base)

        # TF health, scored the way the CONSUMER experiences it: a scan's
        # stamp is only judged unresolvable once it is older than the grace
        # a message filter would give it (transform_timeout 0.2 s + one EKF
        # period). Judging the newest scan immediately over-counted failures
        # 4:1 on a healthy 2D session [measured tonight] because a fresh
        # scan's odom TF simply hasn't arrived yet — that is latency, not
        # unavailability.
        TF_GRACE_S = 0.35
        with self.lock:
            due = []
            while self.tf_pending and now - self.tf_pending[0][1] >= TF_GRACE_S:
                due.append(self.tf_pending.popleft()[0])
        for stamp_msg in due:
            ok = self._lookup(self.args.map_frame, self.args.base_frame,
                              self._rclpy.time.Time.from_msg(stamp_msg)) is not None
            self.tf_results.append(ok)
            self.tf_fail_streak = 0 if ok else self.tf_fail_streak + 1

        # Render/score the newest scan that resolves at its own stamp; fall
        # back to the newest scan at latest TF (flagged) so the display never
        # freezes just because the freshest stamp is still in its TF grace.
        scan_payload = None
        fit = None
        if scans:
            scan, scan_rx, tf_pose, resolved = None, 0.0, None, False
            for msg, rx in reversed(scans):
                p = self._lookup(self.args.map_frame, msg.header.frame_id,
                                 self._rclpy.time.Time.from_msg(msg.header.stamp))
                if p is not None:
                    scan, scan_rx, tf_pose, resolved = msg, rx, p, True
                    break
            if scan is None:
                scan, scan_rx = scans[-1]
                tf_pose = self._lookup(self.args.map_frame, scan.header.frame_id)

            # Content lag, every 3rd snapshot (a 15-offset walls-raycast
            # sweep). Sim-only: needs the arena model and a truth history.
            self._lag_tick += 1
            if WALL_SEGS is not None and self._lag_tick % 3 == 0:
                s_newest = scans[-1][0]
                stamp_s = (s_newest.header.stamp.sec
                           + s_newest.header.stamp.nanosec * 1e-9)
                n_beams = len(s_newest.ranges)

                def _ray(pose, _n=n_beams):
                    return raycast((pose[0], pose[1]), pose[2], WALL_SEGS,
                                   n_beams=_n)

                with self.lock:
                    self._lag = content_lag(
                        s_newest.ranges, s_newest.range_min,
                        s_newest.range_max, self.truth_hist.pose_at, _ray,
                        stamp_s)

            if tf_pose is not None:
                pts_l = scan_endpoints(scan.ranges, scan.angle_min,
                                       scan.angle_increment, scan.range_min,
                                       scan.range_max)
                pts_m = transform_points(pts_l, tf_pose)
                hits = np.zeros(pts_m.shape[0], dtype=bool)
                if map_msg is not None:
                    seq, mask = self._mask_cache
                    if seq != map_seq:
                        h, w = map_msg.info.height, map_msg.info.width
                        grid = np.asarray(map_msg.data, dtype=np.int8).reshape(h, w)
                        mask = occupied_mask_dilated(grid)
                        self._mask_cache = (map_seq, mask)
                    fit, hits = scan_map_fit(
                        pts_m, mask, map_msg.info.resolution,
                        map_msg.info.origin.position.x,
                        map_msg.info.origin.position.y)
                scan_payload = {
                    'age': _r3(now - scan_rx),
                    'resolved': resolved,
                    'points': np.round(pts_m, 3).tolist(),
                    'hits': hits.astype(int).tolist(),
                }

        # Ghosts. Both aligners lock their anchor on the first snapshot where
        # the needed pair exists; everything after is relative motion.
        truth_ghost = None
        if pose is not None and truth is not None:
            self.truth_align.feed(pose, truth)
            truth_ghost = self.truth_align.truth_in_map(truth)
        odom_ghost = None
        if pose is not None and odom is not None:
            self.odom_align.feed(pose, odom)
            odom_ghost = self.odom_align.truth_in_map(odom)

        div_pos = div_yaw = None
        if pose is not None and truth_ghost is not None:
            div_pos, div_yaw = divergence(pose, truth_ghost)

        dt = max(1e-6, now - self.t0)
        tf_ok = (sum(self.tf_results) / len(self.tf_results)) if self.tf_results else None

        state = {
            't': _r3(now - self.t0),
            'rates': {k: _r3(v / dt) for k, v in counts.items()},
            'pose': None if pose is None else [_r3(v) for v in pose],
            'truth_ghost': None if truth_ghost is None else [_r3(v) for v in truth_ghost],
            'odom_ghost': None if odom_ghost is None else [_r3(v) for v in odom_ghost],
            'scan': scan_payload,
            'metrics': {
                'fit': _r3(fit),
                'div_pos': _r3(div_pos),
                'div_yaw': _r3(div_yaw),
                'yaw_ratio': _r3(yaw_ratio),
                'yaw_n': yaw_n,
                'stale_run': stale_run,
                'stale_max': stale_max,
                'stale_frac': _r3(stale_frac),
                'tf_ok_frac': _r3(tf_ok),
                'tf_fail_streak': self.tf_fail_streak,
                'lag_s': _r3(self._lag[0]) if self._lag else None,
                'lag_rms': _r3(self._lag[1]) if self._lag else None,
            },
            'map_seq': map_seq,
        }
        map_payload = None
        if map_msg is not None:
            i = map_msg.info
            map_payload = {
                'seq': map_seq, 'w': i.width, 'h': i.height,
                'res': i.resolution,
                'ox': _r3(i.origin.position.x), 'oy': _r3(i.origin.position.y),
                'rle': rle_encode(map_msg.data),
            }
        return state, map_payload


async def serve(node: LensNode, args):
    import websockets

    history = deque(maxlen=HISTORY_LEN)
    latest = {'state': None, 'map': None}

    stop = asyncio.Event()

    async def sampler():
        # Also the process's shutdown watcher: rclpy's signal handlers absorb
        # SIGINT/SIGTERM and shut the ROS context down WITHOUT exiting, which
        # left the first smoke-test's server running headless until SIGKILL.
        # rclpy.ok() going false is therefore the one reliable stop signal.
        period = 1.0 / SNAPSHOT_HZ
        while True:
            if not node._rclpy.ok():
                stop.set()
                return
            state, map_payload = node.build_state()
            latest['state'] = state
            latest['map'] = map_payload
            m = state['metrics']
            history.append([state['t'], m['fit'], m['div_pos'],
                            m['yaw_ratio'], m['stale_run'], m['lag_s']])
            await asyncio.sleep(period)

    async def handler(ws):
        # A client hanging up mid-send is the NORMAL end of a connection
        # (page closed, probe finished) — swallow it, or every disconnect
        # writes a 12-line traceback into the session log.
        sent_map_seq = -1
        try:
            await ws.send(json.dumps({'type': 'hello', 'history': list(history),
                                      'config': {'snapshot_hz': SNAPSHOT_HZ}}))
            while True:
                state = latest['state']
                if state is not None:
                    msg = {'type': 'snapshot', 'state': state}
                    if latest['map'] is not None and latest['map']['seq'] != sent_map_seq:
                        msg['map'] = latest['map']
                        sent_map_seq = latest['map']['seq']
                    await ws.send(json.dumps(msg))
                await asyncio.sleep(1.0 / SNAPSHOT_HZ)
        except websockets.ConnectionClosed:
            return

    async def process_request(path, request_headers):
        if path.split('?')[0] in ('/', '/index.html'):
            with open(PAGE, 'rb') as f:
                body = f.read()
            return (http.HTTPStatus.OK,
                    [('Content-Type', 'text/html; charset=utf-8'),
                     ('Cache-Control', 'no-store')], body)
        if path == '/healthz':
            return (http.HTTPStatus.OK, [('Content-Type', 'text/plain')], b'ok\n')
        return None      # anything else: proceed with the WebSocket handshake

    asyncio.create_task(sampler())
    async with websockets.serve(handler, args.host, args.port,
                                process_request=process_request):
        print(f'slam_lens: http://{args.host}:{args.port}/   '
              f'(domain {os.environ.get("ROS_DOMAIN_ID", "?")}, '
              f'map {args.map_topic}, scan {args.scan_topic})', flush=True)
        await stop.wait()
        print('slam_lens: ROS context shut down, exiting', flush=True)


def main():
    ap = argparse.ArgumentParser(
        description=__doc__.splitlines()[0], allow_abbrev=False)
    ap.add_argument('--domain', type=int, default=66,
                    help='ROS_DOMAIN_ID (66 = the sim convention; the hardware '
                         'domain is never a default in this repo)')
    ap.add_argument('--host', default='127.0.0.1')
    ap.add_argument('--port', type=int, default=8765)
    ap.add_argument('--map-topic', default='/map')
    ap.add_argument('--scan-topic', default='/scan')
    ap.add_argument('--truth-topic', default='/sim/ground_truth')
    ap.add_argument('--map-frame', default='map')
    ap.add_argument('--base-frame', default='base_footprint')
    ap.add_argument('--sim-time', action='store_true',
                    help='use /clock (replays only, same rule as replay_slam_bag)')
    args = ap.parse_args()

    os.environ.setdefault('ROS_DOMAIN_ID', str(args.domain))

    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    rclpy.init()
    node = LensNode(args)

    # Deliberately SingleThreaded — see the module docstring for the measured
    # reason. The executor runs in a daemon thread; asyncio owns the main one.
    ex = SingleThreadedExecutor()
    ex.add_node(node.node)

    def _spin():
        # ExternalShutdownException at teardown is the normal Ctrl+C path,
        # not an error; the relay's log grew the same traceback until it was
        # understood. Exit the thread quietly.
        from rclpy.executors import ExternalShutdownException
        try:
            ex.spin()
        except ExternalShutdownException:
            pass

    spin = threading.Thread(target=_spin, daemon=True)
    spin.start()

    try:
        asyncio.run(serve(node, args))
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
