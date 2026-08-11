#!/usr/bin/env python3
"""Prove SLAM's TF and timing preconditions hold — BEFORE touching a SLAM parameter.

    ./tools/slam_preflight.py                      # 60 s gate, exits nonzero on failure
    ./tools/slam_preflight.py --seconds 90 --json report.json
    ./tools/slam_preflight.py --odom-frame odom --laser-frame laser_frame

Passive: it subscribes and listens. It commands nothing, so it is safe with the car
powered and stationary. It needs bringup running (something must publish
`odom -> base_footprint`), and it needs NO SLAM — the point is to answer whether SLAM
*can* work here before blaming SLAM for the fact that it doesn't.

WHY THIS EXISTS
---------------
The 2026-08-08 hardware SLAM launch logged **135 message-filter drops, mostly "queue is
full"**, and produced no saved map. A message-filter drop means slam_toolbox asked TF to
put a scan into the odom frame AT THE SCAN'S OWN TIMESTAMP and TF could not answer, so
the scan was queued, and then the queue filled and threw scans away. Tuning
`slam_toolbox.yaml` against that is tuning the wrong layer: the scans never reached the
matcher.

So the gate is exact-time transform availability, not "is TF publishing" — those are
different questions and only the first one predicts whether SLAM will see the data. A
transform that exists at *now* and a transform that exists at *the scan's stamp* differ
by exactly the sensor latency, and this link's latency has been measured between 6 ms
and 0.51 s across bags. That spread is the whole problem, and `transform_timeout` is not
allowed to be the answer: raising it until the warnings stop hides the lag rather than
measuring it.

WHAT IT WILL NOT DO
-------------------
It does not rename frames. `/odom_raw.header.frame_id` is literally the string
`"odom_frame"` on this firmware (deserialized from
`MicroROS-assets/bags/bench-matrix-20260808-093840`, 2026-08-09) while the EKF and the
SLAM configuration both say `odom`, and **no static transform bridging the two exists in
either repo** [verified by grep, both checkouts]. Whether robot_localization drops,
transforms, or accepts that pose component is a live question this tool REPORTS and does
not guess at. Renaming by intuition is how you get two broadcasters fighting over one
edge.

THE GATE
--------
Over `--seconds` (default 60) after a warm-up (default 5 s, because an empty TF buffer
legitimately fails until the first transforms land):

  * >= 99% of scans resolve `odom -> laser_frame` AT THE SCAN'S OWN STAMP, and
  * the longest consecutive failure run is <= `--max-streak` scans.

Anything else exits nonzero and says which clause failed. The streak clause is separate
on purpose: 1% of scans failing at random is a different world from 1% of scans failing
all at once, and only the second one empties a message-filter queue.
"""
import argparse
import json
import math
import os
import sys
import time
from dataclasses import dataclass, field, asdict

# NOTE: no rclpy at module scope. Everything above main() is pure and importable
# without a ROS graph, which is what makes tools/tests/test_slam_preflight.py able to
# run the gate logic against synthetic streams -- including the missing-link negative
# control, which is the only way to know the gate can actually fail.

DEFAULT_ODOM_FRAME = 'odom'
DEFAULT_LASER_FRAME = 'laser_frame'
DEFAULT_BASE_FRAME = 'base_footprint'
# The firmware's own label for the frame it stamps /odom_raw with. Not a typo on our
# side and not silently rewritten: see the module docstring.
FIRMWARE_ODOM_FRAME_LABEL = 'odom_frame'

GATE_SUCCESS_PCT = 99.0
GATE_MAX_STREAK = 3
DEFAULT_WARMUP_S = 5.0
DEFAULT_SECONDS = 60.0

# Rates the firmware contract promises (CLAUDE.md), with the tolerance car_selftest.py
# already uses. Reported for context; they are not part of the pass/fail gate, because
# a slow link is a different finding from an unresolvable transform.
CONTRACT_HZ = {'/scan': 12.0, '/odom_raw': 11.0, '/odom': 11.0}


def percentile(values, pct):
    """Linear-interpolated percentile of a list. Empty -> nan.

    Hand-rolled so this module stays importable with nothing but the stdlib.
    """
    if not values:
        return float('nan')
    ordered = sorted(values)
    if len(ordered) == 1:
        return float(ordered[0])
    pos = (len(ordered) - 1) * (pct / 100.0)
    low = int(math.floor(pos))
    high = int(math.ceil(pos))
    if low == high:
        return float(ordered[low])
    return float(ordered[low] + (ordered[high] - ordered[low]) * (pos - low))


@dataclass
class RateStats:
    topic: str
    count: int = 0
    hz: float = float('nan')
    gap_p50_ms: float = float('nan')
    gap_p95_ms: float = float('nan')
    gap_max_ms: float = float('nan')
    contract_hz: float = float('nan')

    @property
    def within_contract(self):
        if not math.isfinite(self.hz) or not math.isfinite(self.contract_hz):
            return False
        # Same +/-3 Hz shape car_selftest.py uses for the sensor topics.
        return abs(self.hz - self.contract_hz) <= 3.0


def rate_stats(topic, arrival_times, contract_hz=float('nan')):
    """Arrival wall-times -> RateStats. Gaps are consecutive-arrival deltas."""
    st = RateStats(topic=topic, count=len(arrival_times), contract_hz=contract_hz)
    if len(arrival_times) < 2:
        return st
    span = arrival_times[-1] - arrival_times[0]
    if span > 0:
        st.hz = (len(arrival_times) - 1) / span
    gaps_ms = [(b - a) * 1000.0
               for a, b in zip(arrival_times[:-1], arrival_times[1:])]
    st.gap_p50_ms = percentile(gaps_ms, 50)
    st.gap_p95_ms = percentile(gaps_ms, 95)
    st.gap_max_ms = max(gaps_ms)
    return st


@dataclass
class LatencyStats:
    topic: str
    count: int = 0
    p50_ms: float = float('nan')
    p95_ms: float = float('nan')
    max_ms: float = float('nan')
    negative_count: int = 0     # header stamped in the FUTURE relative to receipt


def latency_stats(topic, pairs):
    """pairs: [(receive_wall_time, header_stamp_seconds)] -> LatencyStats.

    This is the number that decides whether an exact-time lookup can succeed at all:
    TF must already hold a transform that old (or be able to interpolate to it) by the
    time the scan arrives. Measured across existing bags it ranges 6-80 ms in some and
    0.33-0.51 s in others, which is precisely why it is measured per session rather
    than assumed.
    """
    st = LatencyStats(topic=topic, count=len(pairs))
    if not pairs:
        return st
    lat_ms = [(recv - stamp) * 1000.0 for recv, stamp in pairs]
    st.p50_ms = percentile(lat_ms, 50)
    st.p95_ms = percentile(lat_ms, 95)
    st.max_ms = max(lat_ms)
    st.negative_count = sum(1 for v in lat_ms if v < 0)
    return st


@dataclass
class TfGateResult:
    attempted: int = 0
    succeeded: int = 0
    success_pct: float = float('nan')
    longest_failure_streak: int = 0
    streak_started_at: float = float('nan')   # relative seconds into the run
    window_s: float = 0.0
    warmup_discarded: int = 0
    threshold_pct: float = GATE_SUCCESS_PCT
    max_streak: int = GATE_MAX_STREAK
    failures: list = field(default_factory=list)   # sample of (t_rel, reason)

    @property
    def pct_ok(self):
        return math.isfinite(self.success_pct) and self.success_pct >= self.threshold_pct

    @property
    def streak_ok(self):
        return self.longest_failure_streak <= self.max_streak

    @property
    def passed(self):
        return self.attempted > 0 and self.pct_ok and self.streak_ok

    def reasons(self):
        """Why it failed, named. Empty when it passed."""
        out = []
        if self.attempted == 0:
            out.append('no scans were evaluated at all (is /scan publishing?)')
            return out
        if not self.pct_ok:
            out.append(f'exact-time TF success {self.success_pct:.2f}% is below the '
                       f'{self.threshold_pct:.0f}% gate')
        if not self.streak_ok:
            out.append(f'longest consecutive failure run {self.longest_failure_streak} '
                       f'scans exceeds the {self.max_streak}-scan limit '
                       f'(a burst empties a message-filter queue; scattered '
                       f'failures do not)')
        return out


def evaluate_tf_gate(samples, warmup_s=DEFAULT_WARMUP_S,
                     threshold_pct=GATE_SUCCESS_PCT, max_streak=GATE_MAX_STREAK,
                     failure_sample_limit=10):
    """samples: [(t_rel_seconds, ok_bool, reason_str)] in time order -> TfGateResult.

    `t_rel` is seconds since the listener started. Samples inside `warmup_s` are
    DISCARDED, not counted as failures: an empty TF buffer cannot answer a question
    about a timestamp it has no data for, and calling that a SLAM defect would make the
    gate lie in the one direction that matters (a false alarm trains people to ignore
    it).
    """
    res = TfGateResult(threshold_pct=threshold_pct, max_streak=max_streak)
    scored = []
    for t_rel, ok, reason in samples:
        if t_rel < warmup_s:
            res.warmup_discarded += 1
            continue
        scored.append((t_rel, bool(ok), reason))
    res.attempted = len(scored)
    if not scored:
        return res
    res.window_s = scored[-1][0] - scored[0][0]
    res.succeeded = sum(1 for _, ok, _ in scored if ok)
    res.success_pct = 100.0 * res.succeeded / res.attempted
    streak = 0
    streak_start = float('nan')
    for t_rel, ok, reason in scored:
        if ok:
            streak = 0
            continue
        if streak == 0:
            streak_start = t_rel
        streak += 1
        if streak > res.longest_failure_streak:
            res.longest_failure_streak = streak
            res.streak_started_at = streak_start
        if len(res.failures) < failure_sample_limit:
            res.failures.append((round(t_rel, 3), reason))
    return res


@dataclass
class FrameReport:
    """What the graph actually calls things, versus what the configuration expects."""
    odom_raw_frame: str = ''
    odom_raw_child: str = ''
    odom_frame: str = ''
    odom_child: str = ''
    scan_frame: str = ''
    configured_odom_frame: str = DEFAULT_ODOM_FRAME
    configured_laser_frame: str = DEFAULT_LASER_FRAME

    def findings(self):
        """Named mismatches. Reported, never auto-corrected."""
        out = []
        if self.odom_raw_frame and self.odom_raw_frame != self.configured_odom_frame:
            out.append(
                f'/odom_raw is stamped frame_id={self.odom_raw_frame!r} but the '
                f'configuration uses {self.configured_odom_frame!r}. This is the known '
                'firmware label (see the module docstring): NOT auto-renamed here. '
                'Whether robot_localization drops, transforms, or accepts that pose '
                'component is decided by the live EKF, not by this tool.')
        if self.scan_frame and self.scan_frame != self.configured_laser_frame:
            out.append(f'/scan is stamped frame_id={self.scan_frame!r} but the '
                       f'configuration uses {self.configured_laser_frame!r}')
        if self.odom_frame and self.odom_frame != self.configured_odom_frame:
            out.append(f'/odom (the EKF output) is stamped frame_id='
                       f'{self.odom_frame!r}, not {self.configured_odom_frame!r}')
        return out


def duplicate_findings(publishers_by_topic):
    """publishers_by_topic: {topic: [node_name, ...]} -> list of finding strings.

    Two broadcasters on one TF edge is the failure this repo has already met once
    (`base_node_X3` alongside the EKF, both sending odom->base_footprint). It presents
    as a robot that jitters between two poses, which reads as a sensor fault.
    """
    out = []
    for topic, nodes in sorted(publishers_by_topic.items()):
        if len(nodes) > 1 and topic in ('/tf', '/odom', '/odom_raw', '/scan'):
            out.append(f'{len(nodes)} publishers on {topic}: {", ".join(sorted(nodes))}'
                       + ('  <-- two broadcasters on one edge fight; expect pose jitter'
                          if topic == '/tf' else ''))
    return out


def format_report(gate, rates, latencies, frames, dup_findings, tf_age):
    """-> (text, ok). The single place the human-readable verdict is composed."""
    lines = []
    lines.append('=== slam_preflight ===')
    lines.append('')
    lines.append('  RATES (context, not the gate)')
    for st in rates:
        mark = 'ok ' if st.within_contract else '   '
        lines.append(f'    {mark}{st.topic:<12} {st.count:5d} msgs  {st.hz:6.2f} Hz  '
                     f'gap p50 {st.gap_p50_ms:6.1f}  p95 {st.gap_p95_ms:6.1f}  '
                     f'max {st.gap_max_ms:6.1f} ms')
    lines.append('')
    lines.append('  LATENCY  receive-time minus header-stamp')
    for st in latencies:
        lines.append(f'      {st.topic:<12} p50 {st.p50_ms:7.1f}  p95 {st.p95_ms:7.1f}  '
                     f'max {st.max_ms:7.1f} ms'
                     + (f'   ({st.negative_count} stamped in the future)'
                        if st.negative_count else ''))
    lines.append('')
    lines.append('  FRAMES observed')
    lines.append(f'      /odom_raw  frame_id={frames.odom_raw_frame!r} '
                 f'child={frames.odom_raw_child!r}')
    lines.append(f'      /odom      frame_id={frames.odom_frame!r} '
                 f'child={frames.odom_child!r}')
    lines.append(f'      /scan      frame_id={frames.scan_frame!r}')
    findings = frames.findings()
    if findings:
        lines.append('')
        lines.append('  FRAME FINDINGS (reported, not corrected)')
        for f in findings:
            lines.append(f'      * {f}')
    if dup_findings:
        lines.append('')
        lines.append('  DUPLICATE PUBLISHERS')
        for f in dup_findings:
            lines.append(f'      * {f}')
    if tf_age:
        lines.append('')
        lines.append(f'  {tf_age}')
    lines.append('')
    lines.append('  THE GATE  exact-time odom -> laser_frame at each scan stamp')
    lines.append(f'      window          {gate.window_s:.1f} s '
                 f'({gate.warmup_discarded} warm-up scans discarded)')
    lines.append(f'      scans evaluated {gate.attempted}')
    lines.append(f'      resolved        {gate.succeeded}  '
                 f'({gate.success_pct:.2f}%, gate {gate.threshold_pct:.0f}%)')
    lines.append(f'      longest failure run {gate.longest_failure_streak} scans '
                 f'(limit {gate.max_streak})')
    if gate.failures:
        lines.append('      first failures:')
        for t_rel, reason in gate.failures:
            lines.append(f'        t+{t_rel:7.3f}s  {reason}')
    lines.append('')
    if gate.passed:
        lines.append('  PASS: scans can be transformed at their own timestamps.')
        lines.append('  This clears the TF/timing precondition ONLY. It is not a SLAM '
                     'result and not a map.')
    else:
        lines.append('  FAIL:')
        for r in gate.reasons():
            lines.append(f'      - {r}')
        lines.append('  Do not tune slam_toolbox.yaml against this. Scans that cannot '
                     'be transformed')
        lines.append('  never reach the matcher, so no matcher parameter can change '
                     'the outcome.')
    return '\n'.join(lines), gate.passed


# --------------------------------------------------------------------- ROS shell
def run_live(args):
    """Subscribe, sample, and score. Everything ROS lives below this line."""
    import rclpy
    from rclpy.node import Node
    from rclpy.duration import Duration
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.time import Time
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import LaserScan
    import tf2_ros
    import threading

    rclpy.init()
    node = Node('slam_preflight')
    buf = tf2_ros.Buffer(cache_time=Duration(seconds=30.0))
    listener = tf2_ros.TransformListener(buf, node)          # noqa: F841

    t0 = time.time()
    arrivals = {'/scan': [], '/odom_raw': [], '/odom': []}
    stamped = {'/scan': [], '/odom_raw': [], '/odom': []}
    samples = []
    frames = FrameReport(configured_odom_frame=args.odom_frame,
                         configured_laser_frame=args.laser_frame)

    def stamp_seconds(header):
        return header.stamp.sec + header.stamp.nanosec * 1e-9

    def on_scan(msg):
        now = time.time()
        arrivals['/scan'].append(now)
        stamped['/scan'].append((now, stamp_seconds(msg.header)))
        if not frames.scan_frame:
            frames.scan_frame = msg.header.frame_id
        # THE question: can this scan be put into the odom frame at ITS OWN stamp?
        stamp = Time(seconds=msg.header.stamp.sec,
                     nanoseconds=msg.header.stamp.nanosec)
        try:
            ok = buf.can_transform(args.odom_frame, msg.header.frame_id, stamp)
            reason = '' if ok else 'can_transform returned False'
        except Exception as e:                      # tf2 raises several distinct types
            ok, reason = False, f'{type(e).__name__}: {e}'
        samples.append((now - t0, bool(ok), reason))

    def on_odom_raw(msg):
        now = time.time()
        arrivals['/odom_raw'].append(now)
        stamped['/odom_raw'].append((now, stamp_seconds(msg.header)))
        if not frames.odom_raw_frame:
            frames.odom_raw_frame = msg.header.frame_id
            frames.odom_raw_child = msg.child_frame_id

    def on_odom(msg):
        now = time.time()
        arrivals['/odom'].append(now)
        stamped['/odom'].append((now, stamp_seconds(msg.header)))
        if not frames.odom_frame:
            frames.odom_frame = msg.header.frame_id
            frames.odom_child = msg.child_frame_id

    node.create_subscription(LaserScan, '/scan', on_scan, qos_profile_sensor_data)
    node.create_subscription(Odometry, '/odom_raw', on_odom_raw,
                             qos_profile_sensor_data)
    node.create_subscription(Odometry, '/odom', on_odom, 10)

    ex = SingleThreadedExecutor()
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()

    print(f'listening {args.seconds:.0f} s on ROS_DOMAIN_ID='
          f'{os.environ.get("ROS_DOMAIN_ID", "0")} '
          f'(first {args.warmup:.0f} s are warm-up)...', flush=True)
    deadline = t0 + args.seconds
    while time.time() < deadline:
        time.sleep(2.0)
        el = time.time() - t0
        n = len(samples)
        good = sum(1 for _, ok, _ in samples if ok)
        print(f'  t+{el:5.1f}s  scans {n:5d}  resolved {good:5d}', flush=True)

    # Duplicate publishers, sampled once at the end when discovery is settled.
    pubs = {}
    for topic in ('/tf', '/odom', '/odom_raw', '/scan'):
        try:
            infos = node.get_publishers_info_by_topic(topic)
            pubs[topic] = [i.node_name for i in infos]
        except Exception:
            pubs[topic] = []

    # odom -> base_footprint cadence, from the buffer rather than the wire: this is the
    # edge slam_toolbox actually walks.
    tf_age = ''
    try:
        latest = buf.lookup_transform(args.odom_frame, args.base_frame, Time())
        age = time.time() - (latest.header.stamp.sec
                             + latest.header.stamp.nanosec * 1e-9)
        tf_age = (f'TF {args.odom_frame} -> {args.base_frame}: newest transform is '
                  f'{age * 1000:.0f} ms old at the end of the run')
    except Exception as e:
        tf_age = (f'TF {args.odom_frame} -> {args.base_frame}: LOOKUP FAILED '
                  f'({type(e).__name__}) -- the edge SLAM needs does not exist')

    gate = evaluate_tf_gate(samples, warmup_s=args.warmup,
                            threshold_pct=args.threshold, max_streak=args.max_streak)
    rates = [rate_stats(t, arrivals[t], CONTRACT_HZ.get(t, float('nan')))
             for t in ('/scan', '/odom_raw', '/odom')]
    lats = [latency_stats(t, stamped[t]) for t in ('/scan', '/odom_raw', '/odom')]
    dups = duplicate_findings(pubs)

    text, ok = format_report(gate, rates, lats, frames, dups, tf_age)
    print()
    print(text)

    if args.json:
        payload = {
            'generated': time.strftime('%Y-%m-%dT%H:%M:%S'),
            'domain': os.environ.get('ROS_DOMAIN_ID', '0'),
            'seconds': args.seconds,
            'gate': asdict(gate),
            'gate_passed': gate.passed,
            'gate_reasons': gate.reasons(),
            'rates': [asdict(r) for r in rates],
            'latency': [asdict(v) for v in lats],
            'frames': asdict(frames),
            'frame_findings': frames.findings(),
            'duplicate_findings': dups,
            'tf_edge': tf_age,
        }
        with open(args.json, 'w') as f:
            json.dump(payload, f, indent=2)
        print(f'\n  json: {args.json}')

    node.destroy_node()
    rclpy.shutdown()
    return 0 if ok else 1


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--seconds', type=float, default=DEFAULT_SECONDS)
    ap.add_argument('--warmup', type=float, default=DEFAULT_WARMUP_S,
                    help='discard this many seconds while the TF buffer fills')
    ap.add_argument('--threshold', type=float, default=GATE_SUCCESS_PCT,
                    help='minimum %% of scans resolvable at their own stamp')
    ap.add_argument('--max-streak', type=int, default=GATE_MAX_STREAK,
                    help='longest tolerated run of consecutive failures')
    ap.add_argument('--odom-frame', default=DEFAULT_ODOM_FRAME)
    ap.add_argument('--laser-frame', default=DEFAULT_LASER_FRAME)
    ap.add_argument('--base-frame', default=DEFAULT_BASE_FRAME)
    ap.add_argument('--json', default='', help='also write a machine-readable report')
    args = ap.parse_args()
    return run_live(args)


if __name__ == '__main__':
    sys.exit(main())
