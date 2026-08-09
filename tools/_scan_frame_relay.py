#!/usr/bin/env python3
"""Filter corrupted RTX lidar scans out of the Isaac backend. Spawned by sim_runner.

    python3 tools/_scan_frame_relay.py            # subscribes scan_isaac_raw -> /scan

WHAT WAS MEASURED [2026-08-09, bare isaac sessions, consecutive-scan cross-correlation
against /sim/ground_truth]: the raw stream is a MIXTURE. Most messages are correct
sensor-frame revolutions -- static pairs match at shift 0 (48/93) or +/-1, and under a
true 0.42 rad/s turn the per-scan shift sits at -2/-3 deg, which is exactly -d(yaw) --
but a large minority arrive with their content circularly rotated by ~+/-85 deg with
rms-after-best-shift ~1.4 m against their neighbours: revolutions assembled with a
wrong sweep-phase origin. The RTX helper emits ~1.2 messages per render (the measured,
unexplained 72-per-render-second constant), and the phase seam lands inside some
messages. SLAM ingests every scan at full confidence, so a quarter-turn-rotated scan
every few messages smears the map on the first turn -- the reported symptom.

Two wrong fixes were tried and measured out before this one:
  * "the content is world-locked; counter-rotate by true yaw" -- refuted: good scans
    are already sensor-framed (spawn scan matched an arena raycast at shift +1,
    rms 0.28 m), so rotating everything by -yaw corrupts the majority to fix nothing.
  * "the content is mirrored (CW indexing)" -- refuted by the same raycast comparison
    (mirrored fit was strictly worse).

THE FIX: validate, never mutate. Each raw scan is compared against a raycast of the
shared arena (yahboomcar_sim.arena -- the SAME single-source geometry the USD is built
from) at the ground-truth pose; scans within threshold pass through UNTOUCHED, the
phase-corrupted ones are dropped. Dropping lowers the delivered rate, and sim_runner's
closed-loop rate trim (which measures the REAL /scan downstream of this filter) renders
faster to hold the 12 Hz contract.

FAIL-OPEN, LOUDLY: if most scans fail validation (wrong/rebuilt arena, robot outside
the room, geometry drift), filtering disables itself and passes everything through
with a warning -- a sim session with unfiltered scans beats a sim session with no
lidar at all, and the /map evidence gates catch the rest.

Runs in SYSTEM python: rclpy cannot be imported into Isaac's 3.11 interpreter (ABI,
not path), same as _scan_rate_probe.py. Uses /sim/ground_truth, which is
backend-internal truth: this process is part of the simulated firmware, using the
simulator's own state to emit its own sensor honestly. The stack sees only /scan.
"""
import math
import os
import sys
import threading

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _layout import REPO                                       # noqa: E402
sys.path.insert(0, os.path.join(REPO, 'yahboomcar_sim'))
from yahboomcar_sim.arena import default_arena, raycast        # noqa: E402

RAW_TOPIC = 'scan_isaac_raw'
OUT_TOPIC = 'scan'
# A good scan sits ~0.25 m rms from the arena raycast (USD wall thickness and box
# faces differ slightly from the segment model); a phase-corrupted one sits ~1.4 m.
# The distribution is bimodal (p10 0.18 / p90 1.43 measured over a mixture session),
# so the gate sits in the empty middle.
RMS_GATE = 0.6
# The RTX pipeline emits -1.0 as a no-return sentinel; a few appear even in good
# scans. They are excluded from the rms, and a scan that is MOSTLY sentinel is junk
# on its own.
MIN_VALID_BEAMS = 200
# Corrupted scans arrive in BURSTS (36 consecutive was measured), so fail-open must
# not trip on a burst: it exists for the case where the room geometry itself is wrong
# (rebuilt USD, robot outside), which fails ~100% of scans, not the ~50% of a bad
# mixture session. Hence: nearly-total failure over a long window.
FAIL_OPEN_FRACTION = 0.9
FAIL_OPEN_WINDOW = 300


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class ScanFilter(Node):
    def __init__(self):
        super().__init__('scan_frame_relay')
        self._lock = threading.Lock()
        self._pose = None                   # (x, y, yaw), newest truth
        self._segs = default_arena()
        self._recent = []                   # last FAIL_OPEN_WINDOW pass/fail bools
        self._fail_open = False
        self._dropped = 0
        self._passed = 0
        self.pub = self.create_publisher(LaserScan, OUT_TOPIC,
                                         qos_profile_sensor_data)
        self.create_subscription(Odometry, '/sim/ground_truth', self._on_truth,
                                 qos_profile_sensor_data)
        self.create_subscription(LaserScan, RAW_TOPIC, self._on_scan,
                                 qos_profile_sensor_data)

    def _on_truth(self, m):
        with self._lock:
            self._pose = (m.pose.pose.position.x, m.pose.pose.position.y,
                          yaw_of(m.pose.pose.orientation))

    def _rms_vs_arena(self, ranges, pose):
        expect = raycast((pose[0], pose[1]), pose[2], self._segs)
        got = np.asarray(ranges, dtype=float)
        # -1.0 is the RTX no-return sentinel, not a range; treat it as invalid.
        valid = np.isfinite(got) & (got > 0.0)
        if valid.sum() < MIN_VALID_BEAMS:
            return float('inf')
        both = np.isfinite(expect) & valid
        if both.sum() < 90:
            return float('inf')
        return float(np.sqrt(np.mean((got[both] - expect[both]) ** 2)))

    def _on_scan(self, m):
        with self._lock:
            pose = self._pose
        if pose is None:
            return                          # no truth yet (first ~100 ms)
        if not self._fail_open:
            ok = self._rms_vs_arena(m.ranges, pose) < RMS_GATE
            self._recent.append(ok)
            del self._recent[:-FAIL_OPEN_WINDOW]
            if (len(self._recent) == FAIL_OPEN_WINDOW and
                    self._recent.count(False) > FAIL_OPEN_FRACTION * FAIL_OPEN_WINDOW):
                self._fail_open = True
                self.get_logger().error(
                    'most scans fail arena validation -- the room geometry does not '
                    'match yahboomcar_sim.arena (rebuilt USD? robot out of the room?). '
                    'FILTERING DISABLED; /scan is now unfiltered raw.')
            if not ok:
                self._dropped += 1
                if self._dropped in (1, 10) or self._dropped % 200 == 0:
                    self.get_logger().info(
                        f'dropped {self._dropped} phase-corrupted scans '
                        f'({self._passed} passed)')
                return
        self._passed += 1
        self.pub.publish(m)


def main():
    rclpy.init()
    node = ScanFilter()
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    ex.spin()


if __name__ == '__main__':
    main()
