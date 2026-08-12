#!/usr/bin/env python3
"""Filter corrupted RTX lidar scans out of the Isaac backend. Spawned by sim_runner.

    python3 tools/_scan_frame_relay.py            # subscribes scan_isaac_raw -> /scan

WHAT WAS MEASURED [2026-08-09, bare isaac sessions, consecutive-scan cross-correlation
against /sim/ground_truth]: the raw stream is a MIXTURE. Most messages are correct
sensor-frame revolutions -- static pairs match at shift 0, and under a true 0.42 rad/s
turn the per-scan shift sits at -2/-3 deg, exactly -d(yaw) -- but a large minority
arrive with their content circularly rotated ~+/-85 deg in BURSTS (36 consecutive
measured): revolutions assembled with a wrong sweep-phase origin, plausibly at the
~1.2-messages-per-render emission seam. SLAM ingests every scan at full confidence,
so the corrupted ones smear the map on the first turn. Session-nondeterministic:
back-to-back boots measured ~50% corrupted, then 100% clean. Hence a per-scan filter.

WHY THE METRIC IS WALLS-ONLY AND ONE-SIDED [2026-08-10, the fun+patrol lesson]: the
first version validated against the full arena raycast -- walls AND boxes at their
canonical spots -- by rms. The boxes are MOVABLE BY DESIGN, and fun mode exists to
punt them: as soon as a patrol shoved the feather boxes, good scans stopped matching
the static model and were dropped in bursts, which stuttered the scan display,
tripped the governor's stale-scan stops, starved SLAM, and sent the rate-feedback
loop chasing the losses. The walls, by contrast, never move -- and geometry gives a
one-sided test that box positions cannot touch: a real return may come back SHORTER
than the wall distance at its azimuth (a box, wherever it currently is) but never
LONGER, because that is seeing through a wall. Phase-rotated scans mislocate the
walls themselves, so a large fraction of their beams instantly violate the bound;
a displaced box can never violate it.

FAIL-OPEN, LOUDLY: if ~all scans fail over a long window, the room geometry itself
does not match yahboomcar_sim.arena (rebuilt USD, robot outside a wall, different
--size) -- filtering disables itself with an error rather than silently starving
/scan. Bursts (36 measured) and mixtures (~50%) can never trip it.

Runs in SYSTEM python: rclpy cannot be imported into Isaac's 3.11 interpreter (ABI,
not path), same as _scan_rate_probe.py. Uses /sim/ground_truth, which is
backend-internal truth: this process is part of the simulated firmware, using the
simulator's own state to emit its own sensor honestly. The stack sees only /scan.
"""
import math
import json
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
from yahboomcar_sim.arena import raycast, segments_room        # noqa: E402

RAW_TOPIC = 'scan_isaac_raw'
OUT_TOPIC = 'scan'
# One-sided tolerance past the wall. Clean scans sit ~0.02-0.03 m from the raycast
# (measured), so 0.25 m is ~10 sigma; the corrupted population mislocates walls by
# ~1.4 m rms. Beams the walls-only model cannot explain UNDER the bound (boxes) are
# legitimate and ignored.
BEYOND_WALL_TOL_M = 0.25
# Fraction of valid beams allowed beyond the wall before the scan is corrupt. A
# phase-rotated scan violates on a large share of its beams at once; clean scans
# essentially never do.
IMPOSSIBLE_GATE = 0.10
# The RTX pipeline emits -1.0 as a no-return sentinel; a few appear even in good
# scans. Excluded from the test, and a scan that is MOSTLY sentinel is junk alone.
#
# 200 of 360 is a CLOSED ROOM's number. In the 4 x 4 m test arena every beam
# finds a wall inside range; in an open scene most of a revolution can see
# nothing at all. Measured on the corridor over 5293 scans: median 175 valid
# beams, mean 181, min 72 -- so this gate alone rejects 64.3% of perfectly good
# scans, before any geometry is considered. That, not the wall model, is what
# kept the filter failing open there after it was given the right walls: replayed
# against the corridor's own geometry only 1.1% of those scans are actually
# impossible.
#
# Overridable for the same reason the wall model is, and defaulting to 200 so
# every existing fleet caller is unchanged.
MIN_VALID_BEAMS = int(os.environ.get('SCAN_RELAY_MIN_VALID_BEAMS', '200'))
# Fail-open only on near-total failure over a long window: geometry mismatch fails
# ~100%; corruption bursts (36 measured) and bad mixtures (~50%) never reach this.
FAIL_OPEN_FRACTION = 0.9
FAIL_OPEN_WINDOW = 300

# WHICH ROOM. `segments_room()` is the stock 4 x 4 m test arena, and it is still
# the default -- every fleet caller keeps exactly the behaviour it had.
#
# It is not the only arena any more. A caller that loads a different world can
# name a JSON file of wall segments here, in the same [[[x1,y1],[x2,y2]], ...]
# form `raycast` already takes, and the corruption filter then works there too.
#
# WITHOUT it, on any other arena, this node is not a filter. Measured on the
# corridor scenario across 56 of 62 Isaac sessions: essentially every beam
# "sees through the wall" of a room that is not the room, so /scan publishes
# NOTHING for the ~21 s it takes to fill the fail-open window, and then passes
# raw scans -- including the phase-corrupted revolutions this node exists to
# drop -- to slam_toolbox, both costmaps and the governor for the rest of the
# run. Silent in both halves: the blackout looks like a slow twin, and the
# passthrough looks like a working filter.
WALLS_ENV = 'SCAN_RELAY_WALLS_JSON'


def load_walls():
    """The wall model to validate scans against: the stock room, or a caller's.

    Fail CLOSED on a bad path, deliberately. A missing or malformed file means
    the caller believes it supplied geometry and did not, and falling back to
    the 4 x 4 m room there would reproduce exactly the silent-blackout failure
    this exists to end.
    """

    path = os.environ.get(WALLS_ENV)
    if not path:
        return segments_room(), f'stock {segments_room.__module__} room'
    with open(path) as handle:
        raw = json.load(handle)
    walls = [((float(a[0]), float(a[1])), ((float(b[0]), float(b[1])))) for a, b in raw]
    if not walls:
        raise ValueError(f'{path} carries no wall segments')
    return walls, f'{len(walls)} segments from {path}'


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def impossible_fraction(ranges, pose, walls):
    """Fraction of valid beams that return LONGER than the wall at their azimuth.

    -> (fraction, n_valid). Boxes anywhere only ever SHORTEN returns, so this is
    immune to the movable-box layout; only mislocated geometry (phase corruption)
    or a wrong room can raise it.
    """
    expect = raycast((pose[0], pose[1]), pose[2], walls)
    got = np.asarray(ranges, dtype=float)
    valid = np.isfinite(got) & (got > 0.0)
    n_valid = int(valid.sum())
    both = np.isfinite(expect) & valid
    n_both = int(both.sum())
    if n_both < 90:
        return 1.0, n_valid
    frac = float(np.mean(got[both] > expect[both] + BEYOND_WALL_TOL_M))
    return frac, n_valid


class ScanFilter(Node):
    def __init__(self):
        super().__init__('scan_frame_relay')
        self._lock = threading.Lock()
        self._pose = None                   # (x, y, yaw), newest truth
        self._walls, source = load_walls()
        self.get_logger().info(f'wall model: {source}')
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

    def _on_scan(self, m):
        with self._lock:
            pose = self._pose
        if pose is None:
            return                          # no truth yet (first ~100 ms)
        if not self._fail_open:
            frac, n_valid = impossible_fraction(m.ranges, pose, self._walls)
            ok = n_valid >= MIN_VALID_BEAMS and frac <= IMPOSSIBLE_GATE
            self._recent.append(ok)
            del self._recent[:-FAIL_OPEN_WINDOW]
            if (len(self._recent) == FAIL_OPEN_WINDOW and
                    self._recent.count(False) > FAIL_OPEN_FRACTION * FAIL_OPEN_WINDOW):
                self._fail_open = True
                self.get_logger().error(
                    'nearly all scans see through the walls of the room model -- the '
                    'geometry does not match yahboomcar_sim.arena (rebuilt USD? robot '
                    'outside the room? different --size?). FILTERING DISABLED; /scan '
                    'is now unfiltered raw.')
            if not ok:
                self._dropped += 1
                if self._dropped in (1, 10) or self._dropped % 200 == 0:
                    self.get_logger().info(
                        f'dropped {self._dropped} corrupted scans '
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
