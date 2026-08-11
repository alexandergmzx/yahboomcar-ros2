"""ROS-free core for the SLAM lens (tools/slam_lens.py): metrics + encodings.

Kept separate from the server for the same reason fleet-console keeps
`summarize.py` apart from `ros_bridge.py`: the reduction/metric policy is
testable with stubs and duck-typed arrays, the transport is not. Every
function here is exercised by tools/tests/test_slam_lens_core.py without
rclpy in the room.

THE FOUR METRICS, and what each convicts:

  scan-to-map fit     fraction of valid scan endpoints landing within one
                      cell of an occupied map cell, at the TF-resolved pose.
                      This is Hector SLAM's alignment score, normalized —
                      the standard "does the sensor agree with the map"
                      number. Low fit while driving = the matcher is being
                      dragged (bad prior) or fed stale/garbage scans.
  pose divergence     TF pose (map->base) vs the simulator's ground truth,
                      aligned once at start. Grows = accumulated pose error
                      that SLAM did not absorb.
  yaw-rate ratio      /odom_raw wz vs ground-truth wz, median over a turning
                      window. The Isaac encoder lie (measured ~2.9x under
                      turn slip) shows here LIVE instead of post-mortem.
  scan staleness      consecutive bit-identical `ranges` arrays. The Isaac
                      render-pacing runaway (render 3.4/s vs publish 12.8 Hz,
                      simctl-isaac.log 2026-08-09) makes ~3-4 published scans
                      share one rendered frame; a healthy backend never
                      repeats a scan bit-for-bit (0/614 measured at 10
                      renders/s).

Occupancy values follow the map_saver trinary convention: -1 unknown,
0 free, 100 occupied; anything >= OCCUPIED_THRESHOLD counts as occupied.
"""
from __future__ import annotations

import math
from collections import deque

import numpy as np

OCCUPIED_THRESHOLD = 65        # int8 occupancy >= this counts as a wall
FIT_TOLERANCE_CELLS = 1        # Chebyshev dilation applied to the wall mask
TURN_RATE_FLOOR = 0.15         # rad/s of |truth wz| below which ratio samples are noise
PAIR_MAX_DT = 0.15             # s, max timestamp gap when pairing odom/truth samples


def wrap_angle(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


# ------------------------------------------------------------------ SE(2)

def se2_mul(a, b):
    """Compose two (x, y, yaw) poses: a then b."""
    ax, ay, ath = a
    bx, by, bth = b
    c, s = math.cos(ath), math.sin(ath)
    return (ax + c * bx - s * by, ay + s * bx + c * by, wrap_angle(ath + bth))


def se2_inv(p):
    x, y, th = p
    c, s = math.cos(th), math.sin(th)
    return (-c * x - s * y, s * x - c * y, wrap_angle(-th))


# ------------------------------------------------- occupancy grid handling

def rle_encode(data) -> list:
    """Run-length encode int8 occupancy data -> flat [value, count, ...].

    A 200x200 grid is 40k cells; walls-and-unknown maps run-length encode to
    a few hundred pairs, which is what makes shipping the map over JSON at
    every update a non-event instead of a design problem.
    """
    a = np.asarray(data, dtype=np.int8)
    if a.size == 0:
        return []
    change = np.flatnonzero(a[1:] != a[:-1]) + 1
    starts = np.concatenate(([0], change))
    ends = np.concatenate((change, [a.size]))
    out = []
    for s, e in zip(starts, ends):
        out.append(int(a[s]))
        out.append(int(e - s))
    return out


def rle_decode(rle: list) -> np.ndarray:
    if not rle:
        return np.zeros(0, dtype=np.int8)
    vals = np.asarray(rle[0::2], dtype=np.int8)
    counts = np.asarray(rle[1::2], dtype=np.int64)
    return np.repeat(vals, counts)


def occupied_mask_dilated(grid: np.ndarray,
                          threshold: int = OCCUPIED_THRESHOLD,
                          cells: int = FIT_TOLERANCE_CELLS) -> np.ndarray:
    """Boolean (h, w) mask of occupied cells, dilated by `cells` (Chebyshev).

    numpy-roll dilation, no scipy: 8 shifted ORs per ring is trivial at these
    grid sizes and keeps the tool importable everywhere the fleet env is.
    """
    mask = grid >= threshold
    out = mask.copy()
    for _ in range(cells):
        m = out.copy()
        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):
                if dx == 0 and dy == 0:
                    continue
                shifted = np.roll(np.roll(m, dy, axis=0), dx, axis=1)
                # np.roll wraps; zero the wrapped edge so a wall on one border
                # cannot "dilate" onto the opposite border.
                if dy == 1:
                    shifted[0, :] = False
                elif dy == -1:
                    shifted[-1, :] = False
                if dx == 1:
                    shifted[:, 0] = False
                elif dx == -1:
                    shifted[:, -1] = False
                out |= shifted
    return out


# --------------------------------------------------------------- the scan

def scan_endpoints(ranges, angle_min: float, angle_increment: float,
                   range_min: float, range_max: float) -> np.ndarray:
    """Valid scan endpoints in the LASER frame -> (n, 2) float array.

    Validity: finite, and strictly inside [range_min, range_max]. This
    excludes inf/nan AND the Isaac RTX -1.0 no-return sentinels (finite,
    below range_min — poison to any consumer that treats them as ranges;
    see docs/slam-research/isaac-scan-quality.md).
    """
    r = np.asarray(ranges, dtype=np.float64)
    n = r.size
    angles = angle_min + angle_increment * np.arange(n)
    ok = np.isfinite(r) & (r >= range_min) & (r <= range_max)
    return np.column_stack((r[ok] * np.cos(angles[ok]),
                            r[ok] * np.sin(angles[ok])))


def transform_points(points: np.ndarray, pose) -> np.ndarray:
    """Apply an (x, y, yaw) pose to (n, 2) points."""
    if points.size == 0:
        return points.reshape(0, 2)
    x, y, th = pose
    c, s = math.cos(th), math.sin(th)
    rot = np.array([[c, -s], [s, c]])
    return points @ rot.T + np.array([x, y])


def scan_map_fit(points_map: np.ndarray, dilated_mask: np.ndarray,
                 resolution: float, origin_x: float, origin_y: float):
    """Hit fraction of map-frame endpoints against the dilated wall mask.

    -> (fit or None, hits bool array). None when there is nothing to score
    (no valid points, or no occupied cell in the map yet) — an empty map
    must read as "no evidence", never as fit 0.0, or the first minute of
    every session would scream red.
    """
    n = points_map.shape[0]
    hits = np.zeros(n, dtype=bool)
    if n == 0 or not dilated_mask.any():
        return None, hits
    cols = np.floor((points_map[:, 0] - origin_x) / resolution).astype(np.int64)
    rows = np.floor((points_map[:, 1] - origin_y) / resolution).astype(np.int64)
    h, w = dilated_mask.shape
    inb = (cols >= 0) & (cols < w) & (rows >= 0) & (rows < h)
    hits[inb] = dilated_mask[rows[inb], cols[inb]]
    return float(hits.sum()) / float(n), hits


# ------------------------------------------------------------ the trackers

class StalenessTracker:
    """Consecutive bit-identical scans. Feed a hashable digest per scan."""

    def __init__(self):
        self._last = None
        self.current_run = 0        # 0 = the latest scan differed from its predecessor
        self.max_run = 0
        self.duplicates = 0
        self.total = 0

    def feed(self, digest) -> None:
        self.total += 1
        if digest == self._last:
            self.current_run += 1
            self.duplicates += 1
            self.max_run = max(self.max_run, self.current_run)
        else:
            self.current_run = 0
        self._last = digest

    @property
    def duplicate_fraction(self) -> float:
        return self.duplicates / self.total if self.total else 0.0


class YawRatioWindow:
    """Median |odom wz| / |truth wz| over the recent turning samples.

    Truth wz is finite-differenced from unwrapped truth yaw (the 2D backend
    publishes no truth twist worth trusting; the pose is authoritative).
    Samples pair nearest-in-time within PAIR_MAX_DT and only count when the
    body is actually turning (|truth wz| > TURN_RATE_FLOOR) — elevated/static
    ratio samples are 0/0 noise, the same rule check_odom_vs_imu.py applies.
    """

    def __init__(self, window_s: float = 20.0):
        self.window_s = window_s
        self._odom = deque()        # (t, wz)
        self._truth = deque()       # (t, wz) finite-differenced
        self._last_truth = None     # (t, yaw)

    def feed_odom(self, t: float, wz: float) -> None:
        self._odom.append((t, wz))
        self._trim(t)

    def feed_truth_yaw(self, t: float, yaw: float) -> None:
        if self._last_truth is not None:
            t0, y0 = self._last_truth
            dt = t - t0
            if 1e-3 < dt < 1.0:
                self._truth.append((t, wrap_angle(yaw - y0) / dt))
        self._last_truth = (t, yaw)
        self._trim(t)

    def _trim(self, now: float) -> None:
        for q in (self._odom, self._truth):
            while q and now - q[0][0] > self.window_s:
                q.popleft()

    def ratio(self):
        """-> (median ratio or None, n turning samples)."""
        if not self._odom or not self._truth:
            return None, 0
        truth = list(self._truth)
        ratios = []
        j = 0
        for t, wz in self._odom:
            while j + 1 < len(truth) and abs(truth[j + 1][0] - t) < abs(truth[j][0] - t):
                j += 1
            tt, twz = truth[j]
            if abs(tt - t) > PAIR_MAX_DT or abs(twz) < TURN_RATE_FLOOR:
                continue
            ratios.append(abs(wz) / abs(twz))
        if not ratios:
            return None, 0
        return float(np.median(ratios)), len(ratios)


class PoseAligner:
    """Anchor the truth frame onto the map frame, once, at first sight.

    The simulators publish ground truth in their own world frame ('map' for
    the 2D backend, 'world' for Isaac) which is NOT slam_toolbox's map frame
    — they merely start out coincident when the robot spawns at the world
    origin. Capturing T_align = T_map_base * inv(T_world_base) at the first
    simultaneous sample makes the ghost exact at t0 and turns every later
    separation into exactly the quantity under diagnosis: pose error accrued
    since the lens started watching. That is an honest lower bound on total
    error, and the page says "since lens start" for that reason.
    """

    def __init__(self):
        self.t_align = None

    def feed(self, map_base_pose, truth_pose) -> None:
        if self.t_align is None:
            self.t_align = se2_mul(map_base_pose, se2_inv(truth_pose))

    def truth_in_map(self, truth_pose):
        if self.t_align is None:
            return None
        return se2_mul(self.t_align, truth_pose)


def divergence(map_base_pose, truth_in_map_pose):
    """-> (position error m, |yaw error| rad) between TF pose and truth ghost."""
    dx = map_base_pose[0] - truth_in_map_pose[0]
    dy = map_base_pose[1] - truth_in_map_pose[1]
    dyaw = abs(wrap_angle(map_base_pose[2] - truth_in_map_pose[2]))
    return math.hypot(dx, dy), dyaw


# ------------------------------------------------------- content lag (sim)

class TruthHistory:
    """Recent ground-truth poses, interpolatable at any time in the window.

    Feeds the content-lag metric. Duplicate scans cannot establish content age
    on Isaac (0/3330 bit-identical at 2.92 renders/s), so this time-offset fit
    asks whether scan content matches an earlier truth pose. The first live-bag
    analysis overstated lag because it included a static robot; guarded
    re-analysis found median -0.04 s and only 1/77 and 2/78 moving samples at
    >=0.2 s across the two runs. That incident is why both history bounds and
    the static-motion refusal below are part of the metric contract.
    """

    def __init__(self, window_s: float = 15.0):
        self.window_s = window_s
        self._q = deque()          # (t, x, y, yaw)

    def feed(self, t: float, pose) -> None:
        self._q.append((t, pose[0], pose[1], pose[2]))
        while self._q and t - self._q[0][0] > self.window_s:
            self._q.popleft()

    def pose_at(self, t: float):
        """Linear interpolation (yaw shortest-arc). None outside the window."""
        q = self._q
        if not q or t < q[0][0] - 0.05 or t > q[-1][0] + 0.05:
            return None
        lo, hi = 0, len(q) - 1
        while lo < hi:
            mid = (lo + hi) // 2
            if q[mid][0] < t:
                lo = mid + 1
            else:
                hi = mid
        b = q[lo]
        a = q[lo - 1] if lo > 0 else b
        dt = b[0] - a[0]
        w = 0.0 if dt <= 1e-9 else min(1.0, max(0.0, (t - a[0]) / dt))
        dyaw = wrap_angle(b[3] - a[3])
        return (a[1] + w * (b[1] - a[1]), a[2] + w * (b[2] - a[2]),
                wrap_angle(a[3] + w * dyaw))


DEFAULT_LAG_OFFSETS = tuple(round(-0.6 + 0.05 * i, 2) for i in range(15))  # -0.6..0.1


def content_lag(ranges, range_min, range_max, pose_at, raycast_at,
                stamp, offsets=DEFAULT_LAG_OFFSETS, min_beams=80,
                wall_slack=0.45):
    """Best time offset explaining the scan against a walls model.

    -> (best_offset_s, rms_at_best_m) or None when unanswerable (no truth
    pose in window, too few scoreable beams, or A STATIC ROBOT). `raycast_at
    (pose) -> expected ranges array` is injected so this stays importable
    without the arena (and testable against synthetic rooms). Beams far
    SHORT of the wall (movable boxes, wherever they are today) are excluded
    per offset via `wall_slack`, the same walls-only reasoning as the relay.

    THE STATIC GUARD IS NOT OPTIONAL: when the pose barely changes across
    the offset window, every offset fits equally and argmin returns noise —
    observed live 2026-08-10 (post-patrol static robot read "80% stale,
    median −0.45 s", which was pure sweep degeneracy, not staleness). If
    the pose moved less than ~half a map cell / a degree-ish across the
    whole window, the question has no answer and None is the honest one.
    """
    p_lo = pose_at(stamp + offsets[0])
    p_hi = pose_at(stamp + offsets[-1])
    if p_lo is None or p_hi is None:
        return None
    moved = math.hypot(p_hi[0] - p_lo[0], p_hi[1] - p_lo[1])
    turned = abs(wrap_angle(p_hi[2] - p_lo[2]))
    if moved < 0.02 and turned < 0.02:
        return None
    r = np.asarray(ranges, dtype=np.float64)
    best = None
    for off in offsets:
        pose = pose_at(stamp + off)
        if pose is None:
            continue
        exp = np.asarray(raycast_at(pose), dtype=np.float64)
        ok = (np.isfinite(r) & (r > range_min) & (r <= range_max)
              & np.isfinite(exp) & (r >= exp - wall_slack))
        if int(ok.sum()) < min_beams:
            continue
        d = r[ok] - exp[ok]
        rms = float(np.sqrt(np.mean(d * d)))
        if best is None or rms < best[1]:
            best = (float(off), rms)
    return best
