"""The test room as line segments, and a lidar that sees it.

ROS-free, so the geometry can be tested against known answers, and so
tools/arena_observability.py and the simulator describe the SAME room rather than two
rooms that drift apart.

Originally written inside tools/arena_observability.py to answer whether a bare 4x4 m room
is degenerate for scan matching. It is not -- median isotropy 0.914, because an 8 m lidar
sees all four walls from everywhere. That measurement lives in
docs/sensor-fusion-research.md; this module is the geometry it was made with.
"""
import math

import numpy as np

# The real device, measured from MicroROS-assets/bags/selftest-20260806-004826:
# 360 beams at 1.000 deg over a full circle, 0.12 to 8.00 m, ~355/360 returns valid.
LIDAR_BEAMS = 360
LIDAR_RANGE_MIN = 0.12
LIDAR_RANGE_MAX = 8.0

# The planned test arena. ONE room for both backends (2026-08-08): these constants
# are the single source of the box layout -- tools/build_arena.py imports them for the
# Isaac USD, so the two rooms cannot drift apart. (They had: 2D carried three 0.4 m
# boxes mid-room while Isaac had four 0.3 m boxes by the corners, and a box count that
# never matched what the SLAM map showed was how the drift was noticed.)
ROOM_W = 4.0
ROOM_H = 4.0
BOX_SIZE = 0.3
# Near the corners, kept clear of the 2x2 m UMBmark square so an obstacle never sits
# inside a calibration run. Deliberately NOT 4-fold symmetric (1.30 vs 1.45): a matcher
# relocalising into the wrong quadrant must not find the same room there.
ARENA_BOXES = ((1.45, 1.45), (-1.45, 1.30), (1.35, -1.40), (-1.30, -1.45))


def segments_room(w=ROOM_W, h=ROOM_H):
    hw, hh = w / 2, h / 2
    return [((-hw, -hh), (hw, -hh)), ((hw, -hh), (hw, hh)),
            ((hw, hh), (-hw, hh)), ((-hw, hh), (-hw, -hh))]


def segments_box(cx, cy, s=BOX_SIZE):
    h = s / 2
    c = [(cx - h, cy - h), (cx + h, cy - h), (cx + h, cy + h), (cx - h, cy + h)]
    return [(c[i], c[(i + 1) % 4]) for i in range(4)]


def default_arena(boxes=ARENA_BOXES):
    """The 4x4 room with the shared box layout in it.

    The boxes are NOT there to make translation observable -- that was the expectation and
    measurement disproved it. They are there because the safety governor needs something
    to stop for, and because they break the rotational symmetry of a square room, where a
    matcher that loses track can otherwise relocalise into the wrong quadrant with
    complete confidence.
    """
    segs = list(segments_room())
    for cx, cy in boxes:
        segs += segments_box(cx, cy)
    return segs


def raycast(origin, yaw, segs, n_beams=LIDAR_BEAMS,
            r_min=LIDAR_RANGE_MIN, r_max=LIDAR_RANGE_MAX):
    """Simulate one scan. Returns ranges in the SENSOR frame, ordered from angle_min.

    Beams that hit nothing within range come back as +inf, which is what a real driver
    reports and what scan_to_xy() already drops. Returning r_max instead would invent a
    wall at exactly 8 m all the way round, and a scan matcher would happily match against
    it.
    """
    ox, oy = origin
    out = np.full(n_beams, np.inf)
    step = 2 * math.pi / n_beams
    for i in range(n_beams):
        a = yaw - math.pi + i * step          # angle_min = -pi, as the real device
        dx, dy = math.cos(a), math.sin(a)
        best = math.inf
        for (x1, y1), (x2, y2) in segs:
            ex, ey = x2 - x1, y2 - y1
            den = dx * ey - dy * ex
            if abs(den) < 1e-12:
                continue
            t = ((x1 - ox) * ey - (y1 - oy) * ex) / den      # along the ray
            u = ((x1 - ox) * dy - (y1 - oy) * dx) / den      # along the segment
            if r_min < t < best and 0.0 <= u <= 1.0:
                best = t
        if best <= r_max:
            out[i] = best
    return out
