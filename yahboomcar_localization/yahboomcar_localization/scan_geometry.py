"""Turning a LaserScan into geometry: points, and the surface normals at them.

ROS-free on purpose, same split as yahboomcar_safety/governor.py, so every claim about
the geometry can be tested against synthetic scans with known answers rather than against
a robot.

NO SCIPY. scipy's compiled submodules (spatial, optimize, linalg, stats) are all broken
on this machine: a pip-installed numpy 2.2.1 in ~/.local shadows apt's 1.26.4, and apt's
scipy 1.11.4 is built against the numpy 1.x ABI, so importing any of them raises
"numpy.dtype size changed". A 2D scan is a few hundred points, which is small enough that
plain numpy is fast enough, so this file depends on nothing but numpy. That is a feature
rather than a workaround -- one fewer undocumented surface in an estimator.
"""
import math

import numpy as np


def scan_to_xy(ranges, angle_min, angle_increment, angle_offset=0.0,
               range_min=0.0, range_max=float('inf')):
    """LaserScan ranges -> (N, 2) array of points in the scanner frame.

    Invalid returns (NaN, inf, or outside [range_min, range_max]) are DROPPED, not
    clamped and not emitted at the origin. A point at the origin reads as an obstacle
    sitting on top of the robot, and to a scan matcher it reads as a feature that never
    moves -- which would quietly anchor the estimate to nothing.
    """
    r = np.asarray(ranges, dtype=float)
    if r.size == 0:
        return np.zeros((0, 2))
    ang = angle_min + angle_offset + np.arange(r.size) * angle_increment
    ok = np.isfinite(r) & (r >= range_min) & (r <= range_max)
    r, ang = r[ok], ang[ok]
    return np.column_stack((r * np.cos(ang), r * np.sin(ang)))


def estimate_normals(xy, k=6, max_neighbour_dist=0.20):
    """Surface normal at each point, from a local line fit. Returns (N, 2), unit length.

    Normals are what make degeneracy visible: a scan matcher can only resist motion
    ALONG a surface normal, so the set of normals in a scan is exactly the set of
    directions the geometry constrains. See scan_matcher.degeneracy().

    Neighbours are taken in INDEX order, which is valid because a lidar scan is already
    sorted by angle, so index-adjacent points are angle-adjacent. That makes this O(N*k)
    instead of O(N^2) and needs no spatial index. `max_neighbour_dist` guards the case
    index adjacency does not handle: a range discontinuity, where consecutive samples
    land on surfaces metres apart and a line fit across the jump would invent a normal
    that describes neither surface.

    Points without enough close neighbours get a zero normal, and callers must treat that
    as "no constraint from this point" rather than as a direction.
    """
    xy = np.asarray(xy, dtype=float)
    n = len(xy)
    normals = np.zeros((n, 2))
    if n < 3:
        return normals
    half = max(1, k // 2)
    for i in range(n):
        lo, hi = max(0, i - half), min(n, i + half + 1)
        nb = xy[lo:hi]
        d = np.linalg.norm(nb - xy[i], axis=1)
        nb = nb[d <= max_neighbour_dist]
        if len(nb) < 3:
            continue
        c = nb - nb.mean(axis=0)
        # 2x2 covariance: the eigenvector of the SMALLER eigenvalue is the direction of
        # least spread, i.e. perpendicular to the fitted line.
        cov = c.T @ c
        w, v = np.linalg.eigh(cov)
        normals[i] = v[:, 0]
    # Sign is irrelevant everywhere it is used: degeneracy works with n n^T, which is
    # invariant to flipping n, and point-to-plane residuals are signed consistently by
    # the correspondence. Not normalising the direction avoids inventing an orientation
    # the local fit cannot actually determine.
    norm = np.linalg.norm(normals, axis=1, keepdims=True)
    with np.errstate(invalid='ignore', divide='ignore'):
        normals = np.where(norm > 0, normals / norm, 0.0)
    return normals


def transform_xy(xy, dx, dy, dtheta):
    """Apply a 2D rigid transform to a point set."""
    xy = np.asarray(xy, dtype=float)
    if xy.size == 0:
        return xy.reshape(0, 2)
    c, s = math.cos(dtheta), math.sin(dtheta)
    R = np.array([[c, -s], [s, c]])
    return xy @ R.T + np.array([dx, dy])


def compose(a, b):
    """Compose two (dx, dy, dtheta) transforms: apply `a`, then `b`."""
    ax, ay, at = a
    bx, by, bt = b
    c, s = math.cos(bt), math.sin(bt)
    return (bx + c * ax - s * ay, by + s * ax + c * ay, wrap(at + bt))


def wrap(a):
    """Angle to [-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))
