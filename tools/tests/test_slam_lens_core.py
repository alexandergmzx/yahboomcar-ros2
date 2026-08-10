"""Tests for the SLAM lens core. ROS-free, synthetic data only.

    python3 -m pytest tools/tests/test_slam_lens_core.py -q

The negative controls matter more than the happy paths: a lens whose fit
metric cannot see a shifted scan, or whose staleness counter reads zero on a
repeated scan, would "diagnose" every session as healthy — the exact failure
mode check_rviz_render's addendum documents for evidence tools.
"""
import math
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from _slam_lens_core import (                                   # noqa: E402
    PoseAligner, StalenessTracker, YawRatioWindow, divergence,
    occupied_mask_dilated, rle_decode, rle_encode, scan_endpoints, scan_map_fit,
    se2_inv, se2_mul, transform_points, wrap_angle)


# ------------------------------------------------------------------ RLE

def test_rle_roundtrip():
    data = np.array([-1] * 100 + [0] * 50 + [100] * 3 + [-1] * 7, dtype=np.int8)
    assert np.array_equal(rle_decode(rle_encode(data)), data)


def test_rle_empty_and_single():
    assert rle_encode([]) == []
    assert rle_decode([]).size == 0
    assert np.array_equal(rle_decode(rle_encode([42])), np.array([42], dtype=np.int8))


def test_rle_compresses_a_mostly_unknown_grid():
    grid = np.full(40000, -1, dtype=np.int8)
    grid[100:300] = 100
    assert len(rle_encode(grid)) <= 6


# ----------------------------------------------------------------- SE(2)

def test_se2_inverse_composes_to_identity():
    p = (1.5, -2.0, 0.7)
    x, y, th = se2_mul(p, se2_inv(p))
    assert abs(x) < 1e-12 and abs(y) < 1e-12 and abs(th) < 1e-12


def test_se2_mul_translates_along_heading():
    # Facing +y, stepping 1 m forward lands at +y.
    x, y, th = se2_mul((0.0, 0.0, math.pi / 2), (1.0, 0.0, 0.0))
    assert abs(x) < 1e-12 and abs(y - 1.0) < 1e-12


# ------------------------------------------------------------- endpoints

def test_scan_endpoints_excludes_sentinels_and_nonfinite():
    # Four beams at 0/90/180/270 deg; -1.0 is the Isaac RTX no-return
    # sentinel (finite, below range_min) and must be excluded, as must inf.
    pts = scan_endpoints([1.0, -1.0, float('inf'), 2.0],
                         angle_min=0.0, angle_increment=math.pi / 2,
                         range_min=0.12, range_max=8.0)
    assert pts.shape == (2, 2)
    assert np.allclose(pts[0], [1.0, 0.0], atol=1e-12)
    assert np.allclose(pts[1], [0.0, -2.0], atol=1e-12)


def test_transform_points_rotates_and_translates():
    pts = transform_points(np.array([[1.0, 0.0]]), (2.0, 3.0, math.pi / 2))
    assert np.allclose(pts, [[2.0, 4.0]], atol=1e-12)


# ------------------------------------------------------------------ fit

def _wall_grid():
    """20x20 grid, resolution 0.1, origin (0,0): a wall column at x=1.0 m."""
    grid = np.zeros((20, 20), dtype=np.int8)
    grid[:, 10] = 100
    return grid


def test_fit_perfect_on_the_wall():
    grid = _wall_grid()
    mask = occupied_mask_dilated(grid)
    pts = np.column_stack((np.full(10, 1.05), np.linspace(0.05, 1.95, 10)))
    fit, hits = scan_map_fit(pts, mask, 0.1, 0.0, 0.0)
    assert fit == 1.0 and hits.all()


def test_fit_low_when_scan_is_shifted():
    # The negative control: the same wall points shifted 0.5 m off the wall
    # (past the 1-cell dilation) must read as misses, not hits.
    grid = _wall_grid()
    mask = occupied_mask_dilated(grid)
    pts = np.column_stack((np.full(10, 1.55), np.linspace(0.05, 1.95, 10)))
    fit, hits = scan_map_fit(pts, mask, 0.1, 0.0, 0.0)
    assert fit == 0.0 and not hits.any()


def test_fit_none_on_empty_map_not_zero():
    # An empty map is "no evidence", never "fit 0.0" — otherwise the first
    # minute of every session reads as a failure.
    grid = np.full((20, 20), -1, dtype=np.int8)
    mask = occupied_mask_dilated(grid)
    fit, _ = scan_map_fit(np.array([[1.0, 1.0]]), mask, 0.1, 0.0, 0.0)
    assert fit is None


def test_fit_none_with_no_points():
    mask = occupied_mask_dilated(_wall_grid())
    fit, hits = scan_map_fit(np.zeros((0, 2)), mask, 0.1, 0.0, 0.0)
    assert fit is None and hits.size == 0


def test_fit_out_of_bounds_points_are_misses():
    mask = occupied_mask_dilated(_wall_grid())
    fit, _ = scan_map_fit(np.array([[50.0, 50.0], [-3.0, 0.1]]), mask, 0.1, 0.0, 0.0)
    assert fit == 0.0


def test_dilation_does_not_wrap_across_borders():
    # A wall on the left border must not dilate onto the right border —
    # np.roll wraps and the implementation must undo that.
    grid = np.zeros((5, 5), dtype=np.int8)
    grid[:, 0] = 100
    mask = occupied_mask_dilated(grid, cells=1)
    assert not mask[:, -1].any()
    assert mask[:, 1].all()


# ------------------------------------------------------------- staleness

def test_staleness_counts_runs_of_identical_scans():
    s = StalenessTracker()
    for d in ('a', 'a', 'a', 'b', 'b', 'c'):
        s.feed(d)
    assert s.duplicates == 3          # a,a + b
    assert s.max_run == 2             # a,a,a = run of 2 repeats
    assert s.current_run == 0         # c differed
    assert s.duplicate_fraction == pytest.approx(0.5)


def test_staleness_zero_on_all_distinct():
    s = StalenessTracker()
    for d in range(100):
        s.feed(d)
    assert s.duplicates == 0 and s.max_run == 0


# ------------------------------------------------------------- yaw ratio

def _feed_turn(win, odom_scale, seconds=10.0, dt=0.05, truth_rate=0.5):
    yaw = 0.0
    t = 0.0
    while t < seconds:
        yaw = wrap_angle(yaw + truth_rate * dt)
        win.feed_truth_yaw(t, yaw)
        win.feed_odom(t, truth_rate * odom_scale)
        t += dt


def test_yaw_ratio_reads_the_lie():
    # Odometry claiming 2.9x the true rate must read ~2.9 — this is the
    # measured Isaac encoder lie the metric exists to surface live.
    win = YawRatioWindow()
    _feed_turn(win, odom_scale=2.9)
    ratio, n = win.ratio()
    assert n > 50
    assert ratio == pytest.approx(2.9, rel=0.05)


def test_yaw_ratio_honest_reads_one():
    win = YawRatioWindow()
    _feed_turn(win, odom_scale=1.0)
    ratio, _ = win.ratio()
    assert ratio == pytest.approx(1.0, rel=0.05)


def test_yaw_ratio_none_when_not_turning():
    # Static robot: truth rate ~0, every sample below the turning floor.
    win = YawRatioWindow()
    for i in range(100):
        t = i * 0.05
        win.feed_truth_yaw(t, 0.001 * math.sin(t))
        win.feed_odom(t, 0.0)
    ratio, n = win.ratio()
    assert ratio is None and n == 0


def test_yaw_ratio_handles_wrap():
    # A sustained turn through +pi must not produce a 2pi/dt spike sample.
    win = YawRatioWindow(window_s=60.0)
    _feed_turn(win, odom_scale=1.0, seconds=30.0, truth_rate=0.8)
    ratio, _ = win.ratio()
    assert ratio == pytest.approx(1.0, rel=0.05)


# ------------------------------------------------------------- alignment

def test_aligner_ghost_exact_at_capture():
    al = PoseAligner()
    map_pose = (2.0, 1.0, 0.3)
    truth_pose = (7.0, -4.0, 1.2)     # arbitrary different world frame
    al.feed(map_pose, truth_pose)
    ghost = al.truth_in_map(truth_pose)
    assert np.allclose(ghost, map_pose, atol=1e-12)


def test_aligner_tracks_relative_motion():
    # After alignment, a 1 m forward step in the truth frame must appear as
    # a 1 m forward step of the ghost in the map frame.
    al = PoseAligner()
    al.feed((0.0, 0.0, math.pi / 2), (5.0, 5.0, 0.0))
    ghost = al.truth_in_map((6.0, 5.0, 0.0))    # truth stepped +1 x (its forward)
    d_pos, d_yaw = divergence((0.0, 1.0, math.pi / 2), ghost)
    assert d_pos < 1e-9 and d_yaw < 1e-9


def test_divergence_reports_separation():
    d_pos, d_yaw = divergence((1.0, 1.0, 0.5), (1.0, 2.0, 0.1))
    assert d_pos == pytest.approx(1.0)
    assert d_yaw == pytest.approx(0.4)


def test_aligner_none_before_first_sample():
    assert PoseAligner().truth_in_map((0, 0, 0)) is None


# ------------------------------------------------------------ content lag

from _slam_lens_core import TruthHistory, content_lag           # noqa: E402


def _square_raycast(pose, n=360, half=2.0):
    """Synthetic walls model: exact ranges to a 2*half square room."""
    x, y, yaw = pose
    out = np.empty(n)
    for i in range(n):
        a = yaw - math.pi + i * (2 * math.pi / n)
        c, s = math.cos(a), math.sin(a)
        best = np.inf
        for wx in (half, -half):
            if abs(c) > 1e-9:
                t = (wx - x) / c
                if t > 0 and abs(y + t * s) <= half + 1e-9:
                    best = min(best, t)
        for wy in (half, -half):
            if abs(s) > 1e-9:
                t = (wy - y) / s
                if t > 0 and abs(x + t * c) <= half + 1e-9:
                    best = min(best, t)
        out[i] = best
    return out


def _spinning_history(rate=0.5, seconds=12.0, dt=0.05):
    h = TruthHistory()
    t, yaw = 0.0, 0.0
    while t <= seconds:
        h.feed(t, (0.3, -0.2, yaw))
        yaw = wrap_angle(yaw + rate * dt)
        t += dt
    return h


def test_truth_history_interpolates_and_windows():
    h = _spinning_history()
    p = h.pose_at(6.025)                      # between samples
    assert p is not None
    assert h.pose_at(-5.0) is None            # before window
    assert h.pose_at(50.0) is None            # after window


def test_content_lag_zero_for_fresh_content():
    # Scan generated from the pose AT its stamp must fit best near offset 0.
    h = _spinning_history()
    stamp = 8.0
    ranges = _square_raycast(h.pose_at(stamp))
    best = content_lag(ranges, 0.12, 8.0, h.pose_at, _square_raycast, stamp)
    assert best is not None
    assert abs(best[0]) <= 0.051
    assert best[1] < 0.02


def test_content_lag_reads_stale_content():
    # THE Isaac render-pacing signature: content from 0.30 s before the
    # stamp, under a body turning at 0.5 rad/s, must read ~-0.30 s.
    h = _spinning_history()
    stamp = 8.0
    ranges = _square_raycast(h.pose_at(stamp - 0.30))
    best = content_lag(ranges, 0.12, 8.0, h.pose_at, _square_raycast, stamp)
    assert best is not None
    assert best[0] == pytest.approx(-0.30, abs=0.051)


def test_content_lag_none_without_truth():
    empty = TruthHistory()
    ranges = np.full(360, 2.0)
    assert content_lag(ranges, 0.12, 8.0, empty.pose_at,
                       _square_raycast, 8.0) is None


def test_content_lag_survives_boxes_short_of_walls():
    # A displaced box (returns SHORT of the wall) must be excluded, not
    # counted as content error — same walls-only rule as the scan relay.
    h = _spinning_history()
    stamp = 8.0
    ranges = _square_raycast(h.pose_at(stamp)).copy()
    ranges[40:80] = 0.6                        # a fat box somewhere close
    best = content_lag(ranges, 0.12, 8.0, h.pose_at, _square_raycast, stamp)
    assert best is not None
    assert abs(best[0]) <= 0.051
    assert best[1] < 0.02
