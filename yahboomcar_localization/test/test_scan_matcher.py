"""Tests for the ICP scan matcher and its degeneracy detector.

Two things are being pinned. First that the matcher recovers transforms it should --
easy, and not the interesting part. Second that it REFUSES when the geometry cannot
support an answer, which is the part that keeps a confident-looking number from a
featureless room out of a state estimate.

Synthetic scenes throughout: the transform is known exactly, so "recovered to 1 mm" is a
real claim rather than agreement between two guesses.
"""
import math

import numpy as np
import pytest

from yahboomcar_localization.scan_geometry import (compose, estimate_normals,
                                                   scan_to_xy, transform_xy, wrap)
from yahboomcar_localization import scan_matcher as sm
from yahboomcar_localization.scan_matcher import (degeneracy, estimate_motion,
                                                  match, nearest,
                                                  nearest_bruteforce,
                                                  rigid_transform)


# ----------------------------------------------------------------- scene builders
def room(w=4.0, h=4.0, n=90):
    """Four walls of a rectangular room, sampled as points."""
    xs = np.linspace(-w / 2, w / 2, n)
    ys = np.linspace(-h / 2, h / 2, n)
    return np.vstack([
        np.column_stack((xs, np.full(n, -h / 2))),
        np.column_stack((xs, np.full(n, h / 2))),
        np.column_stack((np.full(n, -w / 2), ys)),
        np.column_stack((np.full(n, w / 2), ys)),
    ])


def corridor(length=8.0, gap=2.0, n=200):
    """TWO PARALLEL WALLS ONLY. Sliding along them is unobservable."""
    xs = np.linspace(-length / 2, length / 2, n)
    return np.vstack([
        np.column_stack((xs, np.full(n, -gap / 2))),
        np.column_stack((xs, np.full(n, gap / 2))),
    ])


def box(cx, cy, s=0.4, n=20):
    xs = np.linspace(cx - s / 2, cx + s / 2, n)
    ys = np.linspace(cy - s / 2, cy + s / 2, n)
    return np.vstack([
        np.column_stack((xs, np.full(n, cy - s / 2))),
        np.column_stack((xs, np.full(n, cy + s / 2))),
        np.column_stack((np.full(n, cx - s / 2), ys)),
        np.column_stack((np.full(n, cx + s / 2), ys)),
    ])


# ------------------------------------------------------------------ scan_to_xy
def test_scan_to_xy_places_points_at_the_right_angles():
    xy = scan_to_xy([1.0, 1.0], angle_min=0.0, angle_increment=math.pi / 2)
    assert xy[0] == pytest.approx([1.0, 0.0], abs=1e-9)
    assert xy[1] == pytest.approx([0.0, 1.0], abs=1e-9)


def test_scan_to_xy_drops_invalid_rather_than_placing_them_at_the_origin():
    """A dropped return must not become a point at (0,0). To a scan matcher that is a
    feature which never moves, silently anchoring the estimate."""
    xy = scan_to_xy([float('nan'), float('inf'), 0.0, 2.0],
                    angle_min=0.0, angle_increment=0.1, range_min=0.05, range_max=10.0)
    assert len(xy) == 1
    assert not np.any(np.all(np.isclose(xy, 0.0), axis=1))


def test_scan_to_xy_empty():
    assert scan_to_xy([], 0.0, 0.1).shape == (0, 2)


# ----------------------------------------------------------------- primitives
def test_rigid_transform_recovers_a_known_pose():
    p = room()
    truth = (0.13, -0.07, 0.21)
    q = transform_xy(p, *truth)
    got = rigid_transform(p, q)          # exact correspondences
    assert got == pytest.approx(truth, abs=1e-9)


def test_nearest_is_actually_nearest():
    dst = np.array([[0.0, 0.0], [1.0, 0.0], [0.0, 1.0]])
    idx, dist = nearest(np.array([[0.9, 0.05]]), dst)
    assert idx[0] == 1
    assert dist[0] == pytest.approx(math.hypot(0.1, 0.05))


def test_nearest_chunking_matches_unchunked():
    rng = np.random.default_rng(0)
    src, dst = rng.normal(size=(300, 2)), rng.normal(size=(200, 2))
    i1, d1 = nearest(src, dst, chunk=7)
    i2, d2 = nearest(src, dst, chunk=10_000)
    assert np.array_equal(i1, i2) and np.allclose(d1, d2)


def test_compose_matches_sequential_application():
    a, b = (0.2, -0.1, 0.3), (-0.05, 0.4, -0.15)
    pts = room()
    seq = transform_xy(transform_xy(pts, *a), *b)
    comp = transform_xy(pts, *compose(a, b))
    assert np.allclose(seq, comp, atol=1e-12)


# ----------------------------------------------------------------------- ICP
# Per-scan motion the car can actually produce: at 12 Hz, 0.30 m/s is 0.025 m and
# 1.5 rad/s is 0.125 rad. Cases beyond that are not "hard", they are impossible -- the
# (0.12 m, 0.15 rad) case an earlier version tested would need 1.44 m/s.
@pytest.mark.parametrize('truth', [
    (0.025, 0.000, 0.000),
    (0.000, 0.020, 0.000),
    (0.000, 0.000, 0.125),
    (0.020, -0.015, 0.090),
    (-0.025, 0.010, -0.125),
])
def test_icp_recovers_realistic_per_scan_transforms(truth):
    """A room with four walls constrains both axes, so ICP should be accurate."""
    target = np.vstack([room(), box(1.0, 0.8), box(-1.2, -0.6)])
    source = transform_xy(target, *(-t for t in truth))   # source is target moved back
    r = match(source, target)
    assert r.converged, r.reason
    assert (r.dx, r.dy) == pytest.approx((truth[0], truth[1]), abs=0.01)
    assert wrap(r.dtheta - truth[2]) == pytest.approx(0.0, abs=0.02)


def test_rotation_accuracy_is_limited_by_sample_spacing_not_by_the_matcher():
    """The residual rotation error is discretisation of the SCENE, not matcher bias.

    Measured: error tracks (sample spacing / room radius) to within 5% across a 16x
    density sweep -- 88.9 mm spacing gave 33.0 mrad against a predicted 31.4, and
    5.6 mm gave 2.0 against 2.0. Densifying the scene drives it to zero, which a
    genuine algorithmic bias would not do.

    Worth knowing because the real lidar is 360 beams at 1 deg, so at a 0.7 m median
    range its arc spacing is ~12 mm -- finer than the 44 mm scene used in most of
    these tests.
    """
    truth_th = 0.125
    errs = []
    for n in (90, 360):
        target = np.vstack([room(n=n), box(1.0, 0.8, n=n // 5)])
        r = match(transform_xy(target, 0.0, 0.0, -truth_th), target)
        errs.append(abs(r.dtheta - truth_th))
    # 4x the samples must give substantially less error.
    assert errs[1] < errs[0] / 3, errs


def test_icp_is_accurate_to_millimetres_on_a_pure_translation():
    target = np.vstack([room(), box(1.0, 0.8)])
    source = transform_xy(target, -0.05, 0.0, 0.0)
    r = match(source, target)
    assert r.dx == pytest.approx(0.05, abs=0.002)
    assert r.dy == pytest.approx(0.0, abs=0.002)


def test_icp_reports_zero_for_no_motion():
    target = np.vstack([room(), box(0.9, -0.4)])
    r = match(target.copy(), target)
    assert math.hypot(r.dx, r.dy) < 1e-3
    assert abs(r.dtheta) < 1e-3


def test_icp_refuses_an_empty_scan():
    r = match(np.zeros((0, 2)), room())
    assert not r.converged and 'too few points' in r.reason
    assert not r.trustworthy


def test_icp_refuses_when_scenes_do_not_overlap():
    r = match(room() + 50.0, room())
    assert not r.trustworthy


# --------------------------------------------------------------- DEGENERACY
def test_corridor_is_reported_degenerate():
    """THE test. Two parallel walls cannot constrain motion along them, and a matcher
    that returns a confident number there is worse than one that refuses."""
    d = degeneracy(corridor())
    assert d.degenerate
    # The unconstrained direction must be ALONG the corridor (x), not across it.
    assert abs(d.weak_dir[0]) > 0.9, d.weak_dir
    assert d.isotropy < 0.05


def test_match_in_a_corridor_is_marked_untrustworthy():
    target = corridor()
    source = transform_xy(target, -0.10, 0.0, 0.0)     # slide along the corridor
    r = match(source, target)
    assert not r.trustworthy
    assert 'degenerate' in r.reason


def test_a_four_walled_room_is_NOT_degenerate():
    d = degeneracy(room())
    assert not d.degenerate
    assert d.isotropy > 0.5


def test_boxes_rescue_a_degenerate_corridor():
    """Directly relevant to the test arena: obstacles are what make translation
    observable in an otherwise featureless space."""
    bare = degeneracy(corridor())
    boxed = degeneracy(np.vstack([corridor(), box(-2.0, 0.0), box(2.0, 0.3)]))
    assert bare.degenerate
    assert boxed.isotropy > bare.isotropy * 3
    assert not boxed.degenerate


def test_degeneracy_of_too_few_points_fails_closed():
    d = degeneracy(np.array([[1.0, 0.0], [1.0, 0.1]]))
    assert d.degenerate


def test_normals_of_a_flat_wall_all_point_the_same_way():
    xs = np.linspace(-1, 1, 40)
    wall = np.column_stack((xs, np.zeros(40)))
    n = estimate_normals(wall)
    good = n[np.linalg.norm(n, axis=1) > 0.5]
    assert len(good) > 30
    assert np.all(np.abs(good[:, 1]) > 0.95)      # normal is +/- y for a wall along x


def test_normals_are_not_corrupted_by_range_discontinuities():
    """Index-adjacent points can be metres apart at a depth jump. Without the distance
    guard a line fit spans the jump and invents a normal describing neither surface;
    with it, each point is fitted only against its own surface."""
    near = np.column_stack((np.linspace(0, 0.3, 10), np.zeros(10)))
    far = np.column_stack((np.linspace(0, 0.3, 10), np.full(10, 5.0)))
    both = np.vstack([near, far])

    guarded = estimate_normals(both, max_neighbour_dist=0.2)
    # Points either side of the jump still get their OWN wall's normal (+/- y).
    for i in (8, 9, 10, 11):
        if np.linalg.norm(guarded[i]) > 0.5:
            assert abs(guarded[i][1]) > 0.9, f'point {i} normal {guarded[i]}'

    # Without the guard, the fit spans a 5 m jump and the normal rotates away from y.
    unguarded = estimate_normals(both, max_neighbour_dist=100.0)
    corrupted = [i for i in (8, 9, 10, 11)
                 if np.linalg.norm(unguarded[i]) > 0.5 and abs(unguarded[i][1]) < 0.9]
    assert corrupted, 'the guard should be preventing something'



# --------------------------------------------------------------- robustness
def test_rotation_is_not_biased_low_by_outlier_rejection():
    """Regression: a fixed-fraction trim discards the points furthest from the centre of
    rotation, which are exactly the ones that see rotation. Keeping the closest 80%
    converged to a stable 0.0852 rad against a true 0.12 -- a 29% under-estimate that
    looked like clean convergence. Robust MAD rejection must not do that."""
    target = np.vstack([room(), box(1.0, 0.8), box(-1.2, -0.6)])
    for truth_th in (0.06, 0.12, 0.18):
        source = transform_xy(target, 0.0, 0.0, -truth_th)
        r = match(source, target)
        assert r.dtheta == pytest.approx(truth_th, abs=0.02), \
            f'rotation {truth_th} recovered as {r.dtheta}'


def test_moving_obstacle_does_not_drag_the_estimate():
    """Outlier rejection should reject a minority of points that moved differently."""
    target = np.vstack([room(), box(1.0, 0.8)])
    truth = (0.05, 0.02, 0.0)
    source = transform_xy(target, *(-t for t in truth))
    # A person-sized blob that moved somewhere else entirely.
    source = np.vstack([source, box(-1.5, 1.5, s=0.5, n=15)])
    r = match(source, target)
    assert (r.dx, r.dy) == pytest.approx((truth[0], truth[1]), abs=0.02)


def test_noise_does_not_break_convergence():
    rng = np.random.default_rng(3)
    target = np.vstack([room(), box(1.0, 0.8)])
    truth = (0.04, -0.03, 0.05)
    source = transform_xy(target, *(-t for t in truth))
    source = source + rng.normal(scale=0.005, size=source.shape)
    r = match(source, target)
    assert r.converged
    assert (r.dx, r.dy) == pytest.approx((truth[0], truth[1]), abs=0.02)


# ------------------------------------------------- frame convention (regression)
def _simulate_scans(robot_motion, scene):
    """Scene as seen from the sensor before and after the robot moves by robot_motion.

    A world point p is seen as T^-1 p, so moving the robot by dT transforms the observed
    cloud by dT^-1.
    """
    prev = scene
    dx, dy, dth = robot_motion
    c, s_ = math.cos(-dth), math.sin(-dth)
    R = np.array([[c, -s_], [s_, c]])
    curr = (scene - np.array([dx, dy])) @ R.T
    return prev, curr


@pytest.mark.parametrize('motion', [
    (0.02, 0.00, 0.00),
    (0.00, 0.02, 0.00),
    (0.00, 0.00, 0.10),
    (0.00, 0.00, -0.10),
    (0.02, -0.01, 0.06),
])
def test_estimate_motion_returns_ROBOT_motion_not_scene_motion(motion):
    """Regression for a sign error that survived because the magnitude looked right.

    On a real desk run the encoders and gyro both read about +1.8 rad of yaw while the
    naive match(prev, curr) read -1.3. Only the sign exposed it.
    """
    scene = np.vstack([room(n=360), box(1.0, 0.8, n=60), box(-1.2, -0.6, n=60)])
    prev, curr = _simulate_scans(motion, scene)
    r = estimate_motion(prev, curr)
    assert r.converged, r.reason
    assert (r.dx, r.dy) == pytest.approx((motion[0], motion[1]), abs=0.01)
    assert wrap(r.dtheta - motion[2]) == pytest.approx(0.0, abs=0.02)


def test_raw_match_returns_the_INVERSE_of_robot_motion():
    """Pins why estimate_motion swaps its arguments, so the reason survives refactors."""
    scene = np.vstack([room(n=360), box(1.0, 0.8, n=60)])
    motion = (0.0, 0.0, 0.10)
    prev, curr = _simulate_scans(motion, scene)
    raw = match(prev, curr)
    assert wrap(raw.dtheta + motion[2]) == pytest.approx(0.0, abs=0.02), \
        'match(prev, curr) should give the NEGATIVE of the robot rotation'


# ----------------------------------------------- scipy path vs numpy fallback
def test_kdtree_and_bruteforce_agree_exactly():
    """The fast path and the fallback must be interchangeable. scipy is optional here
    because it was broken on this machine for a while (a pip numpy shadowing apt's, with
    apt scipy built against the older ABI), so the fallback is a real code path and not
    a formality."""
    rng = np.random.default_rng(11)
    src, dst = rng.normal(size=(200, 2)), rng.normal(size=(150, 2))
    i_bf, d_bf = nearest_bruteforce(src, dst)
    i_now, d_now = nearest(src, dst)
    assert np.array_equal(i_bf, i_now)
    assert np.allclose(d_bf, d_now)


def test_matching_gives_the_same_answer_on_either_backend():
    target = np.vstack([room(n=180), box(1.0, 0.8, n=40)])
    source = transform_xy(target, -0.02, 0.01, -0.05)
    fast = match(source, target)
    real, sm.HAVE_KDTREE = sm.HAVE_KDTREE, False
    try:
        slow = match(source, target)
    finally:
        sm.HAVE_KDTREE = real
    assert (fast.dx, fast.dy) == pytest.approx((slow.dx, slow.dy), abs=1e-6)
    assert fast.dtheta == pytest.approx(slow.dtheta, abs=1e-6)


def test_fallback_still_works_when_scipy_is_pretended_absent():
    real, sm.HAVE_KDTREE = sm.HAVE_KDTREE, False
    try:
        target = np.vstack([room(n=180), box(1.0, 0.8, n=40)])
        r = match(transform_xy(target, -0.02, 0.0, 0.0), target)
        assert r.converged and r.dx == pytest.approx(0.02, abs=0.005)
    finally:
        sm.HAVE_KDTREE = real
