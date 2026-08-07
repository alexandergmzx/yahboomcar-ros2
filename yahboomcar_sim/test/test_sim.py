"""Tests for the simulated firmware.

The bar is not "does it move plausibly" but "does it reproduce the behaviours that were
MEASURED on the real car" -- because those are the reason it exists. A simulator that
smooths over the awkward parts would let the safety tooling pass against a robot that
does not exist.
"""
import math

import numpy as np
import pytest

from yahboomcar_sim.arena import (LIDAR_RANGE_MAX, default_arena, raycast,
                                  segments_box, segments_room)
from yahboomcar_sim.physics import RobotState, apply_command, step


# ------------------------------------------------------------------- geometry
def test_scan_from_the_centre_of_a_square_room_sees_four_walls():
    r = raycast((0.0, 0.0), 0.0, segments_room(4.0, 4.0))
    assert np.isfinite(r).all()
    # Nearest return is the perpendicular distance to a wall; furthest is a corner.
    assert r.min() == pytest.approx(2.0, abs=0.02)
    assert r.max() == pytest.approx(math.hypot(2, 2), abs=0.05)


def test_scan_rotates_with_the_robot():
    """Rotating the robot must rotate the scan, not change its shape."""
    segs = default_arena()
    a = raycast((0.5, 0.3), 0.0, segs)
    b = raycast((0.5, 0.3), math.pi / 2, segs)
    assert sorted(a[np.isfinite(a)])[:20] == pytest.approx(
        sorted(b[np.isfinite(b)])[:20], abs=0.05)


def test_beams_that_hit_nothing_are_inf_not_range_max():
    """Returning range_max would invent a wall at exactly 8 m all the way round, and a
    scan matcher would happily match against it."""
    r = raycast((0.0, 0.0), 0.0, [((100.0, 100.0), (101.0, 100.0))])
    assert np.isinf(r).all()
    assert not np.any(r == LIDAR_RANGE_MAX)


def test_a_box_casts_a_shadow():
    """Something behind a box must not be visible through it."""
    wall = [((-5.0, 3.0), (5.0, 3.0))]
    clear = raycast((0.0, 0.0), 0.0, wall)
    blocked = raycast((0.0, 0.0), 0.0, wall + segments_box(0.0, 1.0, 0.5))
    # Straight ahead (+y) is beam index 270 for angle_min = -pi.
    i = 270
    assert clear[i] == pytest.approx(3.0, abs=0.05)
    assert blocked[i] < 1.0


# ----------------------------------------------------------------- the chassis
def test_strafe_is_discarded_entirely():
    """Measured on the real car: a commanded strafe produces EXACTLY zero on every
    /odom_raw axis. Not attenuated -- zero."""
    vx, wz = apply_command(0.0, 0.5, 0.0)
    assert vx == 0.0 and wz == 0.0


def test_speed_and_yaw_are_capped():
    vx, wz = apply_command(10.0, 0.0, 99.0)
    assert vx == pytest.approx(0.35)
    assert wz == pytest.approx(1.5)


def test_driving_forward_moves_along_the_heading():
    s = RobotState(yaw=math.pi / 2)
    for _ in range(100):
        s, _, _ = step(s, 0.1, 0.0, 0.01)
    assert s.x == pytest.approx(0.0, abs=1e-9)
    assert s.y == pytest.approx(0.1, abs=1e-3)


# ------------------------------------------------------------------------ slip
def test_no_slip_means_odometry_matches_truth():
    s = RobotState()
    for _ in range(200):
        s, _, _ = step(s, 0.15, 0.1, 0.01, slip=0.0)
    assert s.odom_x == pytest.approx(s.x, abs=1e-9)
    assert s.odom_y == pytest.approx(s.y, abs=1e-9)


def test_full_slip_is_the_car_on_its_stand():
    """slip=1.0: the wheels turn and the body does not move. On the real stand this
    reads as 92% translational slip, and it is the case the EKF A/B was run on."""
    s = RobotState()
    for _ in range(500):
        s, _, _ = step(s, 0.15, 0.0, 0.01, slip=1.0)
    assert s.x == pytest.approx(0.0, abs=1e-12)          # body did not move
    assert s.odom_x > 0.7                                 # wheels say it travelled
    assert s.odom_x == pytest.approx(0.75, abs=0.01)


def test_encoders_cannot_see_slip():
    """The whole reason slip is dangerous: odometry is IDENTICAL with and without it."""
    a = b = RobotState()
    for _ in range(200):
        a, _, _ = step(a, 0.15, 0.05, 0.01, slip=0.0)
        b, _, _ = step(b, 0.15, 0.05, 0.01, slip=0.6)
    assert b.odom_x == pytest.approx(a.odom_x, abs=1e-12)
    assert b.odom_yaw == pytest.approx(a.odom_yaw, abs=1e-12)
    assert b.x < a.x * 0.5          # ...while the truth diverges badly


def test_partial_slip_scales_the_truth_not_the_odometry():
    s = RobotState()
    for _ in range(200):
        s, _, _ = step(s, 0.10, 0.0, 0.01, slip=0.25)
    assert s.x == pytest.approx(0.75 * s.odom_x, abs=1e-9)


# ------------------------------------------------------ the missing watchdog
def test_a_command_is_retained_indefinitely():
    """THE property this simulator exists to reproduce.

    The real firmware has no command watchdog: measured three ways, and confirmed by a
    probe that held 0.15 m/s for 45 s with nothing publishing. A simulator that quietly
    decayed the command would let cmd_vel_deadman and tools/test_failsafe.py pass against
    a robot that does not exist.

    The command lives in the node rather than here, so this pins the physics half: given
    a held command, motion continues without bound.
    """
    s = RobotState()
    for _ in range(6000):            # 60 s at 100 Hz, nothing republished
        s, vx, _ = step(s, 0.15, 0.0, 0.01)
        assert vx == 0.15            # never decays
    assert s.odom_x > 8.0            # still going


def test_zero_command_stops_it():
    s = RobotState()
    for _ in range(100):
        s, _, _ = step(s, 0.15, 0.0, 0.01)
    moved = s.odom_x
    for _ in range(100):
        s, vx, _ = step(s, 0.0, 0.0, 0.01)
        assert vx == 0.0
    assert s.odom_x == pytest.approx(moved, abs=1e-12)
