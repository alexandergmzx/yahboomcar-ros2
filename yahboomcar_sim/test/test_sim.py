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
from yahboomcar_sim.physics import (ACCEL_NOISE, GRAVITY, RobotState,
                                    apply_command, imu_sample, step)


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


# ------------------------------------------- braking test fixture (known values)
def test_fixture_off_by_default_is_instant():
    """decel=0 must keep the honest no-physics behaviour."""
    s = RobotState()
    s, vx, _ = step(s, 0.15, 0.0, 0.01)
    assert vx == 0.15


def test_dead_time_expires_and_the_robot_actually_moves():
    """Regression: comparing the command against the EXECUTED speed re-armed the dead
    timer every tick, so it never expired and the robot never moved. The whole braking
    dry run reported 0 mm stopping distances before this was found."""
    s = RobotState()
    for _ in range(300):                 # 3 s at 100 Hz, dead time 0.25 s
        s, vx, _ = step(s, 0.15, 0.0, 0.01, decel=1.5, dead_time=0.25)
    assert vx == pytest.approx(0.15, abs=1e-6), 'never reached commanded speed'
    assert s.x > 0.3, f'barely moved: {s.x:.3f} m'


def test_dead_time_is_actually_honoured():
    s = RobotState()
    for _ in range(20):                  # 0.2 s, inside a 0.25 s dead time
        s, vx, _ = step(s, 0.15, 0.0, 0.01, decel=1.5, dead_time=0.25)
    assert vx == 0.0, 'moved before the dead time elapsed'


def test_braking_distance_matches_the_known_fixture_values():
    """With dead time T and deceleration a, stopping from v should take
    v*T + v^2/(2a). This is the ground truth the measurement tool must recover."""
    T, A, v = 0.25, 1.5, 0.15
    s = RobotState()
    for _ in range(400):                 # get up to speed
        s, _, _ = step(s, v, 0.0, 0.01, decel=A, dead_time=T)
    x0 = s.x
    for _ in range(400):                 # command zero and coast to rest
        s, _, _ = step(s, 0.0, 0.0, 0.01, decel=A, dead_time=T)
    expected = v * T + v * v / (2 * A)
    assert (s.x - x0) == pytest.approx(expected, rel=0.05), \
        f'stopped in {(s.x-x0)*1000:.0f} mm, expected {expected*1000:.0f} mm'


# ------------------------------------------------------------------------ IMU
# These guard a fix that came from the simulator FAILING tools/sensor_health.py: the
# accelerometer published a constant 9.81, which that tool correctly flags as a stuck
# channel. Reverting any of this to a constant would silently re-break it.
def test_accelerometer_is_noisy_because_a_constant_reads_as_dead():
    rng = np.random.default_rng(1)
    az = [imu_sample(rng, 0.0)[3] for _ in range(400)]
    assert np.std(az) > 0.0, 'a zero-variance accelerometer is what a DEAD one looks like'
    assert np.std(az) == pytest.approx(ACCEL_NOISE, rel=0.25)


def test_gravity_is_the_measured_9_799_not_the_textbook_9_81():
    """The real IMU reads 9.7980-9.8005 at rest. The gap is bigger than the noise."""
    rng = np.random.default_rng(2)
    az = np.array([imu_sample(rng, 0.0)[3] for _ in range(2000)])
    assert az.mean() == pytest.approx(GRAVITY, abs=0.002)
    assert abs(az.mean() - 9.81) > 3 * ACCEL_NOISE / math.sqrt(len(az))


def test_gyro_is_flat_at_rest_exactly_as_the_real_one_is():
    """Not an oversight. Every at-rest bag shows gyro_z std of exactly 0.00000, so there
    is no measurement to model, and inventing noise would defeat sensor_health.py's
    rotate window -- which exists because a gyro cannot be judged while stationary."""
    rng = np.random.default_rng(3)
    gz = [imu_sample(rng, 0.0)[0] for _ in range(200)]
    assert np.std(gz) == 0.0
    assert set(gz) == {0.0}


def test_slip_attenuates_the_gyro_but_never_gravity():
    """Slip means the wheels turn and the body does not, so the BODY's rate falls.
    Gravity is not a function of traction."""
    rng = np.random.default_rng(4)
    gz, _, _, az = imu_sample(rng, 2.0, slip=1.0)
    assert gz == 0.0
    samples = np.array([imu_sample(rng, 2.0, slip=1.0)[3] for _ in range(500)])
    assert samples.mean() == pytest.approx(GRAVITY, abs=0.005)
