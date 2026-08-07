"""Tests for the twin's joint-state bridge.

It had none, which an audit flagged. The bridge is the only thing standing between
/odom_raw and what the twin's wheels appear to do, and its sign conventions are the part
most likely to be silently wrong -- the URDF mirrors the right wheels, so "spins
backwards" is a plausible outcome of any change here.

What is deliberately NOT tested: whether the resulting angles look right in Isaac. That
needs the simulator, and tools/verify_twin.py covers it.
"""
import math

import pytest

from yahboomcar_twin.joint_state_bridge import (WHEEL_LF, WHEEL_LR, WHEEL_RF, WHEEL_RR,
                                                wheel_rates)

R = 0.024        # measured from zq_Link.STL
LY = 0.0675      # half-track, from the URDF joint origins
LXY = 0.115      # lx + ly


def rates(vx=0.0, vy=0.0, wz=0.0, mecanum=False):
    return wheel_rates(vx, vy, wz, R, LY, LXY, mecanum)


# ------------------------------------------------------------------ differential
def test_straight_ahead_turns_all_four_wheels_equally():
    r = rates(vx=0.10)
    assert r[WHEEL_LF] == pytest.approx(0.10 / R)
    assert len(set(round(v, 9) for v in r.values())) == 1, 'no wheel should differ'


def test_a_pure_strafe_produces_exactly_zero():
    """MEASURED on the real robot: commanding (0, +/-0.15, 0) produced exactly zero on
    every axis of /odom_raw -- the motors never ran. The firmware ignores linear.y, and
    a simulator that quietly obeyed it would model a robot that does not exist."""
    r = rates(vy=0.15)
    assert all(v == 0.0 for v in r.values())


def test_spinning_in_place_drives_the_sides_opposite():
    r = rates(wz=1.0)
    assert r[WHEEL_LF] == pytest.approx(-LY / R)
    assert r[WHEEL_RF] == pytest.approx(+LY / R)
    assert r[WHEEL_LF] == -r[WHEEL_RF]


def test_both_wheels_on_a_side_always_match():
    """Skid-steer: there is no steering rack, so a side turns as one."""
    r = rates(vx=0.2, wz=0.7)
    assert r[WHEEL_LF] == pytest.approx(r[WHEEL_LR])
    assert r[WHEEL_RF] == pytest.approx(r[WHEEL_RR])


def test_reverse_is_negative():
    r = rates(vx=-0.10)
    assert all(v < 0 for v in r.values())


def test_rates_are_linear_in_speed():
    a, b = rates(vx=0.05), rates(vx=0.10)
    assert b[WHEEL_LF] == pytest.approx(2 * a[WHEEL_LF])


# ------------------------------------------------------------------------ mecanum
def test_mecanum_branch_does_respond_to_strafe():
    """Kept for the Mcnamu/X3 variant. Off by default because this board is not it --
    so this test documents that the branch is intact, not that it applies here."""
    r = rates(vy=0.15, mecanum=True)
    assert not all(v == 0.0 for v in r.values())
    # A pure strafe drives diagonal pairs opposite ways.
    assert r[WHEEL_LF] == pytest.approx(-r[WHEEL_RF])


def test_mecanum_and_differential_agree_on_pure_forward():
    """Strafe is what separates them; straight ahead they must not disagree."""
    d, m = rates(vx=0.12), rates(vx=0.12, mecanum=True)
    for w in (WHEEL_LF, WHEEL_RF, WHEEL_RR, WHEEL_LR):
        assert d[w] == pytest.approx(m[w])


# -------------------------------------------------------------------------- guards
def test_zero_radius_is_refused_rather_than_dividing_by_zero():
    with pytest.raises(ValueError):
        wheel_rates(0.1, 0.0, 0.0, 0.0, LY, LXY)


def test_geometry_matches_the_measured_urdf():
    """A regression guard on the constants themselves. These are measured values --
    0.024 m from the STL bounding box, 0.0675 m from the URDF joint origins -- and a
    silent edit to either would change every angle the twin displays."""
    assert R == 0.024
    assert LY == 0.0675
    # One rotation of the body at 1 rad/s turns a wheel at ly/r rad/s.
    assert rates(wz=1.0)[WHEEL_RF] == pytest.approx(2.8125)


def test_stationary_is_stationary():
    assert all(v == 0.0 for v in rates().values())


def test_combined_motion_superposes():
    """Forward plus yaw is the sum of the two, which is what makes the IK invertible."""
    fwd, yaw = rates(vx=0.1), rates(wz=0.5)
    both = rates(vx=0.1, wz=0.5)
    for w in (WHEEL_LF, WHEEL_RF):
        assert both[w] == pytest.approx(fwd[w] + yaw[w])


def test_yaw_sign_convention_is_ros_standard():
    """Positive wz is counter-clockwise seen from above, so the RIGHT side must run
    faster forward. Getting this backwards makes the twin turn the wrong way, which is
    exactly the kind of error that looks fine until someone watches it."""
    r = rates(vx=0.10, wz=0.5)
    assert r[WHEEL_RF] > r[WHEEL_LF]


def test_wheel_names_are_the_urdf_names():
    """Isaac binds joints BY NAME; a rename here silently animates nothing."""
    r = rates(vx=0.1)
    assert set(r) == {'zq_Joint', 'yq_Joint', 'yh_Joint', 'zh_Joint'}


def test_angular_rate_scales_inversely_with_radius():
    small = wheel_rates(0.1, 0.0, 0.0, 0.012, LY, LXY)
    big = wheel_rates(0.1, 0.0, 0.0, 0.024, LY, LXY)
    assert small[WHEEL_LF] == pytest.approx(2 * big[WHEEL_LF])


def test_no_nan_or_inf_for_realistic_commands():
    for vx in (-0.35, -0.1, 0.0, 0.1, 0.35):
        for wz in (-1.5, 0.0, 1.5):
            for v in rates(vx=vx, wz=wz).values():
                assert math.isfinite(v)
