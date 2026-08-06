"""Exhaustive offline tests for the safety governor. No robot, no ROS graph.

These are the tests that must pass before the car is allowed on the floor. Each one
states the hazard it guards against, because a safety test whose intent is unclear
tends to get "fixed" by loosening it.
"""
import math

import pytest

from yahboomcar_safety.governor import (GovernorConfig, decide, forward_min_range)

CFG = GovernorConfig()
FRESH = 0.05  # a comfortably recent timestamp age, seconds


def scan(ranges, angle_min=-math.pi, span=2 * math.pi):
    """Build (ranges, angle_min, angle_increment) for a full-circle scan."""
    return ranges, angle_min, span / len(ranges)


# --------------------------------------------------------------- sector geometry

def test_sector_sees_only_forward():
    # 360 beams, one per degree, all 5 m except a 0.5 m return directly BEHIND.
    r = [5.0] * 360
    r[0] = 0.5                      # index 0 -> angle_min = -pi, i.e. behind
    rng = forward_min_range(*scan(r), CFG)
    assert rng == pytest.approx(5.0), 'an obstacle behind must not restrict forward motion'


def test_sector_sees_obstacle_ahead():
    r = [5.0] * 360
    r[180] = 0.4                    # angle 0 -> straight ahead
    assert forward_min_range(*scan(r), CFG) == pytest.approx(0.4)


def test_sector_edges():
    # Just inside vs just outside +/-45 degrees.
    r = [5.0] * 360
    r[180 + 44] = 0.6               # +44 deg, inside
    assert forward_min_range(*scan(r), CFG) == pytest.approx(0.6)
    r = [5.0] * 360
    r[180 + 60] = 0.6               # +60 deg, outside
    assert forward_min_range(*scan(r), CFG) == pytest.approx(5.0)


def test_nan_and_zero_returns_ignored():
    # Lidars emit NaN for no-return and occasional zeros; treating a zero as an
    # obstacle 0 m away would make the robot freeze permanently.
    r = [float('nan')] * 360
    r[180] = 0.0
    r[181] = 1.2
    assert forward_min_range(*scan(r), CFG) == pytest.approx(1.2)


def test_empty_scan_is_unknown_not_clear():
    assert forward_min_range([], -math.pi, 0.0, CFG) == math.inf


# --------------------------------------------------------------- decision rules

def test_clear_path_passes_through_untouched():
    d = decide(0.2, 0.0, 0.0, min_range=3.0, scan_age=FRESH, cmd_age=FRESH, cfg=CFG)
    assert d.vx == pytest.approx(0.2)
    assert not d.limited


def test_hard_stop_inside_stop_distance():
    d = decide(0.3, 0.0, 0.0, min_range=0.20, scan_age=FRESH, cmd_age=FRESH, cfg=CFG)
    assert d.vx == 0.0 and d.limited


def test_slows_progressively_between_thresholds():
    near = decide(0.3, 0, 0, 0.45, FRESH, FRESH, CFG).vx
    far = decide(0.3, 0, 0, 0.80, FRESH, FRESH, CFG).vx
    assert 0.0 < near < far < 0.3, 'speed must decrease monotonically as range closes'


def test_scale_is_continuous_at_the_boundaries():
    # No discontinuity that would make the robot lurch.
    just_inside = decide(0.3, 0, 0, CFG.stop_distance + 1e-6, FRESH, FRESH, CFG).vx
    just_outside = decide(0.3, 0, 0, CFG.slow_distance - 1e-6, FRESH, FRESH, CFG).vx
    assert just_inside == pytest.approx(0.0, abs=1e-3)
    assert just_outside == pytest.approx(0.3, abs=1e-2)


def test_stale_scan_stops_even_with_clear_range():
    # The hazard: lidar dies while the last frame happened to look clear.
    d = decide(0.3, 0, 0, min_range=5.0, scan_age=2.0, cmd_age=FRESH, cfg=CFG)
    assert d.vx == 0.0 and 'stale' in d.reason


def test_missing_scan_stops():
    d = decide(0.3, 0, 0, min_range=5.0, scan_age=None, cmd_age=FRESH, cfg=CFG)
    assert d.vx == 0.0


def test_stale_command_stops():
    # The hazard: operator's process dies mid-command and the robot keeps going.
    d = decide(0.3, 0, 0, min_range=5.0, scan_age=FRESH, cmd_age=1.0, cfg=CFG)
    assert d.vx == 0.0 and 'command stale' in d.reason


def test_no_valid_returns_stops_forward_motion():
    # An all-NaN scan is a blind lidar, not an empty room.
    d = decide(0.3, 0, 0, min_range=math.inf, scan_age=FRESH, cmd_age=FRESH, cfg=CFG)
    assert d.vx == 0.0 and 'no valid lidar' in d.reason


def test_reverse_allowed_near_obstacle():
    # Backing away from a wall must stay possible, and is documented as unprotected
    # because the sector faces forward.
    d = decide(-0.2, 0, 0, min_range=0.10, scan_age=FRESH, cmd_age=FRESH, cfg=CFG)
    assert d.vx == pytest.approx(-0.2)


def test_speed_and_yaw_caps():
    d = decide(10.0, 0.0, 99.0, min_range=5.0, scan_age=FRESH, cmd_age=FRESH, cfg=CFG)
    assert d.vx == pytest.approx(CFG.max_speed)
    assert d.wz == pytest.approx(CFG.max_yaw)


def test_non_finite_command_stops():
    d = decide(float('nan'), 0, 0, min_range=5.0, scan_age=FRESH, cmd_age=FRESH, cfg=CFG)
    assert d.vx == 0.0 and d.wz == 0.0


def test_stale_scan_beats_a_clear_reading():
    # Rule ordering matters: staleness must not be overridable by a good-looking range.
    d = decide(0.3, 0, 0, min_range=10.0, scan_age=99.0, cmd_age=FRESH, cfg=CFG)
    assert d.vx == 0.0


def test_rotation_allowed_when_blocked_ahead():
    # Turning in place is how you escape; blocking it would trap the robot.
    d = decide(0.3, 0.0, 0.8, min_range=0.10, scan_age=FRESH, cmd_age=FRESH, cfg=CFG)
    assert d.vx == 0.0
    assert d.wz == pytest.approx(0.8)


@pytest.mark.parametrize('rng', [0.05, 0.2, 0.34, 0.349])
def test_never_moves_forward_inside_stop_distance(rng):
    assert decide(0.3, 0, 0, rng, FRESH, FRESH, CFG).vx == 0.0


@pytest.mark.parametrize('vx', [0.05, 0.15, 0.3, 1.0])
def test_output_never_exceeds_request_forward(vx):
    # The filter must only ever reduce forward speed, never amplify it.
    d = decide(vx, 0, 0, min_range=0.6, scan_age=FRESH, cmd_age=FRESH, cfg=CFG)
    assert d.vx <= vx + 1e-9
