"""Exhaustive offline tests for the safety governor. No robot, no ROS graph.

These are the tests that must pass before the car is allowed on the floor. Each one
states the hazard it guards against, because a safety test whose intent is unclear
tends to get "fixed" by loosening it.
"""
import math

import pytest

from yahboomcar_safety.governor import (EXPECTED_PUBLISHERS, DockingApproach,
                                        GovernorConfig, bypassing_nodes, decide,
                                        forward_min_range)

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


def test_reverse_allowed_near_obstacle_but_bounded():
    # Backing away from a wall must stay possible -- blocking it would trap the robot
    # against the thing it is trying to escape. But reverse is entirely sensor-blind,
    # so it is bounded well below the forward cap rather than passed through.
    d = decide(-0.2, 0, 0, min_range=0.10, scan_age=FRESH, cmd_age=FRESH, cfg=CFG)
    assert d.vx == pytest.approx(-CFG.max_reverse_speed)
    assert 'reverse capped' in d.reason


def test_slow_reverse_passes_through_untouched():
    d = decide(-0.05, 0, 0, min_range=0.10, scan_age=FRESH, cmd_age=FRESH, cfg=CFG)
    assert d.vx == pytest.approx(-0.05)


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


def test_rotation_allowed_when_blocked_ahead_but_gated():
    # Turning in place is how you escape; blocking it would trap the robot. But rotating
    # sweeps the footprint CORNERS past an obstacle the forward sector cannot watch, so
    # near an obstacle it is gated rather than passed through.
    d = decide(0.3, 0.0, 0.8, min_range=0.10, scan_age=FRESH, cmd_age=FRESH, cfg=CFG)
    assert d.vx == 0.0
    assert d.wz == pytest.approx(CFG.max_yaw_near)
    assert 'yaw gated' in d.reason


def test_rotation_ungated_away_from_obstacles():
    d = decide(0.0, 0.0, 0.8, min_range=5.0, scan_age=FRESH, cmd_age=FRESH, cfg=CFG)
    assert d.wz == pytest.approx(0.8)


def test_slow_rotation_near_obstacle_passes_through():
    d = decide(0.0, 0.0, 0.2, min_range=0.10, scan_age=FRESH, cmd_age=FRESH, cfg=CFG)
    assert d.wz == pytest.approx(0.2)


# ------------------------------------------------- lateral: the audited defect
def test_lateral_zeroed_on_obstacle_stop():
    """The defect this pins: obstacle-stop paths returned out_vy UNCHANGED while
    zeroing vx, so a command carrying linear.y kept its lateral component straight
    through a stop. Inert only because this chassis is differential."""
    d = decide(0.3, 0.25, 0.0, min_range=0.10, scan_age=FRESH, cmd_age=FRESH, cfg=CFG)
    assert d.vx == 0.0
    assert d.vy == 0.0


def test_lateral_zeroed_on_blind_lidar_stop():
    d = decide(0.3, 0.25, 0.0, min_range=math.inf, scan_age=FRESH, cmd_age=FRESH, cfg=CFG)
    assert d.vy == 0.0


def test_lateral_zeroed_in_normal_driving():
    # The chassis is differential -- a commanded strafe produces exactly zero on every
    # /odom_raw axis. Do not rely on the firmware to ignore it.
    d = decide(0.1, 0.25, 0.0, min_range=5.0, scan_age=FRESH, cmd_age=FRESH, cfg=CFG)
    assert d.vy == 0.0
    assert 'lateral zeroed' in d.reason


def test_every_stop_path_zeroes_all_translation():
    """No stop path may leak translation on any axis."""
    stops = [
        decide(0.3, 0.2, 0.0, 5.0, 99.0, FRESH, CFG),            # stale scan
        decide(0.3, 0.2, 0.0, 5.0, FRESH, 99.0, CFG),            # stale command
        decide(float('nan'), 0.2, 0.0, 5.0, FRESH, FRESH, CFG),  # non-finite
        decide(0.3, 0.2, 0.0, math.inf, FRESH, FRESH, CFG),      # blind lidar
        decide(0.3, 0.2, 0.0, 0.10, FRESH, FRESH, CFG),          # obstacle
    ]
    for d in stops:
        assert d.vx == 0.0 and d.vy == 0.0, d.reason


def test_lateral_capped_when_explicitly_enabled():
    cfg = GovernorConfig(allow_lateral=True)
    d = decide(0.0, 10.0, 0.0, min_range=5.0, scan_age=FRESH, cmd_age=FRESH, cfg=cfg)
    assert d.vy == pytest.approx(cfg.max_speed)


@pytest.mark.parametrize('rng', [0.05, 0.2, 0.34, 0.349])
def test_never_moves_forward_inside_stop_distance(rng):
    assert decide(0.3, 0, 0, rng, FRESH, FRESH, CFG).vx == 0.0


@pytest.mark.parametrize('vx', [0.05, 0.15, 0.3, 1.0])
def test_output_never_exceeds_request_forward(vx):
    # The filter must only ever reduce forward speed, never amplify it.
    d = decide(vx, 0, 0, min_range=0.6, scan_age=FRESH, cmd_age=FRESH, cfg=CFG)
    assert d.vx <= vx + 1e-9


# --------------------------------------------------- bypass detection vs the deadman
ME = '/cmd_vel_governor'


def test_deadman_is_not_a_bypass():
    """The audited contradiction: first_floor_launch.py mandates the deadman, the
    deadman must publish /cmd_vel to do its job, and the procedure says abort on
    BYPASSED. The preflight could never pass legitimately."""
    unexpected, seen = bypassing_nodes(['/cmd_vel_deadman'], ME)
    assert unexpected == []
    assert seen == ['/cmd_vel_deadman']


def test_real_bypass_still_reported_alongside_the_deadman():
    unexpected, seen = bypassing_nodes(
        ['/cmd_vel_deadman', '/yahboom_keyboard'], ME)
    assert unexpected == ['/yahboom_keyboard']
    assert seen == ['/cmd_vel_deadman']


def test_self_is_never_a_bypass():
    unexpected, seen = bypassing_nodes([ME, '/cmd_vel_deadman'], ME)
    assert unexpected == [] and seen == ['/cmd_vel_deadman']


def test_namespaced_deadman_still_recognised():
    unexpected, seen = bypassing_nodes(['/robot1/cmd_vel_deadman'], ME)
    assert unexpected == []
    assert seen == ['/robot1/cmd_vel_deadman']


def test_absent_deadman_is_visible_to_the_caller():
    """The caller needs to distinguish "no publishers" from "no deadman" -- the second
    is the dangerous one and must be warnable."""
    unexpected, seen = bypassing_nodes(['/yahboom_keyboard'], ME)
    assert seen == []
    assert unexpected == ['/yahboom_keyboard']


def test_expected_list_is_configurable():
    unexpected, seen = bypassing_nodes(['/my_watchdog'], ME, expected=('my_watchdog',))
    assert unexpected == [] and seen == ['/my_watchdog']


def test_default_expected_contains_the_deadman():
    assert 'cmd_vel_deadman' in EXPECTED_PUBLISHERS


# ------------------------------------------------------------------ docking mode
#
# The concession: a corridor delivery ends in CONTACT, and the proximity floor
# exists to prevent contact. These tests exist to prove the concession is the
# narrow one it claims to be. Every one of them states the hazard, because a
# safety test whose intent is unclear tends to get "fixed" by loosening it.


def _ahead(distance, bearing=0.0, background=5.0):
    """360 beams at `background`, with one return at `distance` on `bearing`.

    Index i maps to angle_min(-pi) + i*(2pi/360), so bearing 0 is index 180.
    """
    r = [background] * 360
    r[180 + int(round(math.degrees(bearing)))] = distance
    return r


def test_docking_masks_only_the_object_it_was_told_about():
    """The hazard: a mask wide enough to hide a second obstacle.

    B is dead ahead at 0.25 m -- well inside the 0.35 m stop -- and the approach
    names it. Without the mask the governor stops; with it, forward motion is
    permitted, because that return is the thing being driven into on purpose.
    """
    r = _ahead(0.25, bearing=0.0)
    approach = DockingApproach(bearing_rad=0.0, range_m=0.25)

    assert forward_min_range(*scan(r), CFG) == pytest.approx(0.25)
    assert forward_min_range(*scan(r), CFG, docking=approach) == pytest.approx(5.0)


def test_an_off_cone_obstacle_still_stops_the_robot_in_docking_mode():
    """**The negative control R1 requires.** A person stepping in from the side.

    B is masked at 0.25 m dead ahead; a second object sits at 0.30 m thirty
    degrees off, outside the 15-degree cone. The stop must still fire, and it
    must name the unmasked object.
    """
    r = [5.0] * 360
    r[180] = 0.25                                   # B, on the cone
    r[180 + 30] = 0.30                              # intruder, off the cone
    approach = DockingApproach(bearing_rad=0.0, range_m=0.25)

    rng = forward_min_range(*scan(r), CFG, docking=approach)
    assert rng == pytest.approx(0.30), 'the off-cone obstacle must survive the mask'

    out = decide(0.05, 0.0, 0.0, rng, FRESH, FRESH, CFG, docking=approach)
    assert out.vx == 0.0
    assert 'obstacle at 0.30 m' in out.reason


def test_the_mask_is_range_gated_so_a_nearer_surprise_is_not_hidden():
    """The hazard: something appearing between the robot and its target.

    The approach was confirmed at 0.25 m, so the gate admits returns out to
    0.35 m. A return at 0.60 m on the same bearing is NOT the object that was
    measured, and must not inherit its permission.
    """
    r = _ahead(0.60, bearing=0.0)
    approach = DockingApproach(bearing_rad=0.0, range_m=0.25, margin_m=0.10)

    assert forward_min_range(*scan(r), CFG, docking=approach) == pytest.approx(0.60)


def test_a_stale_scan_kills_the_creep_even_in_docking_mode():
    """**The negative control R1 requires.** A lidar that stopped reporting.

    Rule 1 is evaluated before the mode is consulted at all, so a terminal
    approach cannot buy permission to drive on stale data.
    """
    approach = DockingApproach(bearing_rad=0.0, range_m=0.25)

    out = decide(0.05, 0.0, 0.0, math.inf, CFG.scan_timeout + 0.1, FRESH, CFG,
                 docking=approach)

    assert out.vx == 0.0 and out.reason == 'scan stale or missing'


def test_a_dead_commander_kills_the_creep_even_in_docking_mode():
    """Rule 2, same argument as rule 1: the mode is consulted after, never before."""
    approach = DockingApproach(bearing_rad=0.0, range_m=0.25)

    out = decide(0.05, 0.0, 0.0, math.inf, FRESH, CFG.cmd_timeout + 0.1, CFG,
                 docking=approach)

    assert out.vx == 0.0 and out.reason == 'command stale'


def test_docking_mode_makes_the_robot_slower_not_faster():
    """The hazard the `--fun` preset would have introduced.

    A terminal phase wants to go SLOWER. The clamp binds even when the ordinary
    speed cap would have allowed more, and it is applied before that cap.
    """
    approach = DockingApproach(bearing_rad=0.0, range_m=0.25)

    fast = decide(0.35, 0.0, 0.0, 5.0, FRESH, FRESH, CFG, docking=approach)

    assert fast.vx == pytest.approx(CFG.docking_creep_max_speed)
    assert 'docking creep' in fast.reason
    # And the clamp is well under the ordinary cap it replaces.
    assert CFG.docking_creep_max_speed < CFG.max_speed


def test_the_empty_sector_still_fails_closed_in_docking_mode():
    """The hazard: a blind lidar reading as a clear field.

    If the mask consumes every return in the sector, the result is inf -- and
    inf must still mean "unknown", not "go".

    Note the construction: the returns must all lie INSIDE the 15-degree cone,
    because that is the only region the mask can reach. A first version of this
    test filled the whole circle at 0.25 m and passed for the wrong reason --
    the beams between 15 and 45 degrees were never maskable, so the sector was
    never actually emptied.
    """
    r = [float('nan')] * 360
    for i in range(180 - 10, 180 + 11):             # +/-10 deg, all inside the cone
        r[i] = 0.25
    approach = DockingApproach(bearing_rad=0.0, range_m=0.25, margin_m=5.0)

    rng = forward_min_range(*scan(r), CFG, docking=approach)
    assert rng == math.inf

    out = decide(0.05, 0.0, 0.0, rng, FRESH, FRESH, CFG, docking=approach)
    assert out.vx == 0.0 and 'no valid lidar returns' in out.reason


def test_without_the_mode_nothing_changes():
    """The regression guard: `docking=None` must be the behaviour that shipped."""
    r = _ahead(0.25, bearing=0.0)

    assert forward_min_range(*scan(r), CFG) == pytest.approx(0.25)
    out = decide(0.20, 0.0, 0.0, 0.25, FRESH, FRESH, CFG)
    assert out.vx == 0.0 and 'obstacle at 0.25 m' in out.reason
