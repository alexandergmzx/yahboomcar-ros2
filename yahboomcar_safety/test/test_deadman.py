"""Tests for the command deadman.

The hazard being guarded is specific: the firmware retains a commanded speed forever
(measured, docs/safety-case.md), so a publisher that dies mid-command leaves the car
driving. These pin the two ways this node could fail at that -- not intervening when it
should, and intervening when it should not.
"""
import math

from yahboomcar_safety.deadman import (DeadmanConfig, is_moving_command,
                                       should_intervene)

CFG = DeadmanConfig(timeout=0.5, hold=2.0)


# --------------------------------------------------------------- is_moving_command
def test_zero_command_is_not_moving():
    assert not is_moving_command(0.0, 0.0, 0.0, CFG)


def test_any_axis_counts_as_moving():
    assert is_moving_command(0.1, 0.0, 0.0, CFG)
    assert is_moving_command(0.0, 0.1, 0.0, CFG)
    assert is_moving_command(0.0, 0.0, 0.1, CFG)


def test_negative_motion_counts():
    """Reversing is still moving. A latched reverse is as dangerous as a latched
    forward, and there is no rear sensor at all."""
    assert is_moving_command(-0.1, 0.0, 0.0, CFG)
    assert is_moving_command(0.0, 0.0, -0.5, CFG)


def test_nan_counts_as_moving():
    """Fail closed. A NaN reaching the firmware is not assumed harmless."""
    assert is_moving_command(float('nan'), 0.0, 0.0, CFG)
    assert is_moving_command(0.0, 0.0, float('nan'), CFG)


def test_infinity_counts_as_moving():
    assert is_moving_command(math.inf, 0.0, 0.0, CFG)


def test_epsilon_noise_is_not_moving():
    assert not is_moving_command(1e-9, -1e-9, 1e-9, CFG)


# ---------------------------------------------------------------- should_intervene
def test_intervenes_after_timeout_following_a_move():
    assert should_intervene(True, 0.6, CFG)


def test_does_not_intervene_before_timeout():
    assert not should_intervene(True, 0.4, CFG)


def test_boundary_is_strictly_greater():
    assert not should_intervene(True, 0.5, CFG)
    assert should_intervene(True, 0.5001, CFG)


def test_does_not_intervene_when_last_command_was_a_stop():
    """The car is already stopped; publishing more zeros achieves nothing and would
    fight anyone trying to start driving."""
    assert not should_intervene(False, 10.0, CFG)


def test_silent_topic_with_no_history_is_not_an_intervention():
    """Nothing has ever been commanded, so nothing is latched. Publishing zeros into a
    graph nobody is driving would just add a spurious publisher to /cmd_vel -- which the
    governor would then report as a bypasser."""
    assert not should_intervene(True, None, CFG)
    assert not should_intervene(False, None, CFG)


def test_steady_publisher_never_triggers():
    """A governor ticking at 20 Hz keeps age near 0.05 s, well under timeout."""
    for _ in range(100):
        assert not should_intervene(True, 0.05, CFG)


def test_timeout_is_configurable():
    tight = DeadmanConfig(timeout=0.1)
    assert should_intervene(True, 0.15, tight)
    assert not should_intervene(True, 0.15, CFG)
