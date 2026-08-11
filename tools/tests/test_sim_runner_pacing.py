"""Tests for the render-pacing trim policy. Pure math, no Isaac.

    python3 -m pytest tools/tests/test_sim_runner_pacing.py -q

The fixture sequences are the four RECORDED runaways (simctl-isaac.log,
sessions of 2026-08-09/10): pacing walked 23.08->3.43, 6.57->2.92,
8.22->2.91, 6.57->3.0 while the measured /scan rate sat ~12.8-13.4 Hz.
The new policy must refuse that walk twice over: the floor at SCAN_HZ,
and the divergence guard that stops dividing an unresponsive plant.
"""
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from sim_runner import trim_render_pacing                       # noqa: E402

SCAN_HZ = 12.0


def run_sequence(start_hz, measured_seq, blend=0.5):
    log, notes = [], []
    hz = start_hz
    for m in measured_seq:
        hz, note = trim_render_pacing(hz, m, SCAN_HZ, blend, log)
        notes.append(note)
    return hz, notes


def test_floor_binds_at_scan_hz():
    # The recorded 2026-08-09 runaway: measured stuck ~12.8-13.4 while the
    # old loop divided pacing toward its 2.0 floor. The floor is now SCAN_HZ.
    measured = [14.4, 15.8, 15.2, 14.2, 13.8, 13.8, 13.8, 13.6, 13.4, 13.2,
                13.2, 13.0, 13.0, 12.8, 12.8, 12.8, 12.8, 12.8, 12.8, 12.8]
    hz, _ = run_sequence(23.08, measured, blend=1.0)
    assert hz >= SCAN_HZ


def test_downward_runaway_is_structurally_impossible_now():
    # With the floor at SCAN_HZ the historical downward walk cannot happen at
    # all — pacing pins at the floor instead. (This is why the divergence
    # guard's remaining live regime is UPWARD; see the next test.)
    measured = [13.4, 13.2, 13.2, 13.0, 13.0, 12.8, 12.8, 12.8]
    hz, _ = run_sequence(9.0, measured, blend=1.0)
    assert hz == SCAN_HZ


def test_divergence_guard_fires_on_upward_runaway():
    # The remaining divergence direction: a starved machine where measured
    # sits far BELOW target no matter how fast we render. Naive control
    # multiplies pacing toward the 40/s ceiling; the guard must hold instead.
    measured = [6.2, 6.0, 6.1, 6.0, 6.0, 6.1, 6.0, 6.0]
    _, notes = run_sequence(12.0, measured, blend=0.5)
    held = [n for n in notes if n]
    assert held and 'HELD' in held[0]
    assert 'proportional assumption' in held[0]


def test_healthy_convergence_unchanged():
    # A plant that genuinely responds: measured tracks pacing toward 12 Hz.
    # No hold, ends near steady state, never below the floor.
    seq = [(20.0, 15.0), (17.0, 14.0), (15.0, 13.0), (14.0, 12.4),
           (13.5, 12.1), (13.2, 12.0)]
    log, hz = [], 20.0
    for _, m in seq:
        hz, note = trim_render_pacing(hz, m, SCAN_HZ, 0.5, log)
        assert note is None
    assert SCAN_HZ <= hz <= 20.0


def test_overshoot_trims_down_toward_floor_not_through_it():
    # Measured ABOVE target with pacing already at the floor: stays at floor.
    log = []
    hz, note = trim_render_pacing(SCAN_HZ, 14.0, SCAN_HZ, 1.0, log)
    assert hz == SCAN_HZ and note is None


def test_ceiling_still_caps():
    log = []
    hz, _ = trim_render_pacing(39.0, 6.0, SCAN_HZ, 1.0, log)
    assert hz <= 40.0


def test_guard_needs_history():
    # Fewer than 4 trims: never holds (no window to judge).
    log = []
    for m in (13.0, 13.0, 13.0):
        hz, note = trim_render_pacing(5.0, m, SCAN_HZ, 1.0, log)
        assert note is None


def test_recorded_session_20260810_night():
    # The 6.57->2.92 walk from run 1, replayed against the new policy: the
    # first trim clamps to the floor and every later trim stays there. No
    # hold note is expected — the floor alone ends this failure mode.
    measured = [13.4, 13.2, 13.0, 12.8, 12.8, 12.8, 12.8, 12.8, 12.8, 12.8]
    hz, notes = run_sequence(6.57, measured, blend=0.5)
    assert hz == SCAN_HZ
    assert all(n is None for n in notes)
