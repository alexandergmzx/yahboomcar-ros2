"""Tests pinning the braking fit's refusal to bless impossible data.

Every case here was supplied by an external audit that fed the tool physically impossible
stopping data and got `identifiable: True` back, plus a "conservative" 33 mm envelope at
0.20 m/s from a dataset whose own worst observed stop was 50 mm.

The failure was not that the maths was wrong. It was that `identifiable` checked only the
NUMBER and SPREAD of speeds -- necessary conditions, treated as sufficient -- so a curve
was fitted through data that could not describe any robot, and the result was handed to a
speed-release gate.
"""
import importlib.util
import math
import os
import sys

import pytest


def _repo_root():
    """Walk up until tools/measure_braking.py is found.

    Counting dirname() calls is how a test ends up looking for
    yahboomcar_ws/tools/measure_braking.py, which is what happened first.
    """
    d = os.path.dirname(os.path.abspath(__file__))
    while d != '/':
        if os.path.exists(os.path.join(d, 'tools', 'measure_braking.py')):
            return d
        d = os.path.dirname(d)
    raise RuntimeError('repo root not found from ' + __file__)


_ROOT = _repo_root()
_spec = importlib.util.spec_from_file_location(
    'measure_braking', os.path.join(_ROOT, 'tools', 'measure_braking.py'))
mb = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(mb)


def runs(pairs):
    return [{'measured_speed_m_s': v, 'stop_distance_m': d} for v, d in pairs]


def physical(T=0.25, a=1.5, speeds=(0.05, 0.10, 0.15, 0.20, 0.30), n=4):
    return runs([(v, v * T + v * v / (2 * a)) for v in speeds for _ in range(n)])


# --------------------------------------------------- the audit's own cases
def test_rejects_distance_that_shrinks_as_speed_rises():
    """A faster robot cannot stop in a shorter distance. This returned
    identifiable: True with T_stop = 5.109 s."""
    f = mb.fit(runs([(0.05, 0.30)] * 4 + [(0.10, 0.20)] * 4 + [(0.20, 0.10)] * 4))
    assert not f['identifiable']
    assert 'monotonic_in_speed' in f['failed_checks']
    assert mb.envelope(f, 0.20) is None


def test_rejects_negative_dead_time():
    """The robot does not begin braking before it is told to. This returned
    identifiable: True with T_stop = -0.295 s."""
    f = mb.fit(runs([(0.05, 0.001)] * 4 + [(0.15, 0.05)] * 4 + [(0.30, 0.30)] * 4))
    assert f['T_stop_s'] < 0
    assert not f['identifiable']
    assert 't_stop_positive' in f['failed_checks']


def test_no_envelope_is_produced_from_an_unusable_fit():
    """The 33 mm envelope came from a fit that should never have produced one. A number
    derived from impossible data is worse than no number, because it gets used."""
    for bad in (runs([(0.05, 0.30)] * 4 + [(0.10, 0.20)] * 4 + [(0.20, 0.10)] * 4),
                runs([(0.05, 0.001)] * 4 + [(0.15, 0.05)] * 4 + [(0.30, 0.30)] * 4)):
        f = mb.fit(bad)
        for v in (0.05, 0.10, 0.20, 0.30):
            assert mb.envelope(f, v) is None


# ------------------------------------------------- other invalid shapes
def test_rejects_too_few_runs_per_speed():
    f = mb.fit(runs([(0.05, 0.02), (0.10, 0.05), (0.20, 0.15)]))
    assert not f['identifiable']
    assert 'enough_runs_per_speed' in f['failed_checks']


def test_rejects_too_few_distinct_speeds():
    f = mb.fit(physical(speeds=(0.05, 0.10)))
    assert not f['identifiable']
    assert 'enough_speeds' in f['failed_checks']


def test_rejects_narrow_speed_spread():
    f = mb.fit(physical(speeds=(0.20, 0.24, 0.28)))
    assert not f['identifiable']
    assert 'enough_spread' in f['failed_checks']


def test_rejects_negative_deceleration():
    f = mb.fit(runs([(0.05, 0.10)] * 4 + [(0.15, 0.28)] * 4 + [(0.30, 0.50)] * 4))
    if f['decel_m_s2'] is None:
        assert not f['identifiable']
        assert 'decel_positive' in f['failed_checks']


# ------------------------------------------------------- good data still works
def test_accepts_physically_consistent_data():
    f = mb.fit(physical())
    assert f['identifiable'], f['failed_checks']
    assert f['T_stop_s'] == pytest.approx(0.25, abs=0.01)
    assert f['decel_m_s2'] == pytest.approx(1.5, abs=0.05)


# ------------------------------------------- the envelope's hard floor
@pytest.mark.parametrize('v', [0.05, 0.10, 0.15, 0.20, 0.30])
def test_envelope_never_predicts_a_stop_shorter_than_one_observed(v):
    """The check that needs no theory: a model may not predict a stop shorter than one
    that has already happened. It is what would have caught the 33 mm case regardless of
    anything else being wrong."""
    data = physical()
    f = mb.fit(data)
    e = mb.envelope(f, v)
    observed = [r['stop_distance_m'] for r in data if r['measured_speed_m_s'] <= v + 1e-9]
    if observed and e is not None:
        assert e >= max(observed), f'{e*1000:.0f} mm < observed {max(observed)*1000:.0f} mm'


def test_envelope_grows_with_speed():
    f = mb.fit(physical())
    es = [mb.envelope(f, v) for v in (0.05, 0.10, 0.20, 0.30)]
    assert all(b >= a for a, b in zip(es, es[1:])), es


def test_envelope_includes_the_bias_sweep_worst_case():
    """Bootstrap cannot see a bias common to every run, so the envelope must take the
    worst of the bias-swept fits too -- 0.5 mm of it once moved fitted `a` from 1.0 to
    2.5."""
    f = mb.fit(physical())
    boot_only = None
    import numpy as np
    bt, bb = f['_boot']
    boot_only = float(np.percentile(bt * 0.30 + bb * 0.09, 95)) + mb.MARGIN_C
    assert mb.envelope(f, 0.30) >= boot_only - 1e-9


# ---------------------------------------------- calibration must be enforced
def test_calibration_with_no_imu_record_is_rejected():
    """All five calibrations recorded before the gyro check exist and say nothing about
    whether the push was straight."""
    bad = {'odom_m': 1.8, 'k': 1.004, 'yaw_change_rad': None}
    assert any('IMU' in p for p in mb.calibration_problems(bad))


def test_short_calibration_is_rejected():
    bad = {'odom_m': 0.5, 'k': 1.08, 'yaw_change_rad': 0.0}
    assert any('push only' in p for p in mb.calibration_problems(bad))


def test_curved_calibration_is_rejected():
    bad = {'odom_m': 1.8, 'k': 1.004, 'yaw_change_rad': math.radians(12)}
    assert any('curved' in p for p in mb.calibration_problems(bad))


def test_implausible_k_is_rejected():
    bad = {'odom_m': 1.8, 'k': 1.9, 'yaw_change_rad': 0.0}
    assert any('implausible' in p for p in mb.calibration_problems(bad))


def test_a_good_calibration_passes():
    good = {'odom_m': 1.83, 'k': 1.004, 'yaw_change_rad': math.radians(0.8)}
    assert mb.calibration_problems(good) == []
