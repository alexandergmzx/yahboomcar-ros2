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
    # gyro_span present, so the yaw figure means something and straightness is judgeable.
    bad = {'odom_m': 1.8, 'k': 1.004, 'yaw_change_rad': math.radians(12),
           'abs_yaw_rad': math.radians(12), 'gyro_span_rad_s': 0.5}
    assert any('curved' in p for p in mb.calibration_problems(bad))


def test_implausible_k_is_rejected():
    bad = {'odom_m': 1.8, 'k': 1.9, 'yaw_change_rad': 0.0}
    assert any('implausible' in p for p in mb.calibration_problems(bad))


def test_a_good_calibration_passes():
    good = {'odom_m': 1.83, 'k': 1.004, 'yaw_change_rad': math.radians(0.8),
            'abs_yaw_rad': math.radians(1.2), 'gyro_span_rad_s': 0.08}
    assert mb.calibration_problems(good) == []


# ---- the gyro gate ---------------------------------------------------------------
# These exist because a DEAD gyro produced the best-looking calibration the tool could
# make: it publishes exactly 0.000000 on all three axes when it fails -- confirmed in 3
# of 5 informative bags -- so it integrates to a flawless zero yaw, which the old gate
# read as a perfectly straight push. Audit finding.
def test_dead_gyro_is_not_mistaken_for_a_straight_push():
    dead = {'odom_m': 1.83, 'k': 1.02, 'yaw_change_rad': 0.0,
            'abs_yaw_rad': 0.0, 'gyro_span_rad_s': 0.0}
    probs = mb.calibration_problems(dead)
    assert any('flat' in p or 'failure signature' in p for p in probs), probs


def test_calibration_without_gyro_evidence_is_refused():
    """Pre-dates gyro_span_rad_s. A dead gyro and a straight push are indistinguishable
    in such a record, so it cannot certify anything -- which correctly invalidates the
    five calibrations recorded before the field existed."""
    old = {'odom_m': 1.83, 'k': 1.02, 'yaw_change_rad': 0.0}
    assert any('gyro-span' in p for p in mb.calibration_problems(old))


def test_s_shaped_push_is_rejected_though_net_yaw_is_zero():
    """Two opposite turns cancel to ~0 net while the wheels trace two arcs and the tape
    measures a chord. Net yaw alone can never see this."""
    snake = {'odom_m': 1.83, 'k': 1.02, 'yaw_change_rad': math.radians(1),
             'abs_yaw_rad': math.radians(40), 'gyro_span_rad_s': 0.9}
    assert any('snaked' in p for p in mb.calibration_problems(snake))


# ---- grouping and the envelope ---------------------------------------------------
def test_runs_group_by_commanded_speed_not_measured():
    """Physically exact data: d = T*v + v^2/(2a), T=0.25, a=1.5. Grouping on the MEASURED
    speed made 15 runs into 9 "speeds" of one run each, so enough_runs_per_speed could
    never pass on real data -- the fit recovered the right answer and refused it."""
    T, a = 0.25, 1.5
    runs = []
    for cmd, meas in ((0.05, (0.049, 0.050, 0.051, 0.0505, 0.0495)),
                      (0.10, (0.099, 0.100, 0.101, 0.1005, 0.0995)),
                      (0.20, (0.199, 0.200, 0.201, 0.2005, 0.1995))):
        for v in meas:
            runs.append({'commanded_speed_m_s': cmd, 'measured_speed_m_s': v,
                         'stop_distance_m': T * v + v * v / (2 * a)})
    f = mb.fit(runs, n_boot=200)
    assert f['distinct_speeds'] == [0.05, 0.10, 0.20]
    assert f['identifiable'], f['failed_checks']
    assert f['T_stop_s'] == pytest.approx(T, abs=0.01)
    assert f['decel_m_s2'] == pytest.approx(a, rel=0.05)


def test_envelope_refuses_when_the_bias_sweep_turns_unphysical():
    """A negative T_stop under 1 mm of assumed bias means the T_stop/`a` split is not
    determined by this data. That was SKIPPED, making the envelope 'the worst of the
    results that happened to be physical'."""
    T, a = 0.25, 1.5
    runs = [{'commanded_speed_m_s': c, 'measured_speed_m_s': c,
             'stop_distance_m': T * c + c * c / (2 * a)}
            for c in (0.05, 0.05, 0.05, 0.10, 0.10, 0.10, 0.20, 0.20, 0.20)]
    f = mb.fit(runs, n_boot=200)
    assert mb.envelope(f, 0.10) is not None
    f['bias_sweep'][0]['T_stop_s'] = -0.295
    assert mb.envelope(f, 0.10) is None


def test_runs_from_another_calibration_are_excluded():
    store = {'odom_scale_k': 1.02, 'calibrations': [{'k': 1.02}], 'runs': [
        {'odom_scale_k': 1.02, 'stop_distance_m': 0.02},
        {'odom_scale_k': 0.94, 'stop_distance_m': 0.05},
        {'stop_distance_m': 0.06},
    ]}
    keep, skip, k = mb.runs_for_active_calibration(store)
    assert k == 1.02
    assert len(keep) == 1
    assert len(skip) == 2, 'a run with no k recorded is foreign, not assumed compatible' 


# ---- calibration provenance ------------------------------------------------------
# Two audit rounds of the same defect. Round 4 fixed run-matching by float k; round 5
# found the SELECTOR still floating and --use-calibration setting k without cal_id, so
# later runs were stamped with the identity of a calibration they were not computed
# under -- corrupted provenance that looks healthy. These pin all of it.
def test_active_calibration_selects_by_id_not_by_float():
    """Two sessions can legitimately share a scale factor; the id cannot collide."""
    store = {'odom_scale_k': 1.02, 'active_cal_id': 'B', 'calibrations': [
        {'cal_id': 'A', 'k': 1.02, 'floor': 'kitchen'},
        {'cal_id': 'B', 'k': 1.02, 'floor': 'garage'},
    ]}
    assert mb.active_calibration(store)['floor'] == 'garage'


def test_dangling_active_cal_id_returns_none_rather_than_guessing():
    store = {'odom_scale_k': 1.02, 'active_cal_id': 'GONE',
             'calibrations': [{'cal_id': 'A', 'k': 1.02}]}
    assert mb.active_calibration(store) is None


def test_legacy_store_without_ids_still_resolves_by_float():
    store = {'odom_scale_k': 1.02, 'calibrations': [{'k': 1.02, 'legacy': True}]}
    assert mb.active_calibration(store)['legacy'] is True


def test_runs_match_by_cal_id_never_by_coincident_k():
    """The float path may not resurrect once ids exist: a run recorded under cal A must
    not pool with active cal B just because both measured k = 1.02."""
    store = {'odom_scale_k': 1.02, 'active_cal_id': 'B',
             'calibrations': [{'cal_id': 'A', 'k': 1.02}, {'cal_id': 'B', 'k': 1.02}],
             'runs': [
                 {'cal_id': 'A', 'odom_scale_k': 1.02, 'stop_distance_m': 0.02},
                 {'cal_id': 'B', 'odom_scale_k': 1.02, 'stop_distance_m': 0.03},
                 {'odom_scale_k': 1.02, 'stop_distance_m': 0.04},   # pre-id legacy
             ]}
    keep, skip, _ = mb.runs_for_active_calibration(store)
    assert len(keep) == 1 and keep[0]['stop_distance_m'] == 0.03
    assert len(skip) == 2, 'same-k runs from another calibration must be foreign'
