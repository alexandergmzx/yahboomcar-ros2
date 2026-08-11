"""Tests for the SLAM TF/timing gate.

    python3 -m pytest tools/tests/ -q

These run against SYNTHETIC sample streams, with no ROS graph and no robot -- which is
the point. A gate that has never been observed to fail is not known to work, so the
negative controls here (a missing TF link, a burst of consecutive failures) matter more
than the passing cases: they are the only evidence that a real failure would actually be
caught rather than averaged away.
"""
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from slam_preflight import (                                    # noqa: E402
    FIRMWARE_ODOM_FRAME_LABEL, FrameReport, TfGateResult, duplicate_findings,
    evaluate_tf_gate, format_report, latency_stats, percentile, rate_stats)


# ------------------------------------------------------------------ small helpers
def test_percentile_interpolates_and_survives_empty():
    assert percentile([1.0, 2.0, 3.0], 50) == 2.0
    assert percentile([1.0, 2.0], 50) == 1.5
    assert percentile([5.0], 95) == 5.0
    assert math.isnan(percentile([], 50))


def test_rate_stats_recovers_a_known_rate():
    arrivals = [i / 12.0 for i in range(120)]        # exactly 12 Hz
    st = rate_stats('/scan', arrivals, contract_hz=12.0)
    assert st.count == 120
    assert st.hz == 12.0
    assert abs(st.gap_p50_ms - (1000.0 / 12.0)) < 1e-6
    assert st.within_contract


def test_rate_stats_flags_a_rate_outside_the_contract():
    arrivals = [i / 4.0 for i in range(40)]          # 4 Hz against a 12 Hz contract
    st = rate_stats('/scan', arrivals, contract_hz=12.0)
    assert not st.within_contract


def test_latency_stats_reports_the_lag_and_notices_future_stamps():
    # 80 ms of lag, the healthy end of what this link has measured
    pairs = [(t, t - 0.080) for t in [i / 12.0 for i in range(60)]]
    st = latency_stats('/scan', pairs)
    assert abs(st.p50_ms - 80.0) < 1e-6
    assert st.negative_count == 0
    # a stamp in the future is a clock problem, and it must not be silently absorbed
    st2 = latency_stats('/scan', pairs + [(1.0, 1.5)])
    assert st2.negative_count == 1


# --------------------------------------------------------------------- the gate
def _stream(n, ok_fn, start=0.0, hz=12.0, reason='can_transform returned False'):
    return [(start + i / hz, ok_fn(i), '' if ok_fn(i) else reason) for i in range(n)]


def test_clean_stream_passes_the_gate():
    samples = _stream(720, lambda i: True)           # 60 s at 12 Hz, all resolvable
    gate = evaluate_tf_gate(samples, warmup_s=5.0)
    assert gate.passed
    assert gate.success_pct == 100.0
    assert gate.longest_failure_streak == 0
    assert gate.reasons() == []


def test_missing_tf_link_fails_the_gate_and_says_so():
    """NEGATIVE CONTROL. The delivery plan requires the runtime gate to be proven
    against a deliberately missing transform -- this is that proof."""
    samples = _stream(720, lambda i: False,
                      reason="LookupException: 'odom' passed to lookupTransform "
                             'argument target_frame does not exist')
    gate = evaluate_tf_gate(samples, warmup_s=5.0)
    assert not gate.passed
    assert gate.success_pct == 0.0
    reasons = ' '.join(gate.reasons())
    assert 'below the 99% gate' in reasons
    assert 'consecutive failure run' in reasons
    # the failure sample must carry the underlying tf2 message, not just "False"
    assert gate.failures
    assert 'LookupException' in gate.failures[0][1]


def test_warmup_failures_are_discarded_not_counted():
    """An empty TF buffer legitimately cannot answer. Counting that as a SLAM defect
    would make the gate cry wolf, which is how gates get ignored."""
    # first 5 s all fail (buffer filling), everything after resolves
    samples = _stream(720, lambda i: (i / 12.0) >= 5.0)
    gate = evaluate_tf_gate(samples, warmup_s=5.0)
    assert gate.passed
    assert gate.warmup_discarded == 60          # 5 s at 12 Hz
    assert gate.success_pct == 100.0


def test_scattered_failures_pass_but_the_same_count_in_a_burst_fails():
    """The two clauses are independent ON PURPOSE: 1% of scans failing at random is a
    different world from 1% failing all at once, and only the burst empties a
    message-filter queue. This test is the whole argument for the streak clause."""
    n = 720                                  # 660 scored after the 5 s warm-up
    # 4 failures scattered every 150th scan -> 99.4%, no run longer than 1
    scattered = _stream(n, lambda i: not (i % 150 == 0 and i > 60))
    g1 = evaluate_tf_gate(scattered, warmup_s=5.0)
    assert g1.succeeded == g1.attempted - 4
    assert g1.success_pct >= 99.0
    assert g1.longest_failure_streak == 1
    assert g1.passed

    # the SAME number of failures, consecutive
    burst = _stream(n, lambda i: not (300 <= i < 304))
    g2 = evaluate_tf_gate(burst, warmup_s=5.0)
    assert g2.succeeded == g2.attempted - 4   # identical loss...
    assert g2.success_pct >= 99.0             # ...so the percentage clause passes it
    assert g2.longest_failure_streak == 4
    assert not g2.passed                      # and the streak clause catches it
    assert 'consecutive failure run' in ' '.join(g2.reasons())


def test_an_empty_stream_is_a_failure_not_a_vacuous_pass():
    gate = evaluate_tf_gate([], warmup_s=5.0)
    assert not gate.passed
    assert 'no scans' in ' '.join(gate.reasons())


def test_streak_start_time_is_reported_for_the_worst_run():
    samples = _stream(720, lambda i: not (240 <= i < 250))
    gate = evaluate_tf_gate(samples, warmup_s=5.0)
    assert gate.longest_failure_streak == 10
    assert abs(gate.streak_started_at - 240 / 12.0) < 1e-6


# -------------------------------------------------------------------- the frames
def test_the_firmware_odom_frame_label_is_reported_not_renamed():
    """/odom_raw really is stamped 'odom_frame' on this firmware [deserialized from
    bench-matrix-20260808-093840]. The tool must name that, and must not pretend to
    have fixed it."""
    fr = FrameReport(odom_raw_frame=FIRMWARE_ODOM_FRAME_LABEL,
                     odom_raw_child='base_footprint',
                     odom_frame='odom', odom_child='base_footprint',
                     scan_frame='laser_frame')
    findings = fr.findings()
    assert len(findings) == 1
    assert 'odom_frame' in findings[0]
    assert 'NOT auto-renamed' in findings[0]
    assert 'robot_localization' in findings[0]


def test_matching_frames_produce_no_findings():
    fr = FrameReport(odom_raw_frame='odom', odom_raw_child='base_footprint',
                     odom_frame='odom', odom_child='base_footprint',
                     scan_frame='laser_frame')
    assert fr.findings() == []


def test_a_wrong_scan_frame_is_its_own_finding():
    fr = FrameReport(scan_frame='lidar_link')
    assert any('lidar_link' in f for f in fr.findings())


def test_two_tf_broadcasters_are_called_out():
    """The failure this repo already met: base_node_X3 alongside the EKF, both sending
    odom->base_footprint."""
    dups = duplicate_findings({'/tf': ['ekf_filter_node', 'base_node_X3'],
                               '/scan': ['YB_Car_Node']})
    assert len(dups) == 1
    assert 'fight' in dups[0]
    assert 'base_node_X3' in dups[0]


def test_a_single_publisher_per_topic_is_silent():
    assert duplicate_findings({'/tf': ['ekf_filter_node'],
                               '/odom': ['ekf_filter_node']}) == []


# -------------------------------------------------------------------- the report
def test_the_report_leads_with_fail_and_names_every_reason():
    gate = evaluate_tf_gate(_stream(720, lambda i: False), warmup_s=5.0)
    text, ok = format_report(gate, [], [], FrameReport(), [], '')
    assert not ok
    assert 'FAIL:' in text
    assert 'below the 99% gate' in text
    # and it must steer away from the wrong fix
    assert 'never reach the matcher' in text


def test_a_passing_report_refuses_to_claim_slam_works():
    gate = evaluate_tf_gate(_stream(720, lambda i: True), warmup_s=5.0)
    text, ok = format_report(gate, [], [], FrameReport(), [], '')
    assert ok
    assert 'PASS' in text
    assert 'not a SLAM result and not a map' in text


def test_gate_result_is_json_serialisable_for_the_run_report():
    from dataclasses import asdict
    import json
    gate = evaluate_tf_gate(_stream(120, lambda i: True), warmup_s=1.0)
    json.dumps(asdict(gate))            # must not raise
    assert isinstance(gate, TfGateResult)
