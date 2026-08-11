"""Nearest-neighbour timestamp pairing for bag analysis. ONE definition.

Extracted from check_odom_vs_imu.py after the 2026-08-10 audit caught its
monotonic advance deadlocking on TIMESTAMP TIES: the old loop only advanced
on STRICTLY closer (`<`), so a run of equal stamps — which mcap recv-time
batching produces routinely — left the cursor pinned at the first element
of the tie forever, and every later sample failed the 0.15 s closeness gate.
Measured: 8 of 7,541 odometry samples paired on bag 20260810-202951, on data
where nearly all should pair. Advancing on `<=` walks through tie runs.
"""
from __future__ import annotations


def nearest_pairs(a_series, b_series, max_dt: float = 0.15):
    """Pair each (t, value) of `a_series` with the nearest-in-time entry of
    `b_series` (both time-sorted). -> list of (a_value, b_value).

    Monotonic two-pointer: O(len(a) + len(b)). `<=` is the tie fix — equal
    distance advances, so duplicate timestamps cannot pin the cursor.
    """
    pairs = []
    j = 0
    for t, va in a_series:
        while j + 1 < len(b_series) and \
                abs(b_series[j + 1][0] - t) <= abs(b_series[j][0] - t):
            j += 1
        if abs(b_series[j][0] - t) < max_dt:
            pairs.append((va, b_series[j][1]))
    return pairs
