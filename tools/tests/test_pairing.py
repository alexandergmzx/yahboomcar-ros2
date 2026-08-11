"""Tests for the shared nearest-neighbour pairing (tools/_pairing.py).

The tie-deadlock regression is the reason this module exists: audit
2026-08-10 measured 8/7,541 samples paired by the old strict-< loop on a
bag whose recv-time batching produced duplicate stamps.
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from _pairing import nearest_pairs                              # noqa: E402


def test_plain_nearest_neighbour():
    a = [(0.0, 'a0'), (1.0, 'a1'), (2.0, 'a2')]
    b = [(0.05, 'b0'), (1.02, 'b1'), (2.10, 'b2')]
    assert nearest_pairs(a, b) == [('a0', 'b0'), ('a1', 'b1'), ('a2', 'b2')]


def test_tie_run_does_not_deadlock():
    # THE regression: a run of identical timestamps early in b. The old
    # strict-< advance pinned the cursor at the first tie element forever,
    # so every later a-sample failed the closeness gate.
    b = [(0.0, 'dup1'), (0.0, 'dup2'), (0.0, 'dup3')] + \
        [(t / 10.0, f'b{t}') for t in range(1, 100)]
    a = [(t / 10.0, f'a{t}') for t in range(0, 100, 5)]
    pairs = nearest_pairs(a, b)
    assert len(pairs) == len(a)          # old code: 1 pair, then starvation


def test_max_dt_gate_still_rejects_far_samples():
    a = [(0.0, 'a'), (5.0, 'far')]
    b = [(0.01, 'b')]
    assert nearest_pairs(a, b, max_dt=0.15) == [('a', 'b')]


def test_all_duplicate_b_stamps():
    # Degenerate but must not hang or crash: b is one big tie run.
    b = [(1.0, f'd{i}') for i in range(50)]
    a = [(1.0, 'a0'), (1.1, 'a1'), (9.0, 'a2')]
    pairs = nearest_pairs(a, b)
    assert len(pairs) == 2               # a2 is out of gate; no deadlock
