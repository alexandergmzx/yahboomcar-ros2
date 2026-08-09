"""Tests for the map scorer.

    python3 -m pytest tools/tests/ -q

Synthetic maps only -- no robot, no SLAM. The three cases the delivery plan requires as
negative controls (corrupted artifact, near-empty map, well-formed map) are here as
tests as well as in `score_slam_map.py --self-test`, so they fail a test run and not
only an operator's eyes.
"""
import os
import sys

import numpy as np
import pytest
import yaml

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from score_slam_map import (                                    # noqa: E402
    MapLoadError, UNKNOWN_VALUE, _write_synthetic, best_alignment_iou,
    duplicate_wall_extent_m, largest_gap_in_wall, load_map, measure_span, read_pgm,
    score_map, score_repeatability, wall_thickness_m)

RES = 0.05


def write_map(dirpath, name, px, resolution=RES, origin=(-2.5, -2.5, 0.0), **extra):
    """Write a PGM+YAML pair from a y-up pixel grid. -> yaml path."""
    pgm = os.path.join(dirpath, f'{name}.pgm')
    with open(pgm, 'wb') as f:
        f.write(f'P5\n{px.shape[1]} {px.shape[0]}\n255\n'.encode())
        f.write(px[::-1].tobytes())          # PGM row 0 is max y
    meta = {'image': f'{name}.pgm', 'resolution': resolution,
            'origin': list(origin), 'negate': 0,
            'occupied_thresh': 0.65, 'free_thresh': 0.25}
    meta.update(extra)
    ypath = os.path.join(dirpath, f'{name}.yaml')
    with open(ypath, 'w') as f:
        yaml.safe_dump(meta, f)
    return ypath


def room(n=100, lo=10, hi=90, thickness=1, doorway=None, doubled=False):
    """A y-up canvas holding a rectangular room. Options reproduce specific defects."""
    px = np.full((n, n), UNKNOWN_VALUE, dtype=np.uint8)
    px[lo:hi + 1, lo:hi + 1] = 254
    for t in range(thickness):
        px[lo + t, lo:hi + 1] = 0
        px[hi - t, lo:hi + 1] = 0
        px[lo:hi + 1, lo + t] = 0
        px[lo:hi + 1, hi - t] = 0
    if doubled:                              # the same wall drawn twice, 3 cells apart
        px[lo + 3, lo:hi + 1] = 0
    if doorway:
        start, width = doorway
        px[hi, start:start + width] = 254
    return px


# ------------------------------------------------- negative control 1: corrupted
def test_truncated_pgm_is_rejected_not_silently_empty(tmp_path):
    yml = os.path.join(tmp_path, 'x.yaml')
    with open(yml, 'w') as f:
        yaml.safe_dump({'image': 'x.pgm', 'resolution': RES,
                        'origin': [0.0, 0.0, 0.0]}, f)
    with open(os.path.join(tmp_path, 'x.pgm'), 'wb') as f:
        f.write(b'P5\n100 100\n255\n' + b'\x00' * 50)
    rep = score_map(yml)
    assert not rep.passed
    assert any('truncated' in e for e in rep.errors)


def test_missing_image_file_is_named(tmp_path):
    yml = os.path.join(tmp_path, 'x.yaml')
    with open(yml, 'w') as f:
        yaml.safe_dump({'image': 'nope.pgm', 'resolution': RES,
                        'origin': [0.0, 0.0, 0.0]}, f)
    rep = score_map(yml)
    assert not rep.passed
    assert any('does not exist' in e for e in rep.errors)


def test_empty_yaml_is_rejected(tmp_path):
    yml = os.path.join(tmp_path, 'empty.yaml')
    open(yml, 'w').close()
    rep = score_map(yml)
    assert not rep.passed
    assert any('empty' in e or 'mapping' in e for e in rep.errors)


def test_a_non_pgm_image_is_rejected(tmp_path):
    with open(os.path.join(tmp_path, 'x.pgm'), 'wb') as f:
        f.write(b'\x89PNG\r\n\x1a\n' + b'\x00' * 100)
    yml = os.path.join(tmp_path, 'x.yaml')
    with open(yml, 'w') as f:
        yaml.safe_dump({'image': 'x.pgm', 'resolution': RES,
                        'origin': [0.0, 0.0, 0.0]}, f)
    with pytest.raises(MapLoadError):
        load_map(yml)


# ------------------------------------------------ negative control 2: near-empty
def test_the_near_empty_map_that_shipped_before_is_rejected(tmp_path):
    """THE regression that motivates this tool. 16 occupied pixels with correct 4 m
    outer dimensions was previously read as a successful map."""
    rep = score_map(_write_synthetic(str(tmp_path), 'near_empty'))
    assert not rep.passed
    occ = next(r for r in rep.rows if r.metric == 'occupied pixels')
    assert int(occ.measured) < 20
    assert not occ.passed


def test_bounding_box_alone_can_never_carry_a_pass(tmp_path):
    """Bounding-box dimensions are specifically forbidden as a success criterion, so
    the row must exist for context but be marked non-required."""
    rep = score_map(_write_synthetic(str(tmp_path), 'near_empty'))
    bbox = next(r for r in rep.rows if r.metric.startswith('bounding box'))
    assert bbox.required_row is False
    assert '4.0' in bbox.measured          # dimensions ARE right...
    assert not rep.passed                   # ...and the map still fails


def test_an_all_unknown_map_fails_on_known_cells(tmp_path):
    px = np.full((100, 100), UNKNOWN_VALUE, dtype=np.uint8)
    px[50, 50:60] = 0
    rep = score_map(write_map(str(tmp_path), 'sparse', px))
    known = next(r for r in rep.rows if r.metric == 'known cells')
    assert not known.passed


# ---------------------------------------------- negative control 3: well-formed
def test_a_well_formed_map_passes_every_required_row(tmp_path):
    ref = {'spans': [{'name': 'x', 'axis': 'x', 'metres': 4.05},
                     {'name': 'y', 'axis': 'y', 'metres': 4.05}],
           'features': [{'name': 'doorway', 'side': 'max_y', 'width_m': 0.80}]}
    facts = {'loop_closure': {'position_error_m': 0.04, 'heading_error_deg': 1.2},
             'runtime_health': {'queue_overflows_after_warmup': 0, 'map_updates': 220}}
    rep = score_map(_write_synthetic(str(tmp_path), 'good'), ref, facts)
    assert rep.passed, rep.table()
    assert all(r.passed for r in rep.rows if r.required_row)


# --------------------------------------------------------------- the geometry
def test_span_measures_the_room_and_a_wrong_tape_value_fails(tmp_path):
    m = load_map(write_map(str(tmp_path), 'r', room()))
    assert measure_span(m, 'x') == pytest.approx(4.05, abs=1e-9)
    ok = score_map(write_map(str(tmp_path), 'r2', room()),
                   {'spans': [{'name': 'x', 'axis': 'x', 'metres': 4.05}]})
    assert next(r for r in ok.rows if r.metric == 'span x').passed
    bad = score_map(write_map(str(tmp_path), 'r3', room()),
                    {'spans': [{'name': 'x', 'axis': 'x', 'metres': 6.00}]})
    assert not next(r for r in bad.rows if r.metric == 'span x').passed


def test_wall_thickness_grows_with_a_smeared_wall(tmp_path):
    thin = load_map(write_map(str(tmp_path), 'thin', room(thickness=1)))
    fat = load_map(write_map(str(tmp_path), 'fat', room(thickness=4)))
    assert wall_thickness_m(thin) < wall_thickness_m(fat)
    assert wall_thickness_m(thin) <= 0.12
    assert wall_thickness_m(fat) > 0.12


def test_a_doubled_wall_is_detected_and_a_clean_one_is_not(tmp_path):
    clean = load_map(write_map(str(tmp_path), 'clean', room()))
    smear = load_map(write_map(str(tmp_path), 'smear', room(doubled=True)))
    assert duplicate_wall_extent_m(clean) <= 0.20
    assert duplicate_wall_extent_m(smear) > 0.20


def test_a_doorway_is_found_with_its_width_and_offset(tmp_path):
    m = load_map(write_map(str(tmp_path), 'door', room(doorway=(40, 16))))
    width, offset = largest_gap_in_wall(m, 'max_y')
    assert width == pytest.approx(0.80, abs=0.06)
    assert offset == pytest.approx((40 - 10) * RES, abs=0.06)


def test_a_wall_with_no_gap_reports_absent(tmp_path):
    m = load_map(write_map(str(tmp_path), 'solid', room()))
    width, _ = largest_gap_in_wall(m, 'max_y')
    assert not np.isfinite(width)


def test_a_missing_doorway_fails_its_row(tmp_path):
    rep = score_map(write_map(str(tmp_path), 'nodoor', room()),
                    {'features': [{'name': 'doorway', 'side': 'max_y',
                                   'width_m': 0.80}]})
    row = next(r for r in rep.rows if 'doorway' in r.metric)
    assert not row.passed
    assert row.measured == 'absent'


# ------------------------------------------------------------------ run facts
def test_loop_closure_and_queue_overflow_rows_fail_when_they_should(tmp_path):
    facts = {'loop_closure': {'position_error_m': 0.42, 'heading_error_deg': 11.0},
             'runtime_health': {'queue_overflows_after_warmup': 135, 'map_updates': 0}}
    rep = score_map(_write_synthetic(str(tmp_path), 'good'), None, facts)
    named = {r.metric: r.passed for r in rep.rows}
    assert not named['loop closure position']
    assert not named['loop closure heading']
    assert not named['scan queue overflows']
    assert not named['map updates during motion']
    assert not rep.passed


# --------------------------------------------------------------- repeatability
def test_identical_maps_align_perfectly(tmp_path):
    a = load_map(write_map(str(tmp_path), 'a', room()))
    b = load_map(write_map(str(tmp_path), 'b', room()))
    v, _, _, _ = best_alignment_iou(a, b)
    assert v == pytest.approx(1.0)


def test_a_translated_map_still_aligns(tmp_path):
    """Two independent runs have unrelated origins; alignment must absorb that."""
    a = load_map(write_map(str(tmp_path), 'a2', room(lo=10, hi=90)))
    b = load_map(write_map(str(tmp_path), 'b2', room(lo=14, hi=94)))
    v, _, _, _ = best_alignment_iou(a, b)
    assert v > 0.9


def test_a_different_room_fails_repeatability(tmp_path):
    ref = {'spans': [{'name': 'x', 'axis': 'x', 'metres': 4.05}]}
    a = write_map(str(tmp_path), 'a3', room(lo=10, hi=90))     # 4.05 m
    b = write_map(str(tmp_path), 'b3', room(lo=10, hi=60))     # 2.55 m
    rep = score_repeatability(a, b, ref)
    assert not rep.passed


def test_repeatability_reports_unreadable_maps_rather_than_scoring_them(tmp_path):
    good = _write_synthetic(str(tmp_path), 'good')
    rep = score_repeatability(good, os.path.join(str(tmp_path), 'missing.yaml'))
    assert not rep.passed
    assert rep.errors


# ------------------------------------------------------------------- plumbing
def test_pgm_reader_handles_comment_lines(tmp_path):
    path = os.path.join(tmp_path, 'c.pgm')
    with open(path, 'wb') as f:
        f.write(b'P5\n# made by map_saver\n4 2\n255\n' + bytes(range(8)))
    arr = read_pgm(path)
    assert arr.shape == (2, 4)


def test_unknown_is_classified_before_thresholds(tmp_path):
    """With the stock free_thresh 0.25, the canonical 205 computes to p=0.196 and would
    score as FREE -- which would make an unexplored map read as fully explored."""
    px = np.full((10, 10), UNKNOWN_VALUE, dtype=np.uint8)
    m = load_map(write_map(str(tmp_path), 'u', px))
    assert m.unknown.all()
    assert not m.free.any()


def test_score_json_is_serialisable(tmp_path):
    import json
    rep = score_map(_write_synthetic(str(tmp_path), 'good'))
    json.dumps(rep.to_json())
