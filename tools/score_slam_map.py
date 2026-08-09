#!/usr/bin/env python3
"""Score a saved occupancy map against TAPE MEASUREMENTS. Existence is not success.

    ./tools/score_slam_map.py --map physical_20260810_run1.yaml \\
        --reference docs/slam-runs/reference-geometry.example.yaml \\
        --run-facts docs/slam-runs/run-facts.example.yaml
    ./tools/score_slam_map.py --map run1.yaml --compare-map run2.yaml ...   # repeatability
    ./tools/score_slam_map.py --self-test        # negative controls, no files needed

Exits nonzero when any required row fails.

WHY THIS EXISTS
---------------
The previous SLAM pass reported maps with **14-18 occupied pixels** and correct outer
dimensions, and the correct dimensions were read as success. A 4.04 x 4.04 m bounding
box around almost nothing is not a map of a room -- it is a map of two opposite walls
glimpsed once. Bounding-box dimensions are therefore SPECIFICALLY FORBIDDEN here as a
success criterion, and `--self-test` proves this scorer rejects exactly that artifact.

Every threshold below comes from the acceptance table in docs/slam-delivery-plan.md.
They may be tightened after measurement. They may NOT be loosened after a failed run
without recording the failed value, the physical reason, and the user's approval --
which is a rule about honesty, not about numbers.

WHAT IT CANNOT DO
-----------------
`--reference` spans are measured as the occupied extent along an axis, which is valid
for the outer walls of a convex room (this robot's 4x4 m test room) and NOT valid for an
L-shaped or cluttered space. Per-wall segmentation is not implemented; a room that needs
it must say so in its run report rather than quietly scoring the wrong thing.

Unknown cells are the canonical map_saver 205 (a +/-5 band), classified as unknown
BEFORE the YAML thresholds are applied. With the stock `free_thresh: 0.25`, 205 computes
to p=0.196 and would otherwise score as free, which would make an empty map read as
100% explored -- the exact failure this tool exists to catch.
"""
import argparse
import json
import math
import os
import sys
from dataclasses import dataclass, field

import numpy as np
import yaml

# --- acceptance thresholds (docs/slam-delivery-plan.md, Phase 3) -----------------
MIN_OCCUPIED_PX = 100
MIN_KNOWN_FRACTION = 0.05
SPAN_TOL_M = 0.15
SPAN_TOL_FRAC = 0.05
FEATURE_TOL_M = 0.15
MAX_WALL_THICKNESS_M = 0.12
MAX_DUPLICATE_WALL_M = 0.20
LOOP_CLOSURE_TOL_M = 0.10
LOOP_CLOSURE_TOL_DEG = 5.0
REPEAT_SPAN_TOL_FRAC = 0.05
REPEAT_MIN_IOU = 0.65

UNKNOWN_VALUE = 205
UNKNOWN_BAND = 5


# ------------------------------------------------------------------- map loading
@dataclass
class MapData:
    occupied: np.ndarray          # bool grid, row 0 = min y (y-up)
    free: np.ndarray
    unknown: np.ndarray
    resolution: float
    origin: tuple
    path: str = ''

    @property
    def shape(self):
        return self.occupied.shape


class MapLoadError(Exception):
    """The artifact itself is unusable. Distinct from a map that loads and scores badly."""


def read_pgm(path):
    """Minimal binary (P5) PGM reader -> uint8 array. Raises MapLoadError on anything
    it cannot honestly parse, rather than returning a plausible-looking empty grid."""
    try:
        with open(path, 'rb') as f:
            data = f.read()
    except OSError as e:
        raise MapLoadError(f'cannot read image {path}: {e}')
    if not data.startswith(b'P5'):
        raise MapLoadError(f'{path} is not a binary PGM (no P5 magic)')
    fields, pos = [], 2
    while len(fields) < 3:
        while pos < len(data) and data[pos:pos + 1].isspace():
            pos += 1
        if data[pos:pos + 1] == b'#':                       # comment line
            while pos < len(data) and data[pos:pos + 1] != b'\n':
                pos += 1
            continue
        start = pos
        while pos < len(data) and not data[pos:pos + 1].isspace():
            pos += 1
        if pos == start:
            raise MapLoadError(f'{path}: truncated PGM header')
        fields.append(int(data[start:pos]))
    pos += 1                                                 # single whitespace byte
    w, h, _maxv = fields
    expected = w * h
    body = data[pos:pos + expected]
    if len(body) < expected:
        raise MapLoadError(f'{path}: PGM body truncated '
                           f'({len(body)} of {expected} bytes for {w}x{h})')
    return np.frombuffer(body, dtype=np.uint8).reshape(h, w)


def load_map(yaml_path):
    """map YAML (+ its PGM) -> MapData. Raises MapLoadError for an unusable artifact."""
    try:
        with open(yaml_path) as f:
            meta = yaml.safe_load(f)
    except OSError as e:
        raise MapLoadError(f'cannot read map yaml {yaml_path}: {e}')
    except yaml.YAMLError as e:
        raise MapLoadError(f'{yaml_path} is not valid YAML: {e}')
    if not isinstance(meta, dict) or not meta:
        raise MapLoadError(f'{yaml_path} is empty or not a mapping')
    for key in ('image', 'resolution', 'origin'):
        if key not in meta:
            raise MapLoadError(f'{yaml_path} has no {key!r} key')
    image = meta['image']
    if not os.path.isabs(image):
        image = os.path.join(os.path.dirname(os.path.abspath(yaml_path)), image)
    if not os.path.exists(image):
        raise MapLoadError(f'{yaml_path} points at {image!r}, which does not exist')
    px = read_pgm(image)
    # Row 0 of a PGM is the TOP row = max y. Flip so row 0 is min y, matching the
    # map frame's y-up convention and the origin field.
    px = px[::-1]
    occ_thresh = float(meta.get('occupied_thresh', 0.65))
    free_thresh = float(meta.get('free_thresh', 0.25))
    if int(meta.get('negate', 0)):
        px = 255 - px
    unknown = np.abs(px.astype(int) - UNKNOWN_VALUE) <= UNKNOWN_BAND
    p = (255.0 - px.astype(float)) / 255.0
    occupied = (p >= occ_thresh) & ~unknown
    free = (p <= free_thresh) & ~unknown
    return MapData(occupied=occupied, free=free, unknown=unknown,
                   resolution=float(meta['resolution']),
                   origin=tuple(meta['origin']), path=os.path.abspath(yaml_path))


# --------------------------------------------------------------------- geometry
def occupied_bbox(m):
    """-> (min_row, max_row, min_col, max_col) of occupied cells, or None."""
    rows, cols = np.nonzero(m.occupied)
    if rows.size == 0:
        return None
    return int(rows.min()), int(rows.max()), int(cols.min()), int(cols.max())


def measure_span(m, axis):
    """Occupied extent along an axis, in metres. 'x' -> columns, 'y' -> rows."""
    bb = occupied_bbox(m)
    if bb is None:
        return float('nan')
    r0, r1, c0, c1 = bb
    cells = (c1 - c0 + 1) if axis == 'x' else (r1 - r0 + 1)
    return cells * m.resolution


def _runs(mask):
    """Index ranges of consecutive True in a 1-D bool array -> [(start, stop_exclusive)]."""
    out, start = [], None
    for i, v in enumerate(mask):
        if v and start is None:
            start = i
        elif not v and start is not None:
            out.append((start, i))
            start = None
    if start is not None:
        out.append((start, len(mask)))
    return out


def wall_thickness_m(m):
    """Median thickness of the outermost occupied run on each side. -> metres.

    A smeared map has thick or doubled walls; a clean one has walls one or two cells
    across. Measured from the OUTSIDE in, per column and per row, so interior clutter
    does not dilute the statistic.
    """
    thick = []
    for grid in (m.occupied, m.occupied.T):
        for line in grid.T if grid is m.occupied else grid:
            runs = _runs(line)
            if not runs:
                continue
            thick.append(runs[0][1] - runs[0][0])
            if len(runs) > 1:
                thick.append(runs[-1][1] - runs[-1][0])
    if not thick:
        return float('nan')
    return float(np.median(thick)) * m.resolution


def duplicate_wall_extent_m(m, band_m=0.40, min_gap_cells=2):
    """Longest consecutive stretch of columns/rows showing a DOUBLED outer wall.

    "Doubled" = two separate occupied runs within `band_m` of the outermost occupied
    cell on that side, split by at least `min_gap_cells` of non-occupied. That is what
    a map smeared by an odometry jump looks like: the same wall drawn twice, a few
    centimetres apart. -> metres of the longest such stretch (0.0 when clean).
    """
    band = max(1, int(round(band_m / m.resolution)))
    worst = 0
    for grid in (m.occupied, m.occupied.T):
        n_lines = grid.shape[1] if grid is m.occupied else grid.shape[1]
        lines = (grid[:, i] for i in range(n_lines))
        flags = []
        for line in lines:
            runs = _runs(line)
            doubled = False
            if len(runs) >= 2:
                for side_runs in (runs, runs[::-1]):
                    first_edge = side_runs[0][0] if side_runs is runs else side_runs[0][1]
                    near = [r for r in runs
                            if abs(r[0] - first_edge) <= band
                            or abs(r[1] - first_edge) <= band]
                    if len(near) >= 2:
                        gaps = [b[0] - a[1] for a, b in zip(near[:-1], near[1:])]
                        if any(g >= min_gap_cells for g in gaps):
                            doubled = True
                            break
            flags.append(doubled)
        run_len = 0
        for f in flags:
            run_len = run_len + 1 if f else 0
            worst = max(worst, run_len)
    return worst * m.resolution


def largest_gap_in_wall(m, side):
    """Width and offset of the largest gap (doorway) in one outer wall, in metres.

    side: 'min_y' | 'max_y' | 'min_x' | 'max_x'. -> (width_m, offset_m) measured from
    the low end of that wall, or (nan, nan) when the wall has no gap.
    """
    bb = occupied_bbox(m)
    if bb is None:
        return float('nan'), float('nan')
    r0, r1, c0, c1 = bb
    band = max(1, int(round(0.30 / m.resolution)))
    if side in ('min_y', 'max_y'):
        rows = slice(r0, r0 + band) if side == 'min_y' else slice(r1 - band + 1, r1 + 1)
        present = m.occupied[rows, c0:c1 + 1].any(axis=0)
    else:
        cols = slice(c0, c0 + band) if side == 'min_x' else slice(c1 - band + 1, c1 + 1)
        present = m.occupied[r0:r1 + 1, cols].any(axis=1)
    gaps = _runs(~present)
    interior = [g for g in gaps if g[0] > 0 and g[1] < len(present)]
    if not interior:
        return float('nan'), float('nan')
    widest = max(interior, key=lambda g: g[1] - g[0])
    return ((widest[1] - widest[0]) * m.resolution,
            widest[0] * m.resolution)


# ----------------------------------------------------------------- repeatability
def iou(a, b):
    inter = np.logical_and(a, b).sum()
    union = np.logical_or(a, b).sum()
    return float(inter) / float(union) if union else 0.0


def best_alignment_iou(m_a, m_b, max_shift_cells=12, angles_deg=(0,)):
    """Occupied-cell IoU after a brute-force rigid alignment. -> (iou, dx, dy, deg).

    Two independent SLAM runs have unrelated map origins, so a raw IoU compares
    nothing. Both grids are cropped to their occupied bounding box and centred first;
    the search then covers small residual offsets and rotations.
    """
    from scipy import ndimage

    def crop(m):
        bb = occupied_bbox(m)
        if bb is None:
            return None
        r0, r1, c0, c1 = bb
        return m.occupied[r0:r1 + 1, c0:c1 + 1]

    a, b = crop(m_a), crop(m_b)
    if a is None or b is None:
        return 0.0, 0, 0, 0.0
    h = max(a.shape[0], b.shape[0]) + 2 * max_shift_cells + 4
    w = max(a.shape[1], b.shape[1]) + 2 * max_shift_cells + 4
    def place(g):
        canvas = np.zeros((h, w), dtype=bool)
        r = (h - g.shape[0]) // 2
        c = (w - g.shape[1]) // 2
        canvas[r:r + g.shape[0], c:c + g.shape[1]] = g
        return canvas
    ca, cb = place(a), place(b)
    best = (0.0, 0, 0, 0.0)
    for deg in angles_deg:
        rot = cb if deg == 0 else ndimage.rotate(
            cb.astype(float), deg, reshape=False, order=0) > 0.5
        for dy in range(-max_shift_cells, max_shift_cells + 1):
            for dx in range(-max_shift_cells, max_shift_cells + 1):
                shifted = np.roll(np.roll(rot, dy, axis=0), dx, axis=1)
                v = iou(ca, shifted)
                if v > best[0]:
                    best = (v, dx, dy, float(deg))
    return best


# ---------------------------------------------------------------------- scoring
@dataclass
class Row:
    metric: str
    measured: str
    required: str
    passed: bool
    required_row: bool = True


@dataclass
class ScoreReport:
    rows: list = field(default_factory=list)
    errors: list = field(default_factory=list)

    def add(self, metric, measured, required, passed, required_row=True):
        self.rows.append(Row(metric, measured, required, passed, required_row))

    @property
    def passed(self):
        return not self.errors and all(r.passed for r in self.rows if r.required_row)

    def table(self):
        if self.errors:
            head = ['UNUSABLE ARTIFACT:'] + [f'  - {e}' for e in self.errors]
        else:
            head = []
        w = max([len(r.metric) for r in self.rows] + [12])
        lines = head + ['', f'  {"metric".ljust(w)}  {"measured":<26}  '
                            f'{"required":<26}  verdict', '  ' + '-' * (w + 62)]
        for r in self.rows:
            mark = 'PASS' if r.passed else 'FAIL'
            if not r.required_row:
                mark += ' (info)'
            lines.append(f'  {r.metric.ljust(w)}  {r.measured:<26}  '
                         f'{r.required:<26}  {mark}')
        lines.append('')
        lines.append(f'  VERDICT: {"PASS" if self.passed else "FAIL"}')
        return '\n'.join(lines)

    def to_json(self):
        return {
            'passed': self.passed,
            'errors': self.errors,
            'rows': [{'metric': r.metric, 'measured': r.measured,
                      'required': r.required, 'passed': r.passed,
                      'required_row': r.required_row} for r in self.rows],
        }


def score_map(map_path, reference=None, run_facts=None):
    """The whole Phase-3 table for one map. -> ScoreReport (never raises for a bad map)."""
    rep = ScoreReport()
    try:
        m = load_map(map_path)
    except MapLoadError as e:
        rep.errors.append(str(e))
        rep.add('map artifact', 'unreadable', 'YAML + PGM resolve', False)
        return rep
    rep.add('map artifact', os.path.basename(map_path), 'YAML + PGM resolve', True)

    total = m.occupied.size
    n_occ = int(m.occupied.sum())
    known_frac = float((m.occupied | m.free).sum()) / total if total else 0.0
    rep.add('occupied pixels', f'{n_occ}', f'>= {MIN_OCCUPIED_PX}',
            n_occ >= MIN_OCCUPIED_PX)
    rep.add('known cells', f'{100 * known_frac:.1f}%',
            f'>= {100 * MIN_KNOWN_FRACTION:.0f}%', known_frac >= MIN_KNOWN_FRACTION)

    # Bounding-box size is reported for context and is NEVER a pass criterion.
    bb = occupied_bbox(m)
    if bb:
        rep.add('bounding box (context only)',
                f'{measure_span(m, "x"):.2f} x {measure_span(m, "y"):.2f} m',
                'not a criterion', True, required_row=False)

    if reference:
        for span in reference.get('spans', []) or []:
            got = measure_span(m, span['axis'])
            want = float(span['metres'])
            tol = max(SPAN_TOL_M, SPAN_TOL_FRAC * want)
            ok = math.isfinite(got) and abs(got - want) <= tol
            rep.add(f'span {span["name"]}', f'{got:.3f} m',
                    f'{want:.3f} +/- {tol:.3f} m', ok)
        for feat in reference.get('features', []) or []:
            width, offset = largest_gap_in_wall(m, feat['side'])
            want_w = float(feat['width_m'])
            ok_w = math.isfinite(width) and abs(width - want_w) <= FEATURE_TOL_M
            rep.add(f'feature {feat["name"]} width',
                    'absent' if not math.isfinite(width) else f'{width:.3f} m',
                    f'{want_w:.3f} +/- {FEATURE_TOL_M:.2f} m', ok_w)
            if 'offset_m' in feat:
                want_o = float(feat['offset_m'])
                ok_o = math.isfinite(offset) and abs(offset - want_o) <= FEATURE_TOL_M
                rep.add(f'feature {feat["name"]} offset',
                        'absent' if not math.isfinite(offset) else f'{offset:.3f} m',
                        f'{want_o:.3f} +/- {FEATURE_TOL_M:.2f} m', ok_o)

    thick = wall_thickness_m(m)
    rep.add('median wall thickness',
            'n/a' if not math.isfinite(thick) else f'{thick:.3f} m',
            f'<= {MAX_WALL_THICKNESS_M:.2f} m',
            math.isfinite(thick) and thick <= MAX_WALL_THICKNESS_M)
    dup = duplicate_wall_extent_m(m)
    rep.add('duplicate wall extent', f'{dup:.3f} m',
            f'<= {MAX_DUPLICATE_WALL_M:.2f} m', dup <= MAX_DUPLICATE_WALL_M)

    if run_facts:
        lc = run_facts.get('loop_closure', {}) or {}
        if 'position_error_m' in lc:
            err = float(lc['position_error_m'])
            rep.add('loop closure position', f'{err:.3f} m',
                    f'<= {LOOP_CLOSURE_TOL_M:.2f} m', err <= LOOP_CLOSURE_TOL_M)
        if 'heading_error_deg' in lc:
            errd = abs(float(lc['heading_error_deg']))
            rep.add('loop closure heading', f'{errd:.2f} deg',
                    f'<= {LOOP_CLOSURE_TOL_DEG:.0f} deg',
                    errd <= LOOP_CLOSURE_TOL_DEG)
        rh = run_facts.get('runtime_health', {}) or {}
        if 'queue_overflows_after_warmup' in rh:
            n = int(rh['queue_overflows_after_warmup'])
            rep.add('scan queue overflows', f'{n}', '0 after warm-up', n == 0)
        if 'map_updates' in rh:
            n = int(rh['map_updates'])
            rep.add('map updates during motion', f'{n}', '> 0', n > 0)
    return rep


def score_repeatability(map_a, map_b, reference=None):
    """Two independent physical runs -> repeatability rows."""
    rep = ScoreReport()
    try:
        ma, mb = load_map(map_a), load_map(map_b)
    except MapLoadError as e:
        rep.errors.append(str(e))
        rep.add('repeatability', 'unreadable', 'both maps load', False)
        return rep
    for span in (reference or {}).get('spans', []) or []:
        a = measure_span(ma, span['axis'])
        b = measure_span(mb, span['axis'])
        if not (math.isfinite(a) and math.isfinite(b) and a > 0):
            rep.add(f'repeat span {span["name"]}', 'unmeasurable', 'both measurable',
                    False)
            continue
        diff = abs(a - b) / a
        rep.add(f'repeat span {span["name"]}', f'{100 * diff:.1f}% apart',
                f'<= {100 * REPEAT_SPAN_TOL_FRAC:.0f}%', diff <= REPEAT_SPAN_TOL_FRAC)
    v, dx, dy, deg = best_alignment_iou(ma, mb, angles_deg=range(-6, 7, 2))
    rep.add('occupied-cell IoU', f'{v:.3f} (dx {dx}, dy {dy}, {deg:.0f} deg)',
            f'>= {REPEAT_MIN_IOU:.2f}', v >= REPEAT_MIN_IOU)
    return rep


# ------------------------------------------------------------------- self-test
def _write_synthetic(dirpath, kind):
    """Build a map artifact of a given `kind` for the negative controls. -> yaml path."""
    res = 0.05
    n = 100                                   # 5 x 5 m canvas
    px = np.full((n, n), UNKNOWN_VALUE, dtype=np.uint8)
    if kind == 'good':
        lo, hi = 10, 90                       # a 4.0 x 4.0 m room
        px[lo:hi + 1, lo:hi + 1] = 254        # interior explored
        px[lo, lo:hi + 1] = 0
        px[hi, lo:hi + 1] = 0
        px[lo:hi + 1, lo] = 0
        px[lo:hi + 1, hi] = 0
        px[hi, 40:56] = 254                   # a 0.80 m doorway in the max-y wall
    elif kind == 'near_empty':
        # The artifact that actually shipped: correct outer dimensions, ~16 pixels.
        px[10, 10:18] = 0
        px[90, 10:18] = 0
    elif kind == 'corrupt':
        path = os.path.join(dirpath, 'corrupt.yaml')
        with open(path, 'w') as f:
            yaml.safe_dump({'image': 'corrupt.pgm', 'resolution': res,
                            'origin': [0.0, 0.0, 0.0]}, f)
        with open(os.path.join(dirpath, 'corrupt.pgm'), 'wb') as f:
            f.write(b'P5\n100 100\n255\n' + b'\x00' * 50)   # truncated body
        return path
    pgm = os.path.join(dirpath, f'{kind}.pgm')
    with open(pgm, 'wb') as f:
        f.write(f'P5\n{n} {n}\n255\n'.encode())
        f.write(px[::-1].tobytes())           # PGM row 0 is max y
    ypath = os.path.join(dirpath, f'{kind}.yaml')
    with open(ypath, 'w') as f:
        yaml.safe_dump({'image': f'{kind}.pgm', 'resolution': res,
                        'origin': [-2.5, -2.5, 0.0], 'negate': 0,
                        'occupied_thresh': 0.65, 'free_thresh': 0.25}, f)
    return ypath


def self_test():
    """Negative-control the scorer, as the delivery plan requires. -> exit code."""
    import tempfile
    ok = True
    with tempfile.TemporaryDirectory() as d:
        print('=== score_slam_map self-test (negative controls) ===\n')
        corrupt = score_map(_write_synthetic(d, 'corrupt'))
        print('1. deliberately CORRUPTED map (truncated PGM body)')
        print(f'   rejected: {not corrupt.passed}   errors: {corrupt.errors}')
        ok &= not corrupt.passed

        empty = score_map(_write_synthetic(d, 'near_empty'))
        occ_row = next(r for r in empty.rows if r.metric == 'occupied pixels')
        print('\n2. NEAR-EMPTY map with correct 4 m outer dimensions')
        print('   (the shape of the artifact that shipped before)')
        print(f'   occupied pixels: {occ_row.measured}   row passed: {occ_row.passed}')
        print(f'   rejected: {not empty.passed}')
        ok &= not empty.passed and not occ_row.passed

        ref = {'spans': [{'name': 'x', 'axis': 'x', 'metres': 4.05},
                         {'name': 'y', 'axis': 'y', 'metres': 4.05}],
               'features': [{'name': 'doorway', 'side': 'max_y', 'width_m': 0.80}]}
        facts = {'loop_closure': {'position_error_m': 0.04, 'heading_error_deg': 1.2},
                 'runtime_health': {'queue_overflows_after_warmup': 0,
                                    'map_updates': 220}}
        good = score_map(_write_synthetic(d, 'good'), ref, facts)
        print('\n3. WELL-FORMED synthetic map + matching reference')
        print(good.table())
        ok &= good.passed
    print(f'\nSELF-TEST: {"PASS" if ok else "FAIL"}')
    return 0 if ok else 1


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--map', help='map YAML to score')
    ap.add_argument('--reference', help='tape-measured reference geometry YAML')
    ap.add_argument('--run-facts', help='run facts YAML (loop closure, runtime health)')
    ap.add_argument('--compare-map', help='second physical run, for repeatability')
    ap.add_argument('--json', help='write the scores as JSON')
    ap.add_argument('--self-test', action='store_true',
                    help='run the negative controls and exit')
    args = ap.parse_args()

    if args.self_test:
        return self_test()
    if not args.map:
        ap.error('--map is required (or use --self-test)')

    def load_yaml(path):
        if not path:
            return None
        with open(path) as f:
            return yaml.safe_load(f)

    reference = load_yaml(args.reference)
    facts = load_yaml(args.run_facts)
    rep = score_map(args.map, reference, facts)
    print(f'=== score_slam_map: {args.map} ===')
    print(rep.table())
    payload = {'map': os.path.abspath(args.map), 'score': rep.to_json()}

    passed = rep.passed
    if args.compare_map:
        rrep = score_repeatability(args.map, args.compare_map, reference)
        print(f'\n=== repeatability vs {args.compare_map} ===')
        print(rrep.table())
        payload['repeatability'] = rrep.to_json()
        payload['compare_map'] = os.path.abspath(args.compare_map)
        passed = passed and rrep.passed

    if args.json:
        with open(args.json, 'w') as f:
            json.dump(payload, f, indent=2)
        print(f'\n  json: {args.json}')
    return 0 if passed else 1


if __name__ == '__main__':
    sys.exit(main())
