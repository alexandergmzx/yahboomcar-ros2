#!/usr/bin/env python3
"""Where in a room can the lidar actually tell that the robot has moved?

    ./tools/arena_observability.py
    ./tools/arena_observability.py --room 4 4 --plot-grid

Needs no robot and no simulator: it raytraces a 360-beam scan against line segments, so
the geometry is exact rather than approximated by a physics engine.

WHAT IT FOUND, WHICH WAS NOT WHAT WAS EXPECTED
----------------------------------------------
This tool was written to confirm that a bare 4x4 m room would be a poor environment for
scan matching, and that boxes would be needed to make translation observable. It showed
the opposite, and the expectation was wrong:

    4.0 x 4.0 m     median isotropy 0.914    well conditioned
    4.0 x 3.0 m                     0.733    well conditioned
    8.0 x 2.0 m                     0.230    marginal
   12.0 x 1.0 m                     0.058    DEGENERATE
   30.0 x 30.0 m                    0.103    marginal, and 0.000 at some positions

The lidar reaches 8 m. A 4 m room therefore shows all four walls from every position, and
four walls constrain both axes. Adding boxes slightly REDUCED median isotropy (0.924 ->
0.772 with three), because box faces are axis-aligned too while occluding wall returns.

Degeneracy needs an aspect ratio near 8:1, or a room larger than the sensor range so the
far walls are not visible at all. A big empty hall is the hazard, not a small square room.

Boxes are still worth having -- they break the rotational symmetry of a square room, and
they are what the safety governor needs to see -- but not for observability.

Isotropy here is the ratio of the weak to the strong eigenvalue of sum(n n^T) over the
observed surface normals -- 1.0 means both directions equally constrained, 0.0 means one
direction carries no information at all. See yahboomcar_localization.scan_matcher.
"""
import argparse
import math
import os
import sys

import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO, 'yahboomcar_ws', 'src', 'yahboomcar_localization'))


def segments_room(w, h):
    hw, hh = w / 2, h / 2
    return [((-hw, -hh), (hw, -hh)), ((hw, -hh), (hw, hh)),
            ((hw, hh), (-hw, hh)), ((-hw, hh), (-hw, -hh))]


def segments_box(cx, cy, s):
    h = s / 2
    c = [(cx - h, cy - h), (cx + h, cy - h), (cx + h, cy + h), (cx - h, cy + h)]
    return [(c[i], c[(i + 1) % 4]) for i in range(4)]


def raycast(origin, segs, n_beams=360, r_max=8.0, r_min=0.12):
    """Simulate a 360-beam scan. Returns (N, 2) hit points in the sensor frame."""
    ox, oy = origin
    angs = np.arange(n_beams) * (2 * math.pi / n_beams)
    pts = []
    for a in angs:
        dx, dy = math.cos(a), math.sin(a)
        best = None
        for (x1, y1), (x2, y2) in segs:
            ex, ey = x2 - x1, y2 - y1
            den = dx * ey - dy * ex
            if abs(den) < 1e-12:
                continue
            t = ((x1 - ox) * ey - (y1 - oy) * ex) / den      # along the ray
            u = ((x1 - ox) * dy - (y1 - oy) * dx) / den      # along the segment
            if t > r_min and 0.0 <= u <= 1.0 and (best is None or t < best):
                best = t
        if best is not None and best <= r_max:
            pts.append((best * dx, best * dy))
    return np.array(pts) if pts else np.zeros((0, 2))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--room', nargs=2, type=float, default=[4.0, 4.0],
                    metavar=('W', 'H'))
    ap.add_argument('--box-size', type=float, default=0.4)
    ap.add_argument('--grid', type=int, default=5,
                    help='sample positions per axis')
    args = ap.parse_args()

    from yahboomcar_localization.scan_matcher import degeneracy

    w, h = args.room
    walls = segments_room(w, h)

    def survey(segs, label):
        """Isotropy over a grid of robot positions, avoiding the walls themselves."""
        margin = 0.6
        xs = np.linspace(-w / 2 + margin, w / 2 - margin, args.grid)
        ys = np.linspace(-h / 2 + margin, h / 2 - margin, args.grid)
        isos, degs = [], 0
        for x in xs:
            for y in ys:
                pts = raycast((x, y), segs)
                if len(pts) < 20:
                    continue
                d = degeneracy(pts)
                isos.append(d.isotropy)
                degs += d.degenerate
        isos = np.array(isos)
        print(f'  {label:34s} isotropy  min {isos.min():.3f}  '
              f'median {np.median(isos):.3f}  degenerate at {degs}/{len(isos)} positions')
        return isos, degs

    print(f'=== observability survey, {w:.1f} x {h:.1f} m room, '
          f'{args.grid}x{args.grid} positions ===')
    print('  (isotropy 1.0 = both directions constrained, 0.0 = one carries no info)')
    print()

    layouts = [
        ('bare walls', []),
        ('1 box, centre', [(0.0, 0.0)]),
        ('2 boxes, diagonal', [(-w / 4, -h / 4), (w / 4, h / 4)]),
        ('3 boxes, scattered', [(-w / 4, -h / 4), (w / 4, h / 4), (w / 4, -h / 3)]),
        ('4 boxes, asymmetric', [(-w / 4, -h / 4), (w / 4, h / 4),
                                 (w / 3, -h / 4), (-w / 3, h / 3.5)]),
    ]
    results = {}
    for label, centres in layouts:
        segs = list(walls)
        for cx, cy in centres:
            segs += segments_box(cx, cy, args.box_size)
        results[label] = survey(segs, f'{label} ({len(centres)} box)')

    print()
    bare_med = float(np.median(results['bare walls'][0]))
    print('  INTERPRETATION')
    print(f'  Bare walls give median isotropy {bare_med:.3f}. With an 8 m sensor range and')
    print('  a room this size the lidar sees all four walls from everywhere, and four')
    print('  walls constrain both axes -- so this room is WELL CONDITIONED, and boxes are')
    print('  not needed to make translation observable. Degeneracy needs an aspect ratio')
    print('  near 8:1, or a room larger than the sensor range.')
    for label, (isos, degs) in results.items():
        if label == 'bare walls':
            continue
        gain = float(np.median(isos)) / bare_med
        print(f'    {label:22s} median isotropy x{gain:.2f} vs bare, '
              f'worst position {isos.min():.3f}')
    print()
    print('  Boxes also matter for a second reason this survey does not capture: they')
    print('  break the ROTATIONAL symmetry of a square room. Four identical walls look')
    print('  the same after a 90 degree turn, so a matcher that loses track can relocalise')
    print('  into the wrong quadrant with total confidence.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
