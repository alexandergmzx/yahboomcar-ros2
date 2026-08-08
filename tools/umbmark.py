#!/usr/bin/env python3
"""UMBmark odometry calibration: run recorder and calculator.

Implements Borenstein & Feng (SPIE 1995) and its companion correction paper. See
docs/odometry-calibration.md for the field procedure and docs/research-log.md for
sources.

    ./tools/umbmark.py record            # walk through the 10 runs, save results
    ./tools/umbmark.py compute           # recompute from the saved file
    ./tools/umbmark.py compute --demo    # worked example, no hardware

Results live in MicroROS-assets/bags/umbmark_results.json (git-ignored) so a session
can be interrupted and resumed.

The point of doing this in software is that the arithmetic -- particularly that alpha
uses the SUM of the two cluster centres while beta uses the DIFFERENCE -- is easy to get
backwards by hand, and getting it backwards silently swaps a wheelbase error for a wheel
diameter error.
"""
import argparse
import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _layout import BAG_DIR                                     # noqa: E402
RESULTS = os.path.join(BAG_DIR, 'umbmark_results.json')

L_DEFAULT = 2.0      # square side, metres
B_NOMINAL = 0.135    # physical track from the URDF joint origins (+/-0.0675)
N_PER_DIR = 5


def mean(xs):
    return sum(xs) / len(xs) if xs else 0.0


def compute(runs, L=L_DEFAULT, b=B_NOMINAL):
    """runs: list of dicts {direction: 'cw'|'ccw', x: mm, y: mm}. Returns a report."""
    cw = [r for r in runs if r['direction'] == 'cw']
    ccw = [r for r in runs if r['direction'] == 'ccw']
    if not cw or not ccw:
        raise ValueError('need runs in BOTH directions -- that is the whole point of '
                         'UMBmark; a single direction cannot separate the two errors')

    # Cluster centres of gravity (Eq. 2), millimetres.
    xcw, ycw = mean([r['x'] for r in cw]), mean([r['y'] for r in cw])
    xccw, yccw = mean([r['x'] for r in ccw]), mean([r['y'] for r in ccw])

    # Accuracy measure (Eq. 3-4).
    r_cw = math.hypot(xcw, ycw)
    r_ccw = math.hypot(xccw, yccw)
    e_max = max(r_cw, r_ccw)

    # Correction factors. L is metres, offsets are mm -> work in mm consistently.
    L_mm = L * 1000.0
    b_mm = b * 1000.0

    # alpha = SUM (Eq. 4.24a), beta = DIFFERENCE (Eq. 4.20a). Not interchangeable.
    alpha = (xcw + xccw) / (-4.0 * L_mm) * (180.0 / math.pi)
    beta = (xcw - xccw) / (-4.0 * L_mm) * (180.0 / math.pi)

    # Independent derivation from y (Eq. 4.20b, 4.24b) -- the contamination check.
    beta_y = (ycw + yccw) / (-4.0 * L_mm) * (180.0 / math.pi)
    alpha_y = (ycw - yccw) / (-4.0 * L_mm) * (180.0 / math.pi)

    # Radius of curvature and wheel-diameter ratio (Eq. 4.21-4.22).
    if abs(beta) < 1e-9:
        R = float('inf')
        Ed = 1.0
    else:
        R = (L_mm / 2.0) / math.sin(math.radians(beta) / 2.0)
        denom = R - b_mm / 2.0
        Ed = (R + b_mm / 2.0) / denom if abs(denom) > 1e-9 else float('nan')

    # Wheelbase error (Eq. 4.26-4.27).
    Eb = 90.0 / (90.0 - alpha) if abs(90.0 - alpha) > 1e-9 else float('nan')
    b_actual = Eb * b

    # Per-wheel factors (Eq. 4.31-4.32). Recorded for completeness; they cannot be
    # applied directly here because the firmware exposes only body twist, not wheels.
    cL = 2.0 / (Ed + 1.0) if Ed == Ed and Ed != -1 else float('nan')
    cR = 2.0 / ((1.0 / Ed) + 1.0) if Ed == Ed and Ed != 0 else float('nan')

    return {
        'n_cw': len(cw), 'n_ccw': len(ccw), 'L_m': L, 'b_nominal_m': b,
        'x_cg_cw': xcw, 'y_cg_cw': ycw, 'x_cg_ccw': xccw, 'y_cg_ccw': yccw,
        'r_cg_cw': r_cw, 'r_cg_ccw': r_ccw, 'E_max_syst_mm': e_max,
        'alpha_deg': alpha, 'beta_deg': beta,
        'alpha_deg_from_y': alpha_y, 'beta_deg_from_y': beta_y,
        'R_mm': R, 'Ed': Ed, 'Eb': Eb, 'b_actual_m': b_actual,
        'cL': cL, 'cR': cR,
    }


def report(res):
    print()
    print(f"runs: {res['n_cw']} cw + {res['n_ccw']} ccw    "
          f"L={res['L_m']} m   b_nominal={res['b_nominal_m']} m")
    print('-' * 66)
    print(f"  cluster c.g. CW    x={res['x_cg_cw']:+8.1f} mm   y={res['y_cg_cw']:+8.1f} mm"
          f"   r={res['r_cg_cw']:7.1f} mm")
    print(f"  cluster c.g. CCW   x={res['x_cg_ccw']:+8.1f} mm   y={res['y_cg_ccw']:+8.1f} mm"
          f"   r={res['r_cg_ccw']:7.1f} mm")
    print()
    print(f"  E_max,syst = {res['E_max_syst_mm']:.1f} mm    <-- the headline number")
    print()
    print(f"  alpha (wheelbase error)      = {res['alpha_deg']:+.4f} deg")
    print(f"  beta  (wheel diameter error) = {res['beta_deg']:+.4f} deg")
    print()

    # Contamination check: x- and y-derived values must agree.
    da = abs(res['alpha_deg'] - res['alpha_deg_from_y'])
    db = abs(res['beta_deg'] - res['beta_deg_from_y'])
    print(f"  cross-check from y:  alpha={res['alpha_deg_from_y']:+.4f}  "
          f"beta={res['beta_deg_from_y']:+.4f}")
    tol = max(0.05, 0.25 * max(abs(res['alpha_deg']), abs(res['beta_deg'])))
    if da > tol or db > tol:
        print(f"  ** WARNING: x- and y-derived values disagree (d_alpha={da:.3f}, "
              f"d_beta={db:.3f}, tol={tol:.3f}).")
        print("     That signals NON-SYSTEMATIC error -- a bump, a cable, a wheel")
        print("     catching. Repeat the run set rather than trusting these numbers.")
    else:
        print(f"  cross-check OK (d_alpha={da:.3f}, d_beta={db:.3f} within {tol:.3f})")
    print()
    print(f"  Ed = D_R/D_L        = {res['Ed']:.6f}   (1.0 = wheels equal)")
    print(f"  Eb                  = {res['Eb']:.6f}   (1.0 = nominal track correct)")
    print(f"  b_actual            = {res['b_actual_m']:.4f} m  "
          f"(nominal {res['b_nominal_m']:.4f} m)")
    print(f"  c_L, c_R            = {res['cL']:.6f}, {res['cR']:.6f}")
    print()
    print('  Interpretation:')
    if res['Eb'] == res['Eb']:
        pct = (res['Eb'] - 1.0) * 100.0
        print(f'    effective track is {pct:+.1f}% vs physical. A large positive value is')
        print('    EXPECTED on this 4-wheel skid-steer: turning is slip by design, so the')
        print('    effective track is genuinely wider than the 0.135 m geometry.')
    if res['Ed'] == res['Ed']:
        pct = (res['Ed'] - 1.0) * 100.0
        print(f'    wheel diameters differ by {pct:+.2f}%; this is what makes "straight"')
        print('    legs curve, and it is the error a one-directional test cannot see.')
    print()


DEMO = [
    {'direction': 'cw', 'x': -131.6, 'y': -134}, {'direction': 'cw', 'x': -127, 'y': -135.1},
    {'direction': 'cw', 'x': -128.7, 'y': -131}, {'direction': 'cw', 'x': -135.3, 'y': -129.1},
    {'direction': 'cw', 'x': -135.6, 'y': -130.1}, {'direction': 'ccw', 'x': 46.3, 'y': -58.1},
    {'direction': 'ccw', 'x': 51.3, 'y': -47.8}, {'direction': 'ccw', 'x': 47.1, 'y': -56.2},
    {'direction': 'ccw', 'x': 54.1, 'y': -46.1}, {'direction': 'ccw', 'x': 53.4, 'y': -53.8},
]


def record(L):
    runs = []
    if os.path.exists(RESULTS):
        with open(RESULTS) as f:
            saved = json.load(f)
        runs = saved.get('runs', [])
        if runs:
            print(f'resuming: {len(runs)} run(s) already recorded in {RESULTS}')

    print('\nUMBmark recorder')
    print(f'  square L = {L} m, {N_PER_DIR} runs per direction')
    print('  Enter the TAPE-MEASURED return offset from the origin, in mm,')
    print('  in the robot\'s STARTING frame:  x = forward, y = left.')
    print('  Blank line to stop and compute with what you have.\n')

    while len(runs) < 2 * N_PER_DIR:
        i = len(runs)
        direction = 'cw' if i < N_PER_DIR else 'ccw'
        try:
            raw = input(f'run {i+1}/10 [{direction.upper():3s}]  x_mm y_mm > ').strip()
        except (EOFError, KeyboardInterrupt):
            print()
            break
        if not raw:
            break
        try:
            parts = raw.replace(',', ' ').split()
            x, y = float(parts[0]), float(parts[1])
        except (ValueError, IndexError):
            print('   need two numbers, e.g.  -125 -80')
            continue
        runs.append({'direction': direction, 'x': x, 'y': y})
        os.makedirs(os.path.dirname(RESULTS), exist_ok=True)
        with open(RESULTS, 'w') as f:
            json.dump({'L_m': L, 'runs': runs}, f, indent=2)

    if not runs:
        print('nothing recorded')
        return 1
    print(f'\nsaved {len(runs)} run(s) to {RESULTS}')
    try:
        report(compute(runs, L=L))
    except ValueError as e:
        print(f'\ncannot compute yet: {e}')
        return 1
    return 0


def main():
    ap = argparse.ArgumentParser(description='UMBmark odometry calibration.')
    ap.add_argument('mode', choices=['record', 'compute'])
    ap.add_argument('--demo', action='store_true', help='compute on synthetic data')
    ap.add_argument('-L', type=float, default=L_DEFAULT, help='square side, m')
    args = ap.parse_args()

    if args.mode == 'compute':
        if args.demo:
            print('DEMO -- synthetic numbers, not from the robot')
            report(compute(DEMO, L=args.L))
            return 0
        if not os.path.exists(RESULTS):
            sys.exit(f'no results at {RESULTS}; run "record" first (or use --demo)')
        with open(RESULTS) as f:
            saved = json.load(f)
        report(compute(saved['runs'], L=saved.get('L_m', args.L)))
        return 0

    return record(args.L)


if __name__ == '__main__':
    sys.exit(main())
