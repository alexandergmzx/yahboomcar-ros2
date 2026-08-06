"""ICP scan matching: the one movement estimate on this robot that does not come from a
wheel or from integrating the robot's own motion.

WHY THIS EXISTS
---------------
Every other estimate the car has is self-referential. Encoders report wheel rotation and
cannot tell rotation from slip. The gyro reports yaw rate and drifts, with no magnetometer
to correct it. The accelerometer needs double integration, which dies within about half a
second. All three answer "how much do I think I moved" by accumulating their own opinion.

The lidar answers a different question: "where am I relative to the room". That makes it
the only sensor able to BOUND drift rather than accumulate it, and the only independent
witness available when the encoders and the tape disagree.

WHAT IT CANNOT DO, AND WHY THAT IS REPORTED RATHER THAN HIDDEN
--------------------------------------------------------------
A scan matcher can only resist motion ALONG a surface normal. Slide a robot down a
corridor of two parallel walls and every scan looks identical, so the estimate along the
corridor is unconstrained -- not noisy, UNCONSTRAINED, and ICP will still converge and
still return a confident-looking number.

CORRECTION, from measurement. An earlier version of this file asserted that a 4x4 m room
with flat walls was "close to the worst case" and that boxes were what made translation
observable there. **Both claims were wrong**, and tools/arena_observability.py disproved
them by raytracing scans across the room:

    4.0 x 4.0 m     median isotropy 0.914    well conditioned
    8.0 x 2.0 m                     0.230    marginal
   12.0 x 1.0 m                     0.058    DEGENERATE
   30.0 x 30.0 m                    0.103    marginal (far walls out of range)

The lidar reaches 8 m and the room is 4 m, so it sees ALL FOUR walls from every position,
and four walls supply normals on both axes. Adding boxes slightly REDUCED isotropy,
because box faces are axis-aligned too and they occlude wall returns.

Degeneracy needs one of two things: an aspect ratio around 8:1 or worse, or a room bigger
than the sensor's range, where the far walls are simply not there to be seen. The 4x4
arena is neither.

So degeneracy is a first-class output. `match()` returns the eigen-decomposition of the
translational information matrix, and callers are expected to refuse the unconstrained
direction rather than average it in. Fail-closed, the same stance as
yahboomcar_safety/governor.py: an estimator that fails permissive is worse than none,
because it invites trust.

The formulation follows Censi, "On achievable accuracy for range-finder localization"
(ICRA 2007), which derives exactly this: the information a scan carries about
translation is sum(n n^T) over the surface normals it observes.

MEASURED ACCURACY ENVELOPE (synthetic 4x4 room + boxes, per-scan motion)
-----------------------------------------------------------------------
Pure translation is recovered EXACTLY at every magnitude tested up to 0.25 m. Error
appears only when translation and rotation combine, and grows with their product:

    motion/scan            xy error
    0.025 m, 0.125 rad      2.7 mm      <- the car's actual per-scan maximum
    0.10 m,  0.125 rad     11.1 mm
    0.25 m,  0.20 rad      47.8 mm

At 12 Hz the car cannot exceed 0.025 m and 0.125 rad in one interval (0.30 m/s and
1.5 rad/s), so it operates in the top row. Rotation carries a floor set by the
DISCRETISATION of the scene rather than by this code: measured error tracked
(sample spacing / room radius) to within 5% over a 16x density sweep. The real lidar is
360 beams at 1 deg, ~12 mm of arc at a 0.7 m range.

NEAREST NEIGHBOURS: scipy WHEN AVAILABLE, numpy WHEN NOT
--------------------------------------------------------
scipy's cKDTree is 6.1x faster than the brute-force fallback here (mean 110 -> 18 ms,
p95 485 -> 43 ms on real 355-point scans), which is the difference between needing
subsampling and a time budget to hit 12 Hz and not needing them.

It is optional rather than required because scipy was broken on this machine for a while
-- a pip numpy shadowing apt's while apt scipy was built against the older ABI -- and a
localisation package that cannot run because a dependency's dependency is mismatched is
worse than one that runs slower. The fallback is exercised by its own test so it cannot
silently rot.
"""
import math
import time
from dataclasses import dataclass, field

import numpy as np

try:                                     # optional: 6.1x faster when present
    from scipy.spatial import cKDTree
    HAVE_KDTREE = True
except Exception:                        # ImportError, or a numpy/scipy ABI mismatch
    cKDTree = None
    HAVE_KDTREE = False

from yahboomcar_localization.scan_geometry import (compose, estimate_normals,
                                                   transform_xy, wrap)


@dataclass
class Degeneracy:
    """Which translation directions the geometry actually constrains."""
    strong_dir: tuple           # unit vector, best-constrained direction
    weak_dir: tuple             # unit vector, worst-constrained direction
    strong_eig: float           # information along strong_dir
    weak_eig: float             # information along weak_dir
    isotropy: float             # weak/strong in [0, 1]; 0 = fully degenerate
    rotational_info: float      # information about yaw
    degenerate: bool            # weak direction is not usefully constrained

    def as_dict(self):
        return {'strong_dir': list(self.strong_dir), 'weak_dir': list(self.weak_dir),
                'strong_eig': self.strong_eig, 'weak_eig': self.weak_eig,
                'isotropy': self.isotropy, 'rotational_info': self.rotational_info,
                'degenerate': self.degenerate}


@dataclass
class MatchResult:
    dx: float
    dy: float
    dtheta: float
    converged: bool
    iterations: int
    inliers: int
    rms: float                  # residual of the accepted correspondences, metres
    degeneracy: Degeneracy = None
    reason: str = ''            # why this result should be distrusted; '' when fine
    seed_dominated: bool = False
    budget_exceeded: bool = False

    @property
    def trustworthy(self):
        return self.converged and not self.reason


def nearest_bruteforce(src, dst, chunk=256):
    """Nearest neighbour of each src point in dst, in plain numpy.

    Chunked over src so peak memory stays at chunk*len(dst)*2 floats rather than
    len(src)*len(dst)*2. Kept as the fallback for machines without a working scipy, and
    tested directly so it cannot rot unnoticed.
    """
    n = len(src)
    idx = np.zeros(n, dtype=int)
    dist = np.zeros(n)
    if n == 0 or len(dst) == 0:
        return idx, dist
    for a in range(0, n, chunk):
        b = min(n, a + chunk)
        d = np.linalg.norm(src[a:b, None, :] - dst[None, :, :], axis=2)
        j = np.argmin(d, axis=1)
        idx[a:b] = j
        dist[a:b] = d[np.arange(b - a), j]
    return idx, dist


def nearest(src, dst, chunk=256):
    """Nearest neighbour of each src point in dst. Returns (indices, distances).

    Uses scipy's cKDTree when it imports, and the numpy fallback otherwise. Both return
    identical results; only the speed differs. A test asserts that equivalence.
    """
    if not HAVE_KDTREE or len(src) == 0 or len(dst) == 0:
        return nearest_bruteforce(src, dst, chunk)
    d, i = cKDTree(dst).query(src)
    return np.asarray(i, dtype=int), np.asarray(d, dtype=float)


def rigid_transform(p, q):
    """Least-squares 2D rigid transform taking p onto q (Procrustes / Kabsch).

    Closed form, no iteration: centre both sets, then the optimal rotation is the angle
    of the summed cross/dot products.
    """
    if len(p) == 0:
        return 0.0, 0.0, 0.0
    pc, qc = p.mean(axis=0), q.mean(axis=0)
    pp, qq = p - pc, q - qc
    num = float(np.sum(pp[:, 0] * qq[:, 1] - pp[:, 1] * qq[:, 0]))   # cross
    den = float(np.sum(pp[:, 0] * qq[:, 0] + pp[:, 1] * qq[:, 1]))   # dot
    theta = math.atan2(num, den)
    c, s = math.cos(theta), math.sin(theta)
    R = np.array([[c, -s], [s, c]])
    t = qc - R @ pc
    return float(t[0]), float(t[1]), float(theta)


def degeneracy(xy, normals=None, weak_threshold=0.05, min_isotropy=0.02):
    """How well does this geometry constrain translation, and in which directions?

    The translational information matrix is sum(n n^T) over surface normals, normalised
    by the number of contributing points so the result is a fraction rather than a count.
    Its eigenvalues are the information available along each eigenvector.

      two parallel walls  -> all normals parallel  -> one eigenvalue ~0 -> DEGENERATE
      a corner or a box   -> normals in two axes   -> both eigenvalues large -> fine

    `min_isotropy` is deliberately small. The point is to catch geometry that constrains
    a direction hardly at all, not to demand a well-conditioned room.
    """
    xy = np.asarray(xy, dtype=float)
    if normals is None:
        normals = estimate_normals(xy)
    valid = np.linalg.norm(normals, axis=1) > 0.5
    n = normals[valid]
    if len(n) < 3:
        return Degeneracy((1., 0.), (0., 1.), 0.0, 0.0, 0.0, 0.0, True)

    M = (n.T @ n) / len(n)
    w, v = np.linalg.eigh(M)                      # ascending
    weak_eig, strong_eig = float(w[0]), float(w[1])
    iso = weak_eig / strong_eig if strong_eig > 0 else 0.0

    # Rotational information: how much a yaw change would move points along their
    # normals. (p x n)^2 summed -- a point far from the origin whose normal is
    # perpendicular to its radius resists rotation most.
    p = xy[valid]
    cross = p[:, 0] * n[:, 1] - p[:, 1] * n[:, 0]
    rot_info = float(np.mean(cross ** 2))

    return Degeneracy(
        strong_dir=(float(v[0, 1]), float(v[1, 1])),
        weak_dir=(float(v[0, 0]), float(v[1, 0])),
        strong_eig=strong_eig, weak_eig=weak_eig, isotropy=iso,
        rotational_info=rot_info,
        degenerate=(weak_eig < weak_threshold or iso < min_isotropy),
    )


# Correspondence distances, coarse to fine. Each stage seeds the next.
#
# A single fixed radius cannot work here. Per scan at 12 Hz the car can yaw up to
# 1.5 rad/s = 0.125 rad, which swings a corner point of a 4 m room through 0.34 m -- so a
# radius tight enough to reject wrong matches at convergence is too tight to find the
# right ones at the start, and ICP silently under-converges (measured: 0.085 rad
# recovered from a true 0.12).
#
# The alternative would be to seed from wheel odometry, and that is deliberately NOT done.
# In a direction the geometry does not constrain, ICP returns its seed unchanged -- so a
# seeded matcher would report wheel odometry back as if it were a lidar observation, and
# fusing the two would double-count exactly as the vendor ekf.yaml already does. Keeping
# the matcher unseeded is what keeps it an INDEPENDENT witness.
CORR_SCHEDULE = (1.00, 0.50, 0.25, 0.12)


def match(source, target, init=(0.0, 0.0, 0.0), max_iter=30, tol=1e-4,
          corr_schedule=CORR_SCHEDULE, min_inliers=20, outlier_k=3.0,
          time_budget=None):
    """Align `source` onto `target`. Returns the transform mapping source into target.

    Point-to-point ICP, coarse to fine. Point-to-point rather than point-to-plane for the
    solve because it needs no normals in the loop and degrades more gracefully on sparse
    scans; normals carry the information that matters in the DEGENERACY report instead.

    `init` defaults to zero and should usually stay there -- see CORR_SCHEDULE above for
    why seeding from odometry would destroy the independence this estimate exists for.
    When a non-zero init IS supplied, `seed_dominated` reports whether the answer merely
    came back out of the seed.

    Outliers are rejected by a ROBUST SCALE (median + outlier_k * MAD), not by keeping a
    fixed closest fraction. That distinction is not cosmetic. Under a rotation the
    residual of a correspondence grows with its distance from the centre of rotation, so
    the points carrying the rotational signal are precisely the ones with the largest
    residuals -- and a fixed-fraction trim throws them away. Measured: trimming to the
    closest 80% converged to a stable 0.0852 rad against a true 0.12 and stayed there
    through every stage, an under-estimate of 29% that looked like clean convergence.

    A robust scale does not have that bias, because when a large motion makes every
    residual large it moves with them; it only rejects points that disagree with the bulk,
    which is what a moving obstacle actually looks like.

    `time_budget` (seconds) bounds latency for real-time use. Cost is dominated by
    ITERATION COUNT, not point count: on recorded scans the mean match took 119 ms but
    the p95 took 546 ms, because cluttered real geometry runs the full 4x30 schedule
    where clean synthetic geometry converges in a few. Against a 12 Hz scan interval
    (83 ms) that overruns by 6.6x, and an estimator that silently misses its deadline
    just drops scans until it looks like it is working.

    When the budget is hit the refinement stops and the best estimate so far is returned
    with `budget_exceeded` set, so the degradation is visible rather than inferred from a
    thinning output rate.
    """
    source = np.asarray(source, dtype=float)
    target = np.asarray(target, dtype=float)
    if len(source) < min_inliers or len(target) < min_inliers:
        return MatchResult(*init, converged=False, iterations=0, inliers=0,
                           rms=float('inf'),
                           reason=f'too few points ({len(source)} -> {len(target)})')

    est = tuple(float(x) for x in init)
    total_iters = 0
    inliers = 0
    rms = float('inf')
    converged = False
    over_budget = False
    t_start = time.perf_counter()

    for corr_dist in corr_schedule:
        if over_budget:
            break
        prev_rms = float('inf')
        for _ in range(max_iter):
            if time_budget is not None and \
                    time.perf_counter() - t_start > time_budget:
                over_budget = True
                break
            total_iters += 1
            moved = transform_xy(source, *est)
            idx, dist = nearest(moved, target)

            keep = dist <= corr_dist
            if keep.sum() >= min_inliers and outlier_k > 0:
                d = dist[keep]
                med = float(np.median(d))
                mad = float(np.median(np.abs(d - med)))
                # 1.4826 makes MAD a consistent estimator of sigma for Gaussian noise.
                cutoff = med + outlier_k * 1.4826 * mad
                keep &= dist <= max(cutoff, 1e-6)
            inliers = int(keep.sum())
            if inliers < min_inliers:
                break

            p, q = moved[keep], target[idx[keep]]
            est = compose(est, rigid_transform(p, q))
            rms = float(np.sqrt(np.mean(dist[keep] ** 2)))

            if abs(prev_rms - rms) < tol * 1e-2:
                break
            prev_rms = rms
        converged = inliers >= min_inliers

    if inliers < min_inliers:
        return MatchResult(*est, converged=False, iterations=total_iters,
                           inliers=inliers, rms=float(rms),
                           reason=f'only {inliers} correspondences at the finest stage')

    deg = degeneracy(target)

    # Did the answer come from the scans, or straight back out of the seed? Only
    # meaningful along the direction the geometry cannot constrain.
    moved_from_seed = math.hypot(est[0] - init[0], est[1] - init[1])
    seed_dominated = bool(
        (init[0] or init[1] or init[2]) and deg.degenerate and moved_from_seed < 1e-3)

    res = MatchResult(est[0], est[1], wrap(est[2]), converged, total_iters, inliers,
                      rms, degeneracy=deg, seed_dominated=seed_dominated,
                      budget_exceeded=over_budget)
    if not converged:
        res.reason = f'did not converge ({total_iters} iterations)'
    elif deg.degenerate:
        # Converged is not the same as correct. Say so, rather than emitting a number
        # whose error the caller has no way to see.
        res.reason = (f'geometry degenerate: translation along '
                      f'({deg.weak_dir[0]:+.2f}, {deg.weak_dir[1]:+.2f}) is '
                      f'unconstrained (isotropy {deg.isotropy:.3f})')
    if seed_dominated:
        res.reason += '; estimate returned the seed unchanged -- NOT independent evidence'
    return res


def estimate_motion(prev_xy, curr_xy, **kw):
    """ROBOT motion between two scans, in the robot's own frame. Use this, not match().

    THE CONVENTION, because getting it backwards is silent and plausible-looking.

    Scan points live in the sensor frame. A world point p is seen at time k as
    s_k = T_k^-1 p, so:

        s_{k+1} = T_{k+1}^-1 T_k s_k = dT^-1 s_k,   where dT = T_k^-1 T_{k+1}

    dT is the robot's motion -- the thing odometry reports. So matching the PREVIOUS
    scan onto the CURRENT one yields dT^-1, the inverse of what is wanted: if the robot
    turns left, the room appears to turn right.

    Matching current onto previous yields dT directly, which is why the arguments are
    swapped here rather than the result being negated.

    Caught on real data: encoders and gyro both read about +1.8 rad of yaw over a desk
    run while the naive match(prev, curr) read -1.3. The magnitudes were plausible, so
    only the sign gave it away.
    """
    return match(curr_xy, prev_xy, **kw)
