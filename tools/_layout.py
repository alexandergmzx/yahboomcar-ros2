"""Where things live, resolved once. Import it; do not run it.

    from _layout import REPO, WS_SETUP, LOG_DIR, BAG_DIR, USD_DIR, pkg_dir, vendor_pkg_dir

WHY THIS EXISTS AS A SHARED MODULE
----------------------------------
The D-12 extraction moved this repo out of the MicroROS checkout: packages went from
`yahboomcar_ws/src/<pkg>` to the repo root, the build environment became the fleet's
`ground_station/install` (Amendment A2.3), and MicroROS-assets stayed behind in the
read-only MicroROS checkout. simctl was made layout-aware in session 4 -- and an audit
for the Isaac repair session (2026-08-08) then found EIGHTEEN other tools still
computing `REPO/yahboomcar_ws/...` and `REPO/MicroROS-assets` paths that no longer
exist. The visible symptom was `simctl start --backend isaac` burning its whole 420 s
readiness budget against a sim_runner that had exited in under a second with
"no arena at <path that does not exist post-extraction>".

Eighteen private copies of the same path logic is how that happens. This module is the
single definition, same rationale as _cmd_vel_safety.CAR_DOMAIN: two copies eventually
disagree, and the one that is wrong is the one somebody trusts.

RESOLUTION ORDER, per location
------------------------------
Environment variables win, then the layout is detected by what exists on disk, with the
LAST candidate returned (not None) when nothing exists so error messages can name a
real path. Both repo layouts keep working: the extracted fleet checkout and an
unextracted MicroROS working copy.

USD lives with the OTHER never-extracted artifacts: it is generated from vendor meshes
(R-05 -- unlicensed, never republished here), so its fleet-layout home is the MicroROS
checkout, reached the same way MicroROS-assets is.
"""
import os

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def _first_existing(cands):
    for c in cands:
        if os.path.exists(c):
            return os.path.abspath(c)
    return os.path.abspath(cands[-1])


# The build/run environment. The fleet's ground_station/install is the SOLE environment
# post-extraction (A2.3); the in-repo workspace remains as the unextracted fallback.
WS_SETUP = os.environ.get('FLEET_WS_SETUP') or _first_existing([
    os.path.join(REPO, '..', '..', 'ground_station', 'install', 'setup.bash'),
    os.path.join(REPO, 'yahboomcar_ws', 'install', 'setup.bash')])

# Bags, logs, firmware, maps. Never extracted; reachable from the fleet position via
# the src/MicroROS farm symlink.
ASSETS = os.environ.get('MICROROS_ASSETS') or _first_existing([
    os.path.join(REPO, 'MicroROS-assets'),
    os.path.join(REPO, '..', 'MicroROS', 'MicroROS-assets')])
LOG_DIR = os.path.join(ASSETS, 'logs')
BAG_DIR = os.path.join(ASSETS, 'bags')

# arena.usd, micro4/, twin_scene.usd -- generated from vendor meshes, so they live with
# the vendor checkout, not here.
USD_DIR = os.environ.get('YAHBOOM_USD_DIR') or _first_existing([
    os.path.join(REPO, 'yahboomcar_ws', 'src', 'yahboomcar_twin', 'usd'),
    os.path.join(REPO, '..', 'MicroROS', 'yahboomcar_ws', 'src', 'yahboomcar_twin',
                 'usd')])


def pkg_dir(name):
    """Ament package root of a FIRST-PARTY package (extracted: at repo root).

    Also where that package's measured evidence lives (failsafe_report.json,
    braking_runs.json, measured_params.json sit next to the code they gate).
    """
    return _first_existing([
        os.path.join(REPO, name),
        os.path.join(REPO, 'yahboomcar_ws', 'src', name)])


def vendor_pkg_dir(name):
    """Ament package root of a VENDOR package (bringup, description, nav, ...).

    Vendor packages were never extracted (R-05): in the fleet layout they exist only in
    the MicroROS checkout, served into the build via the farm.
    """
    return _first_existing([
        os.path.join(REPO, 'yahboomcar_ws', 'src', name),
        os.path.join(REPO, '..', 'MicroROS', 'yahboomcar_ws', 'src', name)])
