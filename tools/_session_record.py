"""Session recording: one directory, one manifest, one timeline per simctl run.

Shared module (the _layout/_dds_shm rule: one definition, because the second
private copy is the one that ends up wrong). simctl uses it to give every
session a durable record; slam_lens uses it to file its metric history with
the session it watched.

WHY THIS EXISTS [audit, 2026-08-10]: the 12:02 manual fun session — the one
with the close-box destabilization worth diagnosing — left NO bag, NO saved
map, NO /cmd_vel record, and its logs survived only because nothing started
afterwards: simctl wrote every log to a flat name with mode 'w', so each
`start` destroyed the previous session's evidence. Three sessions of overnight
diagnosis data survived that night only because a human copied files by hand.

THE SHAPE (structured-logging practice, scaled to a bench robot):
  * one CORRELATION ID per session — the directory name — stamped into the
    manifest, the events timeline, the bag directory and the map filename,
    so "what happened at 12:04" is one directory, not five greps;
  * machine surface and human surface kept separate: `session.json`
    (atomic temp+rename writes — a crash never leaves half a manifest) and
    `events.log` (ISO-8601 stamped lines, append-only);
  * component logs keep their familiar names, INSIDE the session dir; the
    old flat paths become symlinks to the newest session so existing habits
    (`tail logs/simctl-slam.log`) still work;
  * recording is FAIL-OPEN: a recording failure is written down and never
    breaks the session it was recording.

Health counters are parsed from the component logs' REAL line formats (the
salvaged 12:02 logs are the fixtures); counters missing a log report None,
never 0 — "no evidence" and "zero events" are different facts.
"""
from __future__ import annotations

import hashlib
import json
import os
import re
import shutil
import time

# Resolved lazily so tests can point everything at a tmpdir.
from _layout import LOG_DIR

SESSIONS_DIRNAME = 'sessions'
MANIFEST = 'session.json'
EVENTS = 'events.log'
# Free-disk floor for default-on bag recording. The unattended rule ("an
# unbounded bag fills the disk by 4am") made structural.
MIN_FREE_GB_FOR_BAG = 5.0

# The topics a session bag must carry for a session to be diagnosable after
# the fact. /cmd_vel and /cmd_vel_raw are the audit's headline gap: no bag
# ever recorded here had the command stream, so no session could correlate
# "the map went weird" with "what was commanded".
SESSION_BAG_TOPICS = ('/scan', '/tf', '/tf_static', '/odom', '/odom_raw',
                      '/imu', '/sim/ground_truth', '/cmd_vel', '/cmd_vel_raw',
                      '/battery', '/odom_laser')
# /odom_laser joined 2026-08-10 evening: the corrected-EKF live A/B failed in
# a way the offline arm could not have (its pose input is /odom_laser, which
# no bag recorded) — the suspect must be on the record to be convictable.


def sessions_root(root: str | None = None) -> str:
    return os.path.join(root or LOG_DIR, SESSIONS_DIRNAME)


def new_session_dir(backend: str, fun: bool, domain: int,
                    root: str | None = None, stamp: str | None = None) -> str:
    """Create the session directory and repoint latest-d<domain>. -> path.

    The name IS the correlation id: <stamp>-<backend>[-fun]-d<domain>.
    latest is domain-scoped because parallel fleet sessions run on separate
    scratch domains and must never repoint each other's symlink.
    """
    stamp = stamp or time.strftime('%Y%m%d-%H%M%S')
    name = f'{stamp}-{backend}' + ('-fun' if fun else '') + f'-d{domain}'
    base = sessions_root(root)
    path = os.path.join(base, name)
    os.makedirs(path, exist_ok=True)
    link = os.path.join(base, f'latest-d{domain}')
    tmp = link + '.tmp'
    try:
        if os.path.lexists(tmp):
            os.unlink(tmp)
        os.symlink(name, tmp)
        os.replace(tmp, link)          # atomic repoint
    except OSError:
        pass                           # a broken symlink must never kill a start
    return path


def latest_session_dir(domain: int, root: str | None = None) -> str | None:
    link = os.path.join(sessions_root(root), f'latest-d{domain}')
    if os.path.isdir(link):
        return os.path.realpath(link)
    return None


def session_id(path: str) -> str:
    return os.path.basename(os.path.realpath(path))


def log_event(path: str, message: str) -> None:
    """Append one ISO-8601-stamped line to the session timeline. Fail-open."""
    try:
        with open(os.path.join(path, EVENTS), 'a') as f:
            f.write(f'{time.strftime("%Y-%m-%dT%H:%M:%S%z")} {message}\n')
    except OSError:
        pass


def write_manifest(path: str, fields: dict) -> None:
    """Merge fields into session.json, atomically (temp + rename).

    Merge, not replace: start writes what it knows, stop finalizes, and any
    failure path in between may add an `errors` entry — none of them may
    destroy what an earlier writer recorded.
    """
    p = os.path.join(path, MANIFEST)
    data = read_manifest(path) or {}
    data.update(fields)
    data.setdefault('session_id', session_id(path))
    tmp = p + '.tmp'
    with open(tmp, 'w') as f:
        json.dump(data, f, indent=2, sort_keys=True)
        f.write('\n')
    os.replace(tmp, p)


def read_manifest(path: str) -> dict | None:
    try:
        with open(os.path.join(path, MANIFEST)) as f:
            return json.load(f)
    except (OSError, ValueError):
        return None


def add_manifest_error(path: str, error: str) -> None:
    data = read_manifest(path) or {}
    errs = data.get('errors', [])
    errs.append(error)
    write_manifest(path, {'errors': errs})


def repoint_flat_symlinks(path: str, names, root: str | None = None) -> None:
    """Make logs/<name> a symlink to this session's copy, for old habits.

    A REAL FILE at the flat path (the pre-session-dir world, or a parallel
    old-simctl writer) is renamed aside once, not deleted — this module's
    whole reason to exist is that those files used to get destroyed.
    """
    base = root or LOG_DIR
    for name in names:
        flat = os.path.join(base, name)
        target = os.path.relpath(os.path.join(path, name), base)
        try:
            if os.path.islink(flat):
                os.unlink(flat)
            elif os.path.exists(flat):
                os.replace(flat, flat + '.pre-sessions')
            os.symlink(target, flat)
        except OSError:
            pass


def free_disk_gb(path: str) -> float:
    u = shutil.disk_usage(path if os.path.isdir(path) else os.path.dirname(path))
    return u.free / (1 << 30)


def bag_checksum(bag_dir: str) -> str:
    """SHA-256 over the bag's data files (same shape as replay_slam_bag's,
    shared here so manifests and replay tables agree byte-for-byte)."""
    h = hashlib.sha256()
    files = []
    for ext in ('.mcap', '.db3'):
        files += [os.path.join(bag_dir, f) for f in sorted(os.listdir(bag_dir))
                  if f.endswith(ext)] if os.path.isdir(bag_dir) else []
    for fp in files:
        with open(fp, 'rb') as f:
            for chunk in iter(lambda: f.read(1 << 20), b''):
                h.update(chunk)
    return h.hexdigest() if files else ''


# ------------------------------------------------------- health counters
# Parsed from the component logs' real formats; the 12:02 salvaged session
# is the fixture set. A missing log yields None ("no evidence"), never 0.

_RELAY_DROP = re.compile(r'\[scan_frame_relay\]: dropped (\d+) corrupted')
_PACING = re.compile(r'render pacing trimmed to ([0-9.]+)/s')
_DEGEN = re.compile(r'matched \d+/\d+, (\d+) degenerate')


def _read(path):
    try:
        with open(path, errors='replace') as f:
            return f.read()
    except OSError:
        return None


def session_log_counters(path: str) -> dict:
    """-> the per-session health numbers a morning reader greps for today."""
    out = {}

    slam = _read(os.path.join(path, 'simctl-slam.log'))
    out['slam_queue_full_drops'] = (None if slam is None
                                    else slam.count('queue is full'))

    isaac = _read(os.path.join(path, 'simctl-isaac.log'))
    if isaac is None:
        out['relay_dropped_scans'] = None
        out['pacing_trims'] = None
        out['pacing_final_renders_per_s'] = None
    else:
        drops = _RELAY_DROP.findall(isaac)
        # the relay logs a CUMULATIVE count; the last line is the total
        out['relay_dropped_scans'] = int(drops[-1]) if drops else 0
        trims = _PACING.findall(isaac)
        out['pacing_trims'] = len(trims)
        out['pacing_final_renders_per_s'] = float(trims[-1]) if trims else None

    lodom = _read(os.path.join(path, 'simctl-laserodom.log'))
    if lodom is None:
        out['laser_odometry_degenerate'] = None
    else:
        degs = _DEGEN.findall(lodom)
        # also cumulative per status line; last line is the total
        out['laser_odometry_degenerate'] = int(degs[-1]) if degs else 0

    return out
