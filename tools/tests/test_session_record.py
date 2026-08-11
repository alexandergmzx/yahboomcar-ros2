"""Tests for the session recorder. ROS-free; fixture log lines are verbatim
from the salvaged 12:02 session (the run whose missing data motivated this).

    python3 -m pytest tools/tests/test_session_record.py -q
"""
import json
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
import _session_record as sr                                    # noqa: E402


def _mk(tmp_path, backend='2d', fun=False, domain=66, stamp='20260810-120000'):
    return sr.new_session_dir(backend, fun, domain, root=str(tmp_path), stamp=stamp)


# ------------------------------------------------------------- naming

def test_session_dir_name_is_the_correlation_id(tmp_path):
    p = _mk(tmp_path, backend='isaac', fun=True, domain=66)
    assert os.path.isdir(p)
    assert os.path.basename(p) == '20260810-120000-isaac-fun-d66'
    assert sr.session_id(p) == '20260810-120000-isaac-fun-d66'


def test_latest_symlink_is_domain_scoped(tmp_path):
    p66 = _mk(tmp_path, domain=66, stamp='20260810-120000')
    p67 = _mk(tmp_path, domain=67, stamp='20260810-120001')
    assert sr.latest_session_dir(66, root=str(tmp_path)) == os.path.realpath(p66)
    assert sr.latest_session_dir(67, root=str(tmp_path)) == os.path.realpath(p67)
    # a second session on 66 repoints 66 and leaves 67 alone
    p66b = _mk(tmp_path, domain=66, stamp='20260810-130000')
    assert sr.latest_session_dir(66, root=str(tmp_path)) == os.path.realpath(p66b)
    assert sr.latest_session_dir(67, root=str(tmp_path)) == os.path.realpath(p67)


def test_latest_none_when_no_sessions(tmp_path):
    assert sr.latest_session_dir(66, root=str(tmp_path)) is None


# ------------------------------------------------------------ manifest

def test_manifest_merges_and_survives_finalize(tmp_path):
    p = _mk(tmp_path)
    sr.write_manifest(p, {'backend': '2d', 'flags': {'fun': False}})
    sr.write_manifest(p, {'stopped': '2026-08-10T13:00:00'})
    m = sr.read_manifest(p)
    assert m['backend'] == '2d'                 # start's fields survive stop's
    assert m['stopped'] == '2026-08-10T13:00:00'
    assert m['session_id'] == sr.session_id(p)


def test_manifest_write_is_atomic_no_tmp_left(tmp_path):
    p = _mk(tmp_path)
    sr.write_manifest(p, {'a': 1})
    assert not os.path.exists(os.path.join(p, sr.MANIFEST + '.tmp'))
    assert json.load(open(os.path.join(p, sr.MANIFEST)))['a'] == 1


def test_manifest_errors_accumulate(tmp_path):
    p = _mk(tmp_path)
    sr.add_manifest_error(p, 'bag recorder died')
    sr.add_manifest_error(p, 'map save timed out')
    assert sr.read_manifest(p)['errors'] == ['bag recorder died',
                                             'map save timed out']


# -------------------------------------------------------------- events

def test_events_are_stamped_append_only(tmp_path):
    p = _mk(tmp_path)
    sr.log_event(p, 'bag recording started')
    sr.log_event(p, 'map saved')
    lines = open(os.path.join(p, sr.EVENTS)).read().splitlines()
    assert len(lines) == 2
    assert lines[0].endswith('bag recording started')
    assert lines[0][:4].isdigit()               # ISO stamp leads the line


# ------------------------------------------------------- flat symlinks

def test_flat_names_become_symlinks_and_real_files_are_preserved(tmp_path):
    # The pre-sessions world: a REAL flat log exists. It must be renamed
    # aside, never destroyed — destroying it is the bug this module fixes.
    flat = tmp_path / 'simctl-slam.log'
    flat.write_text('previous session evidence')
    p = _mk(tmp_path)
    open(os.path.join(p, 'simctl-slam.log'), 'w').write('new session')
    sr.repoint_flat_symlinks(p, ['simctl-slam.log'], root=str(tmp_path))
    assert os.path.islink(flat)
    assert open(flat).read() == 'new session'
    assert (tmp_path / 'simctl-slam.log.pre-sessions').read_text() \
        == 'previous session evidence'


def test_flat_symlink_repoints_to_newest_session(tmp_path):
    p1 = _mk(tmp_path, stamp='20260810-120000')
    open(os.path.join(p1, 'simctl-slam.log'), 'w').write('one')
    sr.repoint_flat_symlinks(p1, ['simctl-slam.log'], root=str(tmp_path))
    p2 = _mk(tmp_path, stamp='20260810-130000')
    open(os.path.join(p2, 'simctl-slam.log'), 'w').write('two')
    sr.repoint_flat_symlinks(p2, ['simctl-slam.log'], root=str(tmp_path))
    assert open(tmp_path / 'simctl-slam.log').read() == 'two'
    assert open(os.path.join(p1, 'simctl-slam.log')).read() == 'one'  # untouched


# ------------------------------------------------------------ counters
# Fixture lines are VERBATIM from the salvaged 12:02 session logs.

def test_counters_parse_the_real_formats(tmp_path):
    p = _mk(tmp_path)
    open(os.path.join(p, 'simctl-slam.log'), 'w').write(
        "[async_slam_toolbox_node-1] [INFO] [1786384975.676532316] [slam_toolbox]:"
        " Message Filter dropping message: frame 'laser_frame' at time"
        " 1786384975.601 for reason 'discarding message because the queue is full'\n"
        * 89)
    open(os.path.join(p, 'simctl-isaac.log'), 'w').write(
        '  scan rate measured 12.8 Hz -> render pacing trimmed to 3.19/s\n'
        '[INFO] [1786385166.135544115] [scan_frame_relay]: dropped 1 corrupted'
        ' scans (2462 passed)\n'
        '  scan rate measured 12.8 Hz -> render pacing trimmed to 3.09/s\n')
    open(os.path.join(p, 'simctl-laserodom.log'), 'w').write(
        '[INFO] [1786384979.808345877] [laser_odometry]: matched 64/64,'
        ' 0 degenerate, 0 over time budget\n'
        '[INFO] [1786384989.808048021] [laser_odometry]: matched 201/201,'
        ' 2 degenerate, 0 over time budget\n')
    c = sr.session_log_counters(p)
    assert c['slam_queue_full_drops'] == 89
    assert c['relay_dropped_scans'] == 1          # cumulative: last line wins
    assert c['pacing_trims'] == 2
    assert c['pacing_final_renders_per_s'] == 3.09
    assert c['laser_odometry_degenerate'] == 2    # cumulative: last line wins


def test_counters_missing_log_is_none_not_zero(tmp_path):
    # "no evidence" and "zero events" are different facts.
    p = _mk(tmp_path)
    c = sr.session_log_counters(p)
    assert c['slam_queue_full_drops'] is None
    assert c['relay_dropped_scans'] is None
    assert c['laser_odometry_degenerate'] is None


def test_counters_present_but_quiet_log_is_zero(tmp_path):
    p = _mk(tmp_path)
    open(os.path.join(p, 'simctl-slam.log'), 'w').write('[INFO] all well\n')
    open(os.path.join(p, 'simctl-isaac.log'), 'w').write('booted fine\n')
    c = sr.session_log_counters(p)
    assert c['slam_queue_full_drops'] == 0
    assert c['relay_dropped_scans'] == 0
    assert c['pacing_trims'] == 0


# ---------------------------------------------------------------- misc

def test_bag_checksum_empty_dir_is_empty_string(tmp_path):
    assert sr.bag_checksum(str(tmp_path)) == ''


def test_bag_checksum_covers_data_files(tmp_path):
    (tmp_path / 'x_0.mcap').write_bytes(b'abc')
    c1 = sr.bag_checksum(str(tmp_path))
    (tmp_path / 'x_1.mcap').write_bytes(b'def')
    c2 = sr.bag_checksum(str(tmp_path))
    assert c1 and c2 and c1 != c2


def test_free_disk_positive(tmp_path):
    assert sr.free_disk_gb(str(tmp_path)) > 0
