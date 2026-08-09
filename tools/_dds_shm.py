"""Which FastDDS shared-memory segments are safe to delete. Import it; do not run it.

    from _dds_shm import stale_segments, mapped_shm_files

WHY THIS IS SHARED AND NOT COPIED
---------------------------------
Because the unscoped version has already caused a real outage, and a second private copy
is how it comes back. `simctl stop` once globbed every `/dev/shm/fastrtps*`
unconditionally: with Isaac running on domain 66, `simctl stop --domain 67` removed 147
segments and Isaac's `/scan` went from 12.9 Hz to SILENT while its process stayed
alive -- DDS transport destroyed under a running participant, with nothing logged
anywhere.

It nearly happened again on 2026-08-10: `tools/replay_slam_bag.py` was written with its
own `glob(...)+unlink` teardown, and the machine turned out to be running two
`robot2_sim_bringup` processes from a PARALLEL FLEET SESSION at the time. Deleting their
transport would have taken down another session's work with no error message anywhere.
Caught before the first run, which is the only reason it is a comment and not a second
incident.

So: one definition, and the word "stale" has to be EARNED -- a segment is stale only
when no live process has it mapped. Same reasoning as `_layout.REPO` and
`_cmd_vel_safety.CAR_DOMAIN`: two copies eventually disagree, and the one that is wrong
is the one somebody trusts.
"""
import glob

SEGMENT_GLOBS = ('/dev/shm/fastrtps*', '/dev/shm/sem.fastrtps*')


def mapped_shm_files():
    """Every /dev/shm file currently mapped by a live process."""
    live = set()
    for maps in glob.glob('/proc/[0-9]*/maps'):
        try:
            with open(maps) as f:
                for line in f:
                    i = line.find('/dev/shm/')
                    if i != -1:
                        live.add(line[i:].strip())
        except OSError:
            continue          # process exited, or not ours to read
    return live


def all_segments():
    out = []
    for pattern in SEGMENT_GLOBS:
        out += glob.glob(pattern)
    return out


def stale_segments():
    """Segments NO live process is using -- the only ones safe to unlink."""
    live = mapped_shm_files()
    return [s for s in all_segments() if s not in live]
