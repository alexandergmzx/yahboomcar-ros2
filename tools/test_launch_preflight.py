#!/usr/bin/env python3
"""Launch first_floor_launch.py and assert its own preflight can actually pass.

    ./tools/test_launch_preflight.py

Needs no car -- it only checks what the nodes say about each other.

WHY: an audit found the mandatory floor launch failed its own safety check. The deadman
must publish /cmd_vel to do its job; the governor called every other /cmd_vel publisher a
bypass; and docs/first-floor-procedure.md says abort if BYPASSED appears. So the
documented preflight could never pass legitimately, and the only ways through were to
ignore a safety warning or to drop the deadman.

Unit tests could not have caught it: each node was correct alone, and the contradiction
only existed once the launch file put them together. Hence a launch-level test.
"""
import os
import re
import subprocess
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _layout import WS_SETUP                                    # noqa: E402


def main():
    env = dict(os.environ)
    env.setdefault('ROS_DOMAIN_ID', '20')
    cmd = ('source /opt/ros/jazzy/setup.bash && '
           f'source {WS_SETUP} && '
           'exec ros2 launch yahboomcar_safety first_floor_launch.py')
    p = subprocess.Popen(['bash', '-c', cmd], env=env, text=True,
                         stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                         start_new_session=True)

    # The governor's bypass check runs on a 5 s timer, so watch across two of them.
    out, deadline = [], time.time() + 22
    try:
        os.set_blocking(p.stdout.fileno(), False)
        while time.time() < deadline:
            line = p.stdout.readline()
            if line:
                out.append(line.rstrip())
                print(line.rstrip(), flush=True)
            else:
                time.sleep(0.05)
    finally:
        try:
            os.killpg(os.getpgid(p.pid), 15)
        except (ProcessLookupError, PermissionError):
            pass

    text = '\n'.join(out)
    fails = []
    if re.search(r'BYPASSED', text):
        fails.append('governor reported BYPASSED -- the procedure says abort, so this '
                     'launch cannot pass its own preflight')
    if re.search(r'NO DEADMAN', text):
        fails.append('governor reports no deadman, but this launch makes it mandatory')
    if not re.search(r'safety companion present', text):
        fails.append('governor never positively confirmed the deadman; absence of a '
                     'complaint is not the same as confirmation')
    if not re.search(r'governor up', text):
        fails.append('governor never started')
    if not re.search(r'deadman up', text):
        fails.append('deadman never started')

    print()
    if fails:
        for f in fails:
            print(f'FAIL: {f}')
        return 1
    print('PASS: first_floor_launch.py preflight is self-consistent')
    print('      governor and deadman both up, deadman confirmed, no bypass reported')
    return 0


if __name__ == '__main__':
    sys.exit(main())
