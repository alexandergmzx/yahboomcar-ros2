#!/usr/bin/env python3
"""Execute every command in README.md. Exits NONZERO if any fails.

    ./tools/test_readme.py              # run the suite
    ./tools/test_readme.py --list       # show what would run, run nothing
    ./tools/test_readme.py --coverage   # print the summary the README quotes

WHY
---
A command that has never been executed is a claim, not a fact, and this repository has
already shipped several -- in both directions.

Nav2's parameters "configured" through four fixed breakages and still could not navigate
until a fifth was found by actually running it. And `map_gmapping_launch.py` was written
off in this repo's own documentation as impossible on Jazzy, in three separate files, on
the strength of nobody having tried it; `slam_gmapping` needed seven include lines moved
from .h to .hpp and then worked. Untested claims rot toward whatever was assumed, and
"this is broken" is as damaging a false claim as "this works".

So the README is not prose here. It is a test fixture: every fenced command is extracted
and run, and the README is only as true as this file's exit code.

HOW A COMMAND DECLARES WHAT IT NEEDS
------------------------------------
Each fenced block carries a tag on the fence line, `” ```bash test:auto ” `:

    auto         needs nothing running. Executed; must exit 0.
    lifecycle    IS the simulator's start/stop. Used as the suite's own bracket, so the
                 two commands a reader types first are executed rather than described.
    sim          needs a simulator. The suite starts one, runs it, tears it down.
    longrunning  never exits (launch files, RViz). Started, checked alive, killed.
    hardware     genuinely needs the car. NOT executed -- and the prose around it must
                 say so, or this suite fails. An untestable command that does not admit
                 it is exactly the kind of claim this file exists to prevent.
    skip         shown for context, deliberately not run (destructive, or an example).

An untagged command block is a FAILURE, not a pass. Silence is how untested commands get
into documentation in the first place.
"""
import argparse
import os
import re
import subprocess
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _layout import REPO, WS_SETUP                              # noqa: E402
README = os.path.join(REPO, 'README.md')
SIM_DOMAIN = 66

TAGS = ('auto', 'lifecycle', 'sim', 'longrunning', 'hardware', 'skip')
# Flags forced onto the README's `simctl start` when the suite runs it. No RViz window to
# open, no robot driving itself while other commands are measured -- and no SLAM, because
# the README shows `slam_toolbox_launch.py` separately and that launch should be what
# proves itself, not a second copy racing simctl's.
LIFECYCLE_START_FLAGS = ' --no-rviz --no-patrol --no-slam'
# Prose near a `hardware` block must contain one of these, so a reader is told before
# they try it.
HARDWARE_WORDS = ('needs the robot', 'needs hardware', 'real robot', 'physical robot',
                  'not tested in ci', 'requires the car', 'hardware only')


def parse(path):
    """-> [(tag, command, line_no, context)] for every fenced block."""
    text = open(path).read()
    lines = text.split('\n')
    blocks, i = [], 0
    while i < len(lines):
        m = re.match(r'^```(\w+)?(?:\s+test:(\w+))?\s*$', lines[i])
        if m:
            tag = m.group(2)
            lang = m.group(1)
            start = i
            body = []
            i += 1
            while i < len(lines) and not lines[i].startswith('```'):
                body.append(lines[i])
                i += 1
            if lang in (None, 'text', 'yaml', 'json'):
                i += 1
                continue                    # not a command block
            context = '\n'.join(lines[max(0, start - 6):start]).lower()
            # Rejoin `\` continuations first: a wrapped command is one command, and
            # splitting it would both mis-count coverage and run two broken halves.
            joined, buf = [], ''
            for raw in body:
                buf += raw.rstrip()
                if buf.endswith('\\'):
                    buf = buf[:-1] + ' '
                    continue
                joined.append(buf)
                buf = ''
            if buf:
                joined.append(buf)
            for cmd in joined:
                # A trailing `  # ...` is prose for the reader, not part of the command,
                # and it prevents the lifecycle bracket from appending its own flags.
                cmd = re.sub(r'\s+#.*$', '', cmd).strip()
                if not cmd or cmd.startswith('#'):
                    continue
                blocks.append((tag, cmd, start + 1, context))
        i += 1
    return blocks


def run(cmd, timeout, domain=None, cwd=REPO):
    env = dict(os.environ)
    if domain is not None:
        env['ROS_DOMAIN_ID'] = str(domain)
    env.setdefault('DISPLAY', ':0')
    full = (f'source /opt/ros/jazzy/setup.bash 2>/dev/null; '
            f'source {WS_SETUP} 2>/dev/null; {cmd}')
    try:
        p = subprocess.run(['bash', '-c', full], cwd=cwd, env=env,
                           capture_output=True, text=True, timeout=timeout)
        # 500 chars once truncated away the actual verdict line and left only a
        # table of channel names, which cost a whole re-run to diagnose.
        return p.returncode, (p.stdout + p.stderr)[-2000:]
    except subprocess.TimeoutExpired:
        return 'timeout', ''


def run_longrunning(cmd, domain, settle=20):
    """Start it, confirm it is alive after `settle`, then kill its process group."""
    env = dict(os.environ)
    env['ROS_DOMAIN_ID'] = str(domain)
    env.setdefault('DISPLAY', ':0')
    full = (f'source /opt/ros/jazzy/setup.bash 2>/dev/null; '
            f'source {WS_SETUP} 2>/dev/null; exec {cmd}')
    p = subprocess.Popen(['bash', '-c', full], cwd=REPO, env=env,
                         start_new_session=True,
                         stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    time.sleep(settle)
    alive = p.poll() is None
    try:
        os.killpg(os.getpgid(p.pid), 15)
    except (ProcessLookupError, PermissionError):
        pass
    time.sleep(2)
    try:
        os.killpg(os.getpgid(p.pid), 9)
    except (ProcessLookupError, PermissionError):
        pass
    if alive:
        return 0, 'stayed up'
    out = ''
    try:
        out = (p.stdout.read() or '')[-400:]
    except Exception:
        pass
    return 1, f'exited early: {out}'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--list', action='store_true')
    ap.add_argument('--coverage', action='store_true')
    ap.add_argument('--readme', default=README)
    ap.add_argument('--timeout', type=float, default=120)
    args = ap.parse_args()

    # Runs take minutes and are usually redirected to a log. Without this the whole
    # report appears only at the end, and a hung command looks identical to a slow one.
    sys.stdout.reconfigure(line_buffering=True)

    blocks = parse(args.readme)
    if not blocks:
        print(f'no command blocks found in {args.readme}')
        return 2

    counts = {t: 0 for t in TAGS}
    untagged = [b for b in blocks if b[0] is None]
    for t, *_ in blocks:
        if t in counts:
            counts[t] += 1

    if args.list or args.coverage:
        for tag, cmd, ln, _ in blocks:
            print(f'  {str(tag or "UNTAGGED"):12s} L{ln:<4} {cmd[:80]}')
        print()
        print(f'  {len(blocks)} commands: ' +
              ', '.join(f'{n} {t}' for t, n in counts.items() if n))
        if untagged:
            print(f'  {len(untagged)} UNTAGGED -- these would fail the suite')
        return 0 if not untagged else 1

    print(f'=== README command suite: {len(blocks)} commands ===')
    failures = []

    # An untagged block is a failure. Silence is how untested commands get into docs.
    for tag, cmd, ln, _ in untagged:
        failures.append((ln, cmd, 'UNTAGGED -- add test:auto|sim|longrunning|hardware|skip'))

    # `hardware` commands are not run, but the prose must warn the reader.
    for tag, cmd, ln, context in blocks:
        if tag == 'hardware' and not any(w in context for w in HARDWARE_WORDS):
            failures.append((ln, cmd,
                             'tagged hardware but the surrounding text never says so'))

    auto = [b for b in blocks if b[0] == 'auto']
    print(f'\n--- {len(auto)} auto (need nothing running) ---')
    for tag, cmd, ln, _ in auto:
        rc, out = run(cmd, args.timeout)
        ok = rc == 0
        print(f'  [{"ok" if ok else "FAIL"}] L{ln} {cmd[:70]}')
        if not ok:
            failures.append((ln, cmd, f'exit {rc}: {out.strip()[-800:]}'))

    # The simulator's own start/stop are the first two commands anyone types, so the suite
    # uses the README's literal versions as its bracket rather than a private copy that
    # could drift away from what the README claims.
    lifecycle = [b for b in blocks if b[0] == 'lifecycle']
    start_b = next((b for b in lifecycle if re.search(r'\bstart\b', b[1])), None)
    stop_b = next((b for b in lifecycle if re.search(r'\bstop\b', b[1])), None)
    for tag, cmd, ln, _ in lifecycle:
        if (tag, cmd, ln) not in [(b[0], b[1], b[2]) for b in (start_b, stop_b) if b]:
            failures.append((ln, cmd, 'tagged lifecycle but is neither a start nor a stop'))

    sim_cmds = [b for b in blocks if b[0] in ('sim', 'longrunning')]
    if sim_cmds or lifecycle:
        print(f'\n--- {len(sim_cmds) + len(lifecycle)} needing a simulator ---')
        start_cmd = (start_b[1] + LIFECYCLE_START_FLAGS) if start_b \
            else f'{REPO}/tools/simctl start --no-rviz --no-patrol'
        rc, out = run(start_cmd, 400)
        ok = rc == 0
        if start_b:
            print(f'  [{"ok" if ok else "FAIL"}] L{start_b[2]} {start_cmd[:70]}')
        if not ok:
            failures.append((start_b[2] if start_b else 0, start_cmd,
                             f'could not start a simulator: exit {rc}: {out[:200]}'))
        else:
            for tag, cmd, ln, _ in sim_cmds:
                if tag == 'longrunning':
                    rc, out = run_longrunning(cmd, SIM_DOMAIN)
                else:
                    rc, out = run(cmd, args.timeout, domain=SIM_DOMAIN)
                ok = rc == 0
                print(f'  [{"ok" if ok else "FAIL"}] L{ln} {cmd[:70]}')
                if not ok:
                    failures.append((ln, cmd, f'exit {rc}: {out.strip()[-800:]}'))

        # Tear down even if start failed -- a half-started stack still has processes.
        stop_cmd = stop_b[1] if stop_b else f'{REPO}/tools/simctl stop'
        rc, out = run(stop_cmd, 300)
        if stop_b:
            print(f'  [{"ok" if rc == 0 else "FAIL"}] L{stop_b[2]} {stop_cmd[:70]}')
            if rc != 0:
                failures.append((stop_b[2], stop_cmd, f'exit {rc}: {out.strip()[:200]}'))

    hw = [b for b in blocks if b[0] == 'hardware']
    if hw:
        print(f'\n--- {len(hw)} hardware-only (NOT executed) ---')
        for tag, cmd, ln, _ in hw:
            print(f'  [skip] L{ln} {cmd[:70]}')

    print()
    print('=== coverage ===')
    verified = (counts['auto'] + counts['lifecycle'] + counts['sim']
                + counts['longrunning'])
    print(f'  {len(blocks)} commands: {verified} executed by this suite, '
          f'{counts["hardware"]} hardware-only, {counts["skip"]} skipped')

    if failures:
        print()
        print(f'=== {len(failures)} FAILURES ===')
        for ln, cmd, why in failures:
            print(f'  L{ln} {cmd[:70]}')
            print(f'      {why}')
        return 1
    print('  every command in the README ran and succeeded.')
    return 0


if __name__ == '__main__':
    code = main()
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(code)
