#!/usr/bin/env python3
"""Assert RViz is actually DRAWING the displays you think it is. By pixels.

    ./tools/check_rviz_render.py --expect map,scan,robot
    ./tools/check_rviz_render.py --expect map,scan --save /tmp/rviz.png --json out.json
    ./tools/check_rviz_render.py --list          # what it can see, no pass/fail

Exits nonzero naming whichever expected display is not on screen.

WHY THIS EXISTS
---------------
Because "rviz2 stayed alive for 18 s with zero error lines" was accepted as verification
that two rewritten RViz configs worked, and it proved nothing at all. RViz renders an
empty world just as quietly as a full one: a display with no publisher, a wrong topic, a
missing frame and a display that was never created all produce the same silence.

**Alex found the actual defect by looking at the screen** -- the map was missing from
every fun session for hours, because SLAM had been switched off upstream, and no check
in this repo could have told the difference. That is the same failure the SLAM audit
named ("a green RViz status is not acceptance evidence"), and the fix for it is not more
care. It is a measurement.

So this captures the window and counts pixels. A map that is drawn puts thousands of
light-grey cells on screen; a map that is absent puts none. That difference is
observable, so it should be observed.

WHAT IT CANNOT TELL YOU
-----------------------
That the map is CORRECT. Pixels prove a display is populated, not that its contents mean
anything -- a smeared map and a good map both pass. Map quality is
`tools/score_slam_map.py`, against tape measurements. This tool answers exactly one
question: is it on the screen.

It also cannot see an occluded or scrolled-away display, and it needs the window
visible (not minimised, not behind another). A false FAIL from a hidden window is
possible; a false PASS is not, which is the safer direction.
"""
import argparse
import json
import os
import re
import subprocess
import sys
import tempfile

import numpy as np

# Minimum pixels for a display to count as drawn. Set well above antialiasing noise on
# a few stray marks and well below anything genuinely rendered: a 4 m map at the default
# zoom is tens of thousands of cells, a scan ring is thousands of points.
DEFAULT_MIN_PIXELS = {
    'map': 2000,
    'scan': 300,
    'robot': 200,
    'grid': 500,
}


def find_window():
    """-> (render_panel_id, top_level_id). Either may be None.

    RViz's 3D view is its OWN X window -- a child named "rviz2" under the top-level,
    e.g. `0x5c00013 "rviz2": ("rviz2" "rviz2")  988x852+496+67`. Capturing THAT means
    the frame is the viewport and nothing else, which is worth far more than it
    sounds: every cropping heuristic tried before this was content-dependent and
    wrong. Cropping to where the background colour appears shrank the "viewport" as
    the map filled it (measured: y 67-919 with little map, y 67-440 with more, and
    then 0 map pixels because the map had been cropped away by its own presence).
    A window id does not move when the picture changes.
    """
    try:
        tops = subprocess.run(['xdotool', 'search', '--name', 'RViz'],
                              capture_output=True, text=True, timeout=15).stdout.split()
        for wid in reversed(tops):
            geo = subprocess.run(['xdotool', 'getwindowgeometry', wid],
                                 capture_output=True, text=True, timeout=10).stdout
            m = re.search(r'Geometry: (\d+)x(\d+)', geo)
            if not (m and int(m.group(1)) > 400 and int(m.group(2)) > 300):
                continue
            tree = subprocess.run(['xwininfo', '-id', wid, '-tree'],
                                  capture_output=True, text=True, timeout=15).stdout
            best, best_area = None, 0
            for line in tree.splitlines():
                cm = re.search(r'(0x[0-9a-f]+)\s+"rviz2".*?(\d+)x(\d+)\+', line)
                if cm:
                    area = int(cm.group(2)) * int(cm.group(3))
                    if area > best_area:
                        best, best_area = cm.group(1), area
            return best, wid
    except (subprocess.SubprocessError, OSError):
        pass
    return None, None


def capture(window_id=None, path=None):
    """Grab the window (or the root) to a PNG. -> path."""
    path = path or os.path.join(tempfile.gettempdir(), 'rviz_capture.png')
    cmd = ['import', '-silent']
    cmd += ['-window', window_id or 'root']
    cmd.append(path)
    r = subprocess.run(cmd, capture_output=True, text=True, timeout=60)
    if r.returncode != 0 or not os.path.exists(path):
        raise RuntimeError(f'screen capture failed: {r.stderr.strip() or r.returncode}')
    return path


def parse_rviz_colors(cfg_path):
    """Pull the palette OUT OF THE CONFIG, so the checker follows a retheme.

    -> {'background': (r,g,b), 'scan': (r,g,b) or None, 'grid': (r,g,b) or None}
    Deliberately not a YAML load: these configs are hand-written with comments and
    duplicate keys across displays, and a structural parse would need the display order
    anyway. The keys are unambiguous by name.
    """
    colors = {'background': (48, 48, 48), 'scan': None, 'grid': None}
    if not cfg_path or not os.path.exists(cfg_path):
        return colors
    text = open(cfg_path).read()
    m = re.search(r'Background Color:\s*(\d+);\s*(\d+);\s*(\d+)', text)
    if m:
        colors['background'] = tuple(int(g) for g in m.groups())
    # Colour of whichever display block mentions LaserScan first.
    scan_block = text.split('rviz_default_plugins/LaserScan', 1)
    if len(scan_block) > 1:
        m = re.search(r'\n\s+Color:\s*(\d+);\s*(\d+);\s*(\d+)', scan_block[1])
        if m:
            colors['scan'] = tuple(int(g) for g in m.groups())
    grid_block = text.split('rviz_default_plugins/Grid', 1)
    if len(grid_block) > 1:
        m = re.search(r'\n\s+Color:\s*(\d+);\s*(\d+);\s*(\d+)', grid_block[1])
        if m:
            colors['grid'] = tuple(int(g) for g in m.groups())
    return colors


def near(px, rgb, tol):
    """Bool mask of pixels within `tol` (per channel, Chebyshev) of an RGB triple."""
    return (np.abs(px.astype(int) - np.array(rgb, dtype=int)).max(axis=2) <= tol)


def viewport_bbox(px, background):
    """Bounding box of the 3D view: (top, bottom, left, right). RViz's own UI excluded.

    TWO false positives were measured building this, and both would have made the tool
    report "map drawn" for a session with no map:

      1. a FIXED 22% left crop left a 140 px strip of the white Displays panel in
         frame (RViz's panel was ~31% wide), and white scores as map free space;
      2. cropping only the LEFT edge still left the menu bar (67,796 px), the status
         bar (31,187 px) and the right-edge panel strip -- all light grey UI -- which
         together classified as 112,575 px of "map" in a mapless session.

    So the crop is taken from where the configured background colour actually appears,
    in both axes: the 3D viewport is the only region rendering it. Requiring a whole
    row/column to be >2% background keeps a stray antialiased pixel in the UI from
    widening the box.
    """
    bg = near(px, background, 12)
    rows = np.nonzero(bg.mean(axis=1) > 0.02)[0]
    cols = np.nonzero(bg.mean(axis=0) > 0.02)[0]
    if rows.size == 0 or cols.size == 0:
        # No background visible at all -- a map filling the view could do this. Fall
        # back to the whole frame and let the caller see the box was not found.
        return 0, px.shape[0], 0, px.shape[1]
    return int(rows.min()), int(rows.max()) + 1, int(cols.min()), int(cols.max()) + 1


def analyse(png_path, colors, crop_left=None):
    """Classify the frame. -> (counts dict, total pixels considered, bbox).

    `crop_left` forces a left-crop fraction (whole height); None auto-detects the 3D
    viewport in both axes, which is what excludes RViz's own UI from the counts.
    """
    from PIL import Image
    img = Image.open(png_path).convert('RGB')
    px = np.array(img)
    if crop_left is not None:
        bbox = (0, px.shape[0], int(px.shape[1] * crop_left), px.shape[1])
    else:
        bbox = viewport_bbox(px, colors['background'])
    top, bottom, left, right = bbox
    px = px[top:bottom, left:right, :]
    total = px.shape[0] * px.shape[1]
    counts = {}

    bg = near(px, colors['background'], 12)
    counts['background'] = int(bg.sum())

    # MAP: occupancy is drawn in greys. Free space is light (~205-255 in the 'map'
    # colour scheme), walls near-black. Require near-neutral saturation so the amber
    # scan and coloured robot links cannot be mistaken for it, and exclude the
    # background grey explicitly.
    mx = px.astype(int)
    spread = mx.max(axis=2) - mx.min(axis=2)
    lightness = mx.mean(axis=2)
    neutral = spread <= 18
    counts['map'] = int((neutral & (lightness >= 150) & ~bg).sum())

    if colors['scan']:
        counts['scan'] = int(near(px, colors['scan'], 60).sum())
    else:
        counts['scan'] = 0

    # ROBOT: the URDF meshes render coloured and shaded -- anything with real
    # saturation that is not the scan colour.
    scan_mask = near(px, colors['scan'], 60) if colors['scan'] else np.zeros(
        px.shape[:2], dtype=bool)
    counts['robot'] = int(((spread > 30) & ~scan_mask).sum())

    if colors['grid']:
        counts['grid'] = int(near(px, colors['grid'], 14).sum())
    else:
        counts['grid'] = 0
    return counts, total, bbox


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--expect', default='',
                    help='comma-separated displays that MUST be drawn: '
                         'map,scan,robot,grid')
    ap.add_argument('--config', default='',
                    help='the .rviz file in use, for the palette (recommended)')
    ap.add_argument('--save', default='', help='keep the captured PNG here')
    ap.add_argument('--json', default='')
    ap.add_argument('--list', action='store_true',
                    help='report counts and exit 0, asserting nothing')
    ap.add_argument('--crop-left', type=float, default=None,
                    help='force a crop fraction; default auto-detects the 3D viewport')
    ap.add_argument('--min-pixels', default='',
                    help='override thresholds, e.g. map=5000,scan=100')
    args = ap.parse_args()

    thresholds = dict(DEFAULT_MIN_PIXELS)
    for item in filter(None, args.min_pixels.split(',')):
        k, v = item.split('=')
        thresholds[k.strip()] = int(v)

    expected = [e.strip() for e in args.expect.split(',') if e.strip()]
    render_id, top_id = find_window()
    wid = render_id or top_id
    if render_id:
        print(f'=== check_rviz_render  3D render panel {render_id} '
              f'(top level {top_id})')
    else:
        print(f'=== check_rviz_render  window {wid or "(none found)"}')
    if wid is None and expected:
        # HARD FAIL, not a warning. Measured while building this: with RViz stopped,
        # capturing the root window photographed the DESKTOP and passed every display
        # -- 426,101 px of "robot" from ordinary window chrome. A gate that passes when
        # the thing under test is not running is worse than no gate.
        print('  FAIL: no RViz window exists. Nothing can be drawn, so nothing can be')
        print('  asserted. (Use --list to inspect the screen anyway.)')
        return 1
    if wid is None:
        print('  WARNING: capturing the whole screen -- no RViz window found. Counts')
        print('  below describe the desktop, not RViz.')
    png = capture(wid, args.save or None)
    colors = parse_rviz_colors(args.config)
    # Capturing the render panel means the frame IS the viewport: no cropping, and
    # nothing content-dependent. Only a whole-window/root capture needs the fallback.
    crop = args.crop_left if not render_id else 0.0
    counts, total, bbox = analyse(png, colors, crop if render_id else args.crop_left)
    top, bottom, left, right = bbox

    print(f'  capture: {png}')
    if render_id:
        print(f'  frame is the 3D viewport itself ({total} px; no crop needed)')
    else:
        print(f'  cropped viewport: x {left}-{right}, y {top}-{bottom} '
              f'({total} px analysed)')
    print(f'  palette: background {colors["background"]}  scan {colors["scan"]}  '
          f'grid {colors["grid"]}')
    print()
    for name in ('map', 'scan', 'robot', 'grid', 'background'):
        pct = 100.0 * counts.get(name, 0) / total if total else 0.0
        thr = thresholds.get(name)
        mark = ''
        if thr is not None:
            mark = 'drawn' if counts.get(name, 0) >= thr else 'ABSENT'
        print(f'    {name:<12} {counts.get(name, 0):8d} px  {pct:5.1f}%   {mark}')

    missing = [e for e in expected
               if counts.get(e, 0) < thresholds.get(e, DEFAULT_MIN_PIXELS.get(e, 1))]
    payload = {'window': wid, 'capture': png, 'counts': counts, 'total': total,
               'viewport': {'top': top, 'bottom': bottom, 'left': left,
                            'right': right},
               'expected': expected, 'missing': missing,
               'thresholds': {k: thresholds.get(k) for k in expected}}
    if args.json:
        with open(args.json, 'w') as f:
            json.dump(payload, f, indent=2)
        print(f'\n  json: {args.json}')

    print()
    if args.list:
        print('  (--list: asserting nothing)')
        return 0
    if not expected:
        print('  nothing expected; pass --expect to make this a gate')
        return 0
    if missing:
        print(f'  FAIL: not drawn: {", ".join(missing)}')
        for name in missing:
            print(f'    {name}: {counts.get(name, 0)} px < '
                  f'{thresholds.get(name)} px required')
        print('  A display can be absent because it has no publisher, its topic or QoS')
        print('  is wrong, its frame does not exist, or it was never created. This tool')
        print('  says WHICH is missing, not why.')
        return 1
    print(f'  PASS: {", ".join(expected)} all drawn')
    return 0


if __name__ == '__main__':
    sys.exit(main())
