#!/usr/bin/env python3
"""Live top-down lidar view in the terminal.

    ./tools/lidar_view.py                 # live from the robot
    ./tools/lidar_view.py --range 3.0     # wider window, metres
    ./tools/lidar_view.py --once          # single frame, for logs or CI

No GUI, no rviz, no X11 -- so it works over ssh and cannot fail the way a simulator can.
Forward is UP. The robot sits at the centre marked 'R'.

It also draws the safety governor's zones, so you can see what the governor sees rather
than trusting its logs:

    !  inside stop_distance  -> governor commands a hard stop
    o  inside slow_distance  -> governor scales speed down
    .  clear

The forward sector the governor actually watches (+/-45 deg) is marked on the readout,
because an obstacle to the side is not what stops the robot -- a distinction that is easy
to lose when looking at a full 360 degree picture.
"""
import argparse
import math
import os
import sys


def render(ranges, angle_min, angle_inc, max_range, cols=61, rows=25,
           stop_d=0.35, slow_d=0.90):
    """ASCII top-down plot. Returns a list of lines."""
    grid = [[' '] * cols for _ in range(rows)]
    cx, cy = cols // 2, rows // 2

    for i, r in enumerate(ranges):
        if r is None or not math.isfinite(r) or r <= 0.02 or r > max_range:
            continue
        a = angle_min + i * angle_inc
        # ROS: x forward, y left, yaw CCW. Screen: up = forward, left = +y.
        x = r * math.cos(a)
        y = r * math.sin(a)
        col = int(round(cx - y / max_range * (cols // 2)))
        row = int(round(cy - x / max_range * (rows // 2)))
        if 0 <= col < cols and 0 <= row < rows:
            mark = '!' if r <= stop_d else ('o' if r <= slow_d else '.')
            # Never let a farther point overwrite a nearer, more urgent one.
            cur = grid[row][col]
            rank = {' ': 0, '.': 1, 'o': 2, '!': 3}
            if rank[mark] >= rank.get(cur, 0):
                grid[row][col] = mark

    grid[cy][cx] = 'R'
    if cy - 1 >= 0:
        if grid[cy - 1][cx] == ' ':
            grid[cy - 1][cx] = '|'      # nose, so orientation is unambiguous
    return [''.join(r) for r in grid]


def sector_min(ranges, angle_min, angle_inc, lo, hi):
    best = math.inf
    for i, r in enumerate(ranges):
        if r is None or not math.isfinite(r) or r <= 0.02:
            continue
        a = angle_min + i * angle_inc
        a = math.atan2(math.sin(a), math.cos(a))
        if lo <= a <= hi:
            best = min(best, r)
    return best


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--range', type=float, default=2.5, help='view radius, metres')
    ap.add_argument('--once', action='store_true', help='one frame then exit')
    ap.add_argument('--domain', type=int, default=20)
    ap.add_argument('--stop', type=float, default=0.35)
    ap.add_argument('--slow', type=float, default=0.90)
    args = ap.parse_args()

    if os.environ.get('ROS_DOMAIN_ID') is None:
        os.environ['ROS_DOMAIN_ID'] = str(args.domain)

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan

    rclpy.init()
    node = Node('lidar_view')
    state = {'msg': None, 'n': 0}

    def cb(m):
        state['msg'] = m
        state['n'] += 1

    node.create_subscription(LaserScan, '/scan', cb, qos_profile_sensor_data)

    import time
    t0 = time.time()
    while state['msg'] is None and time.time() - t0 < 8.0:
        rclpy.spin_once(node, timeout_sec=0.1)

    if state['msg'] is None:
        print(f'No /scan received on domain {os.environ["ROS_DOMAIN_ID"]} after 8 s.')
        print('Is the car powered and connected?  ros2 node list  should show /YB_Car_Node')
        node.destroy_node(); rclpy.shutdown()
        return 1

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.2)
            m = state['msg']
            rr = list(m.ranges)
            lines = render(rr, m.angle_min, m.angle_increment, args.range,
                           stop_d=args.stop, slow_d=args.slow)

            fwd = sector_min(rr, m.angle_min, m.angle_increment, -0.785, 0.785)
            left = sector_min(rr, m.angle_min, m.angle_increment, 0.785, 2.356)
            right = sector_min(rr, m.angle_min, m.angle_increment, -2.356, -0.785)
            valid = sum(1 for r in rr if r is not None and math.isfinite(r) and r > 0.02)

            out = []
            out.append(f'lidar  frame={m.header.frame_id}  {len(rr)} beams, '
                       f'{valid} valid   view +/-{args.range:.1f} m   frame #{state["n"]}')
            out.append('+' + '-' * len(lines[0]) + '+')
            for ln in lines:
                out.append('|' + ln + '|')
            out.append('+' + '-' * len(lines[0]) + '+')

            def fmt(d):
                return f'{d:5.2f} m' if math.isfinite(d) else '  --  '

            gov = ('STOP' if fwd <= args.stop else
                   'SLOW' if fwd <= args.slow else 'clear')
            out.append(f'  forward +/-45 deg (what the governor watches): {fmt(fwd)}  -> {gov}')
            out.append(f'  left {fmt(left)}   right {fmt(right)}')
            out.append(f'  legend: ! <{args.stop} m stop   o <{args.slow} m slow   . clear'
                       f'   R robot, | nose/forward')

            if args.once:
                print('\n'.join(out))
                break
            sys.stdout.write('\033[2J\033[H' + '\n'.join(out) + '\n')
            sys.stdout.flush()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
