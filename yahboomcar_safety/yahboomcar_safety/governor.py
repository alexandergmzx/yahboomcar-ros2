"""Speed-limiting logic for the lidar safety governor.

Kept free of ROS types on purpose, so the decision function can be unit-tested
exhaustively without a robot, a simulator, or a running ROS graph. `cmd_vel_governor.py`
is the thin ROS wrapper around it.

Design stance: this is a FILTER, not a behaviour. It never invents motion, it only
reduces what was asked for. The vendor's yahboomcar_laser/laser_Avoidance.py is an
autonomous wanderer that publishes straight to /cmd_vel; that is not what protects a
robot from a wall.

Fail-closed everywhere. Missing data, stale data, malformed data, NaNs -- all mean stop.
A safety filter that fails permissive is worse than none, because it invites trust.
"""
import math
from dataclasses import dataclass


@dataclass
class GovernorConfig:
    stop_distance: float = 0.35      # m: hard stop inside this
    slow_distance: float = 0.90      # m: begin scaling down here
    sector_half_angle: float = 0.785  # rad (45 deg), matches the vendor's LaserAngle
    max_speed: float = 0.35          # m/s: absolute cap on forward speed
    max_yaw: float = 1.5             # rad/s: absolute cap on yaw
    scan_timeout: float = 0.5        # s: no lidar this recent -> stop
    cmd_timeout: float = 0.3         # s: no operator input this recent -> stop
    min_valid_range: float = 0.02    # m: below this the reading is noise, not an object


@dataclass
class Decision:
    vx: float
    vy: float
    wz: float
    reason: str          # why the output differs from the request; '' when untouched
    limited: bool
    min_range: float     # nearest obstacle seen in the forward sector, inf if none


def forward_min_range(ranges, angle_min, angle_increment, cfg):
    """Nearest valid return within +/- sector_half_angle of straight ahead.

    Returns inf when the sector holds no valid reading. Callers must treat inf as
    "unknown", not "clear" -- an empty sector can equally mean the lidar is blind.
    """
    if not ranges or angle_increment == 0:
        return math.inf
    nearest = math.inf
    for i, r in enumerate(ranges):
        if r is None:
            continue
        # Reject NaN/inf and physically impossible values rather than trusting them.
        if not math.isfinite(r) or r < cfg.min_valid_range:
            continue
        angle = angle_min + i * angle_increment
        # Normalise to [-pi, pi] so a 0..2pi scan is handled identically.
        angle = math.atan2(math.sin(angle), math.cos(angle))
        if abs(angle) <= cfg.sector_half_angle:
            nearest = min(nearest, r)
    return nearest


def decide(vx, vy, wz, min_range, scan_age, cmd_age, cfg):
    """Body twist + lidar state -> the twist that may actually be sent.

    Rules, in priority order. Earlier rules win, so a stale scan cannot be overridden
    by a comfortable-looking range.
    """
    # 1. No recent lidar means no basis for any forward motion.
    if scan_age is None or scan_age > cfg.scan_timeout:
        return Decision(0.0, 0.0, 0.0, 'scan stale or missing', True, min_range)

    # 2. No recent operator input: assume the operator is gone.
    if cmd_age is None or cmd_age > cfg.cmd_timeout:
        return Decision(0.0, 0.0, 0.0, 'command stale', True, min_range)

    # 3. Garbage in the request is not something to pass through.
    if not all(math.isfinite(v) for v in (vx, vy, wz)):
        return Decision(0.0, 0.0, 0.0, 'non-finite command', True, min_range)

    reasons = []
    out_vx, out_vy, out_wz = vx, vy, wz

    # 4. Absolute caps, independent of obstacles.
    if abs(out_vx) > cfg.max_speed:
        out_vx = math.copysign(cfg.max_speed, out_vx)
        reasons.append('speed capped')
    if abs(out_wz) > cfg.max_yaw:
        out_wz = math.copysign(cfg.max_yaw, out_wz)
        reasons.append('yaw capped')

    # 5. Obstacle rules apply to FORWARD motion only; the lidar sector faces forward,
    #    so reversing away from an obstacle stays allowed. This is a real limitation:
    #    nothing here protects the rear.
    if out_vx > 0.0:
        if min_range == math.inf:
            # Empty sector. Could be a clear field, could be a blind lidar. Fail closed.
            return Decision(0.0, out_vy, out_wz, 'no valid lidar returns', True, min_range)
        if min_range <= cfg.stop_distance:
            return Decision(0.0, out_vy, out_wz,
                            f'obstacle at {min_range:.2f} m', True, min_range)
        if min_range < cfg.slow_distance:
            span = cfg.slow_distance - cfg.stop_distance
            scale = (min_range - cfg.stop_distance) / span if span > 0 else 0.0
            scale = max(0.0, min(1.0, scale))
            out_vx *= scale
            reasons.append(f'slowed to {scale:.2f} at {min_range:.2f} m')

    return Decision(out_vx, out_vy, out_wz, '; '.join(reasons), bool(reasons), min_range)
