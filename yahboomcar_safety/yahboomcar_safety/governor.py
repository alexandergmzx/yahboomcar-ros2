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

# Nodes that are SUPPOSED to publish /cmd_vel alongside the governor. The deadman's whole
# job is to publish zeros there when a driver dies, so flagging it as a bypass made the
# mandatory floor launch fail its own preflight -- the procedure says abort on BYPASSED,
# and first_floor_launch.py always starts both. Found by external audit.
EXPECTED_PUBLISHERS = ('cmd_vel_deadman',)


def bypassing_nodes(publishers, me, expected=EXPECTED_PUBLISHERS):
    """Split /cmd_vel publishers into (unexpected, expected_seen).

    Matched on node name rather than full path so a namespaced deadman still counts.
    That does mean anything calling itself cmd_vel_deadman is trusted -- acceptable,
    because a hostile publisher on this graph can simply drive the car anyway; there is
    no authorisation boundary (see the TODO in docs/safety-case.md).
    """
    unexpected, seen = [], []
    for full in publishers:
        if full == me:
            continue
        (seen if full.rsplit('/', 1)[-1] in expected else unexpected).append(full)
    return unexpected, seen


@dataclass
class GovernorConfig:
    stop_distance: float = 0.35      # m: hard stop inside this
    slow_distance: float = 0.90      # m: begin scaling down here
    sector_half_angle: float = 0.785  # rad (45 deg), matches the vendor's LaserAngle
    max_speed: float = 0.35          # m/s: absolute cap on forward speed
    max_yaw: float = 1.5             # rad/s: absolute cap on yaw
    # Rotation near an obstacle sweeps the footprint CORNERS past it, and the forward
    # sector cannot see that happen. Gated rather than forbidden: rotating in place is
    # how the operator escapes a stop.
    max_yaw_near: float = 0.4        # rad/s: cap on yaw inside stop_distance
    # Reverse is entirely sensor-blind -- there is no rear lidar. "Unprotected" should
    # not also mean "at full speed".
    max_reverse_speed: float = 0.10  # m/s
    # The chassis is differential; a commanded strafe produces exactly zero on every
    # /odom_raw axis. Zeroed here rather than relying on the firmware to ignore it.
    allow_lateral: bool = False
    scan_timeout: float = 0.5        # s: no lidar this recent -> stop
    cmd_timeout: float = 0.3         # s: no operator input this recent -> stop
    min_valid_range: float = 0.02    # m: below this the reading is noise, not an object
    # DOCKING MODE. A terminal-approach concession, and the narrowest one that
    # works. See `DockingApproach` and rule 7 in `decide`.
    docking_cone_half_angle: float = 0.2618   # rad (15 deg)
    docking_creep_max_speed: float = 0.05     # m/s: the governor's own creep clamp


@dataclass(frozen=True)
class DockingApproach:
    """Permission to close the last few centimetres onto ONE known object.

    A corridor delivery ends in contact: the robot must touch its target, and
    the touch is the arrival. The proximity floor exists to prevent exactly
    that, so something has to give -- and the choice is between BYPASSING this
    filter and INFORMING it. This is the informed version, and it is deliberately
    the narrowest concession that still permits the contact.

    What it suppresses: the obstacle stop and the slow zone, for returns falling
    inside a narrow cone toward a bearing the caller has already confirmed, and
    no farther than a range the caller has already measured, plus a margin.

    What it does NOT suppress, which is the point:

      * the stale-scan stop and the command-timeout stop (rules 1 and 2);
      * the obstacle stop for everything OUTSIDE the cone, at full strength;
      * the empty-sector fail-closed;
      * the yaw gate near an obstacle;
      * the speed cap -- it TIGHTENS it, to `docking_creep_max_speed`.

    A mask this shape cannot hide a wall the robot is about to strike while
    turning, a person stepping in from the side, or a lidar that has stopped
    reporting. It can hide exactly one thing: the object the caller is
    deliberately driving into.

    `bearing_rad` and `range_m` come from the caller's own confirmed detection
    and must be refreshed as the robot closes. A stale mask is a wide mask.
    """

    bearing_rad: float
    range_m: float
    margin_m: float = 0.10

    def masks(self, angle: float, distance: float, cfg) -> bool:
        """Is this return the object we are deliberately driving into?"""

        offset = math.atan2(math.sin(angle - self.bearing_rad),
                            math.cos(angle - self.bearing_rad))
        return (abs(offset) <= cfg.docking_cone_half_angle
                and distance <= self.range_m + self.margin_m)


@dataclass
class Decision:
    vx: float
    vy: float
    wz: float
    reason: str          # why the output differs from the request; '' when untouched
    limited: bool
    min_range: float     # nearest obstacle seen in the forward sector, inf if none


def forward_min_range(ranges, angle_min, angle_increment, cfg, docking=None):
    """Nearest valid return within +/- sector_half_angle of straight ahead.

    Returns inf when the sector holds no valid reading. Callers must treat inf as
    "unknown", not "clear" -- an empty sector can equally mean the lidar is blind.

    With `docking` supplied, returns the approach masks are skipped, so the
    number handed to `decide` means "the nearest thing I am NOT deliberately
    driving into". Everything else in the sector still counts, at full strength,
    and an empty result is still inf and still fails closed.
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
        if abs(angle) > cfg.sector_half_angle:
            continue
        if docking is not None and docking.masks(angle, r, cfg):
            continue
        nearest = min(nearest, r)
    return nearest


def decide(vx, vy, wz, min_range, scan_age, cmd_age, cfg, docking=None):
    """Body twist + lidar state -> the twist that may actually be sent.

    Rules, in priority order. Earlier rules win, so a stale scan cannot be overridden
    by a comfortable-looking range.

    `docking` does NOT appear in rules 1-3. A terminal approach is not a reason
    to accept a stale scan, a dead commander, or a NaN, and the ordering here is
    what guarantees that: the mode is consulted only after those three have
    passed. Its effect on `min_range` has already happened in
    `forward_min_range`, upstream of this function; all `decide` itself does
    with the mode is tighten the speed cap.
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

    # 3a. DOCKING CREEP CLAMP. The mode exists to let the robot touch something,
    #     so it makes the robot SLOWER, never faster -- applied before the
    #     ordinary caps so it binds regardless of what they would have allowed.
    if docking is not None and out_vx > cfg.docking_creep_max_speed:
        out_vx = cfg.docking_creep_max_speed
        reasons.append(f'docking creep {cfg.docking_creep_max_speed:.2f} m/s')

    # 4. Lateral motion. The forward sector cannot vouch for sideways clearance, so
    #    strafing is not something this filter can govern.
    if not cfg.allow_lateral:
        if out_vy != 0.0:
            reasons.append('lateral zeroed')
        out_vy = 0.0
    elif abs(out_vy) > cfg.max_speed:
        out_vy = math.copysign(cfg.max_speed, out_vy)
        reasons.append('lateral capped')

    # 5. Absolute caps, independent of obstacles. Forward and reverse have separate
    #    ceilings because only forward is sensed.
    if out_vx > cfg.max_speed:
        out_vx = cfg.max_speed
        reasons.append('speed capped')
    elif out_vx < -cfg.max_reverse_speed:
        out_vx = -cfg.max_reverse_speed
        reasons.append('reverse capped (no rear sensor)')
    if abs(out_wz) > cfg.max_yaw:
        out_wz = math.copysign(cfg.max_yaw, out_wz)
        reasons.append('yaw capped')

    # 6. Rotation near an obstacle, whether or not the robot is also translating.
    #    Applied before the forward rules so it still binds on a full stop, where
    #    rotating in place is exactly what an operator will try next.
    if math.isfinite(min_range) and min_range <= cfg.stop_distance \
            and abs(out_wz) > cfg.max_yaw_near:
        out_wz = math.copysign(cfg.max_yaw_near, out_wz)
        reasons.append(f'yaw gated at {min_range:.2f} m')

    # 7. Obstacle rules apply to FORWARD motion only; the lidar sector faces forward,
    #    so reversing away from an obstacle stays allowed (bounded above). This is a
    #    real limitation: nothing here protects the rear.
    #
    #    Every stop path zeroes BOTH translation axes. Returning out_vy unchanged here
    #    was a real defect: a command carrying linear.y kept its lateral component
    #    through an obstacle stop, inert only because this chassis happens to be
    #    differential.
    #    The stop reason is appended to whatever else already applied, rather than
    #    replacing it. An operator told only "obstacle at 0.10 m" would not know their
    #    yaw was gated too, and would read the sluggish turn as a fault.
    if out_vx > 0.0:
        if min_range == math.inf:
            # Empty sector. Could be a clear field, could be a blind lidar. Fail closed.
            return Decision(0.0, 0.0, out_wz,
                            '; '.join(reasons + ['no valid lidar returns']),
                            True, min_range)
        if min_range <= cfg.stop_distance:
            return Decision(0.0, 0.0, out_wz,
                            '; '.join(reasons + [f'obstacle at {min_range:.2f} m']),
                            True, min_range)
        if min_range < cfg.slow_distance:
            span = cfg.slow_distance - cfg.stop_distance
            scale = (min_range - cfg.stop_distance) / span if span > 0 else 0.0
            scale = max(0.0, min(1.0, scale))
            out_vx *= scale
            reasons.append(f'slowed to {scale:.2f} at {min_range:.2f} m')

    return Decision(out_vx, out_vy, out_wz, '; '.join(reasons), bool(reasons), min_range)
