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

    **SUPERSEDED by `DockingDisc`. Kept only as a negative control.**

    A fixed angular cone cannot admit a contact. A target of radius R subtends
    asin(R/r), which for R = 0.12 exceeds 15 deg everywhere inside 0.464 m and
    reaches 33.5 deg at contact -- so below about 0.41 m the target's own
    shoulders fall OUTSIDE this cone while lying INSIDE the 0.35 m stop, and the
    filter brakes on the very object it was told to ignore. Measured on the
    corridor bench: pins at 0.4176 m, contact never happens. Measured on a real
    session bag: the governed duty cycle collapses 98% -> 28% -> 12% -> 0% as
    the target closes from 0.70 to 0.35 m.

    The closing sentence above is wrong in both directions, and `DockingDisc`
    corrects both. This shape hides MORE than the target: it masks every on-cone
    return nearer than `range + margin`, i.e. the entire segment between sensor
    and target, so a foot planted on the approach line is invisible to the
    filter. And it hides LESS than the target: past 0.41 m the shoulders leak,
    which is the paragraph above. It is the wrong shape, not a small one.
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


@dataclass(frozen=True)
class DockingDisc:
    """Permission to close onto one known object, shaped like the OBJECT.

    Mask a return iff its point lies within `target_radius_m + margin_m` of the
    declared target centre. Not a cone: a disc, sized by the thing being
    approached, in the place it was last seen.

    WHY THE SHAPE MATTERS, in one line each:

      * **It admits contact at every range.** Returns from the target's surface
        are within one radius of its centre by definition, at 0.62 m and at
        0.22 m alike. A cone cannot say that -- see `DockingApproach`.
      * **It self-sizes.** Far away the disc subtends almost nothing; up close
        it opens exactly as fast as the target does, and no faster.
      * **It closes a hole the cone had.** The cone masked every on-cone return
        nearer than the target, i.e. the whole segment between sensor and
        target -- a foot planted on the approach line was invisible. A disc
        masks nothing nearer than `centre - radius`, so that foot stops the
        robot again.

    WHAT IT STILL DOES NOT TOUCH: the stale-scan and command-timeout stops
    (evaluated before this is consulted at all), the obstacle stop for
    everything outside the disc, the empty-sector fail-closed, the yaw gate,
    and the speed cap -- which docking tightens rather than relaxes.

    `target_radius_m` is the AUTHORED radius of the target, never a fitted one.
    Measured across 38 runs, the detector's fitted radius spans 0.072-0.168 m --
    it fills its own acceptance band edge to edge -- so a disc sized from a fit
    would intermittently be smaller than the object and unmask its own target's
    nose. The authored number is the only stable one.

    `margin_m` covers declaration staleness, not fit error: one scan period at
    the creep speed is ~4 mm of translation, and rotation during a pivot adds
    up to ~34 mm of lateral shift at handoff range. 0.10 m carries both with
    room. It must stay well under the clearance to the nearest real hazard --
    in the corridor the east wall sits 0.362 m from the target's centre and the
    stub 0.568 m, so a 0.22 m disc keeps at least 0.06 m of slack even at worst
    staleness.
    """

    bearing_rad: float
    range_m: float
    target_radius_m: float
    margin_m: float = 0.10

    def masks(self, angle: float, distance: float, cfg) -> bool:
        """Does this return lie on the object we are deliberately driving into?"""

        centre_x = self.range_m * math.cos(self.bearing_rad)
        centre_y = self.range_m * math.sin(self.bearing_rad)
        point_x = distance * math.cos(angle)
        point_y = distance * math.sin(angle)
        return math.hypot(point_x - centre_x, point_y - centre_y) <= (
            self.target_radius_m + self.margin_m
        )


def disc_from_declaration(bearing_rad, range_m, target_radius_m, *,
                          margin_m, max_target_radius_m):
    """Validate a caller's disc declaration. Returns the disc, or None to refuse.

    Pure so it can be tested without a ROS graph, which is the only reason the
    node does not inline it.

    The radius is the one number in a declaration that WIDENS the masked region,
    so it is bounded rather than trusted. A declaration outside the bound is
    REFUSED, not clamped: a caller asking to mask a metre of corridor has a bug,
    and quietly granting it the maximum would hide the bug while still moving
    the robot. Non-finite values are refused for the same reason -- a NaN
    bearing would otherwise produce a disc centred nowhere, and `math.hypot`
    against NaN is False, so every return would read as unmasked. That happens
    to fail safe, but by accident rather than by decision.
    """

    values = (bearing_rad, range_m, target_radius_m)
    if not all(math.isfinite(v) for v in values):
        return None
    if not 0.0 < target_radius_m <= max_target_radius_m:
        return None
    if range_m <= 0.0:
        return None
    return DockingDisc(
        bearing_rad=bearing_rad,
        range_m=range_m,
        target_radius_m=target_radius_m,
        margin_m=margin_m,
    )


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
            # THE SLOW ZONE DOES NOT APPLY TO A COMMAND ALREADY AT CREEP SPEED.
            #
            # The zone exists to bleed off speed before a hard stop. A command
            # already clamped to `docking_creep_max_speed` has nothing to bleed:
            # stopping distance at 0.05 m/s is millimetres, and the 0.35 m hard
            # stop above is untouched and still ahead of it.
            #
            # Without this the terminal approach is unreachable for a reason
            # that has nothing to do with the target. In the corridor a wall
            # stub sits 0.315 m off the approach line, entering the +/-45 deg
            # sector at 0.4455 m for the WHOLE creep, which scales 0.05 m/s down
            # to 0.0087 -- 46 s to cover 0.40 m against a 25 s budget. Worse,
            # 8.7 mm/s is below the docking controller's own 10 mm/s stall
            # threshold, so a healthy creep reads as a contact and the robot
            # reports an arrival it never made. Measured on the bench: that
            # false arrival fires 1.1 s after the creep begins.
            #
            # Scoped as narrowly as it can be: only while a declaration is live,
            # and only for a command at or under the creep clamp. Anything
            # faster is slowed exactly as before.
            creeping = (
                docking is not None
                and out_vx <= cfg.docking_creep_max_speed + 1e-9
            )
            if creeping:
                reasons.append(f'creep exempt from slow zone at {min_range:.2f} m')
            else:
                span = cfg.slow_distance - cfg.stop_distance
                scale = (min_range - cfg.stop_distance) / span if span > 0 else 0.0
                scale = max(0.0, min(1.0, scale))
                out_vx *= scale
                reasons.append(f'slowed to {scale:.2f} at {min_range:.2f} m')

    return Decision(out_vx, out_vy, out_wz, '; '.join(reasons), bool(reasons), min_range)
