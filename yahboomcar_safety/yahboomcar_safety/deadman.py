"""Decision logic for the command deadman.

Kept free of ROS types so it can be unit-tested without a robot, same split as
governor.py / cmd_vel_governor.py.

WHY THIS EXISTS
---------------
The firmware has no command watchdog. Measured three ways on 2026-08-06: commanded to
0.15 m/s and then left alone, the car held that speed for the full 45 s of the probe, and
stopped only on an explicit zero. See docs/safety-case.md.

So a retained command is permanent, and the governor's `finally` block -- which only runs
on a clean shutdown -- is the single thing standing between a crashed process and a car
that drives until it hits something.

This node closes that specific gap: it watches /cmd_vel itself, and when the last thing
anyone said was "move" and nobody has said anything since, it says "stop".

Watching the TOPIC rather than the governor's liveness is deliberate. It covers any
publisher dying -- governor, teleop, a test tool, a Nav2 node -- without those publishers
needing to know this node exists, and without a heartbeat protocol to get out of sync.

WHAT IT CANNOT DO
-----------------
It runs on this PC and talks over the same Wi-Fi. If the link drops, the agent dies, or
this machine freezes, it is as cut off as everything else -- and case 3 of the fail-safe
test measured that the car keeps driving through exactly that. It reduces the risk
surface; it does not make the system fail-safe. Only the physical power switch does.
"""
from dataclasses import dataclass


@dataclass
class DeadmanConfig:
    timeout: float = 0.5        # s: no /cmd_vel this recent, after a move -> intervene
    hold: float = 2.0           # s: keep sending zeros this long once triggered
    zero_epsilon: float = 1e-6  # below this a command counts as a stop


def is_moving_command(vx, vy, wz, cfg):
    """Did this command ask the car to move?

    Non-finite values count as moving: a NaN reaching the firmware is not something to
    assume is harmless, and intervening on it is the fail-closed choice.
    """
    for v in (vx, vy, wz):
        if v != v:                      # NaN
            return True
        if abs(v) > cfg.zero_epsilon:
            return True
    return False


def should_intervene(last_was_moving, age, cfg):
    """True when the car was last told to move and nobody has spoken since.

    `age` is seconds since the last /cmd_vel message, or None if none has ever arrived.
    A silent topic with no history is NOT an intervention case -- nothing has been
    commanded, so there is nothing latched to expire.
    """
    if not last_was_moving:
        return False                    # last word was "stop"; already handled
    if age is None:
        return False
    return age > cfg.timeout
