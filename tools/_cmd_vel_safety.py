#!/usr/bin/env python3
"""Guards for anything that publishes a velocity. Import it; do not run it.

    from _cmd_vel_safety import SafeCmdVel, target_of, require_simulator

WHY THIS EXISTS AS A SHARED MODULE
----------------------------------
The firmware has NO COMMAND WATCHDOG. A commanded speed is retained indefinitely --
measured three ways, and a follow-up probe held 0.15 m/s for 45 s with nothing publishing.
So a tool that commands a nonzero velocity and then exits without publishing a zero leaves
a real robot driving across a room forever.

`sim_patrol.py` learned this the expensive way. It caught KeyboardInterrupt but not
SIGTERM, `pkill` terminated it outright, its `finally` never ran, and the simulated robot
was 12 m outside a 4 m room before anyone stopped it. That is not a simulator quirk -- it
is the real firmware's behaviour, faithfully reproduced.

The code below is that lesson, extracted from sim_patrol.py's working implementation
rather than reinvented, because an audit found THREE first-party tools still publishing
0.12-0.15 m/s with no guard and no cleanup at all -- two of them defaulting to
ROS_DOMAIN_ID=20, which is the car:

    check_isaac_contract.py   0.12 m/s, 0.3 rad/s   no guard, no zeros, no domain pin
    measure_latency.py        0.15 m/s              no guard, no zeros, domain 20
    test_failsafe.py          0.15 m/s both topics   no guard, no zeros, domain 20

WHAT THIS CANNOT DO
-------------------
SIGKILL cannot be caught, and a Wi-Fi drop cannot be caught either. Nothing in this file
saves you from those. That is precisely why `cmd_vel_deadman` runs as a SEPARATE process,
and why the power switch on the floor is the only stop that always works. This module
removes the failure modes that ARE catchable; it does not make driving safe.
"""
import os
import signal
import sys
import time

# Node names that identify what you are actually talking to.
REAL_ROBOT_NODE = 'YB_Car_Node'
SIMULATOR_NODE = 'fake_robot'

# THE CAR'S DOMAIN. Single definition; simctl imports it from here. It must match the
# ROS_DOMAIN_ID provisioned onto the board (tools/provision_board.py; currently 20, per
# CLAUDE.md). If the board is ever re-provisioned onto a different domain, change it
# HERE, in one place -- a stale value here turns the structural refusal below into a
# refusal of the wrong domain.
CAR_DOMAIN = 20


def forbid_car_domain(what='This tool'):
    """Refuse to run on the car's domain AT ALL, before any discovery.

    STRUCTURAL, NOT OBSERVATIONAL -- and that distinction is the whole fix. Discovery
    based checks race: DDS discovery is incremental with no upper bound, and an audit
    reproduced a car surfacing at 7.5 s being classified 'simulator' at 7 s, straight
    through the 6 s confirmation dwell. Lengthening the timer just lengthens the race.
    A TIMER CANNOT PROVE THAT HARDWARE IS ABSENT.

    What a timer cannot do, an invariant can: simulators are never started on the car's
    domain (simctl refuses that too), so a simulator-driving tool has no business there
    under any circumstances -- not even if discovery currently shows nothing. This check
    is a constant comparison. It cannot race, cannot time out, and cannot be beaten by
    slow discovery.

    The dwell-based discovery check in target_of() remains as the SECOND layer, for the
    case a car is ever provisioned onto some other domain.
    """
    dom = int(os.environ.get('ROS_DOMAIN_ID', 0) or 0)
    if dom == CAR_DOMAIN:
        print(f'REFUSED: ROS_DOMAIN_ID={dom} is the CAR\'S domain.', flush=True)
        print(f'  {what} drives simulators and is prohibited there STRUCTURALLY -- no', flush=True)
        print('  discovery check is consulted, because discovery races and a timer', flush=True)
        print('  cannot prove the car is absent. Simulators live on 66; the car on '
              f'{CAR_DOMAIN}.', flush=True)
        sys.exit(2)


# A topic the real firmware CANNOT publish. Both simulators do, precisely so tests can
# score themselves against truth, and its presence is therefore positive evidence of a
# simulator rather than the mere absence of evidence of a robot.
SIMULATOR_TOPIC = '/sim/ground_truth'


def target_of(node, seconds=15.0):
    """-> 'hardware' | 'simulator' | 'both' | 'nothing'.

    POLLS rather than looking once. Discovery is not instant: a single look 2 s in once
    reported "no simulator" while the simulator was demonstrably running, and a guard
    that reports 'nothing' when the car is actually there is worse than no guard at all.

    A simulator is recognised by POSITIVE evidence, two ways, because the two backends
    present differently:

      * the 2D sim runs a node called `fake_robot`;
      * the ISAAC backend advertises NO ROS NODES AT ALL -- its isaacsim.ros2.bridge
        OmniGraph publishes without creating discoverable named nodes, so a node-name
        check simply cannot see it. It is identified by /sim/ground_truth instead.

    Never by absence. "I did not find the car" is not evidence that the car is not
    there -- discovery may just be slow -- and a guard built on that would fail open.
    """
    # ASYMMETRIC EXIT, because DDS discovery is INCREMENTAL: participants appear one by
    # one over several seconds, in no particular order. This loop used to break on the
    # FIRST evidence of either kind -- so a simulator discovered at t=2s concluded
    # 'simulator' while the car, discovered at t=4s, was still invisible, and the tool
    # then drove a domain the car was on. Fail-open, found by audit.
    #
    # Now: finding the CAR ends the search immediately (the conclusion is refusal, and
    # more waiting cannot make that wrong). Finding a SIMULATOR does not -- the search
    # continues for `confirm` more seconds specifically looking for a car before
    # 'simulator' may be concluded. The cost is a few seconds on every guarded start;
    # the alternative was a race whose losing case drives the real robot.
    confirm = 6.0
    deadline = time.time() + seconds
    sim_seen_at = None
    real = sim = False
    while time.time() < deadline:
        time.sleep(1.0)
        names = [n for n, _ in node.get_node_names_and_namespaces()]
        topics = [t for t, _ in node.get_topic_names_and_types()]
        real = REAL_ROBOT_NODE in names
        sim = SIMULATOR_NODE in names or SIMULATOR_TOPIC in topics
        if real:
            break
        if sim:
            if sim_seen_at is None:
                sim_seen_at = time.time()
            elif time.time() - sim_seen_at >= confirm:
                break
    if real and sim:
        return 'both'
    if real:
        return 'hardware'
    if sim:
        return 'simulator'
    return 'nothing'


def require_simulator(node, what='This tool', seconds=15.0):
    """Exit nonzero unless the ONLY thing listening is the simulator.

    'both' is refused too. The original sim_patrol guard read

        if 'YB_Car_Node' in names and 'fake_robot' not in names:

    which let a patrol drive whenever a simulator happened to share the domain with the
    car -- the one case where you are most likely to believe you are safe.
    """
    forbid_car_domain(what)
    t = target_of(node, seconds)
    if t == 'simulator':
        return
    print('', flush=True)
    if t in ('hardware', 'both'):
        print(f'REFUSED: {REAL_ROBOT_NODE} is on this domain '
              f'(ROS_DOMAIN_ID={os.environ.get("ROS_DOMAIN_ID", "unset")}).')
        if t == 'both':
            print('  A simulator is here too, but that does not make it safe: a command '
                  'published now reaches BOTH.')
        print(f'  {what} must never drive hardware. The firmware has no command '
              'watchdog, so if this process dies mid-command the car keeps going.')
    else:
        print(f'REFUSED: nothing is listening on '
              f'ROS_DOMAIN_ID={os.environ.get("ROS_DOMAIN_ID", "unset")}.')
        print('  Start a simulator with:  ./tools/simctl start')
    sys.stdout.flush()
    sys.exit(2)


def install_stop_handlers(*publishers, reps=15, gap=0.04):
    """Publish zeros on SIGINT/SIGTERM using publishers that ALREADY exist.

    For tools that already zero correctly from a `finally` block and are only missing
    the signal half. `finally` covers Ctrl+C and exceptions but NOT SIGTERM, whose
    default action terminates the process before any cleanup runs -- which is exactly
    how `pkill` once left a robot driving 12 m out of a 4 m room.

    Prefer SafeCmdVel for new code; this exists so an existing, working cleanup path does
    not have to be rewritten just to gain a signal handler.
    """
    from geometry_msgs.msg import Twist

    def on_signal(signum, _frame):
        print(f'\n  signal {signum}: stopping the robot before exiting', flush=True)
        m = Twist()
        for _ in range(reps):
            for p in publishers:
                p.publish(m)
            time.sleep(gap)
        sys.stdout.flush()
        os._exit(143 if signum == signal.SIGTERM else 130)

    for sig in (signal.SIGINT, signal.SIGTERM):
        try:
            signal.signal(sig, on_signal)
        except ValueError:
            pass          # not the main thread; the caller's `finally` still applies


class SafeCmdVel:
    """Publisher wrapper that ALWAYS leaves the robot stopped.

        with SafeCmdVel(node, ['/cmd_vel', '/cmd_vel_raw']) as safe:
            safe.publish(vx=0.15)
            ...

    Zeros are published on normal exit, on an exception, on SIGINT and on SIGTERM.
    SIGTERM matters most and is the one everybody forgets: its default action terminates
    the process immediately, so `finally` never runs.
    """

    def __init__(self, node, topics=('/cmd_vel',), zero_reps=20, zero_gap=0.04):
        from geometry_msgs.msg import Twist
        self._Twist = Twist
        self.node = node
        self.pubs = [node.create_publisher(Twist, t, 10) for t in topics]
        self.topics = list(topics)
        self.zero_reps = zero_reps
        self.zero_gap = zero_gap
        self.stopping = False
        self._prev_handlers = {}

    def __enter__(self):
        def on_signal(signum, _frame):
            # Do the stopping HERE, not by setting a flag: the caller's loop may be
            # blocked in a sleep, and SIGTERM's default action would kill us first.
            print(f'\n  signal {signum}: stopping the robot before exiting', flush=True)
            self.stopping = True
            self.stop()
            sys.stdout.flush()
            os._exit(143 if signum == signal.SIGTERM else 130)

        for sig in (signal.SIGINT, signal.SIGTERM):
            try:
                self._prev_handlers[sig] = signal.signal(sig, on_signal)
            except ValueError:
                # Not on the main thread; the `finally` path still covers normal exit.
                pass
        return self

    def __exit__(self, exc_type, exc, tb):
        self.stop()
        for sig, prev in self._prev_handlers.items():
            try:
                signal.signal(sig, prev)
            except (ValueError, TypeError):
                pass
        return False

    def publish(self, vx=0.0, wz=0.0, vy=0.0):
        """Publish to every configured topic. vy is carried but this chassis ignores it."""
        if self.stopping:
            return
        m = self._Twist()
        m.linear.x, m.linear.y, m.angular.z = float(vx), float(vy), float(wz)
        for p in self.pubs:
            p.publish(m)

    def stop(self):
        """Publish zeros repeatedly. Repetition is deliberate: these are BEST_EFFORT."""
        m = self._Twist()
        for _ in range(self.zero_reps):
            for p in self.pubs:
                p.publish(m)
            time.sleep(self.zero_gap)
