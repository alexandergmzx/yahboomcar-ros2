"""Refuse to start when another publisher of the same topic is already live.

Written after fourteen stray joint_state_bridge processes accumulated across test runs
and all published conflicting commands to the same joints -- /joint_states ran at 100 Hz
instead of 30, and the twin behaved erratically for reasons that looked like a wiring
bug. Killing a shell job (``kill %1``) does not kill the process it spawned, so tests
that "cleaned up" left publishers behind every time.

Two guards, because they catch different failures:

  * `assert_sole_publisher` asks the ROS graph who else publishes the topic. This
    catches strays from *any* source, including ones started by hand.
  * `install_signal_handlers` makes Ctrl-C and SIGTERM shut the node down cleanly
    rather than leaving it orphaned.
"""
import os
import signal


def count_other_publishers(node, topic: str) -> int:
    """How many publishers of `topic` already exist.

    Uses count_publishers rather than walking the node list and excluding "me" by name.
    That earlier approach had a fatal flaw for this exact purpose: a duplicate process
    runs under the SAME node name, so name-matching skipped the very process it was
    meant to catch, and the guard never fired.

    Call this BEFORE creating your own publisher, so anything counted is somebody else.
    """
    return node.count_publishers(topic)


def assert_sole_publisher(node, topic: str, allow_override: bool = True) -> None:
    """Raise unless this node is the only publisher of `topic`.

    Set ALLOW_DUPLICATE_PUBLISHERS=1 to bypass deliberately (e.g. comparing two
    implementations side by side).
    """
    if allow_override and os.environ.get('ALLOW_DUPLICATE_PUBLISHERS') == '1':
        node.get_logger().warn(
            f'duplicate-publisher check bypassed for {topic} '
            '(ALLOW_DUPLICATE_PUBLISHERS=1)')
        return

    n = count_other_publishers(node, topic)
    if n:
        raise RuntimeError(
            f'{n} other node(s) already publish {topic}. Competing publishers fight for '
            f'the same actuators and produce behaviour that looks like a wiring bug.\n'
            f'  find them : ros2 node info <node>, or  pgrep -af joint_state_bridge\n'
            f'  stop them : pkill -f joint_state_bridge\n'
            f'  override  : ALLOW_DUPLICATE_PUBLISHERS=1')


def install_signal_handlers(on_shutdown) -> None:
    """Run `on_shutdown` on SIGINT/SIGTERM so nothing is left orphaned."""
    def handler(signum, _frame):
        try:
            on_shutdown()
        finally:
            signal.signal(signum, signal.SIG_DFL)
            os.kill(os.getpid(), signum)

    for sig in (signal.SIGINT, signal.SIGTERM):
        try:
            signal.signal(sig, handler)
        except (ValueError, OSError):
            pass  # not on the main thread; caller handles cleanup itself
