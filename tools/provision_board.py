#!/usr/bin/env python3
"""Provision the microROS control board — safe replacement for config_robot.py.

Why this exists: the vendor's config_robot.py has a __main__ block that immediately
writes placeholder config (ssid123 / 192.168.2.116) *and* resets the servo offsets and
both PID sets. Running it as shipped clobbers a working board. This script:

  * writes ONLY wifi / agent-endpoint / car-type / domain-id
  * never touches PID or servo-offset calibration
  * shows current vs. new and asks before writing
  * reads the config back afterwards and verifies it stuck
  * takes the password interactively or from an env var, so it never lands in the
    repo or in shell history

Usage
-----
  ./tools/provision_board.py --ssid MyNetwork --agent-ip 192.168.1.123
  ./tools/provision_board.py --ssid MyNetwork --agent-ip 192.168.1.123 --dry-run

  Password: prompted, or set MICROROS_WIFI_PASSWORD in the environment.

Notes
-----
  * Connect USB-C to the port marked "Serial" (not "5V OUT") and switch the board on.
  * Stop any running micro-ROS agent first — it holds /dev/ttyUSB0.
  * The board only accepts config in a ~5 s window after reset; reboot_device() (a
    DTR/RTS toggle) opens it, which is why this script resets first.
  * ROS_DOMAIN_ID on this PC must match --domain-id, or `ros2 node list` shows nothing.
"""
import argparse
import getpass
import os
import sys
import time

VENDOR_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "MicroROS-assets", "Factory-Firmware",
)

# The vendor setters take ints, but EVERY vendor read_* method returns a *string*
# (read_car_type yields "CAR_TYPE_COMPUTER"; read_agent_ip_port and read_ros_domain_id
# format their ints with "%d"). Comparing a read-back against an int silently fails, so
# all verification below is done as strings.
CAR_TYPE_VALUE = {"computer": 0, "rpi5": 1}
CAR_TYPE_NAME = {0: "CAR_TYPE_COMPUTER", 1: "CAR_TYPE_RPI5"}
CAR_TYPE_DESC = {"CAR_TYPE_COMPUTER": "CAR_TYPE_COMPUTER (WiFi-UDP)",
                 "CAR_TYPE_RPI5": "CAR_TYPE_RPI5 (serial)"}


def load_vendor_class():
    if not os.path.isdir(VENDOR_DIR):
        sys.exit(f"error: vendor bundle not found at {VENDOR_DIR}\n"
                 "       See MicroROS-assets/README.md for what to download.")
    sys.path.insert(0, VENDOR_DIR)
    try:
        from config_robot import MicroROS_Robot  # noqa: E402
    except ImportError as e:
        sys.exit(f"error: could not import config_robot.py: {e}")
    return MicroROS_Robot


def read_config(robot):
    """Read-only snapshot. Values may be None if the config window has closed."""
    def clean(v):
        return v.rstrip("\x00").strip() if isinstance(v, str) else v
    return {
        "firmware":  clean(robot.read_version()),
        "ssid":      clean(robot.read_wifi_ssid()),
        "agent_ip":  clean(robot.read_agent_ip_addr()),
        "agent_port": robot.read_agent_ip_port(),
        "car_type":  robot.read_car_type(),
        "domain_id": robot.read_ros_domain_id(),
    }


def show(cfg, title):
    print(f"\n{title}")
    print("-" * 52)
    for k, v in cfg.items():
        if k == "car_type":
            v = CAR_TYPE_DESC.get(v, v)
        print(f"{k:>12} : {v}")
    print("-" * 52)


def main():
    ap = argparse.ArgumentParser(description="Provision the microROS board for WiFi-UDP.")
    ap.add_argument("--ssid", required=True, help="WiFi network name the CAR joins")
    ap.add_argument("--agent-ip", required=True,
                    help="IP of THIS PC, where the micro-ROS agent listens")
    ap.add_argument("--agent-port", type=int, default=8090)
    ap.add_argument("--domain-id", type=int, default=20,
                    help="0-101; must match ROS_DOMAIN_ID on this PC (default 20)")
    ap.add_argument("--serial", default="/dev/ttyUSB0")
    ap.add_argument("--rpi5", action="store_true",
                    help="set CAR_TYPE_RPI5 (serial) instead of CAR_TYPE_COMPUTER")
    ap.add_argument("--dry-run", action="store_true", help="read and report, write nothing")
    ap.add_argument("--yes", action="store_true", help="skip the confirmation prompt")
    args = ap.parse_args()

    octets = args.agent_ip.split(".")
    if len(octets) != 4 or not all(o.isdigit() and 0 <= int(o) <= 255 for o in octets):
        sys.exit(f"error: --agent-ip {args.agent_ip!r} is not a dotted-quad IPv4 address")
    octets = [int(o) for o in octets]

    if not 0 <= args.domain_id <= 101:
        sys.exit("error: --domain-id must be 0-101")

    if not os.path.exists(args.serial):
        sys.exit(f"error: {args.serial} not found. Is the board connected and switched on?")

    password = None
    if not args.dry_run:
        password = os.environ.get("MICROROS_WIFI_PASSWORD")
        if not password:
            password = getpass.getpass(f"WiFi password for {args.ssid!r}: ")
        if not password:
            sys.exit("error: empty password")

    MicroROS_Robot = load_vendor_class()
    robot = MicroROS_Robot(port=args.serial, debug=False)

    print("Resetting board into its ~5 s config window...")
    robot.reboot_device()
    time.sleep(0.2)
    before = read_config(robot)
    show(before, "current configuration")

    if before["firmware"] is None:
        sys.exit("error: no response from the board.\n"
                 "       Is a micro-ROS agent still holding the port? "
                 "(docker ps | grep micro-ros-agent)")

    car_type = CAR_TYPE_VALUE["rpi5"] if args.rpi5 else CAR_TYPE_VALUE["computer"]
    car_type_name = CAR_TYPE_NAME[car_type]
    print("\nwill write:")
    print(f"        ssid : {before['ssid']!r} -> {args.ssid!r}")
    print(f"    agent_ip : {before['agent_ip']} -> {args.agent_ip}")
    print(f"  agent_port : {before['agent_port']} -> {args.agent_port}")
    print(f"    car_type : {CAR_TYPE_DESC.get(before['car_type'], before['car_type'])}"
          f" -> {CAR_TYPE_DESC[car_type_name]}")
    print(f"   domain_id : {before['domain_id']} -> {args.domain_id}")
    print("\n(PID and servo-offset calibration are left untouched.)")

    if args.dry_run:
        print("\n--dry-run: nothing written.")
        return

    if not args.yes:
        if input("\nProceed? [y/N] ").strip().lower() not in ("y", "yes"):
            print("Aborted; nothing written.")
            return

    # Re-open the config window: the reads above may have consumed it.
    print("\nResetting again to open a fresh config window...")
    robot.reboot_device()
    time.sleep(0.2)

    robot.set_wifi_config(args.ssid, password)
    robot.set_udp_config(octets, args.agent_port)
    robot.set_car_type(car_type)
    robot.set_ros_domain_id(args.domain_id)
    time.sleep(0.2)

    print("Rebooting so the new config takes effect...")
    robot.reboot_device()
    time.sleep(0.2)
    after = read_config(robot)
    show(after, "configuration read back")

    # Compared as strings: every vendor read_* returns str, so `== args.agent_port`
    # against an int would be False even on a perfectly successful write.
    expected = {
        "ssid": args.ssid,
        "agent_ip": args.agent_ip,
        "agent_port": str(args.agent_port),
        "car_type": car_type_name,
        "domain_id": str(args.domain_id),
    }
    mismatches = {k: (after.get(k), v) for k, v in expected.items()
                  if str(after.get(k)) != v}
    ok = not mismatches

    del robot

    if ok:
        print("\n✅ verified — the board stored what we asked for.\n")
        print("Next:")
        print(f"  export ROS_DOMAIN_ID={args.domain_id}")
        print("  docker run -it --rm -v /dev:/dev -v /dev/shm:/dev/shm --privileged \\")
        print("    --net=host microros/micro-ros-agent:jazzy udp4 --port "
              f"{args.agent_port} -v6")
        print("  ros2 node list        # expect /YB_Car_Node")
    else:
        print("\n⚠️  read-back does NOT match what was written:")
        for k, (got, want) in mismatches.items():
            print(f"      {k}: got {got!r}, wanted {want!r}")
        print("\n    The ~5 s config window may have closed mid-write. Re-run;")
        print("    the board is still reconfigurable over serial, so this is recoverable.")
        sys.exit(1)


if __name__ == "__main__":
    main()
