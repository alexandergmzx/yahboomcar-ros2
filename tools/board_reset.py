#!/usr/bin/env python3
"""Reset the ESP32 over USB serial, without reaching for the power switch.

    ./tools/board_reset.py                    # one reset, then report sensor health
    ./tools/board_reset.py --until-gyro-live  # retry until the gyro comes back
    ./tools/board_reset.py --no-verify        # just pulse reset and exit

A RESET IS NOT A POWER CYCLE, and the difference matters here
-------------------------------------------------------------
The CP2102 bridge drives the ESP32's auto-reset circuit: RTS is wired to EN (reset) and
DTR to GPIO0 (boot select). Pulsing RTS with DTR held high restarts the SoC in normal run
mode -- the same thing esptool does before flashing, minus the download-mode part.

What that restarts is the **processor**. It does NOT remove power from the peripherals:
the ICM-42670-P IMU, the lidar and the motor drivers keep their supply rails throughout.
So a reset clears firmware state and re-runs the sensor init, but it cannot clear a state
held inside a peripheral chip that only losing power would clear.

That distinction is the whole question for the intermittently dead gyro. If the fault is
the firmware's IMU init failing, a reset should fix it. If it is the IMU chip latched into
a bad state, a reset will not, and a power cycle is still needed. **Measurement decides,
not argument** -- which is what --until-gyro-live is for: it resets, checks, and reports
what actually happened rather than assuming.

Worth knowing: a power cycle did NOT revive the gyro on 2026-08-06, so "power cycling
always fixes it" was never true either.

SAFETY
------
A reset restarts the firmware, which re-initialises the motor outputs, so it should stop a
moving car. That is a REASONABLE EXPECTATION AND NOT A MEASUREMENT -- do not treat this as
an emergency stop until it has been tested against a moving robot with the wheels
elevated. The physical power switch remains the only stop that is known to work.

It does, though, work over a channel the Wi-Fi cannot take away, which is more than
anything else in this stack can claim.
"""
import argparse
import os
import subprocess
import sys
import time

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def pulse_reset(port, hold=0.15, settle=0.05):
    """Drive the ESP32 auto-reset circuit: pulse EN low with GPIO0 held high.

    DTR -> GPIO0, RTS -> EN, both inverted by the transistor pair on the board, which is
    why `True` here pulls the line LOW. Setting DTR False first keeps GPIO0 high so the
    chip boots the application rather than the ROM bootloader.
    """
    import serial
    with serial.Serial(port, 115200, timeout=0.2) as s:
        s.dtr = False          # GPIO0 high -> normal boot, not download mode
        s.rts = True           # EN low     -> hold in reset
        time.sleep(hold)
        s.rts = False          # EN high    -> release
        time.sleep(settle)
        s.dtr = False


def sensor_health(domain, seconds):
    """-> (exit_code, stdout). 0 means every channel is alive and measuring."""
    env = dict(os.environ, ROS_DOMAIN_ID=str(domain))
    cmd = (f'source /opt/ros/jazzy/setup.bash && '
           f'python3 {os.path.join(REPO, "tools", "sensor_health.py")} '
           f'--seconds {seconds} --domain {domain}')
    p = subprocess.run(['bash', '-c', cmd], env=env, capture_output=True, text=True)
    return p.returncode, p.stdout


def gyro_state(text):
    if 'imu gyro z' not in text:
        return 'no data'
    for line in text.splitlines():
        if 'imu gyro z' in line:
            return 'DEAD' if 'DEAD' in line else 'live'
    return 'unknown'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--port', default='/dev/ttyUSB0')
    ap.add_argument('--domain', type=int, default=20)
    ap.add_argument('--wait', type=float, default=25.0,
                    help='seconds to allow for the micro-ROS session to come back')
    ap.add_argument('--check-seconds', type=float, default=10.0)
    ap.add_argument('--no-verify', action='store_true')
    ap.add_argument('--until-gyro-live', action='store_true',
                    help='reset repeatedly until the gyro reports variance')
    ap.add_argument('--max-attempts', type=int, default=5)
    args = ap.parse_args()

    if not os.path.exists(args.port):
        print(f'no such port: {args.port}')
        print('The board must be connected by USB-C to the port marked "Serial", not '
              '"5V OUT".')
        return 2

    attempts = args.max_attempts if args.until_gyro_live else 1
    for i in range(1, attempts + 1):
        print(f'--- reset {i}/{attempts} on {args.port} ---', flush=True)
        try:
            pulse_reset(args.port)
        except Exception as e:
            print(f'FAIL: could not drive the reset lines: {e}')
            print('Is something else holding the port? The micro-ROS agent holds it only '
                  'in the serial transport; this setup uses udp4, so it should be free.')
            return 2
        print('  EN pulsed; ESP32 restarting', flush=True)

        if args.no_verify:
            return 0

        # The board reconnects to Wi-Fi and re-establishes its XRCE session; nothing
        # publishes until it does.
        print(f'  waiting up to {args.wait:.0f} s for the session to return...',
              flush=True)
        deadline = time.time() + args.wait
        rc, out = 1, ''
        while time.time() < deadline:
            time.sleep(5.0)
            rc, out = sensor_health(args.domain, args.check_seconds)
            if 'SILENT' not in out and 'core topics are silent' not in out:
                break

        state = gyro_state(out)
        print(f'  gyro after reset: {state}', flush=True)
        for line in out.splitlines():
            if any(k in line for k in ('imu gyro z', 'imu accel z', 'LIDAR', 'BATTERY',
                                       'SENSOR HEALTH')):
                print(f'    {line.strip()}', flush=True)

        if rc == 0:
            print()
            print('  PASS: every channel alive after a SERIAL RESET -- no power cycle '
                  'needed.')
            return 0
        if state == 'live':
            print()
            print('  gyro recovered, though another check still failed (see above)')
            return 1
        if not args.until_gyro_live:
            break
        print('  gyro still dead; retrying', flush=True)

    print()
    print('  Reset did not revive the gyro.')
    print('  A reset restarts the PROCESSOR but leaves the IMU powered, so it cannot')
    print('  clear a fault latched inside the chip. If a power cycle does not fix it')
    print('  either -- and on 2026-08-06 one did not -- that points at the sensor or')
    print('  its wiring rather than at firmware init, and is worth raising with Yahboom.')
    return 1


if __name__ == '__main__':
    code = main()
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(code)
