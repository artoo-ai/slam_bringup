#!/usr/bin/env python3
"""yahboom_configure.py — one-shot setup for a fresh YB-ERF01 board.

Run this ONCE on a new (or factory-reset) Yahboom board to provision it
for the X3 mecanum chassis used by this rover. Persists settings to the
STM32's flash so they survive power cycles.

What it does (in order):
  1. Probe firmware version + read current car_type (sanity check the
     serial channel actually reaches the MCU).
  2. Optional factory reset (--factory-reset) — wipes flash to defaults.
  3. set_car_type(1) — X3 mecanum, persisted (FUNC_SET_CAR_TYPE = 0x15).
  4. set_auto_report_state(False, forever=True) — disable the MCU's
     unsolicited encoder/IMU stream. Our SLAM stack uses FAST-LIO + the
     Mid-360 IMU; redundant reports just add serial noise.
  5. Beep + LED flash to confirm the firmware is acknowledging commands.
  6. Optional motor sanity test (--spin-wheels) — verifies each wheel
     individually and a brief mecanum-kinematics roll. WHEELS OFF GROUND.
  7. Read back car_type from flash to confirm persistence.

Run with the ROS bridge stopped — only one process can hold /dev/myserial:
    ./kill_yahboom.sh
    python3 scripts/yahboom_configure.py [--factory-reset] [--spin-wheels]

After this finishes, ./start_yahboom.sh + Nav2 / teleop should drive the
rover. The bridge node also calls set_car_type at runtime as a belt-and-
suspenders, but persisting it here means the board survives a bridge-less
cold boot in the right configuration.
"""
import argparse
import sys
import time

try:
    from Rosmaster_Lib import Rosmaster
except ModuleNotFoundError:
    sys.stderr.write(
        "Rosmaster_Lib not installed. Run install.sh, or:\n"
        "  pip3 install --user ~/slam_ws/src/slam_bringup/vendor/Rosmaster_Lib_3.3.9\n"
    )
    sys.exit(1)


PORT = '/dev/myserial'
CAR_TYPE_X3_MECANUM = 1   # Yahboom firmware constant 0x01

# X3 default PID for closed-loop motor speed control. These match the
# values shipped on a virgin Yahboom YB-ERF01 X3 (per Rosmaster_Lib
# docstring + Yahboom wiki). Override here if you've measured better
# gains on your specific motors.
PID_KP, PID_KI, PID_KD = 1.0, 0.1, 0.0


def banner(s):
    print(f"\n=== {s}")


def confirm(prompt, default_yes=False):
    suffix = " [Y/n] " if default_yes else " [y/N] "
    ans = input(prompt + suffix).strip().lower()
    if not ans:
        return default_yes
    return ans in ("y", "yes")


def main():
    ap = argparse.ArgumentParser(
        description="Configure a fresh Yahboom YB-ERF01 board for X3 mecanum."
    )
    ap.add_argument("--port", default=PORT, help=f"Serial device (default: {PORT})")
    ap.add_argument("--car-type", type=int, default=CAR_TYPE_X3_MECANUM,
                    help="Yahboom car_type (default: 1 = X3 mecanum, 2 = X3plus, 5 = R2)")
    ap.add_argument("--factory-reset", action="store_true",
                    help="Wipe the board's flash to factory defaults BEFORE configuring. "
                         "Use this on a board with unknown prior state.")
    ap.add_argument("--spin-wheels", action="store_true",
                    help="Run the motor sanity test (each wheel + a brief mecanum roll). "
                         "WHEELS MUST BE OFF THE GROUND.")
    ap.add_argument("--no-disable-reports", action="store_true",
                    help="Keep the MCU's auto-report stream enabled. Default is to "
                         "disable it (we don't consume those topics on this stack).")
    args = ap.parse_args()

    banner(f"Opening {args.port} (car_type={args.car_type})")
    bot = Rosmaster(car_type=args.car_type, com=args.port)
    bot.create_receive_threading()
    time.sleep(0.5)   # let the receive thread settle

    banner("Probe: firmware version + current car_type")
    version = bot.get_version()
    print(f"  firmware version: {version!r} "
          f"(returns -1 if no response → board not actually responding)")
    if version in (-1, 0):
        print("  ERROR: board didn't respond to FUNC_VERSION request.")
        print("  → Check /dev/myserial maps to the YB-ERF01 (udevadm info -a /dev/myserial).")
        print("  → Check the USB-C is the *data* port, not power-only.")
        print("  → Check no other process holds the serial port (./kill_yahboom.sh).")
        sys.exit(2)
    pre_car_type = bot.get_car_type_from_machine()
    print(f"  current car_type in flash: {pre_car_type} "
          f"(1=X3 mecanum, 2=X3plus, 3=X1, 5=R2, 6=X3 mini)")

    if args.factory_reset:
        banner("Factory reset (FUNC_RESET_FLASH = 0xA0)")
        if not confirm("This wipes ALL persisted settings on the board. Continue?"):
            print("Aborted.")
            sys.exit(0)
        bot.reset_flash_value()
        time.sleep(1.0)
        print("  flash reset issued.")

    banner(f"set_car_type({args.car_type}) — persisted via FUNC_SET_CAR_TYPE")
    bot.set_car_type(args.car_type)
    time.sleep(0.3)

    if not args.no_disable_reports:
        banner("set_auto_report_state(False, forever=True) — silence unsolicited reports")
        bot.set_auto_report_state(False, forever=True)
        time.sleep(0.3)

    banner(f"set_pid_param({PID_KP}, {PID_KI}, {PID_KD}, forever=True)")
    bot.set_pid_param(PID_KP, PID_KI, PID_KD, forever=True)
    time.sleep(0.5)

    banner("Confirm comms: beep + RGB flash")
    bot.set_beep(150)
    time.sleep(0.3)
    try:
        bot.set_colorful_lamps(0xFF, 0, 60, 0)   # all LEDs green
        time.sleep(0.4)
        bot.set_colorful_lamps(0xFF, 0, 0, 0)    # off
    except Exception as exc:
        print(f"  (LED command not supported on this firmware: {exc})")

    if args.spin_wheels:
        if not confirm("WHEELS OFF GROUND — run motor test?", default_yes=False):
            print("Skipping motor test.")
        else:
            banner("Per-wheel test (1.0s each at PWM 30)")
            for idx, label in enumerate(["FL", "FR", "BL", "BR"], start=1):
                speeds = [0, 0, 0, 0]
                speeds[idx - 1] = 30
                print(f"  spinning {label} (motor {idx}) ...")
                bot.set_motor(*speeds)
                time.sleep(1.0)
            bot.set_motor(0, 0, 0, 0)

            banner("Mecanum kinematics test: forward 0.3 m/s for 2s, then strafe right")
            for _ in range(20):
                bot.set_car_motion(0.3, 0.0, 0.0)
                time.sleep(0.1)
            bot.set_car_motion(0.0, 0.0, 0.0)
            time.sleep(0.5)
            for _ in range(20):
                bot.set_car_motion(0.0, -0.3, 0.0)   # strafe right
                time.sleep(0.1)
            bot.set_car_motion(0.0, 0.0, 0.0)

    banner("Read back persisted state")
    post_car_type = bot.get_car_type_from_machine()
    pid = bot.get_motion_pid()
    print(f"  car_type in flash: {post_car_type} (expected {args.car_type})")
    print(f"  motion PID:        {pid}")
    if post_car_type != args.car_type:
        print("  WARNING: car_type readback doesn't match. set_car_type packet")
        print("  may not have been acknowledged. Re-run with --factory-reset.")

    bot.set_beep(80)
    time.sleep(0.2)
    bot.set_beep(80)
    print("\nDone. Now run ./start_yahboom.sh and verify with teleop_twist_keyboard.")


if __name__ == "__main__":
    main()
