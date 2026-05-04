#!/usr/bin/env python3
"""yahboom_diag.py — direct hardware test, bypassing the ROS bridge.

Run with the slam_bringup yahboom_bridge STOPPED (only one process can
hold /dev/myserial). Walks through buzzer → individual motor PWM →
mecanum kinematics in three escalating tests. Whichever step fails
points at the layer to debug:

  1. Buzzer fails  → serial / firmware / power dead. Check 24V, /dev/myserial,
                     dialout group, USB-C cable orientation.
  2. set_motor works but set_car_motion doesn't → firmware mecanum path
                     bug. Try set_car_type(1) explicitly, or check that
                     car_type=1 matches your hardware (X3 mecanum vs X3plus).
  3. set_motor fails → motor power not enabled. Check the motor-power
                     switch on the YB-ERF01 (separate from logic power!),
                     emergency stop, motor cable seating.
"""
import sys, time
from Rosmaster_Lib import Rosmaster

PORT = '/dev/myserial'
CAR_TYPE = 1   # 1=X3 mecanum (matches yahboom_bridge_node.py default)

print(f"Opening {PORT}, car_type={CAR_TYPE}...")
bot = Rosmaster(car_type=CAR_TYPE, com=PORT)
bot.create_receive_threading()
bot.set_car_type(CAR_TYPE)
time.sleep(0.5)

print("\n[1/3] Beep test (firmware reachable?)")
bot.set_beep(50)             # 50 ms beep
time.sleep(0.3)
print("  → Did you hear a beep? If NO, serial/firmware path dead.")

input("Press Enter to continue to motor test (wheels OFF GROUND)...")

print("\n[2/3] Direct motor PWM (bypass mecanum kinematics)")
print("  Spinning all 4 wheels forward at PWM 30 for 2s...")
bot.set_motor(30, 30, 30, 30)   # speed range -100..100
time.sleep(2.0)
bot.set_motor(0, 0, 0, 0)
print("  → Did all 4 wheels spin? If NO, motor power not enabled "
      "(check YB-ERF01 motor power switch / E-stop / battery).")

input("Press Enter to continue to mecanum kinematics test...")

print("\n[3/3] set_car_motion (the path the ROS bridge uses)")
print("  Forward at vx=0.3 m/s for 2s...")
for _ in range(20):              # 20×0.1s = 2s, refresh every loop
    bot.set_car_motion(0.3, 0.0, 0.0)
    time.sleep(0.1)
bot.set_car_motion(0.0, 0.0, 0.0)
print("  → Did the rover roll forward? If NO but step 2 worked, the"
      "    firmware's FUNC_MOTION path is the issue — try a different"
      "    car_type, or fall back to set_motor() in the bridge.")

print("\nDone.")
