#!/usr/bin/env bash
# Start the Yahboom YB-ERF01 cmd_vel bridge (mecanum X3, USB-serial).
# Idempotent — kills any prior bridge before relaunching.
#
# Usage:
#   ./start_yahboom.sh                                 # /dev/myserial, car_type=1 mecanum
#   ./start_yahboom.sh serial_port:=/dev/ttyUSB0      # if udev pin not in place
#   ./start_yahboom.sh max_vx:=0.3 max_vy:=0.2        # tighter velocity caps for indoor
#
# Pre-reqs (one-time):
#   1. Yahboom plugged in via USB → /dev/ttyUSB0 (CH340 1a86:7523)
#   2. udev pin: /etc/udev/rules.d/99-yahboom.rules with
#      SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523",
#      SYMLINK+="myserial", MODE="0666"
#   3. pip3 install --user .  (from Yahboom's Rosmaster_Lib/ tarball)
#
# Bench-test sequence (per Obsidian "Yahboom Mecanum Configuration" §4):
#   - Wheels on blocks
#   - vx=+0.1 → all four wheels forward
#   - vy=+0.1 → MA+MD forward, MB+MC backward (strafe right)
#   - vz=+0.1 → MA+MB backward, MC+MD forward (yaw left / CCW)

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

source /opt/ros/humble/setup.bash
source ~/slam_ws/install/setup.bash
# shellcheck source=start_helpers.sh
source "$SCRIPT_DIR/start_helpers.sh"
ensure_foxglove

"$SCRIPT_DIR/kill_yahboom.sh" 2>/dev/null

if [ ! -e /dev/myserial ] && [ ! -e /dev/ttyUSB0 ]; then
  echo "start_yahboom: ERROR — no Yahboom serial device found." >&2
  echo "  Plug in the USB-C cable and verify with:  ls -l /dev/serial/by-id/" >&2
  echo "  Expected: usb-1a86_USB_Serial-if00-port0 → ttyUSB0" >&2
  exit 1
fi

exec ros2 launch slam_bringup yahboom.launch.py "$@"
