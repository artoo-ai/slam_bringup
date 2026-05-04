#!/usr/bin/env bash
# Start the Yahboom YB-ERF01 cmd_vel bridge (mecanum X3, USB-serial).
# Idempotent — kills any prior bridge before relaunching.
#
# Usage:
#   ./start_yahboom.sh                                  # auto-pick port via env / by-path / /dev/myserial
#   ./start_yahboom.sh serial_port:=/dev/ttyUSB0        # explicit override
#   ./start_yahboom.sh max_vx:=0.3 max_vy:=0.2          # tighter velocity caps for indoor
#
# Picking the serial port (in order):
#   1. serial_port:=... CLI arg                         (highest priority)
#   2. $YAHBOOM_SERIAL_PORT env var                     (set in ~/.bashrc for persistence)
#   3. ~/.yahboom_serial_port file                      (one line, the path)
#   4. /dev/myserial                                    (legacy udev SYMLINK)
#   5. /dev/ttyUSB0                                     (last-resort guess)
#
# When the WitMotion (also CH340 1a86:7523) is plugged in too, /dev/myserial
# may race between the two devices. Prefer a /dev/serial/by-path/<...> path
# (auto-generated, stable per physical USB port, no udev rule needed):
#
#   ls -l /dev/serial/by-path/
#   echo /dev/serial/by-path/platform-3610000.usb-usb-0:2.1.3:1.0-port0 \
#     > ~/.yahboom_serial_port
#
# Pre-reqs (one-time):
#   1. Yahboom plugged in via USB-C (data port, NOT power-only)
#   2. pip3 install --user vendor/Rosmaster_Lib_3.3.9   (install.sh does this)
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

# Resolve the serial port unless the caller passed serial_port:=... explicitly.
HAS_SERIAL_OVERRIDE=0
for arg in "$@"; do
  case "$arg" in serial_port:=*) HAS_SERIAL_OVERRIDE=1 ;; esac
done

if [ "$HAS_SERIAL_OVERRIDE" -eq 0 ]; then
  RESOLVED_PORT=""
  if [ -n "${YAHBOOM_SERIAL_PORT:-}" ] && [ -e "$YAHBOOM_SERIAL_PORT" ]; then
    RESOLVED_PORT="$YAHBOOM_SERIAL_PORT"
    echo "start_yahboom: using \$YAHBOOM_SERIAL_PORT → $RESOLVED_PORT"
  elif [ -f "$HOME/.yahboom_serial_port" ]; then
    CFG_PORT="$(head -n1 "$HOME/.yahboom_serial_port" | tr -d '[:space:]')"
    if [ -n "$CFG_PORT" ] && [ -e "$CFG_PORT" ]; then
      RESOLVED_PORT="$CFG_PORT"
      echo "start_yahboom: using ~/.yahboom_serial_port → $RESOLVED_PORT"
    fi
  fi
  if [ -z "$RESOLVED_PORT" ] && [ -e /dev/myserial ]; then
    RESOLVED_PORT="/dev/myserial"
  fi
  if [ -z "$RESOLVED_PORT" ] && [ -e /dev/ttyUSB0 ]; then
    RESOLVED_PORT="/dev/ttyUSB0"
  fi
  if [ -z "$RESOLVED_PORT" ]; then
    echo "start_yahboom: ERROR — no Yahboom serial device found." >&2
    echo "  Plug in the USB-C cable and verify with:  ls -l /dev/serial/by-path/" >&2
    echo "  Then either:" >&2
    echo "    export YAHBOOM_SERIAL_PORT=/dev/serial/by-path/<...>" >&2
    echo "    or echo '/dev/serial/by-path/<...>' > ~/.yahboom_serial_port" >&2
    exit 1
  fi
  set -- "serial_port:=$RESOLVED_PORT" "$@"
fi

exec ros2 launch slam_bringup yahboom.launch.py "$@"
