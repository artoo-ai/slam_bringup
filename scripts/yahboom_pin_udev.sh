#!/usr/bin/env bash
# yahboom_pin_udev.sh — pin /dev/myserial to a specific YB-ERF01 USB port
# when more than one CH340 (1a86:7523) is on the bus.
#
# How to use:
#   1. Unplug EVERY CH340 device (YB-ERF01, debug adapters, etc.).
#   2. Plug the YB-ERF01 ALONE into the USB-C port you'll use long-term.
#   3. Run: ./scripts/yahboom_pin_udev.sh
#   4. Plug your other CH340 devices back in.
#
# The script reads the USB path the YB-ERF01 just enumerated on, writes
# /etc/udev/rules.d/99-yahboom.rules with KERNELS=="<that path>", and
# triggers udev. /dev/myserial will land on that physical port forever
# regardless of plug order or other CH340s on the bus.

set -euo pipefail

UDEV_FILE="/etc/udev/rules.d/99-yahboom.rules"

# Find every CH340 currently enumerated as a tty device
CH340_TTYS=()
for dev in /dev/ttyUSB*; do
  [ -e "$dev" ] || continue
  if udevadm info -q property "$dev" 2>/dev/null \
      | grep -q "ID_VENDOR_ID=1a86"; then
    CH340_TTYS+=("$dev")
  fi
done

case ${#CH340_TTYS[@]} in
  0) echo "ERROR: no CH340 USB-Serial devices found. Plug the YB-ERF01 in first."; exit 1 ;;
  1) ;;
  *) echo "ERROR: ${#CH340_TTYS[@]} CH340 devices currently plugged in."
     echo "Unplug everything except the YB-ERF01 and re-run."
     printf '  %s\n' "${CH340_TTYS[@]}"
     exit 1 ;;
esac

DEV="${CH340_TTYS[0]}"
echo "Found single CH340 → $DEV"

# Pull the USB physical port path (KERNELS attribute of the usb-serial parent)
USB_PATH="$(udevadm info -a "$DEV" \
  | awk '/SUBSYSTEMS=="usb-serial"/ {flag=1} flag && /KERNELS==/ {gsub(/.*"|"/,""); print; exit}')"
if [ -z "$USB_PATH" ]; then
  echo "ERROR: could not determine USB port path from $DEV."
  echo "Run: udevadm info -a $DEV   and inspect manually."
  exit 1
fi
echo "USB port path: $USB_PATH"

RULE='SUBSYSTEM=="tty", KERNELS=="'"$USB_PATH"'", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", SYMLINK+="myserial", MODE="0666"'

echo "Writing $UDEV_FILE:"
echo "  $RULE"
echo "$RULE" | sudo tee "$UDEV_FILE" >/dev/null

sudo udevadm control --reload-rules
sudo udevadm trigger

# Wait briefly for the symlink to materialize
for _ in 1 2 3 4 5; do
  if [ -e /dev/myserial ]; then break; fi
  sleep 0.2
done

if [ -e /dev/myserial ]; then
  echo "OK: /dev/myserial → $(readlink -f /dev/myserial)"
else
  echo "WARNING: /dev/myserial symlink didn't appear. Try unplug/replug."
fi
