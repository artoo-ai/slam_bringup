#!/usr/bin/env bash
# Ctrl-C usually stops realsense2_camera_node cleanly, but occasionally
# the USB video device stays in a bad state — next start_d435.sh then
# fails with "Device or resource busy", libuvc "couldn't resolve
# requests", or the streams open but no frames ever arrive ("Frames
# didn't arrive within 5 seconds"). Force a clean kill so the USB handle
# is released before the next launch, and deauth/reauth the RealSense
# at the sysfs level so a stuck camera firmware is reset too.

pkill -SIGINT -f realsense2_camera_node          # try graceful first
sleep 2
pkill -9      -f realsense2_camera_node          # nuke if still alive
pkill -9      -f "ros2 launch slam_bringup d435" # and the launch wrapper

# Wait for the rs node to actually exit before touching sysfs, so the
# USB deauth doesn't race the node still closing /dev/video*.
for _ in 1 2 3; do
  pgrep -f realsense2_camera_node > /dev/null || break
  sleep 1
done

ros2 daemon stop                                 # clear stale DDS discovery cache

# USB-level reset of every attached Intel RealSense camera. Writing 0
# then 1 to /sys/.../authorized triggers a full UVC re-enumeration,
# which is the only reliable way out of the "streams opened but zero
# frames delivered" firmware hang. VID:PIDs covered here —
#   8086:0b07 D435, 8086:0b3a D435i, 8086:0b64 D455.
# Sysfs writes need root; sudo may prompt for a password. If sudo
# refuses (no tty, user Ctrl-C's the prompt), we skip — the pkill
# cleanup above is still useful on its own.
found=0
for dev in /sys/bus/usb/devices/*/; do
  [ -f "${dev}idVendor" ] || continue
  vid=$(cat "${dev}idVendor" 2>/dev/null)
  pid=$(cat "${dev}idProduct" 2>/dev/null)
  [ "$vid" = "8086" ] || continue
  case "$pid" in 0b07|0b3a|0b64) ;; *) continue ;; esac
  found=1
  echo "kill_d435: USB reset $dev ($vid:$pid)"
  sudo sh -c "echo 0 > ${dev}authorized && sleep 1 && echo 1 > ${dev}authorized" \
    || echo "kill_d435: USB reset failed (sudo refused?) — next launch may still see the frames-timeout hang" >&2
done
[ "$found" = "0" ] && echo "kill_d435: no Intel RealSense USB device found — skipping USB reset"
