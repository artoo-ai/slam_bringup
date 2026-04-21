#!/usr/bin/env bash
# Start FAST-LIO2 mapping against the Mid-360 + its onboard IMU.
# Idempotent: cleans a stale fastlio_mapping process before relaunching.
# Ctrl-C is usually clean, but if the node gets stuck in its TBB worker
# pool (rare but seen during long heavy-motion runs) it can leave the
# process holding the input topics in a way that confuses the next launch.
#
# Pre-requisite: Mid-360 must already be publishing in CustomMsg mode
# (xfer_format=1). Easiest way:
#   ./start_sensors.sh lidar_xfer_format:=1 enable_d435:=false enable_witmotion:=false
# (D435 + WitMotion are not used by FAST-LIO2 itself — drop them to free
# CPU during pose-quality bring-up tests.)

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

source /opt/ros/humble/setup.bash
source ~/slam_ws/install/setup.bash

if pgrep -f fastlio_mapping > /dev/null \
   || pgrep -f "ros2 launch slam_bringup fast_lio" > /dev/null; then
  echo "start_fast_lio: fastlio_mapping already running — cleaning up first"
  "$SCRIPT_DIR/kill_fast_lio.sh"

  for _ in 1 2 3 4 5; do
    pgrep -f fastlio_mapping > /dev/null || break
    sleep 1
  done

  if pgrep -f fastlio_mapping > /dev/null; then
    echo "start_fast_lio: ERROR — fastlio_mapping still running after kill_fast_lio.sh" >&2
    exit 1
  fi
fi

# Preflight: FAST-LIO2 is useless without a Mid-360 publishing CustomMsg.
# The silent-failure mode is painful: the node comes up, subscribes to
# /livox/lidar and /livox/imu, and just waits forever with no errors — you
# notice later by staring at an empty Foxglove. Catch both failure modes
# here:
#   1) Mid-360 driver isn't running at all (no publisher on /livox/lidar).
#   2) Driver IS running but in PointCloud2 mode (xfer_format=0). FAST-LIO
#      expects livox_ros_driver2/CustomMsg and silently drops everything
#      else. Fix: relaunch sensors with lidar_xfer_format:=1.
# Set START_FAST_LIO_SKIP_PREFLIGHT=1 to bypass (e.g. bag replay).
if [ "${START_FAST_LIO_SKIP_PREFLIGHT:-0}" != "1" ]; then
  # Wait up to 10s for /livox/lidar to appear + settle; sensor bringup
  # often lags the user by a couple seconds.
  lidar_type=""
  for _ in 1 2 3 4 5 6 7 8 9 10; do
    lidar_type=$(ros2 topic info /livox/lidar 2>/dev/null | awk -F': ' '/^Type:/ {print $2}')
    [ -n "$lidar_type" ] && break
    sleep 1
  done

  if [ -z "$lidar_type" ]; then
    echo "start_fast_lio: ERROR — /livox/lidar has no publisher." >&2
    echo "  The Mid-360 driver is not running. FAST-LIO2 needs Mid-360 + IMU data." >&2
    echo "  Fix: ./start_sensors.sh lidar_xfer_format:=1 enable_d435:=false enable_witmotion:=false" >&2
    exit 1
  fi

  # Publisher type must be CustomMsg (xfer_format=1). Note ros2 renders the
  # type as a python list when multiple endpoints disagree, e.g.
  # "['livox_ros_driver2/msg/CustomMsg', 'sensor_msgs/msg/PointCloud2']" —
  # that case means the publisher is fine but some subscriber (often
  # foxglove_bridge) is on the wrong type, which is harmless for FAST-LIO.
  # So match CustomMsg anywhere in the string, and fail only if it's absent.
  if ! echo "$lidar_type" | grep -q 'livox_ros_driver2/msg/CustomMsg'; then
    echo "start_fast_lio: ERROR — /livox/lidar is publishing type '$lidar_type'." >&2
    echo "  FAST-LIO2 requires livox_ros_driver2/CustomMsg (xfer_format=1)." >&2
    echo "  Mid-360 was likely started in PointCloud2 mode. Restart with:" >&2
    echo "    ./kill_sensors.sh && ./start_sensors.sh lidar_xfer_format:=1 enable_d435:=false enable_witmotion:=false" >&2
    exit 1
  fi

  # /livox/imu check is a cheap sanity: same driver, so if the lidar
  # publisher is up the IMU one almost always is too, but catching the
  # "wrong driver config" case is worth the one extra call.
  if ! ros2 topic info /livox/imu 2>/dev/null | grep -q '^Publisher count: [1-9]'; then
    echo "start_fast_lio: WARNING — /livox/imu has no publisher." >&2
    echo "  FAST-LIO2 will not converge without the Mid-360 onboard IMU. Continuing anyway." >&2
  fi
fi

exec ros2 launch slam_bringup fast_lio.launch.py "$@"
