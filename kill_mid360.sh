#!/usr/bin/env bash
# Ctrl-C does not always fully stop livox_ros_driver2_node — it 
# can leave an orphan process holding UDP ports 56101/56201/56301/56401. 
# Symptoms: the next ros2 launch slam_bringup mid360.launch.py 
# starts but ros2 topic info /livox/lidar shows two publishers, 
# or the driver fails to bind its listener sockets. Clean kill:

pkill -SIGINT -f livox_ros_driver2_node     # try graceful first
sleep 2
pkill -9      -f livox_ros_driver2_node     # nuke if still alive
pkill -9      -f "ros2 launch slam_bringup mid360" # and the launch wrapper
ros2 daemon stop                            # clear stale DDS discovery cache

# Verify — a surviving livox_ros_driver2_node is the #1 cause of FAST-LIO's
# "lidar loop back, clear buffer" flood (two publishers interleave scans
# with slightly different timestamps; FAST-LIO drops them all).
if pgrep -f livox_ros_driver2_node >/dev/null; then
  echo "kill_mid360: WARNING — livox_ros_driver2_node still running after pkill -9:" >&2
  pgrep -af livox_ros_driver2_node >&2
  echo "  Try: sudo pkill -9 -f livox_ros_driver2_node" >&2
  exit 1
fi

