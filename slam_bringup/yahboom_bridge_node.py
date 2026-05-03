#!/usr/bin/env python3
"""Bridge /cmd_vel → Yahboom YB-ERF01 (STM32F103RCT6) over USB-serial.

Path A from the Obsidian note "Mecanum UGV - GitHub - AutomaticAddison
ROSMASTER X3 ROS2": skip mecanum_drive_controller, skip ros2_control,
skip cmd_vel_relay. The STM32 firmware does the mecanum inverse
kinematics internally when car_type=0x01 (X3) is set; we just need to
forward body-frame (vx, vy, wz) from /cmd_vel to set_car_motion().

Why this lives in slam_bringup and not a separate package:
    The whole rover stack should come up with one apt install + one
    colcon build. Forking AutomaticAddison's repo and porting it from
    Jazzy → Humble is a multi-day Yak shave — and 90% of what that repo
    provides (URDF, mecanum kinematics in software, EKF) is either
    redundant with our existing slam_bringup stack or done in firmware.
    A 100-line Python node IS the bridge. See companion vault notes:
      - "Yahboom Mecanum Configuration - Motor Wiring and Taranis SBUS Setup"
      - "Mecanum UGV - GitHub - AutomaticAddison ROSMASTER X3 ROS2"

Telemetry (encoder odom, IMU) is NOT published here — FAST-LIO + Mid-360
already own /Odometry and the IMU pipeline. Re-introducing /odom or
/imu/data from this node would create duplicate publishers and break
the stack we already built. If you ever want the Yahboom IMU as a
fallback, expose it on a namespaced topic (/yahboom/imu) so it doesn't
conflict.

Required: pip install Rosmaster-Lib (or build from Yahboom wiki tarball).
The import is at top-level so missing-package failures surface
immediately at launch with a clear traceback rather than as a silent
"why isn't the rover moving?"
"""

import math
import sys

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import Twist


def _import_rosmaster():
    """Defer the Rosmaster_Lib import behind a helpful error.

    Yahboom's library isn't on PyPI; if it's missing, print install
    guidance instead of dumping a raw ModuleNotFoundError that doesn't
    tell the user what to do.
    """
    try:
        from Rosmaster_Lib import Rosmaster
        return Rosmaster
    except ModuleNotFoundError:
        sys.stderr.write(
            "\n[yahboom_bridge] ERROR: Rosmaster_Lib not installed.\n"
            "  Yahboom does not publish this lib to PyPI. Install from\n"
            "  the kit microSD/USB stick, or download from\n"
            "    https://www.yahboom.net/study/ROSMASTER-X3 → Resources\n"
            "  Inside that zip, find Rosmaster_Lib/ with setup.py and:\n"
            "    pip3 install --user .\n\n"
        )
        raise


class YahboomBridge(Node):
    def __init__(self):
        super().__init__('yahboom_bridge')

        Rosmaster = _import_rosmaster()

        # Parameters — keep defaults aligned with our udev pin
        # (/dev/myserial → /dev/ttyUSB0, see kill_helpers / install docs).
        self.declare_parameter('serial_port', '/dev/myserial')
        # car_type=1 == X3 mecanum (Yahboom firmware constant 0x01).
        # If you ever swap this stack onto an X1/R2 platform, change the
        # arg via launch:  ros2 launch slam_bringup yahboom.launch.py car_type:=4
        self.declare_parameter('car_type', 1)
        # Watchdog timeout — if /cmd_vel goes silent for this many seconds,
        # command zero. Stops the rover dead if Nav2 / teleop crashes.
        self.declare_parameter('cmd_timeout', 0.5)
        # Velocity safety clamps. Mecanum Nav2 can request strafe (vy);
        # the X3 firmware accepts it, but we cap to chassis-safe values
        # to keep wheel slip from confusing FAST-LIO odom.
        self.declare_parameter('max_vx',     0.5)
        self.declare_parameter('max_vy',     0.3)
        self.declare_parameter('max_wz',     1.0)

        port      = self.get_parameter('serial_port').value
        car_type  = int(self.get_parameter('car_type').value)
        self.cmd_timeout = float(self.get_parameter('cmd_timeout').value)
        self.max_vx = float(self.get_parameter('max_vx').value)
        self.max_vy = float(self.get_parameter('max_vy').value)
        self.max_wz = float(self.get_parameter('max_wz').value)

        # Open the board. set_car_type belt-and-suspenders: even if the
        # board's flash already has X3 from a previous 0x15 packet, this
        # ensures we don't silently inherit some other car_type from a
        # prior bring-up.
        try:
            self.bot = Rosmaster(car_type=car_type, com=port)
            self.bot.create_receive_threading()
            self.bot.set_car_type(car_type)
        except Exception as exc:
            self.get_logger().error(
                f"yahboom_bridge: failed to open {port} (car_type={car_type}): {exc}\n"
                f"  Check: ls -l /dev/myserial   ;   groups | grep dialout\n"
                f"  And the udev rule at /etc/udev/rules.d/99-yahboom.rules"
            )
            raise

        # /cmd_vel — the only thing we subscribe to. Plain Twist (NOT
        # TwistStamped) is what every standard producer publishes:
        # Nav2 controller_server, teleop_twist_keyboard, twist_mux.
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_cb, 10)

        self._last_cmd_time = self.get_clock().now()
        self.create_timer(0.1, self._watchdog)

        self.get_logger().info(
            f"yahboom_bridge up — port={port}, car_type={car_type} "
            f"(1=X3 mecanum). Listening on /cmd_vel."
        )

    @staticmethod
    def _clamp(v, lim):
        return max(-lim, min(lim, v))

    def cmd_vel_cb(self, msg: Twist):
        vx = self._clamp(msg.linear.x,  self.max_vx)
        vy = self._clamp(msg.linear.y,  self.max_vy)
        wz = self._clamp(msg.angular.z, self.max_wz)
        # set_car_motion takes body-frame velocities m/s, m/s, rad/s.
        # The STM32 firmware does the mecanum inverse kinematics — we
        # don't compute per-wheel velocities here.
        self.bot.set_car_motion(vx, vy, wz)
        self._last_cmd_time = self.get_clock().now()

    def _watchdog(self):
        elapsed_ns = (self.get_clock().now() - self._last_cmd_time).nanoseconds
        if elapsed_ns / 1e9 > self.cmd_timeout:
            # Stop motors. Don't log every tick — only on edge transitions.
            self.bot.set_car_motion(0.0, 0.0, 0.0)

    def destroy_node(self):
        # Belt-and-suspenders: command zero on shutdown so SIGINT during
        # a Nav2 command doesn't leave the rover coasting.
        try:
            self.bot.set_car_motion(0.0, 0.0, 0.0)
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = YahboomBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
