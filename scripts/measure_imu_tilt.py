#!/usr/bin/env python3
"""
Measure the Mid-360 IMU's mounting tilt relative to gravity, for
FAST-LIO2 calibration.

Why this exists
---------------
FAST-LIO2 sets `camera_init`'s z-axis to whatever direction the IMU's
accelerometer reports as "down" at startup. If the Mid-360 sits on a
tilted sensor plate (or the plate is level but the LiDAR's mount cup
is not), the world frame inherits that tilt for the entire session —
manifests as a tilted Foxglove grid / floor that looks fine in
camera_init but skewed against the actual floor.

There are three ways to fix this once measured:
  1. Physically shim the sensor plate flat (best if the tilt is
     mechanical and >2°). This script tells you the angles.
  2. Bake an offsetting rotation into FAST-LIO's `extrinsic_R` in
     config/fast_lio_mid360.yaml so FAST-LIO knows the IMU's body
     frame is rotated relative to the LiDAR's body frame. This script
     prints the matrix to paste in.
  3. Live with it — if the tilt is < 0.5° it doesn't matter for the
     grid/visualization at typical room scale.

What it does
------------
1. Subscribes to /livox/imu.
2. Collects N samples while the rig sits perfectly still.
3. Averages the linear_acceleration vector (this is gravity in the
   IMU's body frame, with the IMU's bias term — but bias is small
   relative to 9.81 m/s², so the average direction is gravity to
   well under 0.1°).
4. Computes:
   - Magnitude (sanity check: should be ~9.80–9.82 m/s²).
   - Roll  = rotation about IMU +X needed to make accel +Z (or -Z,
     depending on the sign convention) point along world -gravity.
   - Pitch = rotation about IMU +Y, similarly.
5. Reports both the angles and the 3x3 rotation matrix that maps
   IMU body-frame vectors into a gravity-aligned frame. That matrix
   is what you'd paste into `extrinsic_R` (right-multiplying the
   identity already there) to compensate.

Sign convention assumption
--------------------------
The Livox Mid-360 IMU's body frame, per the Mid-360 datasheet, has
+x forward, +y left, +z up. When the rig is level, the accelerometer
reads (0, 0, +9.81) m/s² (the IMU senses the upward normal force from
the table — equivalent to gravity pointing down in the body frame's
sign convention is flipped to up because it's a specific-force
sensor). This script assumes that convention; if your accel reads
~(0, 0, -9.81) when level, run with --gravity-sign=-1.

Usage
-----
  # Rig stationary on a flat horizontal surface:
  python3 ~/slam_ws/src/slam_bringup/scripts/measure_imu_tilt.py \
      --ros-args -p num_samples:=600 -p imu_topic:=/livox/imu

  # If your IMU reports gravity as -9.81 in z when level:
  python3 ~/slam_ws/src/slam_bringup/scripts/measure_imu_tilt.py \
      --ros-args -p gravity_sign:=-1.0
"""

import math
import sys

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Imu


class IMUTiltMeasurer(Node):
    def __init__(self):
        super().__init__('measure_imu_tilt')

        self.declare_parameter('imu_topic',     '/livox/imu')
        self.declare_parameter('num_samples',   600)       # ~3 s at 200 Hz
        self.declare_parameter('gravity_sign',  1.0)        # +1 if level z reads +9.81; -1 if -9.81

        topic               = self.get_parameter('imu_topic').value
        self._target_n      = int(self.get_parameter('num_samples').value)
        self._gravity_sign  = float(self.get_parameter('gravity_sign').value)

        # Match the Livox driver's QoS — BEST_EFFORT/VOLATILE.
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=10,
        )

        self._samples = np.zeros((self._target_n, 3), dtype=np.float64)
        self._n = 0
        self._sub = self.create_subscription(Imu, topic, self._on_imu, sensor_qos)

        self.get_logger().info(
            f"Listening on {topic}; collecting {self._target_n} samples "
            f"(~{self._target_n/200.0:.1f} s at 200 Hz). KEEP THE RIG ABSOLUTELY STILL."
        )

    def _on_imu(self, msg: Imu) -> None:
        if self._n >= self._target_n:
            return
        self._samples[self._n, 0] = msg.linear_acceleration.x
        self._samples[self._n, 1] = msg.linear_acceleration.y
        self._samples[self._n, 2] = msg.linear_acceleration.z
        self._n += 1
        if self._n == self._target_n:
            self._report()
            rclpy.shutdown()

    def _report(self) -> None:
        accel = self._samples[:self._n].mean(axis=0)
        std   = self._samples[:self._n].std(axis=0)

        magnitude = float(np.linalg.norm(accel))
        # Direction the IMU reports gravity in IMU body frame.
        # We want to find the rotation R that maps this direction to
        # the body-frame "up" axis (0,0,1) (or "down" if gravity_sign=-1).
        target = np.array([0.0, 0.0, self._gravity_sign], dtype=np.float64)
        observed = accel / magnitude
        target_unit = target / np.linalg.norm(target)

        # Roll & pitch (small-angle, intuitive readout).
        # Define +pitch = nose up (rotation about IMU +y).
        # Define +roll  = right-side down (rotation about IMU +x).
        # When level, accel = (0, 0, ±g). Tilts pull the x/y components.
        if self._gravity_sign > 0:
            ax, ay, az = observed
        else:
            ax, ay, az = -observed[0], -observed[1], -observed[2]
        # asin works because the components are in [-1, 1] for unit vector.
        pitch_rad = math.atan2(-ax, math.sqrt(ay * ay + az * az))
        roll_rad  = math.atan2( ay, az)

        # Rotation matrix that, when applied to a vector in the IMU body
        # frame, expresses it in the gravity-aligned frame. Built as
        # R = R_x(-roll) · R_y(-pitch); applying it to `observed` should
        # give (0, 0, sign).
        cr, sr = math.cos(-roll_rad),  math.sin(-roll_rad)
        cp, sp = math.cos(-pitch_rad), math.sin(-pitch_rad)
        R_x = np.array([[1, 0, 0],
                        [0, cr, -sr],
                        [0, sr,  cr]])
        R_y = np.array([[ cp, 0, sp],
                        [  0, 1, 0],
                        [-sp, 0, cp]])
        R = R_x @ R_y
        check = R @ observed   # should be ~(0, 0, gravity_sign)

        print()
        print("=" * 70)
        print("Mid-360 IMU tilt calibration")
        print("=" * 70)
        print(f"Samples averaged:        {self._n}")
        print(f"Mean accel (m/s²):       [{accel[0]:+.4f}, {accel[1]:+.4f}, {accel[2]:+.4f}]")
        print(f"Per-axis std (m/s²):     [{std[0]:.4f}, {std[1]:.4f}, {std[2]:.4f}]"
              f"  (large = rig wasn't still)")
        print(f"Magnitude:               {magnitude:.4f} m/s²  "
              f"(expected 9.80–9.82; off by >0.05 → IMU scale issue)")
        print()
        print(f"Roll  (about IMU +x):    {math.degrees(roll_rad):+7.3f}°  "
              f"(positive = right-side down)")
        print(f"Pitch (about IMU +y):    {math.degrees(pitch_rad):+7.3f}°  "
              f"(positive = nose up)")
        print()
        if abs(math.degrees(roll_rad)) < 0.3 and abs(math.degrees(pitch_rad)) < 0.3:
            print("→ Tilt is < 0.3° on both axes. IMU is level within sensor noise;")
            print("  no correction needed in fast_lio_mid360.yaml.")
        else:
            print("Compensating extrinsic_R for fast_lio_mid360.yaml")
            print("(right-multiply this onto the existing extrinsic_R, or replace")
            print(" if extrinsic_R is currently identity):")
            print()
            print("            extrinsic_R: [")
            for i in range(3):
                row = R[i]
                comma = "," if i < 2 else ""
                print(f"                          {row[0]:+.6f}, {row[1]:+.6f}, {row[2]:+.6f}{comma}")
            print("                         ]")
            print()
            print(f"Sanity check: R @ observed = "
                  f"[{check[0]:+.4f}, {check[1]:+.4f}, {check[2]:+.4f}] "
                  f"(should be ~[0, 0, {self._gravity_sign:+.0f}])")
            print()
            print("If the tilt is >2° AND mechanical (sensor plate visibly off-level),")
            print("shim the plate flat instead of baking it into extrinsic_R — that")
            print("avoids carrying a software correction for a hardware problem.")
        print("=" * 70)


def main():
    rclpy.init()
    node = IMUTiltMeasurer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    if node._n < node._target_n:
        print(f"\nAborted early: only {node._n}/{node._target_n} samples collected.",
              file=sys.stderr)
        sys.exit(1)


if __name__ == '__main__':
    main()
