"""Scale Livox Mid-360 IMU acceleration from g to m/s².

Livox's livox_ros_driver2 (the version this stack pins via install.sh)
publishes /livox/imu with linear_acceleration in units of g (1.0 at
rest), not m/s². FAST-LIO2 — and every other ROS consumer — expects
sensor_msgs/Imu to be in m/s² per REP-145 / sensor_msgs convention.

The mismatch is an off-by-9.81 scale on the IMU prediction step in
FAST-LIO's ESKF: gravity init reads 1.0 instead of 9.81, accel
covariance tuned for m/s² is over-weighted by ~96×, and IMU
integration between scans is essentially scrambled. ICP pulls state
back into metric units so the map still resolves, but pose smoothing
and fast-motion handling are degraded.

This node is a thin republisher: subscribe /livox/imu, multiply each
linear_acceleration component by g, leave angular_velocity (rad/s)
and orientation alone, republish on /livox/imu_ms2. FAST-LIO is then
remapped to subscribe to /livox/imu_ms2 (see fast_lio.launch.py).

Auto-detection: the node samples the first 50 messages, checks the
magnitude. If it's already ~9.81 (driver in m/s² mode — possibly a
future Livox release) it logs a warning and passes the messages
through untouched, so this node is safe to leave in the launch even
if the driver behavior changes.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Imu

GRAVITY = 9.80665


class IMUUnitsNode(Node):
    def __init__(self):
        super().__init__('imu_units_g_to_ms2')

        self.declare_parameter('input_topic',  '/livox/imu')
        self.declare_parameter('output_topic', '/livox/imu_ms2')
        self.declare_parameter('autodetect_samples', 50)

        in_topic    = self.get_parameter('input_topic').value
        out_topic   = self.get_parameter('output_topic').value
        self._autodetect_n = int(self.get_parameter('autodetect_samples').value)

        self._magnitudes = []
        self._scale = None  # set after autodetect: 9.80665 (g→m/s²) or 1.0 (passthrough)

        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=20,
        )
        self._pub = self.create_publisher(Imu, out_topic, sensor_qos)
        self._sub = self.create_subscription(Imu, in_topic, self._on_imu, sensor_qos)

        self.get_logger().info(
            f"imu_units_g_to_ms2: {in_topic} -> {out_topic}  "
            f"(autodetect over first {self._autodetect_n} msgs)"
        )

    def _on_imu(self, msg: Imu) -> None:
        if self._scale is None:
            self._magnitudes.append(math.sqrt(
                msg.linear_acceleration.x ** 2
                + msg.linear_acceleration.y ** 2
                + msg.linear_acceleration.z ** 2
            ))
            if len(self._magnitudes) >= self._autodetect_n:
                self._decide_scale()
            else:
                # Don't publish until we know whether to scale —
                # otherwise we'd briefly emit raw-g values labeled m/s².
                return

        out = Imu()
        out.header                = msg.header
        out.orientation           = msg.orientation
        out.orientation_covariance = msg.orientation_covariance
        out.angular_velocity      = msg.angular_velocity
        out.angular_velocity_covariance = msg.angular_velocity_covariance
        out.linear_acceleration.x = msg.linear_acceleration.x * self._scale
        out.linear_acceleration.y = msg.linear_acceleration.y * self._scale
        out.linear_acceleration.z = msg.linear_acceleration.z * self._scale
        out.linear_acceleration_covariance = msg.linear_acceleration_covariance
        self._pub.publish(out)

    def _decide_scale(self) -> None:
        avg = sum(self._magnitudes) / len(self._magnitudes)
        # Three regimes:
        #   ~1.0   → driver in g; multiply by 9.80665
        #   ~9.81  → driver in m/s²; passthrough
        #   neither → IMU was in motion during startup; warn and assume
        #             g (the historical livox_ros_driver2 behavior).
        if abs(avg - 1.0) < 0.15:
            self._scale = GRAVITY
            self.get_logger().info(
                f"detected unit: g (mean magnitude {avg:.4f}). "
                f"Scaling accel by {GRAVITY:.5f}."
            )
        elif abs(avg - GRAVITY) < 1.0:
            self._scale = 1.0
            self.get_logger().warn(
                f"detected unit: m/s² (mean magnitude {avg:.4f}). "
                f"Driver appears to publish in correct units — passing through unchanged. "
                f"This node is safe but redundant; consider removing the remap in fast_lio.launch.py."
            )
        else:
            self._scale = GRAVITY
            self.get_logger().warn(
                f"could not autodetect IMU unit (mean magnitude {avg:.4f}, "
                f"expected ~1.0 or ~9.81). The rig may have been moving during "
                f"startup. Assuming g and scaling by {GRAVITY:.5f}; verify with "
                f"measure_imu_tilt.py once stationary."
            )


def main():
    rclpy.init()
    node = IMUUnitsNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
