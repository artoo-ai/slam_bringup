"""Visualization-only z-axis clip on a PointCloud2 stream.

Subscribes to an input PointCloud2 (default /cloud_registered, FAST-LIO2's
camera_init-frame per-scan cloud), drops every point whose z is outside
[z_min, z_max], and republishes on /cloud_viz_clipped. Used by RViz /
Foxglove top-down floorplan views — see launch/viz_clip.launch.py and
the README "Top-down floorplan view" section for context.

Implementation note: we replaced the original pcl_ros::PassThrough
composable node here because the pcl_ros 2.4.5 deb on Humble does not
expose that filter as a loadable component on this build (class_loader
fails the lookup at container start). A small rclpy node sidesteps the
packaging issue and is plenty fast at FAST-LIO2's ~10 Hz scan rate on
the Orin Nano. Implemented as a copy of the byte buffer with a numpy
mask on the structured view of the z field — no per-point Python loop.
"""

import numpy as np
import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import PointCloud2


def _z_field_offset(msg: PointCloud2):
    """Return (offset, numpy dtype) of the z field, or None if not present."""
    type_to_np = {
        1: np.int8, 2: np.uint8, 3: np.int16, 4: np.uint16,
        5: np.int32, 6: np.uint32, 7: np.float32, 8: np.float64,
    }
    for f in msg.fields:
        if f.name == 'z':
            return f.offset, type_to_np[f.datatype]
    return None


class VizClipNode(Node):
    def __init__(self):
        super().__init__('viz_z_clip')

        self.declare_parameter('input_topic',  '/cloud_registered')
        self.declare_parameter('output_topic', '/cloud_viz_clipped')
        self.declare_parameter('z_min', -1.0)
        self.declare_parameter('z_max',  2.0)

        in_topic  = self.get_parameter('input_topic').value
        out_topic = self.get_parameter('output_topic').value
        self._z_min = float(self.get_parameter('z_min').value)
        self._z_max = float(self.get_parameter('z_max').value)

        # Allow z_min / z_max to be retuned live via:
        #     ros2 param set /viz_z_clip z_max 3.5
        # input_topic / output_topic are intentionally NOT runtime-tunable
        # (changing them would require tearing down + recreating the
        # subscriber/publisher; not worth the complexity).
        self.add_on_set_parameters_callback(self._on_param_change)

        # FAST-LIO2 publishes /cloud_registered with BEST_EFFORT reliability
        # and VOLATILE durability; match that or we'll drop every message
        # silently with no visible error.
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=5,
        )

        self._pub = self.create_publisher(PointCloud2, out_topic, sensor_qos)
        self._sub = self.create_subscription(
            PointCloud2, in_topic, self._on_cloud, sensor_qos,
        )

        self.get_logger().info(
            f"viz_z_clip: {in_topic} -> {out_topic}  "
            f"z in [{self._z_min:.2f}, {self._z_max:.2f}] m  "
            f"(retune live: ros2 param set {self.get_name()} z_max <value>)"
        )

    def _on_param_change(self, params):
        # Validate first; only commit changes if every requested update is sane.
        new_z_min = self._z_min
        new_z_max = self._z_max
        for p in params:
            if p.name == 'z_min':
                new_z_min = float(p.value)
            elif p.name == 'z_max':
                new_z_max = float(p.value)
        if new_z_min >= new_z_max:
            return SetParametersResult(
                successful=False,
                reason=f"z_min ({new_z_min}) must be < z_max ({new_z_max})",
            )
        self._z_min, self._z_max = new_z_min, new_z_max
        self.get_logger().info(
            f"viz_z_clip: z range now [{self._z_min:.2f}, {self._z_max:.2f}] m"
        )
        return SetParametersResult(successful=True)

    def _on_cloud(self, msg: PointCloud2) -> None:
        # Treat the byte payload as an array of point_step-sized records,
        # extract the z field via a structured-dtype view, mask, then copy
        # the surviving rows back out. Avoids decoding/re-encoding every
        # field — works for any PointCloud2 layout (XYZ, XYZI, XYZRGB, ...).
        info = _z_field_offset(msg)
        if info is None:
            self.get_logger().warn_once(
                f"input cloud on '{self._sub.topic_name}' has no 'z' field; dropping."
            )
            return

        z_offset, z_dtype = info
        n_points = (msg.width * msg.height) if not msg.is_dense or msg.height > 1 \
            else (len(msg.data) // msg.point_step)
        if n_points == 0:
            return

        raw = np.frombuffer(msg.data, dtype=np.uint8)
        if raw.size != n_points * msg.point_step:
            # Truncated / padded message — bail rather than mis-slice.
            self.get_logger().warn(
                f"point_step {msg.point_step} * count {n_points} != data {raw.size}; skipping"
            )
            return

        records = raw.reshape(n_points, msg.point_step)
        z_bytes = records[:, z_offset:z_offset + np.dtype(z_dtype).itemsize]
        z = np.frombuffer(z_bytes.tobytes(), dtype=z_dtype)

        mask = (z >= self._z_min) & (z <= self._z_max)
        kept = records[mask]

        out = PointCloud2()
        out.header        = msg.header
        out.height        = 1
        out.width         = int(kept.shape[0])
        out.fields        = msg.fields
        out.is_bigendian  = msg.is_bigendian
        out.point_step    = msg.point_step
        out.row_step      = msg.point_step * out.width
        out.is_dense      = True
        out.data          = kept.tobytes()
        self._pub.publish(out)


def main():
    rclpy.init()
    node = VizClipNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
