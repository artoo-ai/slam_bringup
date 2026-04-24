#!/usr/bin/env python3
"""
Mid-360 LiDAR diagnostics for the 2040 test fixture.

Subscribes to /livox/lidar (CustomMsg or PointCloud2) and /livox/imu for
a configurable duration, then prints a report covering:
  1. Self-hit detection (points inside blind range)
  2. Ground return analysis (blind ring radius)
  3. Feature density per scan
  4. IMU/LiDAR timestamp alignment
  5. Azimuth occlusion map (fixture shadow sectors)
  6. Range histogram

Supports both xfer_format=0 (PointCloud2) and xfer_format=1 (CustomMsg).
Uses MultiThreadedExecutor to capture IMU at full 200 Hz.

Usage:
  python3 lidar_diagnostics.py --ros-args \
      -p duration:=10.0 \
      -p lidar_height:=0.254 \
      -p blind:=0.2
"""

import math
import threading
import time

import numpy as np
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Imu, PointCloud2, PointField

try:
    from livox_ros_driver2.msg import CustomMsg
    HAS_CUSTOM_MSG = True
except ImportError:
    HAS_CUSTOM_MSG = False


def read_points_from_custom_msg(msg):
    """Extract x, y, z arrays from a livox CustomMsg."""
    if len(msg.points) == 0:
        return np.empty(0), np.empty(0), np.empty(0)
    x = np.array([p.x for p in msg.points], dtype=np.float32)
    y = np.array([p.y for p in msg.points], dtype=np.float32)
    z = np.array([p.z for p in msg.points], dtype=np.float32)
    return x, y, z


def read_points_from_pc2(msg: PointCloud2):
    """Extract x, y, z arrays from a PointCloud2 message."""
    field_map = {f.name: f for f in msg.fields}
    if not all(k in field_map for k in ("x", "y", "z")):
        return np.empty(0), np.empty(0), np.empty(0)

    point_step = msg.point_step
    data = np.frombuffer(msg.data, dtype=np.uint8)
    n = msg.width * msg.height
    if n == 0:
        return np.empty(0), np.empty(0), np.empty(0)

    dtype_map = {
        PointField.FLOAT32: np.float32,
        PointField.FLOAT64: np.float64,
    }

    results = []
    for name in ("x", "y", "z"):
        f = field_map[name]
        dt = dtype_map.get(f.datatype, np.float32)
        offset = f.offset
        arr = np.array([
            np.frombuffer(data[i * point_step + offset:i * point_step + offset + np.dtype(dt).itemsize], dtype=dt)[0]
            for i in range(n)
        ])
        results.append(arr)

    return results[0], results[1], results[2]


class LidarDiagnostics(Node):
    def __init__(self):
        super().__init__("lidar_diagnostics")

        self.declare_parameter("duration", 10.0)
        self.declare_parameter("lidar_height", 0.254)
        self.declare_parameter("blind", 0.2)

        self.duration = self.get_parameter("duration").value
        self.lidar_height = self.get_parameter("lidar_height").value
        self.blind = self.get_parameter("blind").value

        self.scan_points_counts = []
        self.all_ranges = []
        self.all_azimuths = []
        self.all_elevations = []
        self.self_hit_counts = []
        self.near_field_counts = []
        self.near_field_azimuths = []
        self.ground_ranges = []

        self.lidar_stamps = []
        self.imu_stamps = []

        self.scan_count = 0
        self.start_time = None
        self.done = False
        self.msg_type_name = "unknown"

        cb_group = ReentrantCallbackGroup()

        if HAS_CUSTOM_MSG:
            self.custom_sub = self.create_subscription(
                CustomMsg, "/livox/lidar", self.custom_msg_cb, 10,
                callback_group=cb_group
            )
            self.pc2_sub = self.create_subscription(
                PointCloud2, "/livox/lidar", self.pc2_cb, 10,
                callback_group=cb_group
            )
            self.get_logger().info(
                "Subscribed to /livox/lidar (CustomMsg + PointCloud2 fallback)"
            )
        else:
            self.pc2_sub = self.create_subscription(
                PointCloud2, "/livox/lidar", self.pc2_cb, 10,
                callback_group=cb_group
            )
            self.get_logger().warn(
                "livox_ros_driver2 not found — PointCloud2 only. "
                "If using xfer_format:=1, run: "
                "source ~/slam_ws/install/setup.bash"
            )

        self.imu_sub = self.create_subscription(
            Imu, "/livox/imu", self.imu_cb, 200,
            callback_group=cb_group
        )

        self.get_logger().info(
            f"Collecting data for {self.duration}s | "
            f"lidar_height={self.lidar_height}m | blind={self.blind}m"
        )

    def custom_msg_cb(self, msg):
        if self.msg_type_name == "PointCloud2":
            return
        if self.msg_type_name == "unknown":
            self.msg_type_name = "CustomMsg"
            self.get_logger().info("Receiving CustomMsg (xfer_format=1)")
        self._process_scan(msg.header, *read_points_from_custom_msg(msg))

    def pc2_cb(self, msg):
        if self.msg_type_name == "CustomMsg":
            return
        if self.msg_type_name == "unknown":
            self.msg_type_name = "PointCloud2"
            self.get_logger().info("Receiving PointCloud2 (xfer_format=0)")
        self._process_scan(msg.header, *read_points_from_pc2(msg))

    def _process_scan(self, header, x, y, z):
        now = self.get_clock().now()
        if self.start_time is None:
            self.start_time = now

        elapsed = (now - self.start_time).nanoseconds / 1e9
        if elapsed > self.duration:
            self.done = True
            return

        self.scan_count += 1
        stamp_ns = Time.from_msg(header.stamp).nanoseconds
        self.lidar_stamps.append(stamp_ns)

        if len(x) == 0:
            self.scan_points_counts.append(0)
            return

        r_xy = np.sqrt(x**2 + y**2)
        r_3d = np.sqrt(x**2 + y**2 + z**2)
        azimuth = np.degrees(np.arctan2(y, x))
        elevation = np.degrees(np.arctan2(z, r_xy))

        self.scan_points_counts.append(len(x))
        self.all_ranges.extend(r_3d.tolist())
        self.all_azimuths.extend(azimuth.tolist())
        self.all_elevations.extend(elevation.tolist())

        self_hits = int(np.sum(r_3d < self.blind))
        near_mask = (r_3d >= self.blind) & (r_3d < 0.5)
        near_field = int(np.sum(near_mask))
        self.self_hit_counts.append(self_hits)
        self.near_field_counts.append(near_field)
        if near_field > 0:
            self.near_field_azimuths.extend(azimuth[near_mask].tolist())

        ground_z_min = -self.lidar_height - 0.05
        ground_z_max = -self.lidar_height + 0.05
        ground_mask = (z >= ground_z_min) & (z <= ground_z_max)
        ground_r = r_xy[ground_mask]
        if len(ground_r) > 0:
            self.ground_ranges.extend(ground_r.tolist())

        if self.scan_count % 10 == 0:
            self.get_logger().info(f"  {elapsed:.1f}s — {self.scan_count} scans collected")

    def imu_cb(self, msg: Imu):
        if self.start_time is None:
            return
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        if elapsed > self.duration:
            return
        stamp_ns = Time.from_msg(msg.header.stamp).nanoseconds
        self.imu_stamps.append(stamp_ns)

    def print_report(self):
        print("\n" + "=" * 65)
        print("  MID-360 LIDAR DIAGNOSTICS REPORT")
        print("=" * 65)

        pts = np.array(self.scan_points_counts) if self.scan_points_counts else np.array([0])
        print(f"\nScans collected: {self.scan_count}")
        print(f"Duration: {self.duration}s")
        print(f"Config: blind={self.blind}m, lidar_height={self.lidar_height}m")

        # 1. Self-hit detection
        print("\n--- 1. SELF-HIT DETECTION ---")
        total_self = sum(self.self_hit_counts)
        total_near = sum(self.near_field_counts)
        total_pts = sum(self.scan_points_counts)

        print(f"Points < {self.blind}m (filtered by FAST-LIO2 blind): {total_self}")
        if total_self > 0:
            pct = 100.0 * total_self / total_pts if total_pts > 0 else 0
            print(f"  {pct:.1f}% of raw points — these are fixture self-hits")
            print(f"  FAST-LIO2 blind={self.blind}m filters ALL of these (OK)")

        print(f"Points {self.blind}-0.5m (PASS through blind filter): {total_near}")
        if total_near > 0:
            near_pct = 100.0 * total_near / total_pts if total_pts > 0 else 0
            print(f"  {near_pct:.1f}% of raw points — these ENTER FAST-LIO2")
            if self.near_field_azimuths:
                nf_az = np.array(self.near_field_azimuths)
                bin_width = 30
                bins = np.arange(-180, 181, bin_width)
                hist, _ = np.histogram(nf_az, bins=bins)
                peak_idx = np.argmax(hist)
                peak_lo, peak_hi = bins[peak_idx], bins[peak_idx + 1]
                peak_pct = 100.0 * hist[peak_idx] / len(nf_az) if len(nf_az) > 0 else 0
                print(f"  Near-field azimuth concentration:")
                for i in range(len(hist)):
                    if hist[i] > 0:
                        lo, hi = bins[i], bins[i + 1]
                        pct = 100.0 * hist[i] / len(nf_az)
                        marker = " << FIXTURE?" if pct > 30 else ""
                        print(f"    {lo:+4d}° to {hi:+4d}°: {hist[i]:5d} ({pct:4.1f}%){marker}")
                if peak_pct > 30:
                    print(f"  LIKELY FIXTURE HITS — concentrated at {peak_lo:+d}° to {peak_hi:+d}°")
                    print(f"  Consider raising blind to 0.5")
                else:
                    print(f"  Spread across azimuths — likely legitimate room returns")
        elif total_self == 0:
            print(f"  OK: no self-hits, blind={self.blind}m is appropriate")

        # 2. Ground return analysis
        print("\n--- 2. GROUND RETURN ANALYSIS ---")
        expected_ring = self.lidar_height / math.tan(math.radians(7.0))
        print(f"Expected blind ring radius: {expected_ring:.2f}m")
        print(f"  (lidar at {self.lidar_height}m, min elevation -7°)")
        if self.ground_ranges:
            gr = np.array(self.ground_ranges)
            print(f"Ground returns found: {len(gr)}")
            print(f"  Nearest ground return: {gr.min():.2f}m")
            print(f"  Farthest ground return: {gr.max():.2f}m")
            print(f"  Median ground range: {np.median(gr):.2f}m")
            if gr.min() < expected_ring * 0.8:
                print(f"  NOTE: ground returns closer than expected —")
                print(f"  floor may not be perfectly level, or multipath")
        else:
            print(f"No ground returns detected.")
            print(f"  Room is likely smaller than {expected_ring:.1f}m radius")
            print(f"  FAST-LIO2 has NO ground plane Z-constraint here")

        # 3. Feature density
        print("\n--- 3. FEATURE DENSITY PER SCAN ---")
        if total_pts > 0:
            useful_per_scan = (total_pts - total_self) / self.scan_count if self.scan_count > 0 else 0
            print(f"Raw points per scan — min: {pts.min()}, max: {pts.max()}, "
                  f"mean: {pts.mean():.0f}")
            print(f"After blind filter — ~{useful_per_scan:.0f} pts/scan enter FAST-LIO2")
            if useful_per_scan < 2000:
                print(f"  WARNING: very low useful density (<2000)")
                print(f"  Check point_filter_num (should be 1 for Mid-360)")
            elif useful_per_scan < 5000:
                print(f"  CAUTION: moderate density — may be marginal indoors")
            else:
                print(f"  OK: sufficient feature density")
        else:
            print(f"Points per scan — min: {pts.min()}, max: {pts.max()}, mean: {pts.mean():.0f}")
            print(f"  WARNING: no point data collected")

        # 4. Timestamp alignment
        print("\n--- 4. IMU/LIDAR TIMESTAMP ALIGNMENT ---")
        if len(self.imu_stamps) > 1 and len(self.lidar_stamps) > 1:
            imu_dt = np.diff(self.imu_stamps) / 1e6
            lidar_dt = np.diff(self.lidar_stamps) / 1e6

            imu_rate = 1000.0 / np.mean(imu_dt) if np.mean(imu_dt) > 0 else 0
            lidar_rate = 1000.0 / np.mean(lidar_dt) if np.mean(lidar_dt) > 0 else 0

            print(f"IMU samples: {len(self.imu_stamps)}, rate: {imu_rate:.1f} Hz (expect ~200)")
            print(f"LiDAR scans: {len(self.lidar_stamps)}, rate: {lidar_rate:.1f} Hz (expect ~10)")

            if imu_rate < 150:
                print(f"  WARNING: IMU rate low — may indicate dropped messages")
            if lidar_rate < 8:
                print(f"  WARNING: LiDAR rate low — may indicate dropped scans")

            imu_arr = np.array(self.imu_stamps)
            lidar_arr = np.array(self.lidar_stamps)
            offsets = []
            for ls in lidar_arr[:10]:
                idx = np.argmin(np.abs(imu_arr - ls))
                offsets.append((ls - imu_arr[idx]) / 1e6)
            mean_offset = np.mean(offsets)
            print(f"Mean LiDAR↔IMU stamp offset (nearest pairs): {mean_offset:.2f} ms")
            if abs(mean_offset) > 50:
                print(f"  WARNING: >50ms offset — check time_sync_en")
            else:
                print(f"  OK: timestamps aligned (same hardware clock)")

            imu_jitter = np.std(imu_dt)
            lidar_jitter = np.std(lidar_dt)
            print(f"IMU period jitter: {imu_jitter:.2f} ms (expect <1ms)")
            print(f"LiDAR period jitter: {lidar_jitter:.2f} ms (expect <5ms)")
        else:
            print(f"  Insufficient data (IMU: {len(self.imu_stamps)}, "
                  f"LiDAR: {len(self.lidar_stamps)})")

        # 5. Azimuth occlusion map
        print("\n--- 5. AZIMUTH OCCLUSION MAP ---")
        if self.all_azimuths:
            az = np.array(self.all_azimuths)
            r = np.array(self.all_ranges)
            valid = r >= self.blind
            az_valid = az[valid]

            bin_width = 10
            bins = np.arange(-180, 181, bin_width)
            hist, _ = np.histogram(az_valid, bins=bins)
            mean_count = np.mean(hist)

            print(f"Points per {bin_width}° azimuth bin (excluding self-hits, mean={mean_count:.0f}):")
            sparse_bins = []
            for i in range(len(hist)):
                lo = bins[i]
                hi = bins[i + 1]
                ratio = hist[i] / mean_count if mean_count > 0 else 0
                bar = "#" * min(int(ratio * 20), 40)
                label = ""
                if ratio < 0.3:
                    label = " << OCCLUDED"
                    sparse_bins.append((lo, hi))
                elif ratio < 0.5:
                    label = " < sparse"
                print(f"  {lo:+4d}° to {hi:+4d}°: {hist[i]:6d} {bar}{label}")

            if sparse_bins:
                print(f"\nOccluded sectors (< 30% of mean):")
                for lo, hi in sparse_bins:
                    print(f"  {lo:+4d}° to {hi:+4d}°")
            else:
                print(f"\nNo significant occlusion detected")
        else:
            print("  No point data")

        # 6. Range histogram
        print("\n--- 6. RANGE HISTOGRAM ---")
        if self.all_ranges:
            r = np.array(self.all_ranges)
            range_bins = [0, 0.1, 0.2, 0.5, 1.0, 2.0, 3.0, 5.0, 10.0, 20.0, 50.0, 100.0]
            hist, _ = np.histogram(r, bins=range_bins)
            total = len(r)
            print(f"Total points: {total}")
            for i in range(len(hist)):
                lo = range_bins[i]
                hi = range_bins[i + 1]
                pct = 100.0 * hist[i] / total if total > 0 else 0
                bar = "#" * min(int(pct), 40)
                in_blind = " [filtered by blind]" if hi <= self.blind else ""
                print(f"  {lo:5.1f} - {hi:5.1f}m: {hist[i]:8d} ({pct:5.1f}%) {bar}{in_blind}")
        else:
            print("  No point data")

        # Summary
        print("\n--- SUMMARY ---")
        issues = []
        if total_near > 100 and self.near_field_azimuths:
            nf_az = np.array(self.near_field_azimuths)
            bins = np.arange(-180, 181, 30)
            hist, _ = np.histogram(nf_az, bins=bins)
            if len(nf_az) > 0 and np.max(hist) / len(nf_az) > 0.3:
                issues.append(
                    f"Near-field fixture hits ({total_near} pts) passing blind filter — "
                    f"consider blind: 0.5"
                )
        if not self.ground_ranges:
            issues.append("No ground returns — FAST-LIO2 lacks Z constraint at this height")
        if total_pts > 0:
            useful = total_pts - total_self
            useful_per = useful / self.scan_count if self.scan_count > 0 else 0
            if useful_per < 2000:
                issues.append(f"Low useful feature density ({useful_per:.0f} pts/scan)")
        if len(self.imu_stamps) > 1:
            imu_dt = np.diff(self.imu_stamps) / 1e6
            imu_rate = 1000.0 / np.mean(imu_dt) if np.mean(imu_dt) > 0 else 0
            if imu_rate < 150:
                issues.append(f"IMU rate low ({imu_rate:.0f} Hz, expect 200)")
            if np.std(imu_dt) > 2.0:
                issues.append(f"High IMU jitter ({np.std(imu_dt):.1f}ms)")

        if issues:
            print("Issues found:")
            for issue in issues:
                print(f"  - {issue}")
        else:
            print("No issues detected. Sensor data looks healthy.")

        print("\n" + "=" * 65)


def main(args=None):
    rclpy.init(args=args)
    node = LidarDiagnostics()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        while rclpy.ok() and not node.done:
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass

    node.print_report()
    node.destroy_node()
    try:
        rclpy.shutdown()
    except Exception:
        pass


if __name__ == "__main__":
    main()
