#!/usr/bin/env python3
"""
Mid-360 LiDAR diagnostics for the 2040 test fixture.

Subscribes to /livox/lidar and /livox/imu for a configurable duration,
then prints a report covering:
  1. Self-hit detection (points inside blind range)
  2. Ground return analysis (blind ring radius)
  3. Feature density per scan
  4. IMU/LiDAR timestamp alignment
  5. Azimuth occlusion map (fixture shadow sectors)
  6. Range histogram

Usage:
  ros2 run slam_bringup lidar_diagnostics --ros-args \
      -p duration:=10.0 \
      -p lidar_height:=0.254 \
      -p blind:=0.2
"""

import math
import struct
import sys
from collections import defaultdict

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Imu, PointCloud2, PointField


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
        self.ground_ranges = []

        self.lidar_stamps = []
        self.imu_stamps = []

        self.scan_count = 0
        self.start_time = None

        self.lidar_sub = self.create_subscription(
            PointCloud2, "/livox/lidar", self.lidar_cb, 10
        )
        self.imu_sub = self.create_subscription(
            Imu, "/livox/imu", self.imu_cb, 50
        )

        self.get_logger().info(
            f"Collecting data for {self.duration}s | "
            f"lidar_height={self.lidar_height}m | blind={self.blind}m"
        )

    def lidar_cb(self, msg: PointCloud2):
        now = self.get_clock().now()
        if self.start_time is None:
            self.start_time = now

        elapsed = (now - self.start_time).nanoseconds / 1e9
        if elapsed > self.duration:
            return

        self.scan_count += 1
        stamp_ns = Time.from_msg(msg.header.stamp).nanoseconds
        self.lidar_stamps.append(stamp_ns)

        x, y, z = read_points_from_pc2(msg)
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
        near_field = int(np.sum((r_3d >= self.blind) & (r_3d < 0.5)))
        self.self_hit_counts.append(self_hits)
        self.near_field_counts.append(near_field)

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
        print(f"Points inside blind ({self.blind}m): {total_self}")
        if total_self > 0:
            print(f"  WARNING: {total_self} points passed through blind filter")
            print(f"  These are self-hits from fixture/plate/battery")
        print(f"Points in near-field ({self.blind}-0.5m): {total_near}")
        if total_near > 100:
            print(f"  NOTE: {total_near} near-field points — check if these are")
            print(f"  fixture reflections or legitimate returns")
        if total_self == 0 and total_near < 50:
            print(f"  OK: minimal self-hits, blind={self.blind}m is appropriate")

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
        print(f"Points per scan — min: {pts.min()}, max: {pts.max()}, "
              f"mean: {pts.mean():.0f}, std: {pts.std():.0f}")
        if pts.mean() < 2000:
            print(f"  WARNING: very low feature density (<2000 mean)")
            print(f"  FAST-LIO2 may fail to find correspondences")
            print(f"  Check point_filter_num (should be 1 for Mid-360)")
        elif pts.mean() < 5000:
            print(f"  CAUTION: moderate density — may be marginal indoors")
        else:
            print(f"  OK: sufficient feature density")

        # 4. Timestamp alignment
        print("\n--- 4. IMU/LIDAR TIMESTAMP ALIGNMENT ---")
        if len(self.imu_stamps) > 1 and len(self.lidar_stamps) > 1:
            imu_dt = np.diff(self.imu_stamps) / 1e6
            lidar_dt = np.diff(self.lidar_stamps) / 1e6

            imu_rate = 1000.0 / np.mean(imu_dt) if np.mean(imu_dt) > 0 else 0
            lidar_rate = 1000.0 / np.mean(lidar_dt) if np.mean(lidar_dt) > 0 else 0

            print(f"IMU samples: {len(self.imu_stamps)}, rate: {imu_rate:.1f} Hz")
            print(f"LiDAR scans: {len(self.lidar_stamps)}, rate: {lidar_rate:.1f} Hz")

            imu_start = self.imu_stamps[0]
            lidar_start = self.lidar_stamps[0]
            offset_ms = (lidar_start - imu_start) / 1e6
            print(f"First-stamp offset (LiDAR - IMU): {offset_ms:.2f} ms")
            if abs(offset_ms) > 50:
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
            bin_width = 10
            bins = np.arange(-180, 181, bin_width)
            hist, _ = np.histogram(az, bins=bins)
            mean_count = np.mean(hist)

            print(f"Points per {bin_width}° azimuth bin (mean={mean_count:.0f}):")
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
                print(f"  {lo:5.1f} - {hi:5.1f}m: {hist[i]:8d} ({pct:5.1f}%) {bar}")
        else:
            print("  No point data")

        # Summary
        print("\n--- SUMMARY ---")
        issues = []
        if total_self > 0:
            issues.append(f"Self-hits detected ({total_self} pts) — consider raising blind")
        if not self.ground_ranges:
            issues.append("No ground returns — FAST-LIO2 lacks Z constraint at this height")
        if pts.mean() < 2000:
            issues.append(f"Low feature density ({pts.mean():.0f} pts/scan)")
        if len(self.imu_stamps) > 1:
            imu_dt = np.diff(self.imu_stamps) / 1e6
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

    try:
        end_time = None
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.start_time is not None:
                elapsed = (node.get_clock().now() - node.start_time).nanoseconds / 1e9
                if elapsed > node.duration + 0.5:
                    break
    except KeyboardInterrupt:
        pass

    node.print_report()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
