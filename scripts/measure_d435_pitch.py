#!/usr/bin/env python3
"""
Measure the residual pitch error of a D435 (or any depth camera) by
fitting a plane to a known-flat horizontal floor and comparing the
plane normal to body +Z.

Why this exists
---------------
The MakerWorld #1788451 D435 mount holds the camera nose-down by ~20°
(see urdf/sensors_common.urdf.xacro). That value is currently set
from a digital-inclinometer reading on a rounded camera housing with
±2° uncertainty. When the rig is mapping with RTABMap, even 2° of
unmodeled pitch tilts the floor in the octomap by ~10 cm at 3 m range,
which breaks Nav2 costmaps.

This script gives a one-shot calibration when:
  - You swap the camera, mount, or platform
  - You see the RTABMap floor visibly tilt
  - You want to verify the URDF pitch is correct after a build

What it does
------------
1. Subscribes to a depth + camera_info pair.
2. For N frames:
   - Reprojects the depth image to 3D points in the depth optical frame.
   - RANSAC-fits a plane to the lower portion of the FOV (where the
     floor lives for a forward-looking camera). The largest inlier
     plane is the assumed floor.
3. Looks up the URDF transform from base_link → depth optical frame.
4. Transforms the plane normal from optical frame to base_link.
5. The residual angle between (transformed normal) and (0, 0, 1) IS
   the pitch error — assuming the rig is sitting on a level horizontal
   floor (verify with a bubble level if you want sub-degree accuracy).
6. Reports:
   - Current URDF pitch (read from the URDF TF chain)
   - Residual pitch error (what's missing from URDF pitch)
   - Suggested new URDF pitch value to drop into d435_front_rpy

Prerequisites
-------------
  source /opt/ros/humble/setup.bash
  source ~/slam_ws/install/setup.bash

  ./start_perception.sh               # URDF + sensors must be running

The rig MUST be sitting on a flat, level, horizontal surface (table or
floor) with at least ~1 m of clear floor in front. The script does NOT
verify rig levelness — that's on you. If your "floor" is tilted, the
script measures rig-tilt + camera-mount-pitch combined, which is not
what you want.

Usage
-----
  python3 scripts/measure_d435_pitch.py --ros-args \
      -p camera_namespace:=d435_front \
      -p num_frames:=30

Optional parameters:
  -p depth_topic:=/d435_front/camera/depth/image_rect_raw
  -p camera_info_topic:=/d435_front/camera/depth/camera_info
  -p body_frame:=base_link        # gravity-aligned reference frame to compare against
  -p ransac_iterations:=200
  -p ransac_threshold:=0.02       # meters; points within this distance count as inliers
  -p min_inlier_fraction:=0.30    # warn if floor plane has fewer inliers than this

Topic defaults are the unaligned depth stream — published whenever the
D435 driver is up, regardless of slam_mode. Alignment to color does not
affect plane-fitting accuracy. To use the aligned stream when available
(slam_mode:=true), override with:
  -p depth_topic:=/d435_front/camera/aligned_depth_to_color/image_raw
  -p camera_info_topic:=/d435_front/camera/aligned_depth_to_color/camera_info

Limitations
-----------
- Only measures pitch (rotation around body Y). Roll and yaw could be
  added by also reporting the X and Z components of the plane normal,
  but for a forward-looking camera those are usually 0 by construction.
- Assumes the dominant horizontal plane in the lower FOV is the floor.
  If you have a low table, low desk, or sloped ground in front of the
  rig, the script will lock onto whatever is most planar — clear the
  area first.
- Depth values <0.3 m are filtered (D435 minimum range; closer values
  are noise/holes).
"""

import math
import sys

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Vector3Stamped
import tf2_ros
import tf2_geometry_msgs  # registers Vector3Stamped transformer
from cv_bridge import CvBridge


def fit_plane_ransac(points: np.ndarray,
                     iterations: int,
                     threshold: float,
                     rng: np.random.Generator) -> tuple[np.ndarray, np.ndarray]:
    """RANSAC plane fit to an Nx3 point array.

    Returns (best_normal, best_inlier_mask). best_normal is unit-length.
    Sign of the normal is not constrained — caller must orient it.
    """
    if len(points) < 3:
        raise ValueError(f"Need >=3 points; got {len(points)}")

    best_inliers = 0
    best_normal = None
    best_mask = None

    n = len(points)
    for _ in range(iterations):
        idx = rng.choice(n, size=3, replace=False)
        p1, p2, p3 = points[idx]
        v1 = p2 - p1
        v2 = p3 - p1
        normal = np.cross(v1, v2)
        norm_mag = np.linalg.norm(normal)
        if norm_mag < 1e-9:
            continue                       # collinear sample
        normal = normal / norm_mag
        d = -np.dot(normal, p1)            # plane: n·p + d = 0
        distances = np.abs(points @ normal + d)
        mask = distances < threshold
        inliers = int(mask.sum())
        if inliers > best_inliers:
            best_inliers = inliers
            best_normal = normal
            best_mask = mask

    if best_normal is None:
        raise RuntimeError("RANSAC found no plane (degenerate geometry?)")

    # Refit normal using ALL inliers via SVD (more accurate than the
    # 3-point sample). Replaces the random draw with a least-squares
    # fit to the inlier set.
    inlier_pts = points[best_mask]
    centroid = inlier_pts.mean(axis=0)
    centered = inlier_pts - centroid
    _, _, vh = np.linalg.svd(centered, full_matrices=False)
    refined_normal = vh[-1]                # smallest-singular-value direction
    refined_normal = refined_normal / np.linalg.norm(refined_normal)
    return refined_normal, best_mask


def depth_to_xyz(depth_m: np.ndarray, K: np.ndarray) -> np.ndarray:
    """Reproject a depth image (meters, HxW) to an Nx3 point array in
    the optical frame. Filters out invalid pixels (<= 0)."""
    h, w = depth_m.shape
    fx, fy = K[0, 0], K[1, 1]
    cx, cy = K[0, 2], K[1, 2]
    u, v = np.meshgrid(np.arange(w), np.arange(h))
    valid = depth_m > 0.3                  # D435 min range
    z = depth_m[valid]
    x = (u[valid] - cx) * z / fx
    y = (v[valid] - cy) * z / fy
    return np.stack([x, y, z], axis=1)


class PitchMeasurement(Node):

    def __init__(self):
        super().__init__('measure_d435_pitch')

        self.declare_parameter('camera_namespace', 'd435_front')
        self.declare_parameter('depth_topic', '')              # auto-derived if empty
        self.declare_parameter('camera_info_topic', '')        # auto-derived if empty
        self.declare_parameter('body_frame', 'base_link')
        self.declare_parameter('num_frames', 30)
        self.declare_parameter('ransac_iterations', 200)
        self.declare_parameter('ransac_threshold', 0.02)
        self.declare_parameter('min_inlier_fraction', 0.30)
        self.declare_parameter('floor_pixel_fraction', 0.5)    # bottom 50% of image

        ns = self.get_parameter('camera_namespace').value
        depth_topic = self.get_parameter('depth_topic').value or \
            f'/{ns}/camera/depth/image_rect_raw'
        info_topic = self.get_parameter('camera_info_topic').value or \
            f'/{ns}/camera/depth/camera_info'
        self._depth_topic = depth_topic
        self._info_topic = info_topic
        self._camera_namespace = ns

        self.body_frame = self.get_parameter('body_frame').value
        self.num_frames = int(self.get_parameter('num_frames').value)
        self.ransac_iterations = int(self.get_parameter('ransac_iterations').value)
        self.ransac_threshold = float(self.get_parameter('ransac_threshold').value)
        self.min_inlier_fraction = float(self.get_parameter('min_inlier_fraction').value)
        self.floor_pixel_fraction = float(self.get_parameter('floor_pixel_fraction').value)

        self.get_logger().info(f"Subscribing to depth: {depth_topic}")
        self.get_logger().info(f"Subscribing to info:  {info_topic}")
        self.get_logger().info(f"Body frame:            {self.body_frame}")
        self.get_logger().info(f"Capturing {self.num_frames} frames")

        # Best-effort QoS for camera streams (matches realsense2_camera default).
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.bridge = CvBridge()
        self.K: np.ndarray | None = None
        self.depth_optical_frame: str | None = None
        self.frames: list[np.ndarray] = []
        self.rng = np.random.default_rng()

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.create_subscription(CameraInfo, info_topic, self._info_cb, sensor_qos)
        self.create_subscription(Image, depth_topic, self._depth_cb, sensor_qos)

    def _info_cb(self, msg: CameraInfo):
        if self.K is None:
            self.K = np.array(msg.k).reshape(3, 3)
            self.depth_optical_frame = msg.header.frame_id
            self.get_logger().info(
                f"Got camera_info: fx={self.K[0,0]:.1f}, fy={self.K[1,1]:.1f}, "
                f"cx={self.K[0,2]:.1f}, cy={self.K[1,2]:.1f}, "
                f"frame_id={self.depth_optical_frame}"
            )

    def _depth_cb(self, msg: Image):
        if self.K is None:
            return     # haven't seen camera_info yet
        if len(self.frames) >= self.num_frames:
            return     # capture is complete

        # realsense2_camera publishes depth as 16UC1 in millimeters, or
        # as 32FC1 in meters depending on encoding settings. Handle both.
        if msg.encoding == '16UC1':
            depth_mm = self.bridge.imgmsg_to_cv2(msg, '16UC1')
            depth_m = depth_mm.astype(np.float32) / 1000.0
        elif msg.encoding == '32FC1':
            depth_m = self.bridge.imgmsg_to_cv2(msg, '32FC1')
        else:
            self.get_logger().error(f"Unsupported depth encoding: {msg.encoding}")
            return

        self.frames.append(depth_m)
        if len(self.frames) % 5 == 0:
            self.get_logger().info(
                f"Captured {len(self.frames)}/{self.num_frames} frames"
            )

    def is_ready(self) -> bool:
        return len(self.frames) >= self.num_frames and self.K is not None

    def _dump_tf_diagnostics(self) -> None:
        """Called when the body_frame ↔ depth_optical_frame transform
        can't be resolved. Prints what frames TF *does* know about so
        the user can see whether the URDF subtree (base_link, sensor_plate,
        ...) is missing, the realsense subtree is missing, or both are
        present but unconnected."""
        all_frames = self.tf_buffer.all_frames_as_string()
        log = self.get_logger().error
        log("=" * 72)
        log("TF tree dump (what tf2 currently knows about):")
        for line in all_frames.splitlines() or ["  (no frames at all)"]:
            log(f"  {line}")
        log("=" * 72)
        log(f"Need: {self.depth_optical_frame} → {self.body_frame}")
        log("Common causes:")
        log(f"  1. URDF didn't load — robot_state_publisher isn't publishing "
            f"{self.body_frame} or sensor_plate → d435_front_link.")
        log("     Check: ros2 topic echo /robot_description --once | head")
        log(f"  2. realsense node's root frame name doesn't match the URDF's "
            f"d435_front_link link.")
        log("     Check: ros2 run tf2_tools view_frames  (writes frames.pdf)")
        log(f"  3. Two unrelated TF trees from two different DDS partitions / "
            f"namespaces. Confirm both publishers see the same /tf, /tf_static.")
        log("Workaround if URDF tree is broken: run with -p body_frame:=" +
            f"{self.depth_optical_frame.rsplit('_', 2)[0]}_link  "
            "(skips the URDF and measures pitch in camera-link coords).")
        log("=" * 72)

    def diagnose_silent_topic(self) -> None:
        """Called when nothing has arrived for a few seconds — tells the
        user *why* the script is sitting idle, so they don't have to
        Ctrl-C and guess. Lists what topics ARE visible under the
        configured camera namespace."""
        all_topics = dict(self.get_topic_names_and_types())
        ns_prefix = f'/{self._camera_namespace}/'
        ns_topics = sorted(t for t in all_topics if t.startswith(ns_prefix))
        info = self.get_logger().warn
        info("Still waiting for camera_info — the configured topic is not publishing.")
        info(f"  expected camera_info: {self._info_topic}")
        info(f"  expected depth:       {self._depth_topic}")
        if ns_topics:
            info(f"  visible topics under {ns_prefix}:")
            for t in ns_topics:
                info(f"    {t}")
            info("  → if you only see /depth/* (not /aligned_depth_to_color/*), "
                 "either pass the unaligned topic overrides (see --help) or "
                 "relaunch with slam_mode:=true to enable depth-to-color alignment.")
        else:
            info(f"  no topics under {ns_prefix} at all — is the D435 driver running? "
                 "(./start_perception.sh in another terminal)")

    def analyze(self) -> int:
        """Run the analysis and print the report. Returns process exit code."""
        if not self.frames:
            self.get_logger().error("No depth frames captured.")
            return 1

        # Stack and median-filter across frames to suppress per-pixel noise.
        depth_stack = np.stack(self.frames, axis=0)
        depth_med = np.median(depth_stack, axis=0)

        # Restrict to the lower portion of the image — that's where the
        # floor lives for a forward-looking, slightly nose-down camera.
        # Avoids accidentally fitting the ceiling or a far wall.
        h, w = depth_med.shape
        cutoff = int(h * (1.0 - self.floor_pixel_fraction))
        depth_lower = depth_med.copy()
        depth_lower[:cutoff, :] = 0.0     # mask top portion

        # Re-derive (u, v) image coords for surviving valid pixels so we
        # can report WHERE the inliers sit in the image (a sanity check
        # that the fit hit the floor and not a wall / table edge).
        valid_mask = depth_lower > 0.3
        vs, us = np.nonzero(valid_mask)
        points = depth_to_xyz(depth_lower, self.K)
        if len(points) < 1000:
            self.get_logger().error(
                f"Too few valid depth points ({len(points)}) — is there a "
                f"flat floor in front of the rig with the lights on?"
            )
            return 1

        try:
            normal_optical, mask = fit_plane_ransac(
                points, self.ransac_iterations, self.ransac_threshold, self.rng
            )
        except RuntimeError as e:
            self.get_logger().error(str(e))
            return 1

        inlier_frac = mask.sum() / len(points)
        # Inlier image-coord stats — tells the user whether the fitted
        # plane occupies the bottom-center of the FOV (floor) or some
        # off-center sliver (wall, table edge, body of the rig).
        inlier_us = us[mask]
        inlier_vs = vs[mask]
        inlier_z = points[mask, 2]
        u_med = float(np.median(inlier_us)) if len(inlier_us) else float('nan')
        v_med = float(np.median(inlier_vs)) if len(inlier_vs) else float('nan')
        z_med = float(np.median(inlier_z))  if len(inlier_z)  else float('nan')
        z_min = float(np.min(inlier_z))     if len(inlier_z)  else float('nan')
        z_max = float(np.max(inlier_z))     if len(inlier_z)  else float('nan')

        # Orient normal so it points "up" in the optical frame. Floor is
        # below the camera, so optical Y is positive on the floor; the
        # surface normal therefore has NEGATIVE Y in the optical frame
        # (points up = points toward decreasing Y).
        if normal_optical[1] > 0:
            normal_optical = -normal_optical

        # Transform normal into the body frame.
        v_in = Vector3Stamped()
        v_in.header.frame_id = self.depth_optical_frame
        v_in.header.stamp = rclpy.time.Time().to_msg()    # latest available TF
        v_in.vector.x = float(normal_optical[0])
        v_in.vector.y = float(normal_optical[1])
        v_in.vector.z = float(normal_optical[2])
        try:
            v_body = self.tf_buffer.transform(v_in, self.body_frame,
                                              timeout=rclpy.duration.Duration(seconds=2.0))
        except Exception as e:
            self.get_logger().error(
                f"TF lookup failed ({self.depth_optical_frame} → {self.body_frame}): {e}"
            )
            self._dump_tf_diagnostics()
            return 1

        n_body = np.array([v_body.vector.x, v_body.vector.y, v_body.vector.z])
        n_body = n_body / np.linalg.norm(n_body)

        # Pitch error: tilt around body Y axis (left-right).
        # If floor normal in body frame has X != 0, the camera is over-
        # or under-pitched. n_body = (sin(err), 0, cos(err)) for pure
        # pitch error. Positive sin(err) means floor normal tilts toward
        # body +X (forward), which means the camera is OVER-pitched
        # (too nose-down) — so the URDF pitch needs to be LESS negative
        # (closer to zero). This is the residual that must be ADDED to
        # current URDF pitch.
        pitch_error_rad = math.asin(np.clip(n_body[0], -1.0, 1.0))

        # Roll error (rotation around body X): n_body Y component.
        roll_error_rad = math.asin(np.clip(-n_body[1], -1.0, 1.0))

        # Read current URDF pitch by extracting it from the TF chain.
        # Quaternion → euler conversion via the rotation that maps
        # body +Z to the optical-frame normal direction (approximately).
        # Easier: lookup the TF directly and decompose.
        try:
            tf_to_optical = self.tf_buffer.lookup_transform(
                self.body_frame, self.depth_optical_frame,
                rclpy.time.Time(), rclpy.duration.Duration(seconds=2.0),
            )
            q = tf_to_optical.transform.rotation
            # Roll/pitch/yaw from quaternion, in body→optical frame.
            # We want the camera_link's pitch in body frame, but the
            # optical frame is rotated 90° relative to camera_link
            # (REP-105 vs REP-103). The simplest measure: take the
            # angle between the optical Z axis (forward in REP-105)
            # transformed into body frame, and body +X.
            # optical_Z in optical = (0, 0, 1)
            # in body = quaternion-rotate (0,0,1)
            qx, qy, qz, qw = q.x, q.y, q.z, q.w
            opt_z_body = np.array([
                2 * (qx*qz + qw*qy),
                2 * (qy*qz - qw*qx),
                1 - 2 * (qx*qx + qy*qy),
            ])
            urdf_pitch_rad = math.asin(np.clip(-opt_z_body[2], -1.0, 1.0))
            # Negative because nose-down (REP-103 negative pitch) makes
            # the optical-Z (forward) point downward, giving a negative
            # body-Z component.
        except Exception as e:
            self.get_logger().warn(f"Could not read URDF pitch from TF: {e}")
            urdf_pitch_rad = float('nan')

        fit_is_reliable = inlier_frac >= self.min_inlier_fraction

        # Output report
        print()
        print("=" * 72)
        print("D435 PITCH MEASUREMENT")
        print("=" * 72)
        print(f"  Frames analyzed:      {len(self.frames)}")
        print(f"  Depth-points fit:     {len(points)}")
        print(f"  Floor inlier fraction: {inlier_frac:.1%}")
        if not fit_is_reliable:
            print(f"  ⚠  Inlier fraction below {self.min_inlier_fraction:.0%} — "
                  f"the fitted plane probably is NOT the floor.")
        print()
        # Inlier image-coord and depth diagnostics — for sanity-checking
        # what the script actually fit. A real floor under a forward-looking
        # camera should have:
        #   - inlier u_centroid near image center (cx ≈ {cx:.0f})
        #   - inlier v_centroid in the LOWER half (v > h/2 = {h//2})
        #   - depth range spanning a wide swath (z_max ≫ z_min)
        # If u_centroid is off-center, you fit a wall. If v_centroid is
        # high (small v), you fit something other than the floor. If
        # z_max ≈ z_min, you fit a small patch (e.g., the rig's own body).
        print(f"  Inlier centroid (u, v):     ({u_med:.0f}, {v_med:.0f})  "
              f"[image is {w}×{h}; floor should sit at u≈{w//2}, v in lower half (v > {h//2})]")
        print(f"  Inlier depth range:         {z_min:.2f} .. {z_max:.2f} m  "
              f"(median {z_med:.2f} m)  "
              f"[real floor spans a wide range; small range = non-floor surface]")
        print()
        print(f"  Floor normal in optical frame: "
              f"({normal_optical[0]:+.4f}, {normal_optical[1]:+.4f}, {normal_optical[2]:+.4f})")
        print(f"  Floor normal in {self.body_frame:>12s} frame: "
              f"({n_body[0]:+.4f}, {n_body[1]:+.4f}, {n_body[2]:+.4f})")
        print(f"  (Should be (0, 0, 1) for a perfectly-pitched camera on a level rig.)")
        print()
        if not math.isnan(urdf_pitch_rad):
            print(f"  Current URDF pitch:        "
                  f"{urdf_pitch_rad:+.4f} rad   ({math.degrees(urdf_pitch_rad):+.2f}°)")
        print(f"  Residual pitch error:      "
              f"{pitch_error_rad:+.4f} rad   ({math.degrees(pitch_error_rad):+.2f}°)")
        print(f"  Residual roll error:       "
              f"{roll_error_rad:+.4f} rad   ({math.degrees(roll_error_rad):+.2f}°)")
        if not math.isnan(urdf_pitch_rad):
            suggested = urdf_pitch_rad - pitch_error_rad
            print()
            if fit_is_reliable:
                print(f"  Suggested URDF pitch:      "
                      f"{suggested:+.6f} rad   ({math.degrees(suggested):+.2f}°)")
                print(f"  → urdf/sensors_common.urdf.xacro:")
                print(f"      <xacro:property name=\"d435_front_rpy\" "
                      f"value=\"0.0 {suggested:.6f} 0.0\"/>")
            else:
                print(f"  Suggested URDF pitch:      "
                      f"WITHHELD (inlier fraction {inlier_frac:.1%} < "
                      f"{self.min_inlier_fraction:.0%} threshold).")
                print(f"  Re-run on a clear flat floor (no obstacles in FOV, no")
                print(f"  furniture in the lower half of the image) before trusting")
                print(f"  any pitch suggestion. Computed-but-WITHHELD value would")
                print(f"  have been {math.degrees(suggested):+.2f}°.")
        print("=" * 72)
        print("Reminder: this assumes the rig is on a level horizontal floor.")
        print("If you didn't verify rig levelness with a bubble level, the")
        print("pitch error includes any rig-tilt and is NOT pure mount pitch.")
        print("=" * 72)
        return 0 if fit_is_reliable else 2


def main():
    rclpy.init()
    node = PitchMeasurement()
    try:
        # Spin until enough frames captured or Ctrl-C. Warn after 3 s
        # if camera_info still hasn't arrived, then again every 5 s, so
        # the user can tell the difference between "capturing" and "topic
        # is not published at all."
        start = node.get_clock().now()
        warn_after_s = 3.0
        warned = False
        while rclpy.ok() and not node.is_ready():
            rclpy.spin_once(node, timeout_sec=0.1)
            elapsed = (node.get_clock().now() - start).nanoseconds / 1e9
            if not warned and node.K is None and elapsed >= warn_after_s:
                node.diagnose_silent_topic()
                warned = True
                start = node.get_clock().now()    # re-arm for 5 s repeat
                warn_after_s = 5.0
        rc = node.analyze()
    except KeyboardInterrupt:
        rc = 130
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(rc)


if __name__ == '__main__':
    main()
