#!/usr/bin/env python3
"""
Measure the Mid-360's mounting tilt (roll/pitch) by fitting the floor plane.

Why this exists: urdf/sensors_common.urdf.xacro hard-codes
`livox_rpy = 0 0 0` (perfectly level). If the real mount is tilted even
2-3°, distant floor returns get projected UP into the pointcloud_to_
laserscan band (scan_z_min..scan_z_max above base_link) and become
phantom obstacles in /scan. slam_toolbox then maps a speckle ring around
the robot, inflation closes the free space, and Nav2's planner can't
reach any frontier goal ("GridBased: failed to create plan"). The leak
starts at range ≈ scan_z_min / sin(tilt): a 2° tilt pollutes everything
beyond ~4.3 m, 3° beyond ~2.9 m.

What it does: collects /livox/lidar for `duration` seconds (PointCloud2
or CustomMsg), selects the points below the sensor, least-squares fits a
plane with iterative outlier rejection, and prints:
  - measured roll/pitch of the lidar relative to the floor
  - the exact livox_rpy line to paste into sensors_common.urdf.xacro
  - the lidar height above the floor (to validate livox_xyz z)
  - the range at which floor returns start leaking into the scan band

Run it with the robot STATIONARY on flat, open floor (a few meters of
clear floor in front; rugs/thresholds skew the fit). Only the Mid-360
driver needs to be running, e.g.:

  ./start_mid360.sh                # or any stack that publishes /livox/lidar
  python3 scripts/measure_lidar_tilt.py --ros-args \
      -p duration:=10.0 -p topic:=/livox/lidar
"""

import math
import time

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2

try:
    from livox_ros_driver2.msg import CustomMsg
    HAS_CUSTOM_MSG = True
except ImportError:
    HAS_CUSTOM_MSG = False

# Matches scan_z_min default in launch/slam_2d.launch.py — used only to
# report the leak-onset range, not to filter points.
SCAN_Z_MIN_DEFAULT = 0.15


def read_points_from_pc2(msg: PointCloud2):
    """Extract x, y, z arrays from a PointCloud2 message via numpy striding."""
    field_map = {f.name: f for f in msg.fields}
    if not all(k in field_map for k in ("x", "y", "z")):
        return np.empty(0), np.empty(0), np.empty(0)

    n = msg.width * msg.height
    if n == 0:
        return np.empty(0), np.empty(0), np.empty(0)

    point_step = msg.point_step
    data = np.frombuffer(msg.data, dtype=np.uint8)

    results = []
    for name in ("x", "y", "z"):
        f = field_map[name]
        arr = np.ndarray(shape=(n,), dtype='<f4',
                         buffer=data, offset=f.offset, strides=(point_step,))
        results.append(arr.copy())

    return results[0], results[1], results[2]


def read_points_from_custom_msg(msg):
    """Extract x, y, z arrays from a livox CustomMsg."""
    n = msg.point_num
    if n == 0:
        return np.empty(0), np.empty(0), np.empty(0)
    x = np.empty(n, dtype=np.float32)
    y = np.empty(n, dtype=np.float32)
    z = np.empty(n, dtype=np.float32)
    for i, p in enumerate(msg.points):
        x[i] = p.x
        y[i] = p.y
        z[i] = p.z
    return x, y, z


class MeasureLidarTilt(Node):
    def __init__(self):
        super().__init__("measure_lidar_tilt")
        self.declare_parameter("duration", 10.0)
        self.declare_parameter("topic", "/livox/lidar")
        # URDF-claimed lidar optical-center height, for the sanity check:
        # mecanum = base_link_to_plate_top 0.29210 + livox z 0.03661.
        self.declare_parameter("expected_height", 0.32871)
        self.duration = self.get_parameter("duration").value
        self.expected_height = self.get_parameter("expected_height").value
        topic = self.get_parameter("topic").value

        self.xs, self.ys, self.zs = [], [], []
        self.scan_count = 0
        self.start = time.monotonic()

        self.create_subscription(PointCloud2, topic, self._pc2_cb, 10)
        if HAS_CUSTOM_MSG:
            self.create_subscription(CustomMsg, topic, self._custom_cb, 10)
        self.get_logger().info(
            f"Collecting {topic} for {self.duration:.0f}s — keep the robot still..."
        )

    def _pc2_cb(self, msg):
        self._accumulate(*read_points_from_pc2(msg))

    def _custom_cb(self, msg):
        self._accumulate(*read_points_from_custom_msg(msg))

    def _accumulate(self, x, y, z):
        if x.size:
            self.xs.append(x)
            self.ys.append(y)
            self.zs.append(z)
            self.scan_count += 1

    def done(self):
        return time.monotonic() - self.start >= self.duration

    # ---------- analysis ----------

    def fit_floor(self):
        if not self.xs:
            print("\nERROR: no points received. Is the Mid-360 driver running "
                  "and publishing on the topic? (param: topic)")
            return False

        x = np.concatenate(self.xs)
        y = np.concatenate(self.ys)
        z = np.concatenate(self.zs)
        r_xy = np.hypot(x, y)

        # Candidates: below the sensor, past the blind ring. Range cap is
        # generous (10 m) because from a ~0.33 m mount the Mid-360's -7.2°
        # FOV edge only reaches the floor beyond ~2.6 m — capping at 6 m
        # nearly excluded the floor on the rover (lesson from 2026-06-10:
        # the fit returned a furniture plane 0.081 m below the sensor).
        cand = (z < -0.05) & (r_xy > 0.4) & (r_xy < 10.0)
        if np.count_nonzero(cand) < 500:
            print(f"\nERROR: only {np.count_nonzero(cand)} candidate floor "
                  f"points (need ≥500). Is the lidar too low, or the floor "
                  f"out of view?")
            return False

        # RANSAC plane hypotheses, then pick the LOWEST well-supported one.
        # Two selection rules were tried and rejected:
        #   - densest z-bin: a tilted floor spreads across bins, loses to
        #     compact distractors (chair seats).
        #   - most inliers: from a ~0.33 m mount the floor is only visible
        #     past ~2.6 m at grazing angle, so it's SPARSE — a couch seat
        #     or bed top beats it on count (this returned "floor 0.081 m
        #     below sensor" on the rover, refuted by tape measure).
        # The floor's defining property is that it is the lowest horizontal
        # plane in view, so: among hypotheses with real support (≥400 pts
        # and ≥20% of the best count), take the lowest.
        cx, cy, cz = x[cand], y[cand], z[cand]
        if cx.size > 30000:
            idx = np.random.default_rng(0).choice(cx.size, 30000, replace=False)
            cx, cy, cz = cx[idx], cy[idx], cz[idx]
        pts3 = np.column_stack([cx, cy, cz])

        rng = np.random.default_rng(1)
        hypotheses = []   # (z_at_origin, count, inlier_mask)
        best_count = 0
        for _ in range(500):
            i, j, k = rng.choice(pts3.shape[0], 3, replace=False)
            v1, v2 = pts3[j] - pts3[i], pts3[k] - pts3[i]
            nrm = np.cross(v1, v2)
            norm_len = np.linalg.norm(nrm)
            if norm_len < 1e-9:
                continue
            nrm /= norm_len
            if abs(nrm[2]) < 0.7:
                continue  # near-vertical plane — wall, not floor
            d = nrm @ pts3[i]
            inliers = np.abs(pts3 @ nrm - d) < 0.015
            count = int(np.count_nonzero(inliers))
            if count < 400:
                continue
            z_at_origin = d / nrm[2]
            hypotheses.append((z_at_origin, count, inliers))
            best_count = max(best_count, count)

        viable = [h for h in hypotheses if h[1] >= max(400, 0.2 * best_count)]
        if not viable:
            print(f"\nERROR: RANSAC found no horizontal plane with enough "
                  f"support. Rerun on flat, open floor with a few meters "
                  f"of clearance.")
            return False
        z_floor, support, best_inliers = min(viable, key=lambda h: h[0])

        higher = [h for h in viable if h[0] > z_floor + 0.10]
        if higher:
            print(f"  note: {len(higher)} higher horizontal plane(s) seen "
                  f"(e.g. {max(h[0] for h in higher):+.3f} m vs floor "
                  f"{z_floor:+.3f} m) — furniture, correctly ignored.")

        fx, fy, fz = cx[best_inliers], cy[best_inliers], cz[best_inliers]

        # Refine: least-squares plane z = a·x + b·y + c with outlier
        # rejection (>2 cm residual). 3 rounds is plenty on a hard floor.
        for _ in range(3):
            A = np.column_stack([fx, fy, np.ones_like(fx)])
            coeffs, *_ = np.linalg.lstsq(A, fz, rcond=None)
            a, b, c = coeffs
            resid = fz - (a * fx + b * fy + c)
            keep = np.abs(resid) < 0.02
            if np.count_nonzero(keep) < 200:
                break
            fx, fy, fz = fx[keep], fy[keep], fz[keep]

        rms = float(np.sqrt(np.mean((fz - (a * fx + b * fy + c)) ** 2)))

        # Upward floor normal in the LIDAR frame.
        n = np.array([-a, -b, 1.0])
        n /= np.linalg.norm(n)
        nx, ny, nz = n

        # URDF rpy (extrinsic XYZ) of the lidar in a floor-level frame,
        # derived from R·n = ez with R = Ry(pitch)·Rx(roll), yaw
        # unobservable from a plane:
        roll = math.atan2(ny, nz)
        pitch = -math.asin(max(-1.0, min(1.0, nx)))
        height = -c  # plane z at origin → sensor sits -c above the floor

        tilt = math.degrees(math.acos(max(-1.0, min(1.0, nz))))

        print("\n" + "=" * 64)
        print("Mid-360 floor-plane fit")
        print("=" * 64)
        print(f"  scans integrated     : {self.scan_count}")
        print(f"  floor points used    : {fx.size}")
        print(f"  fit RMS residual     : {rms * 1000:.1f} mm "
              f"({'good' if rms < 0.015 else 'NOISY — rerun on flat open floor'})")
        dh = height - self.expected_height
        print(f"  lidar height (floor) : {height:.3f} m "
              f"(URDF says {self.expected_height:.3f} — "
              f"{'consistent' if abs(dh) < 0.03 else f'OFF BY {dh:+.3f} m — fix the URDF stack-up or this fit grabbed furniture'})")
        print(f"  roll                 : {math.degrees(roll):+.2f}°")
        print(f"  pitch                : {math.degrees(pitch):+.2f}°")
        print(f"  total tilt           : {tilt:.2f}°")
        print()
        if tilt < 0.5:
            print("  Mount is level within measurement noise — the floor-leak")
            print("  hypothesis is NOT confirmed; look at scan_z_min / clutter")
            print("  instead. No URDF change needed.")
        else:
            leak_r = SCAN_Z_MIN_DEFAULT / math.sin(math.radians(tilt))
            print(f"  Floor returns leak into the scan band (z ≥ "
                  f"{SCAN_Z_MIN_DEFAULT} m) beyond ≈ {leak_r:.1f} m range.")
            print()
            print("  Paste into urdf/sensors_common.urdf.xacro "
                  "(CHECK THE WIKI for mount specs first):")
            print(f'    <xacro:property name="livox_rpy" '
                  f'value="{roll:.6f} {pitch:.6f} 0.0"/>')
            print()
            print("  Then ./build.sh and rerun — the phantom speckle ring in")
            print("  /map should disappear.")
        print("=" * 64)
        return True


def main(args=None):
    rclpy.init(args=args)
    node = MeasureLidarTilt()
    try:
        while rclpy.ok() and not node.done():
            rclpy.spin_once(node, timeout_sec=0.1)
        node.fit_floor()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
