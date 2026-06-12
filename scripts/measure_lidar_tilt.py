#!/usr/bin/env python3
"""
Measure the Mid-360's mounting tilt (roll/pitch) by fitting the floor plane —
with a built-in repeatability check.

Why this exists: urdf/sensors_common.urdf.xacro hard-codes
`livox_rpy = 0 0 0` (perfectly level). A real mount tilt leaks distant floor
returns up into the pointcloud_to_laserscan obstacle band and poisons the
costmap. This script measures the actual tilt from the raw cloud.

Why the repeatability check exists: single fits LIE. Field history
(2026-06-11/12): three consecutive single-shot fits on a mount that a
digital angle gauge proved level (<1°) returned 7.0°, 7.1°, and 5.6° of
"tilt" — in three different directions. A plane fit constrained by sparse
grazing-incidence floor returns can be scene-limited and noise-dominated
while still showing a great RMS. So the capture is split into N=10
time-slices, each fit independently, and the spread is reported:

  - spread (std) ≤ 0.5° in roll AND pitch  → result is trustworthy;
    the paste-ready livox_rpy line is printed.
  - spread > 0.5°  → UNSTABLE verdict, NO paste line. Believe your angle
    gauge, not this script. Improve the scene (more open floor, longer
    duration) or dump diagnostics (debug_dump) for offline analysis.

Floor selection (each sub-fit): RANSAC plane hypotheses; keep near-level
(≤8°) planes whose height matches the tape-measured mount height
(expected_height ± 0.05 m); take the best-supported one; refine with
iterative least squares; re-validate. Selection rules previously tried and
field-rejected: densest z-bin (tilted floor spreads across bins), most
inliers (couch out-votes sparse grazing floor), lowest plane (mirror
reflections form ghost planes below the real floor).

Run with the robot STATIONARY on flat, open floor (a few meters of clear
floor in view; from a ~0.33 m mount the floor is only visible beyond
~2.6 m). Only the Mid-360 driver needs to be running:

  ./start_mid360.sh
  python3 scripts/measure_lidar_tilt.py --ros-args \
      -p duration:=10.0 -p topic:=/livox/lidar \
      -p expected_height:=0.32871 \
      -p debug_dump:=/tmp/tilt_debug.npz     # optional diagnostics
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

# A result is "stable" when the std-dev across sub-samples is below this
# for BOTH roll and pitch (degrees).
STABILITY_STD_DEG = 0.5

N_SUBSAMPLES = 10


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
        # URDF-claimed lidar optical-center height, for floor selection and
        # the sanity check: mecanum = plate_top 0.29210 + livox z 0.03661.
        self.declare_parameter("expected_height", 0.32871)
        # Path for a diagnostic .npz dump (candidates + selected inliers +
        # plane, from the full-capture fit). Set when results disagree with
        # a physical gauge so the fit can be inspected offline. Empty = off.
        self.declare_parameter("debug_dump", "")
        self.duration = self.get_parameter("duration").value
        self.expected_height = self.get_parameter("expected_height").value
        self.debug_dump = self.get_parameter("debug_dump").value.strip()
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

    def _fit_subset(self, x, y, z):
        """Full floor-fit pipeline on one subset of scans.

        Returns (result_dict, None) on success or (None, reason_str).
        """
        r_xy = np.hypot(x, y)
        # Below the sensor, past the blind ring, out to 10 m (from a
        # ~0.33 m mount the -7.2° FOV edge only reaches the floor beyond
        # ~2.6 m, so a tight range cap would exclude the floor entirely).
        cand = (z < -0.05) & (r_xy > 0.4) & (r_xy < 10.0)
        n_cand = int(np.count_nonzero(cand))
        if n_cand < 500:
            return None, f"only {n_cand} candidate points"
        cx, cy, cz = x[cand], y[cand], z[cand]
        if cx.size > 30000:
            idx = np.random.default_rng(0).choice(cx.size, 30000, replace=False)
            cx, cy, cz = cx[idx], cy[idx], cz[idx]
        pts3 = np.column_stack([cx, cy, cz])

        rng = np.random.default_rng(1)
        hypotheses = []   # (z_at_origin, count, inlier_mask, |nz|)
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
            if count < 300:
                continue
            z_at_origin = d / nrm[2]
            hypotheses.append((z_at_origin, count, inliers, abs(nrm[2])))

        # Floor = best-supported near-level plane at the tape-measured
        # height. Window is deliberately tight (±0.05): at ±0.15 a mixed
        # plane slicing couch+floor obliquely won at the window edge.
        WIN = 0.05
        level_min = math.cos(math.radians(8.0))
        floor_hyps = [h for h in hypotheses
                      if abs(h[0] + self.expected_height) <= WIN
                      and h[3] >= level_min]
        rejected_zs = sorted(set(
            round(h[0], 1) for h in hypotheses
            if abs(h[0] + self.expected_height) > WIN))
        if not floor_hyps:
            return None, "no near-level plane in the expected-height window"
        _, support, best_inliers, _ = max(floor_hyps, key=lambda h: h[1])

        fx, fy, fz = cx[best_inliers], cy[best_inliers], cz[best_inliers]

        # Refine: least-squares plane z = a·x + b·y + c with outlier
        # rejection (>2 cm residual).
        a = b = c = 0.0
        for _ in range(3):
            A = np.column_stack([fx, fy, np.ones_like(fx)])
            coeffs, *_ = np.linalg.lstsq(A, fz, rcond=None)
            a, b, c = coeffs
            resid = fz - (a * fx + b * fy + c)
            keep = np.abs(resid) < 0.02
            if np.count_nonzero(keep) < 200:
                break
            fx, fy, fz = fx[keep], fy[keep], fz[keep]

        # Re-validate: a refined fit that drifted out of the window or
        # past 8° came from a mixed seed — report failure, not garbage.
        if abs(-c - self.expected_height) > WIN or \
                math.hypot(a, b) > math.tan(math.radians(8.0)):
            return None, "refined fit drifted out of the expected-floor window"

        rms = float(np.sqrt(np.mean((fz - (a * fx + b * fy + c)) ** 2)))

        # Upward floor normal in the LIDAR frame → URDF rpy (extrinsic
        # XYZ, R = Ry(pitch)·Rx(roll), yaw unobservable from a plane).
        n = np.array([-a, -b, 1.0])
        n /= np.linalg.norm(n)
        nx, ny, nz = n
        roll = math.atan2(ny, nz)
        pitch = -math.asin(max(-1.0, min(1.0, nx)))
        tilt = math.degrees(math.acos(max(-1.0, min(1.0, nz))))

        # Coverage: a fit supported by a narrow azimuth sector or short
        # range span is noise-dominated in the cross direction.
        in_az = np.degrees(np.arctan2(fy, fx))
        az_sectors = int(np.unique((in_az // 15).astype(int)).size)  # of 24
        in_r = np.hypot(fx, fy)
        r_span = float(in_r.max() - in_r.min()) if in_r.size else 0.0

        return {
            "height": -c, "roll": roll, "pitch": pitch, "tilt_deg": tilt,
            "rms": rms, "support": int(fx.size),
            "az_sectors": az_sectors, "r_span": r_span,
            "rejected_zs": rejected_zs,
            "pts3": pts3, "fx": fx, "fy": fy, "fz": fz,
            "abc": np.array([a, b, c]),
        }, None

    def fit_floor(self):
        if not self.xs:
            print("\nERROR: no points received. Is the Mid-360 driver running "
                  "and publishing on the topic? (param: topic)")
            return False

        print("\n" + "=" * 72)
        print("Mid-360 floor-plane fit — repeatability across "
              f"{N_SUBSAMPLES} sub-samples")
        print("=" * 72)

        # Split the capture into N contiguous time-slices and fit each
        # independently. Agreement across slices is the trust signal a
        # single fit cannot provide.
        n_chunks = min(N_SUBSAMPLES, len(self.xs))
        chunk_ids = np.array_split(np.arange(len(self.xs)), n_chunks)
        print(f"   #  height   roll     pitch    tilt    pts   az-cov  r-span")
        sub = []
        for i, ids in enumerate(chunk_ids, 1):
            x = np.concatenate([self.xs[j] for j in ids])
            y = np.concatenate([self.ys[j] for j in ids])
            z = np.concatenate([self.zs[j] for j in ids])
            res, why = self._fit_subset(x, y, z)
            if res is None:
                print(f"  {i:>2}  — failed: {why}")
                continue
            sub.append(res)
            print(f"  {i:>2}  {res['height']:6.3f}  "
                  f"{math.degrees(res['roll']):+7.2f}  "
                  f"{math.degrees(res['pitch']):+7.2f}  "
                  f"{res['tilt_deg']:5.2f}  {res['support']:>5}  "
                  f"{res['az_sectors']:>3}/24  {res['r_span']:5.1f}m")

        # Full-capture fit (primary numbers + debug dump source).
        full, full_why = self._fit_subset(
            np.concatenate(self.xs), np.concatenate(self.ys),
            np.concatenate(self.zs))

        if len(sub) < max(3, n_chunks // 2) or full is None:
            print("-" * 72)
            print(f"VERDICT: NOT MEASURABLE — too few sub-fits succeeded "
                  f"({len(sub)}/{n_chunks}"
                  + (f"; full fit failed: {full_why}" if full is None else "")
                  + ").")
            print("  Move to a few meters of open, flat floor and/or "
                  "increase -p duration:=30.0.")
            return False

        rolls = np.degrees([r["roll"] for r in sub])
        pitches = np.degrees([r["pitch"] for r in sub])
        heights = np.array([r["height"] for r in sub])
        std_roll, std_pitch = float(np.std(rolls)), float(np.std(pitches))
        stable = std_roll <= STABILITY_STD_DEG and std_pitch <= STABILITY_STD_DEG

        print("-" * 72)
        print(f"  spread over {len(sub)} sub-samples: "
              f"roll {np.mean(rolls):+.2f} ± {std_roll:.2f}°,  "
              f"pitch {np.mean(pitches):+.2f} ± {std_pitch:.2f}°,  "
              f"height {np.mean(heights):.3f} ± {np.std(heights):.3f} m")
        if full["rejected_zs"]:
            zs = ", ".join(f"{v:+.1f}" for v in full["rejected_zs"])
            print(f"  ignored plane(s) at z≈[{zs}] m — furniture above, or "
                  f"mirror ghosts below the floor.")

        if self.debug_dump:
            try:
                np.savez_compressed(
                    self.debug_dump,
                    candidates=full["pts3"],
                    inlier_x=full["fx"], inlier_y=full["fy"],
                    inlier_z=full["fz"], plane_abc=full["abc"],
                    expected_height=self.expected_height,
                )
                print(f"  debug dump written: {self.debug_dump}")
            except OSError as exc:
                print(f"  debug dump FAILED ({exc})")

        print("-" * 72)
        if not stable:
            print(f"VERDICT: UNSTABLE — sub-sample spread (roll ±{std_roll:.2f}°, "
                  f"pitch ±{std_pitch:.2f}°) exceeds ±{STABILITY_STD_DEG}°.")
            print("  DO NOT apply any rpy from this run. The fit is "
                  "scene-limited (sparse grazing floor), not measuring the "
                  "mount. A physical angle gauge on the plate is the "
                  "authority. To improve: more open floor in view, longer "
                  "duration, or send the debug_dump for offline analysis.")
            print("=" * 72)
            return False

        dh = full["height"] - self.expected_height
        height_ok = abs(dh) < 0.03
        print(f"VERDICT: STABLE (spread ±{max(std_roll, std_pitch):.2f}°)")
        print(f"  height : {full['height']:.3f} m (URDF says "
              f"{self.expected_height:.3f} — "
              + ("consistent" if height_ok else
                 f"OFF BY {dh:+.3f} m — fix the URDF stack-up") + ")")
        print(f"  roll   : {math.degrees(full['roll']):+.2f}°")
        print(f"  pitch  : {math.degrees(full['pitch']):+.2f}°")
        print(f"  tilt   : {full['tilt_deg']:.2f}°   (fit RMS "
              f"{full['rms'] * 1000:.1f} mm, {full['support']} pts)")
        print()
        if full["tilt_deg"] < 0.5:
            print("  Mount is level within measurement noise. No URDF "
                  "change needed.")
        elif not height_ok:
            print("  Height check failed — resolve the stack-up first; "
                  "do not paste the rpy below until height reads "
                  "consistent.")
            print(f'  (would be: <xacro:property name="livox_rpy" '
                  f'value="{full["roll"]:.6f} {full["pitch"]:.6f} 0.0"/>)')
        else:
            leak_r = SCAN_Z_MIN_DEFAULT / math.sin(math.radians(full["tilt_deg"]))
            print(f"  Floor returns leak into the scan band (z ≥ "
                  f"{SCAN_Z_MIN_DEFAULT} m) beyond ≈ {leak_r:.1f} m range.")
            print()
            print("  Paste into urdf/sensors_common.urdf.xacro "
                  "(CHECK THE WIKI for mount specs first):")
            print(f'    <xacro:property name="livox_rpy" '
                  f'value="{full["roll"]:.6f} {full["pitch"]:.6f} 0.0"/>')
            print()
            print("  Then ./build.sh and rerun.")
        print("=" * 72)
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
