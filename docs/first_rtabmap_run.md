# First RTABMap Run — Bench Fixture Runbook

Step-by-step for the first end-to-end SLAM run on the 2040 bench fixture.
Goal: walk a closed loop ~5–10 m through a furnished room, see the
trajectory snap when RTABMap detects the loop, and end with a clean
`/map` in `/.ros/rtabmap.db`.

Until this runs successfully end-to-end, **every change we've made
since the FAST-LIO2 drift-free run is theoretical.** First contact
finds the bugs.

## 0. Prereqs

- Jetson `gizmo` powered, on the LiDAR network (`192.168.1.100`).
- Mid-360 powered + reachable (`ping 192.168.1.202`).
- D435 plugged in via USB3 to the Jetson.
- Latest code on the Jetson:
  ```bash
  cd ~/slam_ws/src/slam_bringup && git pull && ./build.sh
  ```
- Run the room: at least one closed-loop path 5–10 m across, with
  furniture / door frames / objects (NOT empty walls). RTABMap visual
  loop closure needs texture.
- Fixture on the **floor** (not on the 38" table) so the Mid-360 has
  ground returns within ~2 m. See [`test_fixture.md`](test_fixture.md)
  for why.

## 1. Walk the pre-mapping checklist

Open [`test_fixture.md`](test_fixture.md) → "Pre-RTABMap mapping
checklist". Verify each item. The two that have actually bitten people
in our setup:

- **Doubled D435 in TF**: `ros2 topic echo /tf_static --once | grep -A2 'child_frame_id: "d435_front_link"'` — must show ONE match.
- **IR emitter on**: `./start_rtabmap.sh` preflight does this automatically. Honor the warning if it fires.

## 2. Launch

One terminal, one command:

```bash
cd ~/slam_ws/src/slam_bringup
./start_slam.sh delete_db_on_start:=true rviz:=true
```

This composes:
- `perception.launch.py` (URDF + sensors in slam mode + Mid-360 in CustomMsg mode)
- `body → base_link` static TF (PLATFORM_BRIDGES['bench_fixture'])
- `fast_lio.launch.py` (LiDAR-inertial odometry)
- `rtabmap.launch.py` (graph SLAM + visual loop closure)
- rviz2 with `rviz/perception.rviz` (RobotModel + TF + sensor topics)

`delete_db_on_start:=true` wipes any previous `~/.ros/rtabmap.db` so
this is a clean first map. Drop that arg on subsequent runs to
incrementally extend the map.

**Watch the launch log for** (in this order, ~5–15 s total):
1. `livox_ros_driver2_node` reporting "Lidar Mid360 connected"
2. `realsense2_camera` printing the depth/color profiles
3. `fastlio_mapping` printing `/Odometry` rate
4. `rtabmap` printing "RTAB-Map: rate=1.000000 Hz" and "Initialized"

If any line is missing for >30 s after launch, Ctrl-C and check the
[Common failures](#common-failures) section below.

## 3. Settle (first 30 seconds — DO NOT MOVE THE RIG)

FAST-LIO2's ESKF needs ~30 s of stationary settle time to learn
accel/gyro biases. RTABMap needs camera_info + first depth frame to
initialize the graph. Don't move the rig until:

- `ros2 topic hz /Odometry` shows ~10 Hz steady (run in a second terminal)
- `ros2 topic hz /rtabmap/info` shows ~1 Hz steady
- RViz fixed-frame `base_link` shows the URDF triads + the live LiDAR
  cloud + first D435 pointcloud projected forward and slightly down

## 4. Verify the topics

In a second terminal:

```bash
# Odometry alive
ros2 topic hz /Odometry              # ~10 Hz
ros2 topic hz /cloud_registered_body # ~10 Hz

# Camera alive
ros2 topic hz /d435_front/camera/color/image_raw                       # ~30 Hz
ros2 topic hz /d435_front/camera/aligned_depth_to_color/image_raw      # ~30 Hz

# RTABMap alive
ros2 topic hz /rtabmap/info          # ~1 Hz
ros2 topic info /map                 # Publisher count: 1

# TF tree is single-rooted (no doubled D435)
ros2 run tf2_tools view_frames       # opens frames.pdf — verify single chain
```

If any rate is significantly lower than expected, jump to
[Common failures](#common-failures).

## 5. Walk the room

Carry the rig — overhead pole / cart / shoulder mount, NOT chest-height
handheld (your body occludes 60–90° of Mid-360 azimuth and biases yaw,
per the FAST-LIO2 motion-test note).

Walk a closed loop:
1. Start at point A, stationary, ~30 s.
2. Walk slowly (≤0.5 m/s) along path A → B → C → D → A.
3. At A again, stop and watch the trajectory.

The closing leg of the loop is when RTABMap should detect the closure
and snap the graph.

## 6. Verify loop closure

Three signals — at least one should fire when you return near A:

```bash
# 1. /rtabmap/info — loopClosureId goes non-zero
ros2 topic echo /rtabmap/info --field loop_closure_id

# 2. Visible "trajectory snap" in RViz: /Odometry path jumps as
#    RTABMap re-optimizes the graph

# 3. Console log from the rtabmap node — look for
#    "Loop closure detected with N inliers, residual X m"
```

Loop closure WILL NOT fire if:
- The room is feature-poor (long blank walls, dark areas)
- You're moving too fast (>0.5 m/s = motion blur on RGB)
- `Vis/MinInliers` is too strict (default 15; lower to 10 for textureless rooms)
- You haven't actually closed the loop (RTABMap needs to be within
  ~2–3 m of a previous keyframe location, not necessarily the start)

## 7. Save the map

RTABMap auto-saves to `~/.ros/rtabmap.db` continuously while running.
Ctrl-C the launch — RTABMap flushes the DB on clean shutdown.

Verify:

```bash
ls -lh ~/.ros/rtabmap.db
# Should be 50 MB – 500 MB depending on map size + run length
```

Export the 2D occupancy grid to a PNG for visual review:

```bash
rtabmap-databaseViewer ~/.ros/rtabmap.db
# GUI tool — Tools → Export → 2D map → PNG
```

Or export the 3D point cloud to PLY:

```bash
rtabmap-export --cloud ~/.ros/rtabmap.db
# → produces cloud.ply in cwd
```

## What success looks like

- `/map` shows recognizable wall / door / floor structure on the room scale
- 3D point cloud (from RViz `/cloud_map` or PLY export) has flat floor and vertical walls (no skew)
- Loop closure fired at least once during the closing leg
- Trajectory length ≈ actual walked distance (within 5%)

## What probably-good-enough-but-not-perfect looks like

- Floor tilts ≤2° in the 3D map → re-run `scripts/measure_d435_pitch.py`
  to refine the URDF pitch
- One or two trajectory jumps mid-walk that aren't at obvious loop points
  → `Vis/MinInliers` may be too low (false positives); try 18–20
- Map has Swiss-cheese holes on white walls → IR emitter is off, check
  `ros2 param get /d435_front/camera depth_module.emitter_enabled`

## Common failures

### Empty `/map` (OccupancyGrid all -1)

**Cause**: `Grid/NormalsSegmentation: true` (Livox unorganized clouds
break the default normals seg).
**Check**: `ros2 param get /rtabmap Grid/NormalsSegmentation` should
return `false`.
**Fix**: already set in `launch/rtabmap.launch.py`, but verify the
param made it through.

### `/Odometry` publishes but never updates pose (rig moves, pose stays at origin)

**Cause**: Mid-360 driver is in PointCloud2 mode (`xfer_format=0`)
instead of CustomMsg (`xfer_format=1`).
**Check**: `ros2 topic info /livox/lidar` — type should be
`livox_ros_driver2/msg/CustomMsg`, NOT `sensor_msgs/msg/PointCloud2`.
**Fix**: `slam.launch.py` forces `lidar_xfer_format:=1`. If somehow
overridden, restart with `./start_slam.sh` (no extra args).

### RTABMap waits forever for a topic

**Cause**: One of the 5 required input topics has no publisher.
**Check**: `start_rtabmap.sh` preflight catches this — read its error.
**Fix**: usually means sensors didn't come up or D435 is in raw mode
(no aligned depth). `slam.launch.py` forces `slam_mode:=true`.

### "approx_sync_max_interval too tight" warnings spammed

**Cause**: Jetson is overloaded; one of the camera streams is dropping
behind FAST-LIO2 odom by >50 ms.
**Check**: `htop` — CPU and load.
**Fix**: drop D435 to 640×480 @ 15 Hz (edit `launch/d435.launch.py`)
or bump `approx_sync_max_interval:=0.1` in `start_slam.sh`.

### TF parent conflict / doubled D435

**Cause**: realsense node publishing `d435_front_link` as a TF root,
URDF publishing it as child of `sensor_plate`.
**Check**: `ros2 topic echo /tf_static --once | grep -A2 'child_frame_id: "d435_front_link"'` — should show ONE match.
**Fix**: already wired via `base_frame_id: 'link'` in
`launch/d435.launch.py`. If you see this fail, the realsense version
on the Jetson may have changed parameter semantics — see
`docs/test_fixture.md` "Why are there two D435s in RViz?" for fallback.

### Map looks correct but Nav2 won't plan

**Cause**: usually a 10–20 cm Z error in `PLATFORM_BRIDGES` (body→base_link),
which pushes floor cells out of Nav2's costmap obstacle band.
**Fix**: re-derive from the URDF stack-up:
`body_Z_above_base_link = base_link_to_plate_top + livox_frame_Z = 0.207670 + 0.036610 = 0.244280 m`.
Update `PLATFORM_BRIDGES['bench_fixture']` Z accordingly.

## What to record after the run

Write down (in `obsidian-vault-rico/rico/Robotics/SLAM/First RTABMap Run - Bench Fixture Runbook.md`):

- Date / time / approximate run length
- Room used + what features were visible (texture quality)
- Did loop closure fire? How many times?
- Trajectory length vs actual walked distance
- Any warnings during the run
- Any anomalies in the resulting `/map` or 3D cloud
- One-line conclusion: ready for next phase, or needs more tuning

That note is the gravity well for "what we learned from first-RTABMap"
that will inform the Go2 URDF + Phase 1.10 dual-camera plans.
