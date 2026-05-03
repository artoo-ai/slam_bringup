# Troubleshooting

Every issue we've actually hit on this stack, with diagnosis commands and the exact fix. Index by symptom — search this file before fresh debugging. When a future session asks for help, point at this doc and the issue + commands are pre-loaded.

Entries are sorted roughly by severity / how often we've hit them.

---

## Index

- [No point cloud showing in RViz / Foxglove](#no-point-cloud-showing-in-rviz--foxglove)
- [FAST-LIO drift while stationary (z = -4 m, pitch 17°)](#fast-lio-drift-while-stationary-z---4-m-pitch-17)
- ["lidar loop back, clear buffer" flood from FAST-LIO](#lidar-loop-back-clear-buffer-flood-from-fast-lio)
- [Nav2 goal aborts with "Distance remaining: 0.00 m"](#nav2-goal-aborts-with-distance-remaining-000-m)
- [Nav2 "Behavior tree threw exception: Empty Tree"](#nav2-behavior-tree-threw-exception-empty-tree)
- [Costmap is solid magenta — no traversable cells](#costmap-is-solid-magenta--no-traversable-cells)
- [RTABMap "Rejecting all added loop closures" flood](#rtabmap-rejecting-all-added-loop-closures-flood)
- [RTABMap "Ignoring local loop closure ... transform too large"](#rtabmap-ignoring-local-loop-closure--transform-too-large)
- [RTABMap blank `/map` (occupancy grid all unknown)](#rtabmap-blank-map-occupancy-grid-all-unknown)
- [Two RViz windows when launching with rviz:=true](#two-rviz-windows-when-launching-with-rvizfalse)
- [Two publishers on a topic / orphan processes survive kill](#two-publishers-on-a-topic--orphan-processes-survive-kill)
- [Mid-360 IMU unit bug (gravity reads 0.99 instead of 9.81)](#mid-360-imu-unit-bug-gravity-reads-099-instead-of-981)
- [Jetson CPU saturated — controller missing rate, RTABMap delay > 1 s](#jetson-cpu-saturated--controller-missing-rate-rtabmap-delay--1-s)
- [`/cmd_vel` publishes but rover doesn't move](#cmd_vel-publishes-but-rover-doesnt-move)
- [Yahboom bridge: Rosmaster_Lib not installed](#yahboom-bridge-rosmaster_lib-not-installed)
- [Yahboom bridge: serial port permission denied](#yahboom-bridge-serial-port-permission-denied)
- [Yahboom bench-test: a wheel spins backward / strafe inverted](#yahboom-bench-test-a-wheel-spins-backward--strafe-inverted)
- [Mid-360 driver fails to bind UDP sockets](#mid-360-driver-fails-to-bind-udp-sockets)
- [D435 "Frames didn't arrive within 5 seconds" / "Device or resource busy"](#d435-frames-didnt-arrive-within-5-seconds--device-or-resource-busy)
- [WitMotion: "Device or resource busy" / no /imu/data](#witmotion-device-or-resource-busy--no-imudata)
- [Tilted point cloud / camera_init not level with floor](#tilted-point-cloud--camera_init-not-level-with-floor)
- [URDF / platform-specific issues](#urdf--platform-specific-issues)
- [Sitting stationary issues](#sitting-stationary-issues)
- [Moving / driving issues](#moving--driving-issues)
- [`./build.sh` errors: package not found](#buildsh-errors-package-not-found)
- [CloudCompare shows just a "line" / sparse cloud after `export_map.sh`](#cloudcompare-shows-just-a-line--sparse-cloud-after-export_mapsh)
- [`rtabmap-export` crashes with `--cam_projection` or `--mesh`](#rtabmap-export-crashes-with---cam_projection-or---mesh)

---

## No point cloud showing in RViz / Foxglove

**Symptom:** RViz/Foxglove 3D panel is empty, or shows the URDF / TF axes but no LiDAR data.

**Diagnose:**
```bash
ros2 topic list | grep -E 'livox|cloud_registered|cloud_viz'
ros2 topic info /livox/lidar          # Publisher count must be ≥ 1
ros2 topic info /cloud_registered     # Publisher count must be ≥ 1 (FAST-LIO)
ros2 run tf2_tools view_frames        # generates frames.pdf — confirms TF chain
```

**Common causes & fixes:**

1. **Wrong fixed frame.** `/livox/lidar` is in `livox_frame`; `/cloud_registered` in `camera_init`; `/cloud_registered_body` in `body`; saved RTABMap `/map` in `map`. RViz "Global Status: Error — Frame [X] does not exist" means TF doesn't reach there.
2. **`/livox/lidar` is `livox_ros_driver2/CustomMsg`** when `xfer_format=1` — Foxglove can't render CustomMsg natively. Use `/cloud_registered` (world) or `/cloud_registered_body` (body) instead, OR launch with `xfer_format:=0` (PointCloud2) for sensor-only viewing.
3. **Subscribed to `/cloud_registered` but seeing only "current room" that moves with sensor** — that's a single sweep re-rendered each frame. Use `/Laser_map` (set `publish.map_en: true` in `fast_lio_mid360.yaml`) OR `/cloud_viz_clipped` for accumulation, OR set Foxglove's PointCloud2 **Decay time** to a large value (e.g. `1e9`) for client-side accumulation.
4. **Missing `body→base_link` static TF.** Per-platform; see [URDF / platform-specific issues](#urdf--platform-specific-issues).
5. **Foxglove still subscribed but no publisher** — `Subscription count: 1, Publisher count: 0` is harmless. Once a publisher returns, the existing subscriber attaches automatically.

**Related scripts:** [start_fast_lio.md](scripts/start_fast_lio.md), [start_viz_clip.md](scripts/start_viz_clip.md), [start_perception.md](scripts/start_perception.md).

---

## FAST-LIO drift while stationary (z = -4 m, pitch 17°)

**Symptom:** `tf2_echo map base_link` shows a moving translation/rotation while the rover sits perfectly still on a flat floor. Z drifts negative; pitch oscillates around 15-20°.

**Diagnose:**
```bash
ros2 run tf2_ros tf2_echo map base_link    # watch for several seconds
ros2 run tf2_ros tf2_echo map camera_init  # if non-identity, RTABMap IS correcting; if identity, RTABMap hasn't relocalized
rtabmap-info ~/.ros/rtabmap.db | grep "Optimized graph"
# Look for: x=<huge>->-N, e.g. x=942->-3 means 942 m of unoptimized drift on a 52 m trajectory
```

**Cause:** FAST-LIO is integrating IMU bias as 6 DoF drift. RTABMap was the only thing pulling drift back into shape during mapping (via 15+ loop closures). In **localization mode** (`Mem/IncrementalMemory: false`), no new keyframes are added, so live drift surfaces directly in TF.

**Fix:** Use `force_3dof:=true` on wheeled rovers — RTABMap clamps z/roll/pitch in the loop-closure optimizer.

```bash
./start_nav.sh force_3dof:=true                                       # navigation mode
./start_slam.sh force_3dof:=true delete_db_on_start:=true             # re-map clean
```

**Important:** `force_3dof` clamps **RTABMap's optimizer**, NOT FAST-LIO itself. FAST-LIO will still drift; RTABMap just refuses to bake the drift into the graph. The long-term fix for FAST-LIO drift would be wheel-odom EKF or a planar constraint plugin (not stock).

**Related:** [Sitting stationary issues](#sitting-stationary-issues), [start_nav.md](scripts/start_nav.md) Gotchas.

---

## "lidar loop back, clear buffer" flood from FAST-LIO

**Symptom:** Endless `[fastlio_mapping] lidar loop back, clear buffer` lines, no `/Odometry` published, every scan dropped.

**Cause:** FAST-LIO sees scan timestamps arriving out of order. Two root causes:

### Cause 1: Two publishers on `/livox/lidar` or `/livox/imu_ms2`

Most common. An orphan `livox_ros_driver2_node` or `imu_units_g_to_ms2` from a previous session double-publishes with slightly different timestamps; FAST-LIO's `last_timestamp_lidar` ratchets ahead and rejects every scan.

```bash
ros2 topic info /livox/lidar | grep "Publisher count"     # must be 1
ros2 topic info /livox/imu_ms2 | grep "Publisher count"   # must be 1
pgrep -af livox_ros_driver2_node                          # must show 1 process
pgrep -af imu_units_g_to_ms2                              # must show 1 process
top -bn1 | grep -E 'livox|imu_units|fastlio'              # check for orphans with hours of TIME+
```

**Fix:**
```bash
./kill_slam.sh        # now actually surfaces warnings (no more 2>/dev/null)
pkill -9 -f livox_ros_driver2_node
pkill -9 -f imu_units
pkill -9 -f fastlio
ros2 daemon stop
sleep 3
pgrep -af livox       # blank
pgrep -af imu_units   # blank

./start_slam.sh delete_db_on_start:=true
```

The recent `kill_helpers.sh` (commit `7272f6a`) auto-escalates SIGINT → SIGKILL → `sudo` SIGKILL with verification, so this should not recur. If it does, the script exits 1 with the surviving PID.

### Cause 2: System clock issues / Mid-360 PTP unsynced

Rare. If even after a clean kill the warnings persist, check that the LiDAR clock isn't producing timestamps from device boot (small numbers) instead of epoch (~1.78e9 in 2026).

**Related:** [Two publishers on a topic / orphan processes survive kill](#two-publishers-on-a-topic--orphan-processes-survive-kill), [start_mid360.md](scripts/start_mid360.md).

---

## Nav2 goal aborts with "Distance remaining: 0.00 m"

**Symptom:** RViz Nav2 panel shows `Feedback: aborted` (red) the moment you click 2D Goal Pose. `Recoveries: 0+`. No path appears.

**Diagnose in this order — branches differ:**

```bash
# 1. Check the BT log for the actual exit reason
ros2 topic echo /diagnostics --once 2>/dev/null
# Look at the bt_navigator log lines specifically:
#   "Behavior tree threw exception: Empty Tree"  → see Empty Tree section
#   "Failed to make progress"                     → see /cmd_vel section
#   "Compute path to pose failed"                 → planner can't find path

# 2. Check pose
ros2 run tf2_ros tf2_echo map base_link        # is z ~ 0? matches your physical position?
ros2 run tf2_ros tf2_echo map camera_init      # is RTABMap correcting? if identity, see drift section

# 3. Check map / costmap
ros2 topic echo --once /map | head -5
ros2 topic echo --once /global_costmap/costmap | head -5
```

**Branches:**

- **"Empty Tree"** → see [Nav2 "Behavior tree threw exception: Empty Tree"](#nav2-behavior-tree-threw-exception-empty-tree).
- **`map → base_link` is at origin (0,0,0) but you're not** → RTABMap hasn't relocalized. Use **2D Pose Estimate** in RViz to seed `/initialpose`, OR drive ~1 m forward/back to trigger a loop closure.
- **`map → base_link` says z = -4 m** → see [FAST-LIO drift while stationary](#fast-lio-drift-while-stationary-z---4-m-pitch-17).
- **Costmap is mostly magenta around your robot** → see [Costmap is solid magenta](#costmap-is-solid-magenta--no-traversable-cells).
- **`/cmd_vel` publishes but nothing happens** → see [`/cmd_vel` publishes but rover doesn't move](#cmd_vel-publishes-but-rover-doesnt-move).
- **Nav2 panel says "Localization: inactive"** — **EXPECTED**. That indicator monitors AMCL, which we don't run. Ignore.

**Related:** [start_nav.md](scripts/start_nav.md) Gotchas.

---

## Nav2 "Behavior tree threw exception: Empty Tree"

**Symptom:** Every goal aborts immediately with `[bt_navigator] [ERROR] [BehaviorTreeEngine]: Behavior tree threw exception: Empty Tree. Exiting with failure.`

**Cause:** `bt_navigator` was given an empty path string for its default BT XML. Setting `default_nav_to_pose_bt_xml: ""` does NOT mean "use default" — it means "use this nonexistent path". Fixed in commit `4b9d402`.

**Fix:** `git pull` + `./build.sh` (the config file is a `data_file`, needs install/share refresh).

The fix is to **omit** the `default_nav_to_pose_bt_xml` and `default_nav_through_poses_bt_xml` keys entirely from `config/nav2_params.yaml`. The package-default BT XML at `nav2_bt_navigator/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml` then takes over.

**Verify the fix landed:**
```bash
grep -A1 "default_nav_to_pose_bt_xml" ~/slam_ws/src/slam_bringup/config/nav2_params.yaml
# Should show only the comment, no `default_nav_to_pose_bt_xml: ""` line
```

---

## Costmap is solid magenta — no traversable cells

**Symptom:** RViz costmap displays show the entire saved map covered in cyan (inflated) and magenta (lethal) cells. No corridor for the planner to route through.

**Diagnose:**
```bash
# In RViz, enable Map displays for /global_costmap/costmap and /local_costmap/costmap.
# The robot icon should sit in mostly-clear (white/grey) space, with a thin
# inflation ring near walls. NOT a wall of magenta.

ros2 topic echo --once /scan | head -3   # check the laserscan ranges look sane
```

**Three compounding causes** (all addressed in commit `eee1f5d`, but worth knowing):

1. **`pointcloud_to_laserscan` slice too tall.** Default before the fix was `0.10–2.0 m` above `base_link` — the laserscan caught walls, shelves, ceiling fans, doorframes; everything got marked as obstacles. Fixed: `scan_max_height: 0.45` (rover collision band).

2. **Inflation too aggressive.** Old `inflation_radius: 0.55` + `robot_radius: 0.4` = 0.95 m forbidden ring around every obstacle. Indoors that paints rooms solid. Fixed: `inflation_radius: 0.30` + `robot_radius: 0.25`.

3. **`obstacle_layer` in BOTH global and local costmap.** With FAST-LIO drift, live `/scan` obstacles don't perfectly align with the saved `/map`'s `static_layer`. Two layers → conflicting cells → magenta soup. Fixed: global costmap uses **only** `static_layer + inflation_layer`; local costmap keeps `obstacle_layer + inflation_layer`.

**Per-platform tuning:** if your rover is taller than ~45 cm, raise `scan_max_height`:

```bash
./start_nav.sh scan_max_height:=0.8 force_3dof:=true   # Go2 standing
./start_nav.sh scan_max_height:=1.2                    # bench fixture on a table
```

If the rover is bigger than 25 cm radius, edit `config/nav2_params.yaml`:
```yaml
local_costmap.local_costmap.ros__parameters.robot_radius: 0.35    # bigger rover
global_costmap.global_costmap.ros__parameters.robot_radius: 0.35
```

---

## RTABMap "Rejecting all added loop closures" flood

**Symptom:** `[rtabmap] Rejecting all added loop closures (N, first is X <-> Y) ... maximum graph error ratio of 3.07 of std deviation`. Repeats every iteration.

**Cause:** Saved map's pre-optimized graph has heavy FAST-LIO drift baked in (e.g. 942 m unoptimized error on a 52 m trajectory). Any new legit loop closure, when applied, would shift other nodes by 3σ+, which RTABMap's optimizer treats as "obviously wrong, reject the whole batch."

**Diagnose:**
```bash
rtabmap-info ~/.ros/rtabmap.db | grep "Optimized graph"
# Big numbers like x=942->-3 = severe drift, baked in
```

**Fix (workaround — already applied):** `RGBD/OptimizeMaxError: 5.0` (was `3.0`) in `launch/rtabmap.launch.py`. Lets corrections within 5σ through.

**Fix (root cause):** Re-map with `force_3dof:=true` from scratch — drift never accumulates in the first place.

```bash
cp ~/.ros/rtabmap.db ~/maps/livingroom-original-$(date +%F).db   # save the old one
./start_slam.sh delete_db_on_start:=true force_3dof:=true
# Drive the same area; end where you started for a global closure.
```

---

## RTABMap "Ignoring local loop closure ... transform too large"

**Symptom:** `[rtabmap] Ignoring local loop closure with N because resulting transform is too large!? (1.06m > 1.00m)`.

**Cause:** Default `RGBD/LocalRadius: 1.0` m is too tight given FAST-LIO drift. Neighbouring keyframes look 1.06–1.13 m apart in odom space but are genuinely close in real space — legit local closures get tossed.

**Fix (already applied in commit `4b9d402`):** Bumped `RGBD/LocalRadius: 2.0` m in `launch/rtabmap.launch.py`. If you still see this with the latest code, drift is even worse — re-map with `force_3dof:=true`.

---

## RTABMap blank `/map` (occupancy grid all unknown)

**Symptom:** `/map` topic is published but the OccupancyGrid is all -1 (unknown). RViz Map display is solid grey.

**Cause:** `Grid/NormalsSegmentation: true` (default) expects organized clouds (row/column structure). Mid-360 emits unorganized clouds. Segmentation fails silently and the grid stays empty.

**Fix (already applied):** `Grid/NormalsSegmentation: false` in `launch/rtabmap.launch.py`. **THIS IS THE #1 GOTCHA** for using RTABMap with Livox LiDAR — if you ever copy this config to a new project, do not lose this line.

**Other prerequisites for `/map` to populate:**
- `Reg/Strategy: 1` (ICP) — visual-only registration won't build a grid for unstructured clouds.
- `subscribe_scan_cloud: true` — RTABMap subscribes `/cloud_registered_body`.
- `Grid/Sensor: 0` (laserscan mode).

---

## Two RViz windows when launching with rviz:=false

**Symptom:** `./start_slam.sh rviz:=true` opens TWO RViz windows — one with `perception.rviz` (URDF + sensors) and one with `fastlio.rviz` (FAST-LIO debug view).

**Cause:** Both `perception.launch.py` and the upstream `FAST_LIO_ROS2/mapping.launch.py` spawn their own `rviz2` when `rviz=true`. The `rviz` launch arg cascaded to both.

**Fix (already applied in commit `9be7389`):** `slam.launch.py` now pins `rviz=false` on the FAST-LIO include. `git pull` + (if needed) `./build.sh`.

**If you actually want the FAST-LIO debug RViz** (it has `/Odometry`, `/Path`, `/Laser_map` pre-configured):
```bash
./start_fast_lio.sh rviz:=true   # standalone, bypasses the slam.launch.py pin
```

---

## Two publishers on a topic / orphan processes survive kill

**Symptom:** `ros2 topic info <topic>` shows `Publisher count: 2` after running a kill script. Common offenders: `/livox/lidar`, `/livox/imu_ms2`, `/Odometry`, `/cmd_vel`. Causes a cascade of downstream failures.

**Diagnose:**
```bash
ros2 topic info /livox/lidar | grep -i count       # both counts
pgrep -af livox|fastlio|imu_units|rtabmap         # find the orphan PID
top -bn1 | grep -E 'livox|imu_units|fastlio|rtabmap'  # check TIME+ for hours-old orphans
```

**Cause:** Process held by `pkill -9` (D-state, sudo-required, or just slow to exit). The previous `2>/dev/null` in `kill_slam.sh` was hiding the warnings.

**Fix (commits `7272f6a`, `1b4057f`, `c7e4184`, `a9effa9`):** Every `kill_*.sh` now uses `kill_helpers.sh::nuke_processes`:

1. SIGINT, sleep 2.
2. SIGKILL × 3 retries.
3. `sudo -n` SIGKILL (non-interactive — uses cached creds if present).
4. Interactive `sudo` SIGKILL (Ctrl-C to skip).
5. `pgrep` verifies; exit 1 with diagnostic if still alive.

`kill_slam.sh` propagates the exit code, and the layered scripts no longer suppress stderr. So you'll **see** the `! X survived pkill -9 — escalating to sudo` line if the helper is doing work.

**Manual escalation if all else fails:**
```bash
sudo pkill -9 -f livox_ros_driver2_node
sudo pkill -9 -f imu_units
sudo pkill -9 -f fastlio_mapping
ros2 daemon stop
```

If a process persists past `sudo pkill -9` it's typically D-state (kernel I/O) — reboot.

---

## Mid-360 IMU unit bug (gravity reads 0.99 instead of 9.81)

**Symptom:** Live magnitude check shows `Magnitude: 0.9890 m/s²` while the rig sits stationary — should be ~9.81. FAST-LIO over-trusts the IMU prediction step and produces unstable odometry.

**Cause:** Livox driver 1.2.6 publishes `/livox/imu` with `linear_acceleration` in **units of g** (1.0 at rest), not m/s² as REP-145 specifies.

**Fix (already applied):** `slam_bringup/imu_units_node.py` republishes on `/livox/imu_ms2` (m/s²). Auto-detects unit on the first sample (g vs m/s² window). FAST-LIO's `imu_topic` in `config/fast_lio_mid360.yaml` points at `/livox/imu_ms2`.

**Verify with `scripts/measure_imu_tilt.py`:**
```bash
python3 ~/slam_ws/src/slam_bringup/scripts/measure_imu_tilt.py --ros-args \
    -p num_samples:=600 -p imu_topic:=/livox/imu_ms2
# Magnitude should be 9.79–9.83 m/s²
```

**Related issue:** if the bridge fails to start (`pip` install issue, etc.), `start_fast_lio.sh` has a preflight that catches missing entry points and tells you to `--clean` rebuild.

---

## Jetson CPU saturated — controller missing rate, RTABMap delay > 1 s

**Symptom:**
- `[controller_server] Control loop missed its desired rate of 20.0000Hz`
- `[bt_navigator] Behavior Tree tick rate 100.00 was exceeded!`
- `[rtabmap] Rate=1.00s, RTAB-Map=3.39s, delay=3.71s`
- `Message Filter dropping message: timestamp earlier than transform cache`

**Diagnose:**
```bash
top -bn1 | head -20         # look for >70% on rtabmap, fastlio_mapping, rviz2, foxglove_bridge
uptime                      # load average should be < N_cores
```

**Fixes, in order:**

1. **Move RViz off the Jetson.** Run RViz on your dev machine over the LAN (same `ROS_DOMAIN_ID` + `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`). Single biggest CPU win we've measured — RTABMap delay went 3.7 s → 0.5 s.
2. **Kill non-essential processes** — `rustdesk`, screen recorder, browser, anything else on the Jetson.
3. **Disable D435 depth pointcloud** — `slam_mode:=true` does this automatically.
4. **Lower BT/controller rates** in `config/nav2_params.yaml`:
   ```yaml
   bt_navigator.ros__parameters.bt_loop_duration: 20      # was 10 ms = 100 Hz; bump to 50 Hz
   controller_server.ros__parameters.controller_frequency: 10.0   # was 20.0
   ```
5. **Lower RTABMap detection rate** in `launch/rtabmap.launch.py`:
   ```python
   'Rtabmap/DetectionRate': '0.5',   # was 1.0 Hz
   ```
6. **Enable jetson_clocks** for max performance:
   ```bash
   sudo jetson_clocks
   ```

---

## `/cmd_vel` publishes but rover doesn't move

**Symptom:** `ros2 topic echo /cmd_vel` shows valid Twist messages flying by (e.g. `linear.x: 0.5`), but the rover physically doesn't move. Nav2 eventually logs `Failed to make progress` and triggers recovery behaviors.

**Cause:** No node is consuming `/cmd_vel` and writing to motors.

**Diagnose:**
```bash
ros2 topic info /cmd_vel | grep -i count    # Subscription count: must be ≥ 1
pgrep -af yahboom_bridge                    # bridge running?
ls -l /dev/myserial                         # USB device pinned?
```

**Fix (mecanum):** Run the Yahboom bridge.

```bash
# Option A: standalone (good for bench testing)
./start_yahboom.sh

# Option B: include in the SLAM stack (production navigation)
./start_nav.sh platform:=mecanum force_3dof:=true enable_drive:=true
```

**Fix (other platforms):** Per-platform bridge required. Go2 / R2D2 / Roboscout drive bridges are the open Phase 3 task in PLAN.md §9.5.

**Sanity check before Nav2** — drive the rover with manual teleop first:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
# Press 'b' for holonomic mode (mecanum strafe)
```

If teleop drives the rover, Nav2 will too — same `/cmd_vel` topic.

---

## Yahboom bridge: Rosmaster_Lib not installed

**Symptom:**
```
[yahboom_bridge] ERROR: Rosmaster_Lib not installed.
  Yahboom does not publish this lib to PyPI. Install from
  the kit microSD/USB stick, or download from
    https://www.yahboom.net/study/ROSMASTER-X3 → Resources
```

**Fix:** The library is vendored at `vendor/Rosmaster_Lib_3.3.9/`:

```bash
pip3 install --user ~/slam_ws/src/slam_bringup/vendor/Rosmaster_Lib_3.3.9
python3 -c "from Rosmaster_Lib import Rosmaster; print('OK')"
```

`./install.sh` does this automatically; if you ran it before the vendor/ directory landed (commit `825623d`), re-run `./install.sh`.

---

## Yahboom bridge: serial port permission denied

**Symptom:** `[yahboom_bridge] failed to open /dev/myserial: [Errno 13] Permission denied`.

**Diagnose:**
```bash
ls -l /dev/myserial          # → ../../ttyUSB0 (must be a symlink)
ls -l /dev/ttyUSB0           # check the actual permissions
groups | grep dialout        # must contain dialout
```

**Fixes:**

1. **udev rule missing** — `/dev/myserial` doesn't exist:
   ```bash
   sudo tee /etc/udev/rules.d/99-yahboom.rules <<'EOF'
   SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", SYMLINK+="myserial", MODE="0666"
   EOF
   sudo udevadm control --reload-rules && sudo udevadm trigger
   ```
2. **Not in `dialout` group** — `./install.sh` should add you. If just added, log out/in.
   ```bash
   sudo usermod -aG dialout $USER
   # Log out + back in
   ```

---

## Yahboom bench-test: a wheel spins backward / strafe inverted

**Symptom:** During the §4 verification tests in the Obsidian wiring note, one wheel spins the wrong way, OR strafe (vy=+0.1) doesn't produce MA+MD forward / MB+MC backward.

**Fix (single wheel wrong):** Use the **firmware polarity flag** for that channel. NEVER swap motor leads — keep wiring identical across all four ports so encoder signal mapping stays clean.

**Fix (strafe broken even after individual wheels are right):** Port assignment error. Re-check the MA/MB/MC/MD corner mapping in the Obsidian *"Yahboom Mecanum Configuration - Motor Wiring and Taranis SBUS Setup"* note §2. The firmware kinematics matrix expects MA/MB/MC/MD at specific corners; polarity flags can't compensate for swapped ports.

**Wheel-roller direction:** must form an **"X" pattern when viewed from above**. "O" pattern means diagonals fight each other and strafe disappears.

---

## Mid-360 driver fails to bind UDP sockets

**Symptom:** `livox_ros_driver2_node` starts but throws "address already in use" on UDP port 56301 (or similar).

**Cause:** Orphan driver from a previous session holding the socket. Or `TIME_WAIT` from a recent crash.

**Diagnose:**
```bash
ss -uln | grep -E ':5610|:5620|:5630|:5640'    # any listeners = stale orphan
pgrep -af livox
```

**Fix:**
```bash
./kill_mid360.sh         # now exits 1 with diagnostic if pkill -9 fails (commit c7e4184)
# If still listening:
sudo pkill -9 -f livox_ros_driver2_node
ros2 daemon stop
sleep 30                 # TIME_WAIT window
```

**Prevent:** the new `kill_mid360.sh` verifies a clean kill via `pgrep` and won't return success if a survivor exists.

---

## D435 "Frames didn't arrive within 5 seconds" / "Device or resource busy"

**Symptom:** RealSense node starts but no frames arrive, OR fails with "Device or resource busy".

**Cause:** USB handle stuck — usually a previous `realsense2_camera_node` didn't release `/dev/video0` cleanly, or the camera firmware hung.

**Fix:** `./kill_d435.sh` does a sysfs-level USB reset (`echo 0 > authorized; echo 1 > authorized`) which clears most firmware hangs. Needs sudo (script handles the prompt).

```bash
./kill_d435.sh
./start_d435.sh slam_mode:=true
```

If still failing, **physically replug** the camera. Some USB-3 hubs don't propagate the sysfs reset cleanly.

---

## WitMotion: "Device or resource busy" / no /imu/data

**Symptom:** Either `pyserial.SerialException: Device or resource busy`, or the node starts but `/imu/data` never publishes.

**Diagnose:**
```bash
pgrep -af wt901c_imu             # orphan?
ls -l /dev/ttyUSB*               # which port?
groups | grep dialout            # in group?
```

**Fixes:**

1. **Orphan node** — `./kill_witmotion.sh` (kill_helpers escalation).
2. **Wrong port** — `./start_witmotion.sh serial_port:=/dev/ttyUSB1`.
3. **Not in dialout** — `sudo usermod -aG dialout $USER`, log out/in.
4. **Firmware not in 0x61 mode** — connect WitMotion to a Windows PC with their config tool, set output mode to `0x61` packet (one-time per device).

---

## Tilted point cloud / camera_init not level with floor

**Symptom:** In Foxglove with display frame `camera_init`, the floor appears tilted by several degrees. In `body` frame it looks correct.

**Cause:** FAST-LIO captured the rig's gravity initialization while it was moving / not yet settled. The world-frame Z axis got locked in tilted.

**Fix:**
1. **Restart FAST-LIO with the rig stationary**, give it 30 s to settle before moving.
2. **Permanent fix** — measure mounting tilt with `scripts/measure_imu_tilt.py`:
   ```bash
   python3 ~/slam_ws/src/slam_bringup/scripts/measure_imu_tilt.py --ros-args \
       -p num_samples:=600 -p imu_topic:=/livox/imu_ms2
   ```
   Output is roll + pitch and a 3×3 rotation matrix to paste into `extrinsic_R` in `config/fast_lio_mid360.yaml`.
3. **If tilt > 2° AND mechanical** — shim the sensor plate flat instead of baking the correction into `extrinsic_R`.

---

## URDF / platform-specific issues

**Symptom:** `[ERROR] [launch]: ... no PLATFORM_BRIDGES entry for '<platform>'`.

**Cause:** `slam.launch.py` requires a `body → base_link` static TF offset per platform, in `PLATFORM_BRIDGES`. Stubs aren't trusted by default; you must measure and add the entry.

**Fix:** Edit `launch/slam.launch.py`, add an entry:
```python
PLATFORM_BRIDGES = {
    'bench_fixture': (0.0, 0.0, -0.244280, 0.0, 0.0, 0.0),   # measured
    'mecanum':       (0.0, 0.0, -0.144280, 0.0, 0.0, 0.0),   # PLACEHOLDER — measure!
    'go2':           (X, Y, Z, R, P, Yaw),                    # add yours
}
```

**Measurement procedure:** Z is from the wheel-bottom contact (= `base_link`) to the **top of the sensor plate**. Add the plate-top → `livox_frame` offset (36.61 mm per Livox datasheet). Result is the negative of where `base_link` sits below `body`.

The bench fixture stack-up is documented in `launch/slam.launch.py`'s `PLATFORM_BRIDGES` comments.

**URDF file location:** `urdf/<platform>.urdf.xacro`. Currently only `bench_fixture` exists; mecanum / go2 / r2d2 / roboscout are Phase 1.7.

**Other URDF symptoms:**
- **Robot model not visible in RViz** — `RobotModel` display needs `Description Topic: /robot_description` with `Durability Policy: Transient Local`. `rviz/perception.rviz` has this set.
- **TF chain broken** — `ros2 run tf2_tools view_frames` produces a PDF of the actual tree. Look for missing parents.

---

## Sitting stationary issues

When the rover is supposed to be still but the stack misbehaves:

| Symptom | Section |
|---|---|
| Pose drifts in `tf2_echo map base_link` | [FAST-LIO drift while stationary](#fast-lio-drift-while-stationary-z---4-m-pitch-17) |
| `lidar loop back` flood | [lidar loop back](#lidar-loop-back-clear-buffer-flood-from-fast-lio) |
| RTABMap loop closures rejected | [Rejecting all added loop closures](#rtabmap-rejecting-all-added-loop-closures-flood) |
| Tilted cloud in `camera_init` view | [Tilted point cloud](#tilted-point-cloud--camera_init-not-level-with-floor) |
| Nav2 goal aborts immediately | [Nav2 goal aborts](#nav2-goal-aborts-with-distance-remaining-000-m) |

**General "sitting still" debugging:**
- Wait 30 s after `start_fast_lio.sh` for the ESKF to learn IMU biases.
- Check `tf2_echo map base_link` shouldn't move > a few mm/s.
- Check `ros2 topic hz /Odometry` is 10 Hz.

---

## Moving / driving issues

When the rover is moving and things go wrong:

| Symptom | Likely cause |
|---|---|
| Pose drift accumulates fast | Fast turns + over-tight `acc_cov`. Try `acc_cov: 0.5` (was 0.1). |
| Cloud "ghosts" / double walls in map | Loop closures aren't catching. Drive a closed loop; check `Loop closure detected` log lines. |
| FAST-LIO logs `No Effective Points!` | Voxel grid too coarse for sparse Mid-360 sweeps in narrow space. `filter_size_surf: 0.3` works for indoor; lower if scenes are very sparse. |
| Body cam in Mid-360 FOV biases yaw | **Keep your body out of the Mid-360's horizontal FOV** when handheld. Use a pole / shoulder mount. |
| RTABMap delay grows | Jetson CPU saturated. See [Jetson CPU saturated](#jetson-cpu-saturated--controller-missing-rate-rtabmap-delay--1-s). |

---

## `./build.sh` errors: package not found

**Symptom:** `./build.sh` succeeds but `./start_nav.sh` fails with `package 'pointcloud_to_laserscan' not found` (or similar).

**Cause:** `slam_bringup`'s `package.xml` declares deps as `<exec_depend>` only. colcon doesn't fail at build time for missing exec deps; it surfaces at launch time.

**Fix (commit `48b8249`):** `./build.sh` now verifies via `ros2 pkg list`:

```bash
./build.sh
# Look for "✗ pointcloud_to_laserscan — NOT found" or "✗ nav2_bringup — NOT found"
sudo apt install ros-humble-nav2-bringup ros-humble-pointcloud-to-laserscan
```

The full apt set:
```bash
sudo apt install \
  ros-humble-nav2-bringup \
  ros-humble-pointcloud-to-laserscan \
  ros-humble-nav2-rviz-plugins \
  ros-humble-slam-toolbox \
  ros-humble-rtabmap-rviz-plugins
```

---

## CloudCompare shows just a "line" / sparse cloud after `export_map.sh`

**Symptom:** Open `~/maps/<timestamp>/cloud.ply` in CloudCompare and see only a thin trajectory of dots, not a dense scan cloud.

**Cause:** PLY files contain multiple elements — `vertex` (the cloud) and `camera` (the trajectory poses). CloudCompare's PLY import shows you the camera/trajectory element by default if not specified.

**Fix:**
1. In CloudCompare's PLY-import dialog, **remove** any `camera - scalex/scaley` entries from the Scalar fields box.
2. Click **Apply all**.
3. In the DB Tree, select the `vertex` cloud (the larger sibling).
4. Press **Z** to fit the view onto it.

If still sparse, re-export with `--voxel-size 0.0` (no voxel downsampling) for max density:
```bash
./scripts/export_map.sh --voxel-size 0.0 --no-color
```

---

## `rtabmap-export` crashes with `--cam_projection` or `--mesh`

**Symptom:**
- `rtabmap-export ... --cam_projection ~/.ros/rtabmap.db` → `vector::_M_range_check: __n (which is 18446744073709551615) >= this->size()` crash mid-export.
- `rtabmap-export --mesh --texture ...` → `createTextureMesh() Condition (mesh->polygons.size()) not met!`

**Cause:** Upstream RTABMap bugs.
- `--cam_projection` crash on long sessions (~frame 147/1019 — index overflow in image-cache lookup).
- `--mesh --texture` fails because LiDAR cloud has no `rgb` field, color transfer culls all polygons.

**Fix (workaround):** Use `--no-color` and skip `--mesh --texture`:

```bash
./scripts/export_map.sh --no-color   # plain assembled cloud, no RGB projection
```

The wrapper script (`scripts/export_map.sh`) is tuned to a working flag combination by default.

---

## Quick diagnostic playbook

When something is off and you don't know where to start, run these in order:

```bash
# 1. Process inventory — orphans? right count?
pgrep -af 'fastlio|livox|imu_units|rtabmap|nav|pointcloud_to_laserscan|yahboom'

# 2. Topic publishers/subscribers — count = 1 unless intended otherwise
for t in /livox/lidar /livox/imu_ms2 /Odometry /cloud_registered /cloud_registered_body \
         /map /scan /cmd_vel /tf /tf_static; do
  echo "== $t =="
  ros2 topic info $t 2>&1 | grep -i count
done

# 3. TF — does the chain reach base_link from map?
ros2 run tf2_ros tf2_echo map base_link
ros2 run tf2_ros tf2_echo map camera_init     # is RTABMap correcting?
ros2 run tf2_ros tf2_echo body base_link      # platform offset published?

# 4. Rates — anything stalled?
timeout 5 ros2 topic hz /Odometry             # ~10 Hz
timeout 5 ros2 topic hz /cloud_registered     # ~10 Hz
timeout 5 ros2 topic hz /scan                 # ~10 Hz (when nav2 is up)

# 5. CPU / memory — Jetson healthy?
top -bn1 | head -20
free -h
uptime

# 6. RTABMap stats
rtabmap-info ~/.ros/rtabmap.db | head -30
```

Send the output of these to a fresh debug session (or paste into a future Claude prompt) and most issues localize within a minute.
