# slam_bringup

Portable sensor-rig SLAM stack for ROS2 Humble. One NVIDIA Jetson Orin Nano Super + one sensor plate moves between multiple mobile robots; the package selects per-platform URDF + TF bridge offsets at launch time.

## Documentation map

- **[docs/scripts/](docs/scripts/README.md)** — per-script reference. Every `start_*.sh` has a dedicated page with parameters, defaults, dependency chain, and "when to override" guidance. Look here when you want to know what a specific script does or which arg to flip.
- **[docs/troubleshooting.md](docs/troubleshooting.md)** — issue-by-issue guide indexed by symptom. Every problem we've actually hit on this stack has an entry with diagnosis commands and the exact fix. Search here before fresh debugging.
- **[docs/map_export.md](docs/map_export.md)** — exporting `~/.ros/rtabmap.db` to PLY/PCD/etc. and viewing in CloudCompare, MeshLab, Open3D, Foxglove, RViz, or Potree.
- **[PLAN.md](PLAN.md)** — phase-by-phase architecture plan and task checklist.
- **[TEST_PLAN.md](TEST_PLAN.md)** — Phase 1.6 / Phase 2 hardware validation tests.
- **[docs/](docs/)** — long-form notes (`first_rtabmap_run.md`, `test_fixture.md`).

This README itself is the cross-cutting overview (TF tree, frame conventions, sensor list, network setup). For lookup-by-script-name go to `docs/scripts/`; for lookup-by-symptom go to `docs/troubleshooting.md`.

## Startup commands (current — Phase 3 / Navigation)

Each line is its own terminal (so each driver group gets its own log stream and Ctrl-C kills only that one). All commands assume `~/.bashrc` already sources `/opt/ros/humble/setup.bash` + `~/slam_ws/install/setup.bash` — see [Shell environment](#shell-environment-source-before-ros2-commands) below.

**Sensor-only bench workflow (Foxglove visualization, no SLAM):**

```bash
# Terminal 1 — All three sensors (Mid-360 + D435 + WitMotion) in one launch
cd ~/slam_ws/src/slam_bringup && ./start_sensors.sh

# Terminal 2 — livox_frame → camera_link static TF (bench stopgap until Phase 1.7 URDF)
cd ~/slam_ws/src/slam_bringup && ./start_bench_tf.sh

# Terminal 3 (optional) — Foxglove bridge for remote Studio/App viewing
cd ~/slam_ws/src/slam_bringup && ./start_foxglove.sh
```

In Foxglove: set **Display Frame / Fixed Frame** to `livox_frame` and add `/livox/lidar` + `/d435_front/camera/depth/color/points` as PointCloud2 displays.

**FAST-LIO2 SLAM workflow** (odometry + incremental point-cloud map):

```bash
# Terminal 1 — Mid-360 in CustomMsg mode (FAST-LIO2 subscribes to this format,
# not PointCloud2). Skip D435 + WitMotion since FAST-LIO2 uses the Mid-360's
# onboard IMU only — drops CPU contention during pose bring-up.
cd ~/slam_ws/src/slam_bringup && ./start_sensors.sh lidar_xfer_format:=1 enable_d435:=false enable_witmotion:=false

# Terminal 2 — FAST-LIO2 mapping
cd ~/slam_ws/src/slam_bringup && ./start_fast_lio.sh

# Terminal 3 (optional) — Foxglove bridge
cd ~/slam_ws/src/slam_bringup && ./start_foxglove.sh
```

In Foxglove: set **Display Frame** to `camera_init` (FAST-LIO2's world frame) and add `/Odometry` (Pose or Path) plus a point-cloud display. FAST-LIO2 publishes `/cloud_registered` as **per-scan** clouds in the world frame, not a server-side accumulated map — pick one of:

- **Option A — Foxglove client-side accumulation (recommended).** Add `/cloud_registered` as a PointCloud2 display and set its **Decay time** to a large value (e.g. `1e9`). Foxglove piles up scans locally; the Jetson keeps sending one scan per message, so bandwidth and node memory stay flat. Best for longer runs over WiFi.
- **Option B — server-side accumulated `/Laser_map`.** Set `publish.map_en: true` in `config/fast_lio_mid360.yaml` and relaunch `./start_fast_lio.sh`, then add `/Laser_map` as a PointCloud2 display (decay `0`). FAST-LIO2 appends every scan to an internal buffer and republishes the whole cloud each tick — message size and node memory grow unbounded, so use this only for short demos. For real long-session mapping, switch to RTABMap (Phase 7.4).

**FAST-LIO2 operating conditions** (validated 2026-04-20/21 on gizmo):
- Test in a **furnished space** — living room, lab with equipment, hallway with doorframes. Mid-360 needs vertical structure within ~5 m. An empty table in a mostly empty room diverges even with correct config.
- Give FAST-LIO **~30 s of stationary settle time** after `./start_fast_lio.sh` before moving. That's the ESKF learning accel/gyro biases; don't flag early drift as a defect.
- For walking tests, **keep your body out of the Mid-360's horizontal FOV**. Chest-height handheld occludes 60–90° of azimuth and biases the yaw estimate, producing a slow orbit drift that persists after you stop. Use an overhead pole, a cart-top mount, or helmet/shoulder mount. See [`TEST_PLAN.md`](TEST_PLAN.md) *Phase 2 — hard-won constraints* for details.
- If you see **upside-down point cloud** in Foxglove's 3D panel set to `camera_init` but correct when set to `body`: FAST-LIO's world-frame gravity initialization captured the rig while it was moving. Restart with the rig stationary.
- If Foxglove shows **no lidar data**, remember `/livox/lidar` is `livox_ros_driver2/CustomMsg` and Foxglove can't render it. Use `/cloud_registered` (world) or `/cloud_registered_body` (body) instead.
- If the displayed cloud **only shows the current room and moves/rotates with the sensor instead of accumulating**, you're subscribed to `/cloud_registered` — switch to `/Laser_map`. `/cloud_registered` is a single sweep re-rendered each frame in `camera_init`; it is not the map. Keep/Decay RViz settings won't fix this — wrong topic. (If `/Laser_map` also drifts/rotates as you move, that's real odometry drift — see the Obsidian FAST-LIO2 Troubleshooting note, sections 3/7/10.)

Common `start_sensors.sh` arg overrides:

```bash
./start_sensors.sh slam_mode:=true                  # for RTABMap (D435 align_depth on, pointcloud off)
./start_sensors.sh lidar_xfer_format:=1             # for FAST-LIO2 (Mid-360 emits CustomMsg)
./start_sensors.sh enable_rear:=true                # bring up the D435i rear camera (Phase 1.10)
./start_sensors.sh enable_witmotion:=false          # drop one sensor for isolated debugging
```

**Default platform is `mecanum`.** The mecanum UGV is the working test bed; bench-fixture and handheld testing are rare debug flows. The defaults below assume that. Override with `platform:=bench_fixture` only when you genuinely need it.

**Full SLAM workflow** (URDF + sensors + FAST-LIO2 + RTABMap, builds `~/.ros/rtabmap.db`):

```bash
cd ~/slam_ws/src/slam_bringup && ./start_slam.sh delete_db_on_start:=true
```

`force_3dof` is **on by default** (clamps z/roll/pitch — required on wheeled rovers, see Drift section in Troubleshooting). Foxglove bridge auto-spawns in the background.

Drive the rover through the space; Ctrl-C saves to `~/.ros/rtabmap.db`. View live from Foxglove Studio on your dev machine: `New Connection → Foxglove WebSocket → ws://gizmo.local:8765`. See [Navigation (Nav2)](#navigation-nav2) for the next step and [Map persistence](#map-persistence-and-rosrtabmapdb) for inspecting the saved DB.

**Navigation workflow** (loads the saved map, runs Nav2 on top — no new mapping):

```bash
cd ~/slam_ws/src/slam_bringup && ./start_nav.sh
```

That single command brings up: SLAM stack in localization mode + Nav2 + Yahboom drive bridge (`enable_drive:=true` is forced). The rover physically moves on `2D Goal Pose`. Foxglove auto-spawns; connect from your dev machine and click **2D Goal Pose** on the saved `/map`.

`start_nav.sh` refuses to launch against an empty DB. To navigate without driving (planner-visualization on the bench), pass `platform:=bench_fixture enable_drive:=false`.

**Foxglove auto-spawn:** every `start_*.sh` runs `ensure_foxglove` from `start_helpers.sh` — idempotent background spawn of `foxglove_bridge`. Set `SLAM_NO_FOXGLOVE=1` to suppress (e.g. headless / CI). The bridge survives Ctrl-C of the foreground stack.

This section tracks the **latest working command set** for whatever phase is current. Check the [Status](#status) section for what's wired up.

## Sensors (shared across all platforms)

- **Livox Mid-360** — 360° × 59° non-repetitive LiDAR with built-in ICM40609 IMU (primary for FAST-LIO2)
- **Intel RealSense D435** (front) + **D435i** (rear) — depth + RGB, D435i's BMI055 IMU disabled
- **WitMotion WT901** — redundant / backup IMU

## Target platforms

| Platform | `platform:=` | `base_link` source |
|----------|--------------|---------------------|
| Unitree Go2 Pro | `go2` | Go2 SDK |
| Lifesize R2D2 | `r2d2` | URDF via `robot_state_publisher` |
| Sharper Image Roboscout | `roboscout` | URDF via `robot_state_publisher` |
| Mecanum 4-wheel UGV | `mecanum` | URDF via `robot_state_publisher` |

Each platform ships its own `urdf/<platform>.urdf.xacro` with measured offsets and its own `base_link → body` TF bridge entry in `launch/slam.launch.py`.

## SLAM stack

- **FAST-LIO2** — LiDAR-inertial odometry (Mid-360 built-in IMU, rigid coupling)
- **RTABMap** — graph SLAM with visual loop closure via D435 front RGB + depth
- Output: `/Odometry`, `/cloud_registered_body`, `/map`, `/octomap_full`, persistent `~/.ros/rtabmap.db`

## First-time setup (on the Jetson)

```bash
mkdir -p ~/slam_ws/src && cd ~/slam_ws/src
git clone <this-repo-url> slam_bringup
cd slam_bringup
./install.sh
```

`install.sh` clones the three vendor drivers (`livox_ros_driver2`, `witmotion_ros@ros2`, `FAST_LIO_ROS2`), builds Livox-SDK2, apt-installs RealSense + RTABMap + Nav2 + CycloneDDS, runs `rosdep`, adds the user to `dialout`, exports `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`, and runs the first `colcon build` with the livox-required cmake args.

## Network setup (one-time, per Jetson)

The Mid-360 broadcasts LiDAR + IMU data on `192.168.1.0/24`. The Jetson needs a dedicated ethernet interface on that subnet — a USB-to-Gigabit-Ethernet adapter with the Realtek RTL8153 chipset is recommended (the `r8152` driver ships in the mainline kernel and JetPack 6.x).

### 1. Identify the USB-to-GbE adapter

Plug the adapter in, then:

```bash
lsusb | grep Realtek       # should list the adapter
dmesg | grep r8152         # driver bound cleanly
ip -br link                # list all interfaces — find the new one
```

The new interface usually shows up as `eth1`, `enp*s0`, or `enx<MAC>` depending on systemd's naming policy. Note the exact name — below it's referred to as `<IFACE>`.

### 2. Create the LiDAR connection profile

```bash
sudo nmcli con add type ethernet ifname <IFACE> con-name lidar \
  ipv4.method manual ipv4.addresses 192.168.1.100/24
sudo nmcli con up lidar
```

The Jetson's address can be anything on `192.168.1.0/24` **except** `.0`, `.201`, `.202`, `.203`, or `.255` (those are reserved: network/broadcast + the three LiDAR IPs documented in PLAN.md §3.5 — VLP-16, Mid-360, Pandar40P).

### 3. Go2 control network (Go2 only — harmless idle on other platforms)

```bash
sudo nmcli con add type ethernet ifname eth0 con-name go2 \
  ipv4.method manual ipv4.addresses 192.168.123.XX/24
sudo nmcli con up go2
```

Pick a free `.XX` on the Go2's internal subnet. Leaving the profile configured with the cable unplugged on R2D2 / Roboscout / mecanum is a no-op.

### 4. Verify

```bash
ip -br addr show <IFACE>   # shows 192.168.1.100/24 — UP
ping -c 3 192.168.1.202    # Mid-360 reachable (powered + cabled)
```

### 5. Mid-360 driver host IP — keep in sync with step 2

`config/mid360.json` has a `host_net_info` block with four `*_ip` fields (`cmd_data_ip`, `push_msg_ip`, `point_data_ip`, `imu_data_ip`). These are the addresses the **driver binds its UDP listener sockets to** — they must exactly equal the Jetson's IP chosen in step 2 (default `192.168.1.100`). They cannot be `255.255.255.255` or `0.0.0.0`; the driver silently fails to open its listeners and no `/livox/*` topics ever publish. The committed value is `192.168.1.100` — matches the default in step 2, so for most installs this just works.

If you need to run on a Jetson with a different LiDAR-network IP (e.g. multi-Jetson rig, alternate subnet, home LAN), edit all four fields and rebuild:

```bash
sed -i 's/192\.168\.1\.100/<NEW_HOST_IP>/g' ~/slam_ws/src/slam_bringup/config/mid360.json
cd ~/slam_ws && colcon build --packages-select slam_bringup --symlink-install
```

The lidar's own on-device destination (set once via Livox Viewer 2) is already broadcast `255.255.255.255`, so the lidar itself doesn't need reconfiguring when the host IP changes — only this file does.

### Troubleshooting

- **Interface disappears on reboot** — plug order changes the systemd name. Pin it by its MAC with a udev rule, or rely on the nmcli `con-name` (which binds by name, not MAC — so pinning is optional).
- **`ping` hangs** — check the cable, Mid-360 power, and that `<IFACE>` actually corresponds to the USB adapter (not `eth0`).
- **Both interfaces show `NO-CARRIER`** — the cable is unplugged or dead.
- **Multiple LiDARs on the same subnet** — only one can be connected at a time unless you set up host-side routing (VLP-16, Mid-360, and Pandar40P all share `192.168.1.0/24`).

## Shell environment (source before `ros2` commands)

Two `setup.bash` files in order — base ROS2 first, then the workspace overlay:

```bash
source /opt/ros/humble/setup.bash              # base ROS2 Humble
source ~/slam_ws/install/setup.bash            # workspace overlay (slam_bringup + vendor drivers)
```

Order matters; the workspace overlay must come second so it layers on top of base ROS2.

**Make it permanent** — append both lines (and confirm the CycloneDDS export is there) to `~/.bashrc` so every new shell is ready:

```bash
cat >> ~/.bashrc <<'EOF'
source /opt/ros/humble/setup.bash
source ~/slam_ws/install/setup.bash
EOF

# Apply to the current shell without opening a new terminal:
source ~/.bashrc
```

`install.sh` already appended `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` to `~/.bashrc`; these two `source` lines complete the environment.

**Verify:**

```bash
echo $ROS_DISTRO                               # humble
echo $RMW_IMPLEMENTATION                       # rmw_cyclonedds_cpp
ros2 pkg prefix slam_bringup                   # /home/rico/slam_ws/install/slam_bringup
```

## Dev loop (after edits)

```bash
cd ~/slam_ws/src/slam_bringup
git pull
./build.sh                        # fast rebuild of slam_bringup only
source ~/slam_ws/install/setup.bash
```

`build.sh` flags:

| Command | Purpose |
|---------|---------|
| `./build.sh` | Build only `slam_bringup` — fast |
| `./build.sh --all` | Build the whole workspace (vendor + `slam_bringup`) |
| `./build.sh --clean` | `rm -rf build/slam_bringup install/slam_bringup` first |
| `./build.sh --clean --all` | Full clean workspace rebuild |
| `./build.sh <pkg> [<pkg> ...]` | Build specific packages |
| `./build.sh --help` | Show help |

After each build the script runs package / launch / config sanity checks with ✓ / ✗ output so you know the new setup is good before sourcing.

## Running

```bash
# Mid-360 standalone (Phase 1.3 — works today)
ros2 launch slam_bringup mid360.launch.py                   # xfer_format=0, RViz-viewable
ros2 launch slam_bringup mid360.launch.py xfer_format:=1    # CustomMsg for FAST-LIO2

# D435 front (Phase 1.4 — works today; rear defaults off, SLAM mode toggles align_depth)
ros2 launch slam_bringup d435.launch.py                     # front-only, raw mode (pointcloud on)
ros2 launch slam_bringup d435.launch.py slam_mode:=true     # front-only, SLAM mode (align_depth on, pointcloud off)

# Planned (not yet implemented — see PLAN.md phases below)
ros2 launch slam_bringup witmotion.launch.py                          # Phase 1.5
ros2 launch slam_bringup sensors.launch.py                            # Phase 1.6
ros2 launch slam_bringup perception.launch.py platform:=go2           # Phase 1.7
ros2 launch slam_bringup slam.launch.py       platform:=go2           # Phase 2.5
```

Swap `platform:=r2d2|roboscout|mecanum` when the rig moves. `enable_rear:=true` adds the D435i rear camera.

### Helper start/kill scripts

One-liner wrappers around the launches that also handle "the driver got wedged and is still holding sockets/USB" — each `start_*.sh` detects an existing instance and invokes the matching `kill_*.sh` before relaunching:

| Script | Purpose |
|--------|---------|
| `./start_mid360.sh` | Launch Mid-360; auto-clean a stale `livox_ros_driver2_node` first |
| `./kill_mid360.sh` | Force-kill the Mid-360 driver + its launch wrapper + `ros2 daemon stop` |
| `./start_d435.sh` | Launch D435 front; auto-clean a stale `realsense2_camera_node` first |
| `./kill_d435.sh` | Force-kill the D435 driver + its launch wrapper |
| `./start_witmotion.sh` | Launch WT901C `wt901c_imu` Python node; auto-clean a stale instance first |
| `./kill_witmotion.sh` | Force-kill the WitMotion node + its launch wrapper |
| `./start_sensors.sh` | Launch all three sensors via `sensors.launch.py`; pre-cleans every per-sensor driver before relaunching |
| `./kill_sensors.sh` | Chain `kill_mid360.sh` + `kill_d435.sh` + `kill_witmotion.sh`, then nuke the parent launch wrapper |
| `./start_fast_lio.sh` | Launch FAST-LIO2 (`fastlio_mapping`) against `/livox/lidar` CustomMsg + `/livox/imu`; auto-clean a stale instance first. Also auto-spawns `viz_clip` for `/cloud_viz_clipped` (override with `viz_z_max:=…` or disable with `enable_viz_clip:=false`). |
| `./kill_fast_lio.sh` | Force-kill the FAST-LIO2 node + its launch wrapper. Also chains `kill_viz_clip.sh`. |
| `./start_viz_clip.sh` | Standalone viz-clip republisher: subscribes `/cloud_registered` (overridable), publishes z-clipped `/cloud_viz_clipped`. Use against a bag replay or alongside an already-running pipeline. |
| `./kill_viz_clip.sh` | Force-kill the `viz_clip_container` + its launch wrapper. Called automatically by `kill_fast_lio.sh` and `kill_slam.sh`. |
| `./start_rtabmap.sh` | Launch RTABMap on top of FAST-LIO2 + D435 RGB-D (visual loop closure + occupancy grid). Preflights all 5 input topics + body→d435_front_link TF before launching |
| `./kill_rtabmap.sh` | Force-kill the RTABMap node + its launch wrapper |
| `./start_perception.sh` | Launch URDF (`platform:=bench_fixture` default) + all sensors + optional rviz2. No SLAM — for visualizing sensor placement against the URDF tree. |
| `./start_slam.sh` | Launch the full SLAM stack: URDF + sensors (slam_mode) + FAST-LIO2 + RTABMap + per-platform `body→base_link` bridge. Pass-through args: `platform:=`, `delete_db_on_start:=`, `localization:=`, `nav2:=`, `rviz:=`. |
| `./kill_slam.sh` | Force-kill every layer of the SLAM stack (chains `kill_nav.sh`, `kill_rtabmap.sh`, `kill_fast_lio.sh`, `kill_viz_clip.sh`, `kill_sensors.sh`). |
| `./start_nav.sh` | Launch the SLAM stack in **navigation mode**: localization-only RTABMap + Nav2. Requires an existing `~/.ros/rtabmap.db` built by `start_slam.sh` first. Add `enable_drive:=true` for the mecanum rover. |
| `./kill_nav.sh` | Force-kill the Nav2 nodes + `pointcloud_to_laserscan`. Called automatically by `kill_slam.sh`. |
| `./start_yahboom.sh` | Launch the Yahboom YB-ERF01 `/cmd_vel` → mecanum bridge over USB-serial. Standalone (no SLAM) for bench-testing the drive base with `teleop_twist_keyboard`. See [Mecanum drive (Yahboom YB-ERF01)](#mecanum-drive-yahboom-yb-erf01). |
| `./kill_yahboom.sh` | Force-kill the Yahboom bridge. Called automatically by `kill_slam.sh`. |
| `./start_bench_tf.sh` | Publish `livox_frame → camera_link` static TF for multi-sensor visualization (see below) |
| `./start_foxglove.sh` | Start `foxglove_bridge` on the Jetson for remote Studio/App connections |

### Top-down floorplan view (`/cloud_viz_clipped`)

Both `./start_slam.sh` and `./start_fast_lio.sh` run a tiny rclpy node (`slam_bringup/viz_clip_node.py`) alongside the rest of the pipeline that republishes a z-clipped copy of `/cloud_registered` on `/cloud_viz_clipped` — purely for visualization. The raw cloud still flows into FAST-LIO2 / RTABMap unchanged, so ICP, the occupancy grid, and `/octomap_full` keep their full vertical extent (you can still measure rafters in the 3D map).

(Implementation note: this used to be a `pcl_ros::PassThrough` composable node, but the Humble `ros-humble-pcl-ros` 2.4.5 deb does not consistently expose its filter classes to `class_loader` — the container fails the lookup at startup with "Failed to find class with the requested plugin name". The rclpy node sidesteps the packaging issue and is plenty fast for a 10 Hz Mid-360 cloud on the Orin Nano.)

In Foxglove / RViz, set the **Display frame** to `camera_init` and add `/cloud_viz_clipped` as a PointCloud2 display. With the ceiling clipped you can sit the camera straight above and see the floorplan with furniture, walls, and doorways visible inside each room.

`z = 0` is the body pose at FAST-LIO2 startup (i.e. the IMU position when you launched), **not** the floor. Tune `viz_z_max` for the rig's startup height and the space:

```bash
./start_slam.sh                                # defaults: viz_z_min=-3.0, viz_z_max=3.0 (covers typical room)
./start_slam.sh viz_z_max:=4.5                 # garage / shop with high ceilings
./start_slam.sh viz_z_min:=0.5 viz_z_max:=1.5  # narrow band for a clean floorplan section
./start_slam.sh enable_viz_clip:=false         # disable; only /cloud_registered is published
```

The same arg overrides work on `./start_fast_lio.sh` for FAST-LIO-only testing — the script splits viz-related args off and passes them to a backgrounded `viz_clip.launch.py`:

```bash
./start_fast_lio.sh                            # FAST-LIO2 + auto viz_clip with defaults
./start_fast_lio.sh viz_z_max:=4.5             # garage
./start_fast_lio.sh enable_viz_clip:=false     # FAST-LIO2 only, no /cloud_viz_clipped
```

Standalone replay (against a bag, or alongside a manually-launched pipeline):

```bash
./start_viz_clip.sh viz_z_max:=2.5
# or directly:
ros2 launch slam_bringup viz_clip.launch.py viz_z_max:=2.5
```

Args: `enable_viz_clip` (default `true`), `viz_z_min` (default `-1.0`), `viz_z_max` (default `2.0`), `viz_input_topic` (default `/cloud_registered`), `viz_output_topic` (default `/cloud_viz_clipped`).

### Navigation (Nav2)

Once you've built a map with `./start_slam.sh`, you can navigate inside it with Nav2. There are two ways to enable it:

```bash
./start_nav.sh                                  # convenience wrapper: localization=true + nav2=true
./start_slam.sh nav2:=true localization:=true   # equivalent — useful with custom args
```

`start_nav.sh` refuses to launch if `~/.ros/rtabmap.db` doesn't exist (override with `database_path:=...`) and refuses `delete_db_on_start:=true` (you can't navigate against an empty map).

**Required apt packages** (verified by `./build.sh`):

```bash
sudo apt install \
  ros-humble-nav2-bringup ros-humble-pointcloud-to-laserscan \
  ros-humble-nav2-rviz-plugins ros-humble-slam-toolbox \
  ros-humble-rtabmap-rviz-plugins
```

`nav2-bringup` and `pointcloud-to-laserscan` are runtime requirements; the `*-rviz-plugins` packages add the **Navigation 2** and **SlamToolboxPlugin** RViz panels referenced by `rviz/perception.rviz`.

**What it adds on top of the SLAM stack:**

| Layer | Topic / Frame | Purpose |
|-------|---------------|---------|
| `pointcloud_to_laserscan` | `/cloud_registered_body` → `/scan` | 2D laserscan in `base_link`, sliced from the Mid-360 body cloud. Default slice `[scan_min_height=0.10, scan_max_height=0.45]` m above `base_link` — the rover collision band. Nav2 costmaps want LaserScan, not PointCloud2. |
| `camera_init → odom` static TF | TF | Identity alias so Nav2's default `odom` frame name resolves to FAST-LIO2's `camera_init`. |
| `nav2_bringup/navigation_launch.py` | controller_server, planner_server, smoother_server, behavior_server, bt_navigator, waypoint_follower, velocity_smoother, lifecycle_manager_navigation | The full Nav2 pipeline. **No** map_server (RTABMap publishes `/map`) and **no** amcl (RTABMap publishes the `map → camera_init` correction). |

**Launch args worth knowing:**

| Arg | Default | When to flip |
|---|---|---|
| `force_3dof` | `false` | **Set to `true` on any wheeled rover** (Roboscout/Go2/mecanum). Constrains z/roll/pitch to 0; without it FAST-LIO drifts in altitude (we saw `tf2_echo map base_link` reporting z = −4 m, pitch ≈ 17° while stationary). Default false because the bench fixture / handheld rig genuinely uses 6 DoF. |
| `scan_min_height` | `0.10` m | Lower bound of pointcloud→laserscan slice (above `base_link`). Raise above 0 to skip the floor; lower for low-clearance platforms. |
| `scan_max_height` | `0.45` m | Upper bound of slice. Default catches typical rover collision height. **Raise to ~0.8 m for Go2 standing**, **~1.2 m for the bench fixture on a table** so the laserscan still has data. |
| `localization` | `false` (start_slam) / forced `true` (start_nav) | Reuse the saved DB read-only; `Mem/IncrementalMemory: false`. |
| `delete_db_on_start` | `false` (start_slam) / forced `false` (start_nav) | `true` wipes `~/.ros/rtabmap.db` at launch — only on `start_slam.sh`, only when you really mean it. |
| `nav2` | `false` (start_slam) / forced `true` (start_nav) | Toggle Nav2 layer. |
| `nav2_params_file` | `config/nav2_params.yaml` | Per-platform overrides. |

**Send a goal:**

```bash
# RViz: enable the "2D Goal Pose" tool, click on the map.
./start_nav.sh rviz:=true force_3dof:=true

# Or from the CLI:
ros2 topic pub /goal_pose geometry_msgs/PoseStamped \
  "{header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 0.0}, orientation: {w: 1.0}}}" --once
```

Nav2 publishes `geometry_msgs/Twist` on `/cmd_vel`. The bench fixture has no consumer, so `/cmd_vel` is just for visualizing the planner's output. Real platforms need a `/cmd_vel` → drive-base bridge in their own launch file (Go2: unitree_ros2 cmd_vel adapter; mecanum: roboclaw_node; etc.).

**Frames Nav2 sees:**

```
map ──RTABMap──> camera_init ──FAST-LIO2──> body ──slam.launch.py──> base_link
                       ↓
                 (identity, nav2.launch.py)
                       ↓
                      odom
```

Nav2 plugins reference `map`, `odom`, and `base_link`. The `camera_init → odom` identity TF lets the stock plugin defaults work without per-plugin frame overrides.

**Costmap layout:** the **global costmap** uses **only `static_layer` (from `/map`) + `inflation_layer`** — the global plan follows the saved RTABMap occupancy grid. The **local costmap** uses **`obstacle_layer` (from `/scan`) + `inflation_layer`** for transient reactivity. `obstacle_layer` is INTENTIONALLY OMITTED from the global costmap because FAST-LIO drift between sessions misaligns live scans against the saved map; double-stamping would paint global plans solid magenta. See the comment block in `config/nav2_params.yaml > global_costmap`.

**Per-platform tuning:**

`config/nav2_params.yaml` ships defaults tuned for a small indoor rover: `robot_radius: 0.25`, `inflation_radius: 0.30`, `max_vel_x: 0.5`. Override per-platform with `nav2_params_file:=/path/to/<platform>_nav2_params.yaml`. Things you'll typically want to tune:

- `robot_radius` (or `footprint:` for non-circular shapes) — bigger for Go2 (~0.35), smaller for tabletop bots
- `inflation_radius` — increase for cluttered spaces, decrease if rover keeps refusing valid paths
- `max_vel_x` / `max_vel_theta` and `acc_lim_*` in the `FollowPath` (DWB) section
- `max_vel_y`: defaults to `0.3` because mecanum is the primary platform — DWB samples strafe trajectories so Nav2 picks "drive sideways toward goal" over "rotate-then-drive". Override to `0.0` (and zero out `acc_lim_y`/`decel_lim_y` and the Y entries in `velocity_smoother.max_velocity` / `min_velocity` / `max_accel` / `max_decel`) on a diff-drive platform (Go2/R2D2), otherwise DWB will emit vy that the firmware can't execute
- `velocity_smoother.max_velocity` / `max_accel` to match the controller_server limits
- `obstacle_layer.scan.max_obstacle_height` (in `local_costmap`) should match `scan_max_height` from the launch args

**Gotcha — Nav2 RViz panel "Localization: inactive" is EXPECTED.** That indicator monitors `lifecycle_manager_localization` (AMCL + map_server), which we deliberately don't run — RTABMap is our localizer. The indicator will stay grey/red the entire session; ignore it. To verify RTABMap is actually localizing, watch the `rtabmap` log for `Loop closure detected` lines or run `ros2 topic echo /rtabmap/info` in another terminal.

**What's NOT included:**

- A `/cmd_vel` consumer for the bench fixture (it has no wheels). Add a per-platform velocity bridge in the platform's own launch file.
- Recovery behaviors specific to a platform (Go2 stand-up sequence, mecanum gear shift, etc.) — extend `behavior_server.behavior_plugins` per platform.
- Speed-limit zones / dynamic obstacle layers — add `costmap_filter` or `STVL` plugins as needed.

### Mecanum drive (Yahboom YB-ERF01)

The mecanum UGV uses a Yahboom YB-ERF01-V3.0 board (STM32F103RCT6 + DRV8833) running ROSMASTER X3 firmware (`car_type=0x01` = 4-wheel mecanum). Wiring + Taranis SBUS RC config is documented in the Obsidian vault at *Robotics → Robots → Mecanum UGV → "Yahboom Mecanum Configuration - Motor Wiring and Taranis SBUS Setup"*. The driver path we use is **Path A — direct Python bridge** from *"Mecanum UGV - GitHub - AutomaticAddison ROSMASTER X3 ROS2"*: subscribe to `/cmd_vel`, call `Rosmaster_Lib.set_car_motion(vx, vy, wz)` over USB-serial. The STM32 firmware does the mecanum inverse kinematics internally.

**One-time setup on the Jetson:**

```bash
# 1. Plug the Yahboom board's USB-C to the Jetson. Verify enumeration:
ls -l /dev/serial/by-id/      # → usb-1a86_USB_Serial-if00-port0 -> ../../ttyUSB0

# 2. Pin the device path (so /dev/myserial survives reordering across boots):
sudo tee /etc/udev/rules.d/99-yahboom.rules <<'EOF'
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", SYMLINK+="myserial", MODE="0666"
EOF
sudo udevadm control --reload-rules && sudo udevadm trigger
ls -l /dev/myserial           # → ../../ttyUSB0

# 3. Install Yahboom's Python lib. Vendored in this repo at
#    vendor/Rosmaster_Lib_3.3.9/ because Yahboom doesn't publish to PyPI
#    (saves a Google Drive scavenger hunt every time). install.sh does
#    this automatically; manually:
pip3 install --user ~/slam_ws/src/slam_bringup/vendor/Rosmaster_Lib_3.3.9
python3 -c "from Rosmaster_Lib import Rosmaster; print('OK')"
```

**Bench-test BEFORE Nav2** (wheels off the ground — `Verification Order` in the Obsidian wiring note):

```bash
./start_yahboom.sh                            # bridge only, no SLAM
ros2 run teleop_twist_keyboard teleop_twist_keyboard
# Press 'b' to enable holonomic mode (mecanum strafe)
# i = forward, , = backward
# u/o    = forward-strafe-left / forward-strafe-right
# j/l    = pure yaw (left/right)
# Run through the three sanity tests:
#   vx=+0.1 → all four wheels forward
#   vy=+0.1 → MA+MD forward, MB+MC backward (strafe right)
#   vz=+0.1 → MA+MB backward, MC+MD forward (yaw left)
# Fix any wrong wheel via the firmware POLARITY FLAG, never by swapping wires.
```

**Full autonomous nav** (after bench-test passes):

```bash
./start_nav.sh platform:=mecanum force_3dof:=true enable_drive:=true
```

This launches the URDF + sensors + FAST-LIO2 + RTABMap (localization-only) + Nav2 + Yahboom bridge in one command. View from your Mac via Foxglove or RViz with the same `ROS_DOMAIN_ID` / `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`.

**Velocity caps** in `slam_bringup/yahboom_bridge_node.py`: default `max_vx=0.5`, `max_vy=0.3`, `max_wz=1.0` rad/s. Override per-launch:

```bash
./start_yahboom.sh max_vx:=0.3 max_vy:=0.2     # tighter for first runs
```

Also configured:
- **Watchdog** — if `/cmd_vel` goes silent for 0.5 s the bridge commands zero. Stops the rover dead if Nav2 / teleop crashes.
- **No telemetry republished** — the bridge does NOT publish `/odom` or `/imu/data` because FAST-LIO and Mid-360 already own those topics. Re-enabling Yahboom telemetry would create duplicate publishers.
- **SBUS RC stays live** — the Taranis path is independent of `/cmd_vel`; both feed the same firmware kinematics block. Keep RC enabled as the manual-override / kill-switch layer.
- **Chassis-frame inversion** — this rover's YB-ERF01 is mounted with firmware-+X (the direction `set_car_motion(+vx)` drives) pointing OPPOSITE to the sensor mast. The bridge corrects this with `invert_vx:=true invert_vy:=true` (defaults), so Nav2 / teleop "+vx" drives toward the sensors as expected. Do **not** try to fix this by rotating `body→base_link` — that flips Nav2's costmap, footprint orientation, and `/scan` frame and breaks navigation. If a future chassis is mounted with firmware-+X already facing the sensors, override with `./start_yahboom.sh invert_vx:=false invert_vy:=false`. `wz` is never inverted (yaw rotation is the same regardless of in-plane heading flip).
- **Nav2 mecanum config** — `config/nav2_params.yaml` enables Y-axis sampling for DWB (`max_vel_y=0.3`, `vy_samples=10`, matching `acc_lim_y`/`decel_lim_y` and Y entries in `velocity_smoother`). DWB will sample strafe trajectories so Nav2 picks the holonomic shortcut to a goal. Diff-drive platforms must override these to 0.0 in a per-platform params file.

### Map persistence and `~/.ros/rtabmap.db`

`./start_slam.sh` saves to `~/.ros/rtabmap.db` by default. `delete_db_on_start:=false` (the default) **appends** new sessions to the existing DB, so multi-day mapping accumulates without manual concatenation. Inspect a saved map with:

```bash
ls -lh ~/.ros/rtabmap.db                     # size: a multi-room map should be > 10 MB
rtabmap-info ~/.ros/rtabmap.db               # node count, link types, optimized graph error
./scripts/export_map.sh                      # export to ~/maps/<timestamp>/cloud.ply for CloudCompare
```

Snapshot before risky runs:
```bash
cp ~/.ros/rtabmap.db ~/maps/livingroom-$(date +%F).db
```

**RTABMap drift workaround for already-saved maps:** if `rtabmap-info` shows a large pre-optimization error (e.g. `Optimized graph: x=942->-3` on a 52 m trajectory) you'll see live `Loop closure ... rejected!` floods in localization mode because the corrections exceed the rejection threshold. Two fixes:

1. **Re-map with `force_3dof:=true`** — the long-term fix; drift never accumulates.
2. **The threshold has already been loosened** in `launch/rtabmap.launch.py` from `RGBD/OptimizeMaxError: 3.0` to `5.0` so legit-but-large corrections survive. This is the workaround for already-saved maps; further tuning rarely needed.

### View Mid-360 + D435 together in Foxglove / RViz

The Mid-360 publishes clouds in `livox_frame`; the D435 publishes depth + pointcloud under `camera_link`. Until Phase 1.7 wires up a full URDF-driven TF tree via `robot_state_publisher`, those frames are disconnected and a 3D panel will only render one at a time (with a "transform not available" warning on the other). `start_bench_tf.sh` publishes a single static TF between them as a stopgap:

```bash
./start_mid360.sh &                  # terminal 1 — Mid-360 driver
./start_d435.sh &                    # terminal 2 — D435 driver (raw mode → pointcloud on)
./start_bench_tf.sh                  # terminal 3 — livox_frame → camera_link static TF
```

In Foxglove's 3D panel set **Display frame** to `livox_frame`, then add topics:

- `/livox/lidar` — Mid-360 PointCloud2
- `/d435_front/camera/depth/color/points` — D435 depth-projected PointCloud2 (raw mode only)
- `/d435_front/camera/color/image_raw` — optional RGB image overlay

Edit the `X/Y/Z/ROLL/PITCH/YAW` constants at the top of `start_bench_tf.sh` to match your measured rig offsets. Re-run the script after each edit — the idempotency check swaps the new transform in cleanly without leaving the old publisher around.

### Kill scripts and orphan-process handling

Every `kill_*.sh` script routes through a shared helper, `kill_helpers.sh`, that escalates **SIGINT → SIGKILL (3× retry) → `sudo -n` SIGKILL → interactive `sudo` SIGKILL**, verifying with `pgrep` after each step. If a process survives all of that (D-state, kernel hang), the script returns exit 1 with a one-line diagnostic. `kill_slam.sh` propagates that exit so a stuck orphan blocks the next launch instead of silently coexisting with it.

The orphan classes that this prevents (and that we've actually been bitten by):

| Orphan | Symptom | Why it bites |
|---|---|---|
| Second `livox_ros_driver2_node` | FAST-LIO logs `lidar loop back, clear buffer` indefinitely; `ros2 topic info /livox/lidar` → Publisher count: 2 | Two publishers interleave scans with slightly different timestamps; FAST-LIO's `last_timestamp_lidar` ratchets ahead and rejects every scan. |
| Second `imu_units_g_to_ms2` | Same `lidar loop back` flood as above; `top` shows two `imu_units` processes (one with hours of TIME+) | Two republishers double-publish `/livox/imu_ms2` with duplicate timestamps; same FAST-LIO ratchet failure. |
| Surviving `fastlio_mapping` | Next launch fails with "topic /Odometry already exists" | DDS rejects a second publisher on the same topic. |
| Surviving `rtabmap` | `rtabmap.db` locked, next RTABMap fails on DB open | SQLite handle held by orphan. |

If you ever see `! X survived pkill -9 — escalating to sudo` in your terminal, that's the helper doing its job. If sudo prompts and you Ctrl-C, the script reports the survivor and returns 1; investigate manually with `pgrep -af <pattern>`.

**`kill_slam.sh` no longer suppresses stderr** from the layered kill scripts — the previous `2>/dev/null` was hiding exactly the warnings you needed to see when an orphan survived. Per-stage `FAILED=1` propagates up to a final exit code with a diagnostic hint.

Verify the Mid-360 driver in particular is fully gone:

```bash
pgrep -af livox                                    # no output = clean
ss -uln | grep -E ':5610|:5620|:5630|:5640'        # no listeners = sockets released
ros2 topic list | grep livox                       # (after re-sourcing) nothing
```

If `ss` still shows sockets held after the `pkill -9`, wait 30–60 s for the kernel's `SO_REUSEADDR` `TIME_WAIT` window to expire, or change the host ports in `config/mid360.json` as a last resort.

### Troubleshooting Nav2 + RTABMap on a real rover

| Symptom | Likely cause | Fix |
|---|---|---|
| `tf2_echo map base_link` reports z = −4 m or pitch = 17° while the rover is **stationary on a flat floor** | FAST-LIO is integrating bias as 6 DoF drift; RTABMap's `map → camera_init` correction can't keep up in `localization:=true` mode (no new keyframes) | `./start_nav.sh force_3dof:=true` (and re-map with the same flag for the long-term fix) |
| Every 2D Goal Pose immediately aborts with `Distance remaining: 0.00 m` | (a) RTABMap hasn't relocalized into the saved map → robot pose is at the saved-map origin instead of where it is, OR (b) costmap is solid magenta with no traversable cells | (a) Drop a **2D Pose Estimate** in RViz at your real position, or drive ~1 m to trigger a loop closure. (b) Check costmap density in RViz — see next row. |
| Costmap renders as a wall of magenta + cyan everywhere | `pointcloud_to_laserscan` slice too tall (capturing ceiling/shelves), inflation_radius too generous, or `obstacle_layer` double-stamping in global_costmap | Already tightened: `scan_max_height: 0.45`, `inflation_radius: 0.30`, global_costmap uses **only** `static_layer + inflation_layer`. Per-platform: raise `scan_max_height` if your rover is taller. |
| RTABMap log spam: `Rejecting all added loop closures... maximum graph error ratio of 3.07 of std deviation` | Saved map's pre-optimized graph drift is large enough that legit closures look "wrong" to the 3σ optimizer | Already loosened: `RGBD/OptimizeMaxError = 5.0` (was 3.0). Re-mapping with `force_3dof:=true` is the long-term fix. |
| `Rate=1.00s, RTAB-Map=3.39s, delay=3.71s` and `Message Filter dropping message: timestamp earlier than transform cache` | Jetson saturated; RTABMap processes scans 3+ s late, by which time TF rolled past | Close `rviz2` (use Foxglove from a dev machine), drop `Rtabmap/DetectionRate` 1.0 → 0.5 in `launch/rtabmap.launch.py`, check `top` for runaway competing processes. |
| Two RViz2 windows open with `rviz:=true` | Both `perception.launch.py` and upstream FAST-LIO's `mapping.launch.py` were spawning their own RViz | Already pinned: `slam.launch.py` forces FAST-LIO's rviz off. Run `./start_fast_lio.sh rviz:=true` standalone if you want the FAST-LIO debug view. |
| Nav2 panel shows `Localization: inactive` | **EXPECTED** — that indicator is for `lifecycle_manager_localization` (AMCL + map_server), which we deliberately don't run | Ignore. Verify RTABMap localization via `Loop closure detected` log lines instead. |

### D435 pitch measurement (`scripts/measure_d435_pitch.py`)

One-shot calibration that fits a plane to the floor in the D435's depth
image and reports the residual pitch error vs the current URDF value.
Useful when you swap the camera or its mount, or when the RTABMap floor
looks tilted in the 3D map. Requires the rig to be sitting on a flat,
level horizontal surface with ~1 m of clear floor in front.

**Run it** (with `./start_perception.sh` already running in another
terminal):

```bash
python3 ~/slam_ws/src/slam_bringup/scripts/measure_d435_pitch.py --ros-args \
    -p camera_namespace:=d435_front \
    -p num_frames:=30
```

Output is the current URDF pitch, the residual error, and a copy-pasteable
new value for `d435_front_rpy` in `urdf/sensors_common.urdf.xacro`.

### Map export for CloudCompare (`scripts/export_map.sh`)

`rtabmap-export` with no flags writes the *occupancy-grid voxel cloud* —
sparse and confusing in CloudCompare ("just a trail of dots along my
trajectory"). For visual map inspection you almost always want the
assembled LiDAR scan cloud projected with D435 RGB. This script wraps
the right flag combination:

```bash
./scripts/export_map.sh                    # defaults: scan + cam_projection + voxel 3 cm
./scripts/export_map.sh --voxel-size 0.01  # max detail (1 cm voxels, larger file)
./scripts/export_map.sh --no-color         # skip RGB projection (faster, LiDAR only)
./scripts/export_map.sh --db ~/.ros/rtabmap.db.bak --output ~/maps/before-fix
```

Output goes to `~/maps/<timestamp>/cloud.ply`, opened with
`cloudcompare cloud.ply` (Linux) or `open cloud.ply` after
`brew install --cask cloudcompare` on macOS. In CloudCompare:

1. In the PLY-import dialog, remove any `camera - scalex/scaley`
   entries from the Scalar fields box (those belong to the trajectory
   element, not the points), then click **Apply all**.
2. In the DB Tree, select the `vertex` cloud and press **Z** to fit
   the view onto it. If you see only a thin trajectory of dots, you're
   looking at the camera-pose element — select the larger sibling.

### IMU tilt measurement (`scripts/measure_imu_tilt.py`)

One-shot calibration of the Mid-360 IMU's mounting tilt relative to gravity.
Run this when:
- FAST-LIO2's `camera_init` z-axis comes out tilted (Foxglove grid not level
  with the floor in `camera_init`-anchored views).
- You swap or remount the sensor plate.
- The rover SLAM map shows a slowly-tilting world plane that's not real.

**Run it** with the rig **stationary** on a level horizontal surface — the
script averages 600 IMU samples and the average is meaningless if anything
is moving:

```bash
python3 ~/slam_ws/src/slam_bringup/scripts/measure_imu_tilt.py --ros-args \
    -p num_samples:=600 -p imu_topic:=/livox/imu
```

Output is roll + pitch in degrees and (if tilt > 0.3°) a 3×3 rotation matrix
to paste into `extrinsic_R` in `config/fast_lio_mid360.yaml`. If the tilt
exceeds ~2° **and** is mechanical (sensor plate visibly off-level), shim
the plate flat rather than baking the correction into `extrinsic_R`.

The script does not calibrate the IMU's bias — FAST-LIO2 estimates that
online during the ESKF settle (the ~30 s stationary period after
`./start_fast_lio.sh`) — and it does not calibrate the IMU's scale, which
is set at the Livox factory and can't be re-measured in the field.

**Mid-360 IMU unit gotcha.** The Livox driver in this stack
(`livox_ros_driver2` 1.2.6) publishes `/livox/imu` with
`linear_acceleration` in units of **g** (1.0 at rest), not m/s² as
sensor_msgs convention requires. FAST-LIO2 expects m/s². To bridge
this, `fast_lio.launch.py` spawns a small republisher
(`slam_bringup/imu_units_node.py`) that auto-detects the unit on the
first 50 messages, scales `/livox/imu` by 9.80665 if it sees g, and
republishes on `/livox/imu_ms2`. `config/fast_lio_mid360.yaml`'s
`imu_topic` points at `/livox/imu_ms2`, so FAST-LIO consumes the
corrected stream. If a future Livox driver release publishes in m/s²
natively the autodetect logs a warning and passes the messages through
unchanged — the node is safe to leave wired up. `measure_imu_tilt.py`
auto-detects the unit too, so you can run it against either topic.

### LiDAR diagnostics (`scripts/lidar_diagnostics.py`)

Standalone health-check for the Mid-360 data stream. Subscribes to `/livox/lidar` (CustomMsg or PointCloud2) and `/livox/imu` for a fixed duration, then prints a six-part report. Does NOT touch FAST-LIO2 — safe to run alongside any workflow.

**Prerequisites:**

```bash
source /opt/ros/humble/setup.bash
source ~/slam_ws/install/setup.bash   # REQUIRED so livox_ros_driver2.msg.CustomMsg imports
```

**Run it:**

```bash
# Fixture on floor (LiDAR ~0.254 m above floor)
python3 ~/slam_ws/src/slam_bringup/scripts/lidar_diagnostics.py --ros-args \
    -p duration:=10.0 -p lidar_height:=0.254 -p blind:=0.5

# Fixture on 38" table (LiDAR ~1.2 m above floor)
python3 ~/slam_ws/src/slam_bringup/scripts/lidar_diagnostics.py --ros-args \
    -p duration:=10.0 -p lidar_height:=1.2 -p blind:=0.5
```

Parameters:
- `duration` — seconds to capture (default 10)
- `lidar_height` — LiDAR height above ground in meters (used to predict blind ring radius and filter ground returns)
- `blind` — current FAST-LIO2 `blind` setting; drives the self-hit vs. near-field classification

**What each section tells you:**

| Section | What it checks | How to interpret |
|---------|----------------|------------------|
| 1. Self-hit detection | Points inside `blind` (filtered) vs points in `blind`-`blind+0.3m` band (leaking into FAST-LIO2) | Near-field points clustered in a single azimuth sector = fixture reflections; raise `blind`. Spread evenly = legitimate room returns; leave `blind` alone. |
| 2. Ground return analysis | Expected blind-ring radius vs. observed ground returns | `lidar_height / tan(7°)` = nearest ground radius. No returns = room smaller than that, FAST-LIO2 has no Z-constraint from ground. |
| 3. Feature density | Raw points/scan, useful points after `blind` filter | <2000 useful = FAST-LIO2 will struggle. 15k+ = healthy. |
| 4. IMU/LiDAR timestamps | Captured rates, period jitter, LiDAR↔IMU stamp offset | Script capture rate is GIL-limited (~100 Hz IMU, ~3 Hz LiDAR) — those numbers are lower bounds, not hardware truth. FAST-LIO2 receives full 200/10 Hz. Only flag >50 ms stamp offset if capture rate is ≥150 Hz. |
| 5. Azimuth occlusion map | Points per 10° bin, excluding self-hits | Bins <30% of mean = occluded sector (fixture shadow, body part blocking scan, obstacle in FOV). Even distribution = clean horizontal coverage. |
| 6. Range histogram | Point distribution by distance | Tells you the scale of the surrounding structure. Room with walls at 2-5 m will peak there; empty open space shows sparse distant bins. |

**Findings from the 2040 fixture test session (2026-04-23):**

Running on the benchtop (fixture LiDAR ~1.2 m above floor) with the original `blind: 0.2` revealed two real problems masked by the filter config:

1. **Fixture ghost points** — 2,975 points per 10 s in the 0.2-0.5 m band were passing through `blind: 0.2` and entering the Kalman filter. The azimuth correlation showed they clustered in two sectors (-90° to -60° and +120° to +150°), which matched the 2040 extrusion frame members and the battery/buck-converter assembly. These ghost points were corrupting the pose update.
2. **Missing ground plane** — On the table, the expected blind-ring radius was 9.77 m; the room was smaller, so zero ground returns arrived. FAST-LIO2 had no Z-constraint from floor geometry, so accelerometer bias started driving the pose downward after ~20 seconds.

Fix was to raise `blind` to 0.5 in `config/fast_lio_mid360.yaml`:

- Catches ALL fixture hits (verified: 0 points leak through in follow-up runs)
- Does NOT cost any useful data — on the floor the nearest legitimate ground return is at 2.07 m (well beyond 0.5 m blind), on the table the nearest structure is ~1 m away
- Post-fix floor test showed 8,197 ground returns detected (nearest 1.59 m, median 3.19 m) giving FAST-LIO2 a solid Z anchor
- FAST-LIO2 then ran stable for 3+ minutes with no visible drift (vs. 20 s before)

**Root-cause lesson:** `blind` is not just a self-hit filter — for a compact rig, it's the line between "noise from my own frame corrupts the state estimate" and "clean room returns drive the filter." Set `blind` ≥ the largest radial distance from the LiDAR optical center to any component in the fixture (frame, plate, battery, cabling). Verify with this diagnostic whenever the rig geometry changes.

See `docs/test_fixture.md` for fixture dimensions, derived geometry at each mounting position, and test history.

## Status

Phase 3 (Navigation) landed 2026-05-02. Detailed task checklist in [PLAN.md](./PLAN.md).

- [x] Phase 0 — hardware + Jetson prereqs
- [x] Phase 0.5 — repo + Mac↔Jetson workflow
- [x] Phase 1.1–1.2 — `install.sh` + package skeleton
- [x] Phase 1.3 — Mid-360 standalone
- [x] Phase 1.4 — D435 front standalone (rear launch scaffolded; dual is Phase 1.10)
- [x] Phase 1.5 — WitMotion WT901C (custom 0x61 Python node, 200 Hz on `/imu/data`)
- [x] Phase 1.6 — `sensors.launch.py` integration (single-command bring-up of all three sensors)
- [x] Phase 2.2 — FAST-LIO2 (`/Odometry` at 10 Hz, `/cloud_registered` accumulating in `camera_init`)
- [x] Phase 2.4 — RTABMap graph SLAM + visual loop closure (`./start_slam.sh`)
- [x] Phase 3 — Nav2 layer on top of FAST-LIO2 + RTABMap (`./start_nav.sh`, see [Navigation (Nav2)](#navigation-nav2))
  - [x] `pointcloud_to_laserscan` from `/cloud_registered_body` → `/scan`
  - [x] `force_3dof` launch arg for wheeled rovers
  - [x] `kill_helpers.sh` orphan-process escalation across all `kill_*.sh`
  - [x] **Mecanum** `/cmd_vel` bridge — Yahboom YB-ERF01 / ROSMASTER X3 via USB-serial (`./start_yahboom.sh`, `enable_drive:=true`). Path A (direct Python bridge) per the Obsidian AutomaticAddison note. URDF + measured `body→base_link` offset for `mecanum` platform pending; current PLATFORM_BRIDGES entry is a placeholder.
  - [ ] **Go2** / **R2D2** / **Roboscout** `/cmd_vel` bridges
  - [ ] Per-platform `nav2_<platform>_params.yaml` files
- [ ] Phase 1.7 — URDF (`go2`, `r2d2`, `roboscout`, `mecanum`) — bench_fixture done, others stubbed
- [ ] Phase 1.8 — Go2 SDK integration check
- [ ] Phase 1.9 — `install.sh` smoke test
- [ ] Phase 1.10 — Dual camera (rear D435i)
- [ ] Phase 2.5 — MCAP recording / playback / Foxglove

## Source note

Detailed background, tuning rationale, and hardware history live in the Obsidian vault:
`rico/Robotics/Nvidia Jetson/Jeston Orin Nano Super/Jetson Orin Nano Go2 ROS2 Sensor Bringup.md`
