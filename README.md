# slam_bringup

Portable sensor-rig SLAM stack for ROS2 Humble. One NVIDIA Jetson Orin Nano Super + one sensor plate moves between multiple mobile robots; the package selects per-platform URDF + TF bridge offsets at launch time.

## Startup commands (current — Phase 2.2)

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

This section tracks the **latest working command set** for whatever phase is current. As Phase 2.4+ comes online, lines get added or replaced (eventually `slam.launch.py platform:=...`). Check the [Status](#status) section for what's wired up.

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
| `./start_slam.sh` | Launch the full SLAM stack: URDF + sensors (slam_mode) + FAST-LIO2 + RTABMap + per-platform `body→base_link` bridge. Pass-through args: `platform:=`, `delete_db_on_start:=`, `localization:=`, `rviz:=`. |
| `./kill_slam.sh` | Force-kill every layer of the SLAM stack. |
| `./start_bench_tf.sh` | Publish `livox_frame → camera_link` static TF for multi-sensor visualization (see below) |
| `./start_foxglove.sh` | Start `foxglove_bridge` on the Jetson for remote Studio/App connections |

### Top-down floorplan view (`/cloud_viz_clipped`)

Both `./start_slam.sh` and `./start_fast_lio.sh` run a `pcl_ros::PassThrough` filter alongside the rest of the pipeline that republishes a z-clipped copy of `/cloud_registered` on `/cloud_viz_clipped` — purely for visualization. The raw cloud still flows into FAST-LIO2 / RTABMap unchanged, so ICP, the occupancy grid, and `/octomap_full` keep their full vertical extent (you can still measure rafters in the 3D map).

In Foxglove / RViz, set the **Display frame** to `camera_init` and add `/cloud_viz_clipped` as a PointCloud2 display. With the ceiling clipped you can sit the camera straight above and see the floorplan with furniture, walls, and doorways visible inside each room.

`z = 0` is the body pose at FAST-LIO2 startup (i.e. the IMU position when you launched), **not** the floor. Tune `viz_z_max` for the rig's startup height and the space:

```bash
./start_slam.sh                                # default: viz_z_max=2.0  (typical house)
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

### Stopping the Mid-360 driver

Ctrl-C does not always fully stop `livox_ros_driver2_node` — it can leave an orphan process holding UDP ports 56101/56201/56301/56401. Symptoms: the next `ros2 launch slam_bringup mid360.launch.py` starts but `ros2 topic info /livox/lidar` shows two publishers, or the driver fails to bind its listener sockets. Clean kill:

```bash
pkill -SIGINT -f livox_ros_driver2_node     # try graceful first
sleep 2
pkill -9      -f livox_ros_driver2_node     # nuke if still alive
pkill -9      -f "ros2 launch slam_bringup mid360" # and the launch wrapper
ros2 daemon stop                            # clear stale DDS discovery cache
```

Verify everything is gone:

```bash
pgrep -af livox                                    # no output = clean
ss -uln | grep -E ':5610|:5620|:5630|:5640'        # no listeners = sockets released
ros2 topic list | grep livox                       # (after re-sourcing) nothing
```

If `ss` still shows sockets held after the `pkill -9`, wait 30–60 s for the kernel's `SO_REUSEADDR` `TIME_WAIT` window to expire, or change the host ports in `config/mid360.json` as a last resort.

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

Phase 1 in progress. Detailed task checklist in [PLAN.md](./PLAN.md).

- [x] Phase 0 — hardware + Jetson prereqs
- [x] Phase 0.5 — repo + Mac↔Jetson workflow
- [x] Phase 1.1–1.2 — `install.sh` + package skeleton
- [x] Phase 1.3 — Mid-360 standalone
- [x] Phase 1.4 — D435 front standalone (rear launch scaffolded; dual is Phase 1.10)
- [x] Phase 1.5 — WitMotion WT901C (custom 0x61 Python node, 200 Hz on `/imu/data`)
- [x] Phase 1.6 — `sensors.launch.py` integration (single-command bring-up of all three sensors)
- [x] Phase 2.2 — FAST-LIO2 (`/Odometry` at 10 Hz, `/cloud_registered` accumulating in `camera_init`) *— jumped ahead of Phase 1.7–1.10 since URDF is only needed for the RTABMap bridge*
- [ ] Phase 1.7 — URDF (`go2`, `r2d2`, `roboscout`, `mecanum`) + `perception.launch.py` + rviz
- [ ] Phase 1.8 — Go2 SDK integration check
- [ ] Phase 1.9 — `install.sh` smoke test
- [ ] Phase 1.10 — Dual camera (rear D435i)
- [ ] Phase 2 — FAST-LIO2 + RTABMap SLAM
- [ ] Phase 2.5 — MCAP recording / playback / Foxglove

## Source note

Detailed background, tuning rationale, and hardware history live in the Obsidian vault:
`rico/Robotics/Nvidia Jetson/Jeston Orin Nano Super/Jetson Orin Nano Go2 ROS2 Sensor Bringup.md`
