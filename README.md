# slam_bringup

Portable sensor-rig SLAM stack for ROS2 Humble. One NVIDIA Jetson Orin Nano Super + one sensor plate moves between multiple mobile robots; the package selects per-platform URDF + TF bridge offsets at launch time.

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
| `./start_bench_tf.sh` | Publish `livox_frame → camera_link` static TF for multi-sensor visualization (see below) |
| `./start_foxglove.sh` | Start `foxglove_bridge` on the Jetson for remote Studio/App connections |

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

## Status

Phase 1 in progress. Detailed task checklist in [PLAN.md](./PLAN.md).

- [x] Phase 0 — hardware + Jetson prereqs
- [x] Phase 0.5 — repo + Mac↔Jetson workflow
- [x] Phase 1.1–1.2 — `install.sh` + package skeleton
- [x] Phase 1.3 — Mid-360 standalone
- [x] Phase 1.4 — D435 front standalone (rear launch scaffolded; dual is Phase 1.10)
- [ ] Phase 1.5 — WitMotion
- [ ] Phase 1.6 — `sensors.launch.py` integration
- [ ] Phase 1.7 — URDF (`go2`, `r2d2`, `roboscout`, `mecanum`) + `perception.launch.py` + rviz
- [ ] Phase 1.8 — Go2 SDK integration check
- [ ] Phase 1.9 — `install.sh` smoke test
- [ ] Phase 1.10 — Dual camera (rear D435i)
- [ ] Phase 2 — FAST-LIO2 + RTABMap SLAM
- [ ] Phase 2.5 — MCAP recording / playback / Foxglove

## Source note

Detailed background, tuning rationale, and hardware history live in the Obsidian vault:
`rico/Robotics/Nvidia Jetson/Jeston Orin Nano Super/Jetson Orin Nano Go2 ROS2 Sensor Bringup.md`
