# `start_slam.sh` — full SLAM stack

Builds the layered SLAM bring-up in one command: URDF + sensors + FAST-LIO2 + RTABMap, and saves the resulting map to `~/.ros/rtabmap.db`. Use this to **make a new map** or **append to an existing one**. For navigating *inside* a saved map, use [`start_nav.sh`](start_nav.md) instead.

## What it does

1. Kills every layer of the stack first (idempotent — safe to re-run).
2. Launches `slam.launch.py` which composes:
   - `perception.launch.py` (URDF via `robot_state_publisher` + all three sensors in `slam_mode`)
   - Per-platform `body → base_link` static TF
   - `fast_lio.launch.py` (LiDAR-inertial odometry → `/Odometry`, `/cloud_registered`, `/cloud_registered_body`)
   - `imu_units_g_to_ms2` (Livox `/livox/imu` is in g; rescaled to m/s² on `/livox/imu_ms2`)
   - `viz_clip` (z-clipped `/cloud_viz_clipped` for top-down floorplan view)
   - `rtabmap.launch.py` (graph SLAM, loop closure, `/map` occupancy grid)

After ~30 s of stationary settle, `/Odometry` publishes at 10 Hz and `/map` starts filling in. Drive / walk the rig to extend the map.

## Dependencies

- All three sensors physically connected (Mid-360 over Ethernet, D435 over USB, WitMotion over USB-serial).
- ROS 2 Humble + the workspace built (`./build.sh`).
- apt: `ros-humble-rtabmap-ros`, RealSense, Livox SDK2 — installed by `./install.sh`.
- For wheeled rovers: see [`start_nav.sh`](start_nav.md) once the map exists.

No other scripts need to be running first — `start_slam.sh` brings up the whole chain itself.

## Default usage

```bash
./start_slam.sh
```

What you get:
- Platform: `bench_fixture` (the standalone test rig)
- DB: `~/.ros/rtabmap.db` — **appended** to if it exists
- 6 DoF pose estimation (use `force_3dof:=true` on wheeled rovers)
- No RViz on the Jetson (run RViz on your dev machine)
- Top-down `/cloud_viz_clipped` enabled with z range `[-3.0, 3.0]` m
- Nav2 disabled

## Parameters

| Arg | Default | What it does | When to override |
|---|---|---|---|
| `platform` | `bench_fixture` | Selects URDF + per-platform `body→base_link` static TF from `PLATFORM_BRIDGES` in `launch/slam.launch.py`. | `mecanum` for the Yahboom rover (placeholder offset — measure to make accurate). Other platforms (`go2`, `r2d2`, `roboscout`) are stubbed; passing them errors out with the exact instruction to add a `PLATFORM_BRIDGES` entry. |
| `delete_db_on_start` | `false` | Wipe `~/.ros/rtabmap.db` at launch. **Appending** is the default — multiple sessions accumulate into one DB. | `true` ONLY when you really mean to throw away the saved map. `start_nav.sh` rejects this. |
| `localization` | `false` | RTABMap loads the DB read-only (`Mem/IncrementalMemory: false`) — no new keyframes. | `true` when you want to navigate against a saved map without modifying it. `start_nav.sh` forces this on. |
| `force_3dof` | `false` | RTABMap clamps z/roll/pitch to 0 in the loop-closure optimizer. | **Always `true` on wheeled rovers** — without it, FAST-LIO drifts in altitude/tilt and `tf2_echo map base_link` reports nonsense z. Leave `false` only on the bench fixture / handheld rig. |
| `rviz` | `false` | Spawn `rviz2` on the Jetson. | Almost never — Jetson CPU is precious. Run RViz on your dev machine over the LAN. The fast_lio rviz spawn is pinned off here so you don't get two windows. |
| `nav2` | `false` | Include `nav2.launch.py` in the same launch. | `true` (paired with `localization:=true`) when you want one command for navigation. `start_nav.sh` is the convenience wrapper for this. |
| `nav2_params_file` | `config/nav2_params.yaml` | Path to Nav2 YAML. | Per-platform tuning — point at e.g. `~/cfg/mecanum_nav2.yaml`. |
| `enable_drive` | `false` | Spawn the per-platform `/cmd_vel → drive base` bridge. Mecanum-only today. | `true` on the mecanum rover — exposes the Yahboom bridge. |
| `database_path` | `~/.ros/rtabmap.db` | Where RTABMap writes/reads the graph + visual word DB. | Per-map snapshots: `~/maps/livingroom-2026-05-03.db`. |
| `enable_viz_clip` | `true` | Run the z-clip republisher to publish `/cloud_viz_clipped`. | `false` if you only want the raw `/cloud_registered`. |
| `viz_z_min` | `-3.0` | Lower z bound (m) of the visualization clip in `camera_init` frame. | Raise to skip the floor in odd cases. |
| `viz_z_max` | `3.0` | Upper z bound (m). | `4.5` for high-ceiling spaces (garage, shop). |
| `use_sim_time` | `false` | Subscribe `/clock` from a bag replay. | `true` only with `ros2 bag play`. |

## Examples

```bash
# First time mapping a new space — wipe and start fresh:
./start_slam.sh delete_db_on_start:=true

# Mapping on the mecanum rover (CRITICAL: force_3dof):
./start_slam.sh platform:=mecanum force_3dof:=true delete_db_on_start:=true

# Append a new room to an existing map (default behaviour, called out for clarity):
./start_slam.sh

# High-ceiling space — bump the z-clip ceiling so you don't lose the cloud:
./start_slam.sh viz_z_max:=4.5

# Snapshot map to a named DB instead of clobbering ~/.ros/rtabmap.db:
./start_slam.sh database_path:=~/maps/lab-2026-05-03.db delete_db_on_start:=true

# Navigation in one command instead of using start_nav.sh:
./start_slam.sh nav2:=true localization:=true force_3dof:=true platform:=mecanum enable_drive:=true
```

## After Ctrl-C

RTABMap saves to `database_path` on shutdown. To verify:

```bash
ls -lh ~/.ros/rtabmap.db          # > 1 MB for a real map
rtabmap-info ~/.ros/rtabmap.db    # node count + optimized graph error
```

A small *Optimized graph: x* range (e.g. `x=10->-5`) means low FAST-LIO drift. A large one (e.g. `x=942->-3`) means heavy drift was loop-closed back into shape — re-map with `force_3dof:=true` if you're on a wheeled rover.

## See also

- [start_nav.md](start_nav.md) — once the map exists, navigate inside it
- [start_fast_lio.md](start_fast_lio.md) — LiDAR-inertial odometry alone
- [start_rtabmap.md](start_rtabmap.md) — RTABMap alone (assumes FAST-LIO already up)
- Main README §[Map persistence](../../README.md#map-persistence-and-rosrtabmapdb)
