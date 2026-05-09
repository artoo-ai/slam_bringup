# RViz configs cheat sheet

Each top-level start script auto-loads a tailored RViz config when you pass `rviz:=true`. Override with `rviz_config:=/path/to/your.rviz` if you want something custom.

| Start script | Config | What's on by default |
|---|---|---|
| `./start_slam.sh` | `slam.rviz` | URDF, /map (RTABMap), /cloud_viz_clipped, /Odometry trail |
| `./start_slam_2d.sh` | `slam_2d.rviz` | URDF, /map (slam_toolbox), /scan (cyan), /odom (rf2o), **SlamToolbox panel** |
| `./start_nav.sh` | `nav.rviz` | slam.rviz superset + costmaps, /plan, /local_plan, **Navigation 2 panel** |
| `./start_nav_2d.sh` | `nav_2d.rviz` | slam_2d.rviz superset + costmaps, /plan, /local_plan, **Navigation 2 + SlamToolbox panels** |

`perception.rviz` is the original "everything-on" config kept for `start_perception.sh` / `start_sensors.sh` and pure debug.

---

## Per-config quick reference

### slam.rviz — 3D mapping (FAST-LIO2 + RTABMap)

**Use when:** `./start_slam.sh` is running and you want to watch the 3D map build.

| Display | Topic | Color/Style | On? |
|---|---|---|---|
| Map | `/map` | grayscale occupancy grid | ✓ |
| Cloud (world) | `/cloud_viz_clipped` | rainbow by Z | ✓ |
| Odometry | `/Odometry` | red arrows | ✓ |
| RobotModel | `/robot_description` | URDF mesh | ✓ |
| LiDAR (live) | `/livox/lidar` | flat gray points | off |
| LaserScan (Nav2) | `/scan` | cyan | off |
| D435 Front Pointcloud | `/d435_front/.../points` | RGB | off |

No Nav2 / SlamToolbox panel — this view is for verifying mapping geometry only. Send `2D Pose Estimate` if you want to seed RTABMap relocalization.

### slam_2d.rviz — 2D mapping (slam_toolbox + rf2o)

**Use when:** `./start_slam_2d.sh` is running and you want to drive around to build a map, then **save it via the SlamToolbox panel**.

| Display | Topic | Color/Style | On? |
|---|---|---|---|
| Map | `/map` | grayscale occupancy grid | ✓ |
| LaserScan (live) | `/scan` | cyan single-line ring | ✓ |
| Odometry (rf2o) | `/odom` | green arrows | ✓ |
| RobotModel | `/robot_description` | URDF mesh | ✓ |
| LiDAR (3D context) | `/livox/lidar` | rainbow by Z | off |
| D435 Front Pointcloud | `/d435_front/.../points` | RGB | off |

**Saving the map:** in the SlamToolbox panel, type an absolute path in the filename field (e.g. `/home/rico/maps/livingroom`), then click **Serialize Map**. Produces `livingroom.data` + `livingroom.posegraph` — the pair `start_nav_2d.sh` loads.

> "Save Map" produces `.pgm`/`.yaml` instead, which is NOT what slam_toolbox localization mode wants. Use Serialize Map.

### nav.rviz — 3D navigation (RTABMap localization + Nav2)

**Use when:** `./start_nav.sh` is running.

Adds to slam.rviz (all on by default):
- Global Costmap (`/global_costmap/costmap`, rainbow heatmap)
- Local Costmap (`/local_costmap/costmap`, rolling robot-centered)
- Plan (`/plan`, green line)
- Local Plan (`/local_plan`, blue line)
- LaserScan (Nav2) (`/scan`, cyan)
- **Navigation 2 panel** (lifecycle + goal feedback)

**Send a goal:** click **2D Goal Pose** tool → click+drag on the map.

**Relocalize:** click **2D Pose Estimate** → click+drag the rover's actual location. Publishes `/initialpose` for RTABMap.

### nav_2d.rviz — 2D navigation (slam_toolbox localization + Nav2)

**Use when:** `./start_nav_2d.sh map_file:=~/maps/livingroom` is running.

Adds to slam_2d.rviz (all on by default):
- Global Costmap, Local Costmap, Plan, Local Plan
- **Navigation 2 panel**
- **SlamToolbox panel** still available (useful for "Deserialize Map" or manual loop closure debug)

**Mecanum quirk to expect:** ~60 s of in-place yawing/sliding at goal before "Goal Reached." The Plan/Local Plan keep redrawing during that — that's the controller fighting mecanum static friction. Not a bug. See `docs/why_slam_is_hard_and_how_to_simplify.md` §10/§11.

---

## Common toggles

- **Hide the 3D cloud, keep the 2D scan:** turn off "Cloud (world)" / "LiDAR (3D context)" — the 2D scan ring stays.
- **Turn off costmaps for a cleaner picture:** untick Global Costmap and Local Costmap. /map and /scan are usually all you need.
- **Increase scan visibility:** in LaserScan display, bump Size (Pixels) from 4 → 8.

## Common gotchas

### "0 publishers" on a topic that's clearly running

QoS mismatch. RViz2 does not auto-detect publisher QoS. Each display has a `Reliability Policy` field:

| Topic | QoS to set in RViz |
|---|---|
| `/map`, `/global_costmap/costmap`, `/local_costmap/costmap` | Reliable + Transient Local |
| `/scan`, `/livox/lidar`, `/cloud_viz_clipped`, `/Odometry`, D435 cloud | Best Effort |
| `/odom` (rf2o), `/plan`, `/local_plan` | Reliable |

The configs in this folder pre-set these. If you add a new display via the GUI, double-check the Reliability Policy.

### "No tf data. Actual error: Frame [map] does not exist"

Either you set Fixed Frame to `map` before the SLAM stack came up, or the stack hasn't published `map → odom` yet. Wait ~5 s after launch, then click any Display checkbox off+on to re-subscribe. If still nothing, `slam_toolbox` (or RTABMap) didn't start — check the launch terminal for errors.

### RViz crashes on launch with "could not load plugin slam_toolbox::SlamToolboxPlugin"

`apt install ros-humble-slam-toolbox` (already in `install.sh` as of 2026-05-08).

### RViz crashes with "could not load plugin nav2_rviz_plugins/Navigation 2"

`apt install ros-humble-nav2-rviz-plugins`.

---

## Customizing without losing changes

The configs are tracked in git. If you tweak displays in RViz and want to keep them, **save to a different filename** (`File → Save Config As → my_custom.rviz`), then run with `rviz_config:=$(pwd)/my_custom.rviz`. Don't overwrite the tracked files unless your change is generally useful — otherwise you'll have a noisy `git diff` every session.
