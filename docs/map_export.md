# Map Export & Viewing

How to get an RTABMap-built map out of `~/.ros/rtabmap.db` into formats you can view, share, or process in other tools (CloudCompare, MeshLab, Open3D, Foxglove, RViz, web viewers, etc.).

## TL;DR

```bash
./scripts/export_map.sh                          # defaults — dense LiDAR cloud + RGB → ~/maps/<timestamp>/cloud.ply
open ~/maps/<timestamp>/cloud.ply                # macOS CloudCompare
cloudcompare ~/maps/<timestamp>/cloud.ply        # Linux CloudCompare
```

## What's in `~/.ros/rtabmap.db`?

A single SQLite file containing:
- **Pose graph** — keyframes + neighbour links + loop closures (the optimized trajectory)
- **Sensor data per keyframe** — RGB image, depth image, LiDAR scan, camera intrinsics
- **Occupancy grid** — the 2D `/map` Nav2 uses
- **Visual word dictionary** — RTABMap's bag-of-words for loop closure
- **Settings** — every Reg/* / Grid/* / RGBD/* parameter that was active when the map was built

Inspect with:
```bash
ls -lh ~/.ros/rtabmap.db                # size — multi-room map ≥ 10 MB
rtabmap-info ~/.ros/rtabmap.db          # node count, link types, optimized graph error
rtabmap-databaseViewer ~/.ros/rtabmap.db   # GUI keyframe browser (heavy)
```

## Snapshot before risky operations

```bash
cp ~/.ros/rtabmap.db ~/maps/livingroom-$(date +%F).db
```

The DB is ALWAYS the source of truth — exports below are derived; you can always re-export with different settings later.

---

## `./scripts/export_map.sh` — the wrapper

Tuned for **dense LiDAR cloud + D435 RGB projection**, which is what you almost always want for visual map inspection.

### Default usage

```bash
./scripts/export_map.sh
```

What you get:
- Source: `~/.ros/rtabmap.db`
- Output: `~/maps/<timestamp>/cloud.ply`
- Cloud: assembled LiDAR scan (`--scan`) — dense, the actual map
- Color: D435 keyframe RGB projected onto the cloud (`--cam_projection`)
- Voxel grid: 3 cm (`--voxel 0.03`) — ~10× smaller file with no visible loss
- Max range: 30 m (`--max_range 30`) — drops noise past indoor distances

### Flags

| Flag | Default | What it does |
|---|---|---|
| `--db PATH` | `~/.ros/rtabmap.db` | Source DB. |
| `--output DIR` | `~/maps/<timestamp>` | Output folder (created). |
| `--voxel-size N` | `0.03` m | Voxel grid downsampling. `0.0` = no downsampling. `0.01` for max detail (bigger file). |
| `--max-range N` | `30` m | Drop returns past N meters. |
| `--no-color` | (RGB on) | Skip `--cam_projection` — faster, no RGB. |
| `--help` | — | Print help. |

### Examples

```bash
./scripts/export_map.sh --voxel-size 0.01            # max detail (1 cm voxels)
./scripts/export_map.sh --voxel-size 0.0             # no downsampling at all
./scripts/export_map.sh --no-color                   # plain LiDAR, no RGB projection
./scripts/export_map.sh --db ~/maps/lab.db           # alternate DB
./scripts/export_map.sh --max-range 10               # drop very far returns
```

### Why `--scan` and not the default `rtabmap-export`?

`rtabmap-export` with no flags exports the **occupancy-grid voxel cloud** — sparse, looks like a trail of dots in CloudCompare. `--scan` exports the assembled LiDAR sweeps, which is what you want for visual inspection.

---

## Direct `rtabmap-export` reference

The wrapper covers 95% of cases; for advanced exports go direct.

```bash
rtabmap-export [flags] DATABASE.db
```

### Output flags (pick one or more)

| Flag | Output |
|---|---|
| (none) | Occupancy-grid voxels (sparse — usually not what you want) |
| `--scan` | Assembled LiDAR cloud (dense — recommended for visual inspection) |
| `--cloud` | Per-keyframe local depth/LiDAR clouds (denser than scan in places, but stitched per-keyframe) |
| `--mesh` | Triangle mesh from the cloud — **fragile**, can crash; see [Gotchas](#gotchas) |
| `--mesh --texture` | Mesh with D435 RGB texture — **fragile** |
| `--ground` | Just the ground plane points |
| `--obstacles` | Just the non-ground points |

### Color flags

| Flag | Effect |
|---|---|
| `--cam_projection` | Project D435 keyframe RGB onto the cloud's points (works with `--scan` and `--cloud`) |
| `--cam_projection_keep_all` | Keep points outside any camera FOV (default drops them) |

### Filtering

| Flag | Effect |
|---|---|
| `--max_range N` | Drop returns farther than N m |
| `--min_range N` | Drop returns closer than N m |
| `--voxel N` | Voxel-grid downsample to N m cells (0 = off) |
| `--noise_radius N` | Statistical outlier removal radius |
| `--noise_min_neighbors N` | Min neighbours within radius (companion to above) |

### Output format

`rtabmap-export` always writes **PLY** to `<output_dir>/<basename>.ply`. To convert:

```bash
# CloudCompare — load PLY, File → Save As → PCD/LAS/E57/etc.
cloudcompare cloud.ply

# Open3D — Python conversion to anything:
python3 -c "
import open3d as o3d
pc = o3d.io.read_point_cloud('cloud.ply')
o3d.io.write_point_cloud('cloud.pcd', pc)        # PCD
o3d.io.write_point_cloud('cloud.xyz', pc)        # ASCII XYZ
o3d.io.write_point_cloud('cloud.ply', pc, write_ascii=True)   # ASCII PLY
"

# pcl_tools (apt install pcl-tools):
pcl_ply2pcd cloud.ply cloud.pcd
```

### Standalone `rtabmap-info`

```bash
rtabmap-info ~/.ros/rtabmap.db          # text summary
```

Output to look at:
- **`Sessions:`** — number of separate mapping runs in this DB.
- **`Total odometry length:`** — accumulated trajectory.
- **`Optimized graph: N poses (x=A->B, y=C->D, z=E->F)`** — pre/post optimization error in each axis. Big numbers (e.g. `x=942->-3`) = severe FAST-LIO drift. Small numbers (`x=10->-2`) = clean map.
- **Link counts** — `Neighbor`, `GlobalClosure`, `LocalSpaceClosure`. More closures = more confident map.
- **Database size breakdown** — RGB images / Depth / Grids / Scans / Features.

---

## Viewers

### CloudCompare (recommended for one-shot inspection)

Free, cross-platform, handles PLY natively, has measurement / cropping / outlier-removal tools.

```bash
# macOS
brew install --cask cloudcompare
open ~/maps/<timestamp>/cloud.ply

# Linux
sudo snap install cloudcompare
cloudcompare ~/maps/<timestamp>/cloud.ply
```

**PLY-import dialog gotcha:** PLY files contain multiple "elements" — `vertex` (the cloud) and `camera` (trajectory poses). The dialog defaults to showing scalar fields from the camera element, which is just the trajectory line. If you see only a "thin line of dots":

1. In the import dialog, **remove** any `camera - scalex/scaley` entries from the *Scalar fields* box.
2. Click **Apply all**.
3. In the DB Tree (left panel), select the `vertex` cloud (the larger sibling).
4. Press **Z** to fit view onto it.

Useful CloudCompare actions for SLAM cloud inspection:
- **Display point size**: Properties → Default point size → 3 or 4 (helps when points are sparse).
- **Octree-based culling**: Tools → Octree → Compute, then use crop tools.
- **Statistical outlier removal**: Tools → Clean → SOR Filter.
- **Distance to mesh**: Tools → Distance → Cloud/Mesh Dist (compare two scans).

### MeshLab (if you exported a `--mesh`)

Best for triangle-mesh inspection / texturing.

```bash
brew install --cask meshlab        # macOS
sudo apt install meshlab           # Linux
meshlab cloud.ply
```

### Open3D (Python — programmatic + scripted views)

Best when you want to crop / filter / compare clouds in code:

```python
import open3d as o3d
pc = o3d.io.read_point_cloud('cloud.ply')
print(pc)                                          # point count
o3d.visualization.draw_geometries([pc])           # interactive viewer

# Bounding-box crop:
bbox = o3d.geometry.AxisAlignedBoundingBox(
    min_bound=(-5, -5, -1), max_bound=(5, 5, 2))
cropped = pc.crop(bbox)

# Voxel downsampling:
ds = pc.voxel_down_sample(voxel_size=0.05)

# Statistical outlier removal:
clean, _ = pc.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
```

### Foxglove Studio (live + saved bag clouds)

Foxglove can render PointCloud2 from a live ROS topic OR from a recorded MCAP/bag. To view a saved cloud as a topic, replay it via a small publisher node, OR drag the PLY into Foxglove's File panel (newer builds support PLY directly).

For LIVE accumulation as you map, set the PointCloud2 display's **Decay time** to a high value (e.g. `1e9`) on `/cloud_registered` — Foxglove accumulates per-scan clouds client-side without growing Jetson memory.

### RViz2

```bash
ros2 launch slam_bringup slam.launch.py             # in another terminal, on the Jetson
ros2 run rviz2 rviz2 -d ~/slam_ws/src/slam_bringup/rviz/perception.rviz
```

For a SAVED PLY in RViz, install `pcl_ros`:
```bash
sudo apt install ros-humble-pcl-ros
ros2 run pcl_ros pcd_to_pointcloud cloud.pcd 0.1 _frame_id:=map _topic:=/saved_cloud
# Then add a PointCloud2 display on /saved_cloud in RViz.
```

### Potree / web viewer

For sharing a map with someone who doesn't have CloudCompare, build a Potree web viewer from the PLY.

```bash
# One-time install
git clone https://github.com/potree/PotreeConverter
# Build per the repo's README

# Convert
PotreeConverter cloud.ply -o web_map/ --output-format BINARY

# Serve
cd web_map && python3 -m http.server 8000
# Browse http://localhost:8000/
```

Result: streaming web viewer that handles billion-point clouds smoothly. Good for sending to non-roboticists.

---

## Other useful exports

### 2D occupancy grid as PNG

```bash
# Use map_server's saver against /map (must be active)
ros2 run nav2_map_server map_saver_cli -f ~/maps/livingroom_2d
# Produces livingroom_2d.pgm (image) + livingroom_2d.yaml (metadata)
# Open the .pgm in any image viewer.
```

### Bag-style replay of the original sensor stream

Not stored in the RTABMap DB — you'd need an MCAP/ROS bag of the
original `/livox/lidar` + `/d435/...` topics for that. See PLAN.md
§Phase 2.5 for the recording workflow (`record_sensors.sh`, planned).

### Trajectory export

```bash
rtabmap-export --poses 0 -p path.txt ~/.ros/rtabmap.db
# Writes a TUM-format trajectory: timestamp x y z qx qy qz qw
```

Useful for evo-eval comparisons, ground-truth alignment, or feeding
into other SLAM systems for benchmarking.

---

## Gotchas

### `rtabmap-export --cam_projection` crashes mid-export

```
vector::_M_range_check: __n (which is 18446744073709551615) >= this->size()
```

Upstream RTABMap bug (≥ 0.21). Crashes around frame 147/1019 on long
sessions. **Workaround:** `--no-color` (plain LiDAR cloud).

### `rtabmap-export --mesh --texture` fails

```
createTextureMesh() Condition (mesh->polygons.size()) not met!
```

LiDAR cloud has no `rgb` field; color-transfer culls all polygons.
**Workaround:** skip mesh+texture, use `--scan --cam_projection` for a
colored cloud (no mesh).

### CloudCompare shows just a "line"

Usual cause is selecting the `camera` element instead of `vertex` — see
[CloudCompare](#cloudcompare-recommended-for-one-shot-inspection) above.

If the bounding box dimensions in CloudCompare's Properties panel are
huge (10⁹+), one or more keyframe poses got corrupted. Use Edit → Crop
→ Box, or re-export with `--max_range 30` to drop the outliers.

### File is huge (>1 GB)

You used `--voxel-size 0.0`. Use `--voxel-size 0.05` for a casual viz
or `0.01` for max detail. The default `0.03` is a good middle ground.

### The export takes forever

`--scan` walks every keyframe. On the Jetson it's ~30 s/100 nodes.
Run on your dev machine for faster export by `scp`-ing the DB:

```bash
scp gizmo:.ros/rtabmap.db ~/maps/
./scripts/export_map.sh --db ~/maps/rtabmap.db
```

---

## See also

- [docs/scripts/start_slam.md](scripts/start_slam.md) — building maps
- [docs/troubleshooting.md](troubleshooting.md) — RTABMap-specific issues
- Main README §[Map persistence](../README.md#map-persistence-and-rosrtabmapdb) — DB inspection commands
- [scripts/export_map.sh](../scripts/export_map.sh) — the wrapper itself
