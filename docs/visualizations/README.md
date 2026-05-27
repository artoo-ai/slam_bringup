# Visualizations

Interactive HTML pages that explain how the SLAM stack actually behaves —
the kind of thing that's faster to *see* than to read about. Each file is
self-contained (no server, no build step). Open it in any modern browser.

## Pages

### `slam_config_explorer.html`
**Interactive cards for every tunable parameter** in this repo's config
files — 50+ cards across 7 sections. Slide a value, watch the data
change live. Each card shows:

- A live canvas (for impactful parameters) or a slim slider+description
  layout (for internal tuning knobs)
- The project default with a green/yellow chip showing if you've drifted
  from it, plus per-card and global reset buttons
- **Mode badges** (mapping · localization · Nav2 · 3D stack) so you know
  whether the parameter even applies to your current launch
- **Dependencies** — what other parameters/files must match (e.g. map
  resolution must match Nav2 costmap resolution; IMU covariances depend
  on the unit-corrected `/livox/imu_ms2` topic)
- **Compute cost chip** (none / minor / moderate / significant) with the
  Big-O scaling so you know what maxing it out actually costs the Jetson
  Orin Nano
- **Realistic value range pill** (distinct from the slider's full range,
  which often extends past realistic ranges for teaching purposes)
- **🎬 Real-world test plan** — concrete physical setup (broomstick at
  2 m, tape arrow on the floor, an assistant walking a figure-8), value
  sweep with expected observations, and what to document for video
  coverage. Use this if you're recording parameter-tuning videos.
- **🔄 Cross-device callouts** on sensor parameters listing the equivalent
  parameter name + default for the user's other devices (Velodyne VLP-16,
  Hesai Pandar40P, Unitree L1, RPLidar C1M1, Neato XV-11, TF Mini-Plus,
  D435/D435i/D455, ZED/ZED Mini, OAK-D Pro, Intel Euclid, BNO086, BNO055)

#### Sections

1. **2D stack — sensor processing**: `scan_z_min/max`, `scan_range_min/max`,
   `angle_increment`. Applies to ANY 3D source feeding `pointcloud_to_laserscan`.
2. **2D stack — slam_toolbox**: map resolution, keyframe triggers, scan
   matcher tuning, loop closure thresholds, correlation + loop search
   grids, optimizer penalty weights, TF/publish timing, `max_laser_range`.
3. **Nav2 — costmaps + controller**: "three rings" explainer card
   (sensor exclusion vs robot_radius vs inflation), plus `robot_radius`,
   inflation, persistence, goal tolerances, max_vel, controller_frequency,
   DWB sampling, accelerations, critic weights, RotateToGoal tuning,
   costmap update rates, local costmap extent, max_obstacle_height,
   planner + smoother, behavior server, velocity smoother, BT timing,
   progress checker, velocity thresholds, waypoint follower.
4. **3D stack — FAST-LIO2**: `blind`, voxel filter (surf + map),
   `scan_line` + `fov_degree`, IMU covariances (per-sample + bias),
   time sync, extrinsic calibration, `det_range`.
5. **3D stack — RTABMap**: ICP registration, RGBD keyframe triggers,
   loop closure local search, occupancy grid, visual features, detection
   rate + memory management, mode flags, database path.
6. **Sensor drivers**: Mid-360, D435, WitMotion, viz_clip.
7. **Drive bridge**: Yahboom velocity caps, watchdog + car_type, axis
   inversion + serial port.

Designed for video documentation of parameter testing — each card's
realistic range pill and test plan tell you what physical setup to film
and what to expect at each end of the slider.

### `slam_2d_vs_3d.html`
**Why 2D SLAM works where the 3D stack stalled.** Side-by-side comparison
of FAST-LIO2 + RTABMap (15-state estimator, full cloud) vs.
slam_toolbox + rf2o (3-state estimator, /scan). Interactive Z-band
sliders, angular bin demo with aliasing warnings, "winner takes all"
per-bin range reduction, and a final-output comparison showing what each
stack actually consumes from the same lidar at the same instant.

Use this if you don't fully grok why the 2D path is the right choice on
this rover, or what `pointcloud_to_laserscan` is doing under the hood.

### `lidar_2d_projection.html`
Earlier prototype of the 2D-projection visualization (single canvas,
controls bar at the bottom). Superseded by the more comprehensive
`slam_2d_vs_3d.html` but kept around as a focused, no-distractions view
of just the 3D → 2D step.

### `lidar_scan_pattern_comparison.html`
Spinning-mirror vs. non-repetitive (Livox) scan pattern animation —
shows why the Mid-360 fills in the cloud differently than a Velodyne and
why "scan_line: 4" in `fast_lio_mid360.yaml` is a Livox-specific
preprocess hint.

### `depth_camera_comparison.html`
Field-of-view + range envelope comparison for D435 / D435i / D455 /
Mid-360. Useful when deciding whether to add the D435 back to the 3D
stack (currently off in `slam_2d.launch.py`).

## How to use these

```sh
# macOS
open docs/visualizations/slam_config_explorer.html

# Linux
xdg-open docs/visualizations/slam_config_explorer.html
```

Or just drag the file into a browser tab. Everything is offline-friendly
HTML5 canvas + vanilla JS — no network, no build, no dependencies.

## Editing notes

- Each page is a single self-contained HTML file with inline CSS + JS.
  Keep it that way — these are meant to be email-able / drop-in-able.
- Drawing helpers are duplicated between `slam_2d_vs_3d.html` and
  `slam_config_explorer.html` on purpose — the cost of pulling in a
  shared module isn't worth losing self-containment.
- When a card's parameter default changes in the YAML / launch file,
  update the slider's `value="…"` attribute to match (it's the single
  source of truth for the "def X" chip and the reset button).
