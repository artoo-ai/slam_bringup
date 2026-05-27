# Visualizations

Interactive HTML pages that explain how the SLAM stack actually behaves —
the kind of thing that's faster to *see* than to read about. Each file is
self-contained (no server, no build step). Open it in any modern browser.

## Pages

### `slam_config_explorer.html`
**Interactive cards for every tunable parameter** in this repo's config
files. Slide a value, watch the data change live. Each card shows:

- A live canvas demonstrating the parameter's visual effect
- The project default with a green/yellow chip showing if you've drifted
- Mode badges (mapping · localization · Nav2 · 3D stack) so you know
  whether the parameter even applies to your launch
- Dependencies — what other parameters/files must match
- **Compute cost chip** (none / minor / moderate / significant) with the
  Big-O scaling so you know what maxing it out actually costs the Jetson
- "Tune up if / tune down if" guidance derived from this project's
  YAML comments and engineering memos

Covers 14 parameters across 4 groups: 2D sensor processing
(`scan_z_min/max`, `scan_range_min/max`, `angle_increment`), slam_toolbox
(`resolution`, `minimum_travel_distance`, loop closure), Nav2 costmaps
(`robot_radius`, `inflation_radius` + scaling, goal tolerances, max
velocity), and FAST-LIO2 (`blind`, `filter_size_surf`, IMU covariances,
`det_range`). Per-card and global reset buttons revert to project
defaults.

Includes a wide explainer card — **"The three rings around the rover"** —
that side-by-sides `scan_range_min` / `blind` (sensor exclusion),
`robot_radius` (planner lethal zone), and `inflation_radius` (soft cost
gradient) so you can see why three superficially-similar parameters
solve completely different problems.

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
