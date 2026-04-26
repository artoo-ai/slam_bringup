# 2040 Extrusion Test Fixture

Self-contained benchtop test rig for Mid-360 LiDAR + FAST-LIO2 development.

## Components

| Component | Notes |
|-----------|-------|
| Livox Mid-360 | Sits on top of mounting plate |
| Mid-360 ICM40609 IMU | Built-in, hardware time-synced, used by FAST-LIO2 |
| WitMotion WT901C | On mounting plate, NOT used by FAST-LIO2 |
| Jetson Orin Nano Super | Compute |
| Waveshare USB3.2 Hub (4-port) | Inside frame |
| 12V Buck Converter | Powers Jetson from DeWalt battery |
| DeWalt 20V Battery | Power source |

## Physical Dimensions

- **Extrusion frame footprint**: matches Mid-360 mounting plate (5.5" x 8.5" / 139.7 x 215.9 mm)
- **Fixture height (table contact to top of mounting plate)**: 8.176" (207.670 mm) *(measured 2026-04-26; supersedes the earlier "~9"" rough estimate)*
- **Mounting plate**: 5.5" x 8.5", 7 mm thick ASA  *(measured 2026-04-25; CAD source called for 6 mm — over-extrusion delta is documented here, use the measured value for URDF)*
- **Mid-360 sits on top of the mounting plate** (no tilt, level)
- **Frame extrusion**: **two stacked 2040 perimeter cages** (40 mm vertical, 20 mm horizontal each), connected by short **2020 vertical posts** that form a Jetson + USB-hub cavity between them. The lidar plate bolts to the upper 2040 via rubber isolation pads (no direct metal-to-metal). Footprint of both cages = lidar plate footprint (139.7 × 215.9 mm).
- **Lower 2040 cage**: bottom rail, sits on the table. 40 mm vertical face.
- **2020 vertical posts (Jetson cavity)**: 97.683 mm tall — holds Jetson Orin Nano Super and Waveshare USB3.2 hub between the two 2040 rails.
- **Upper 2040 cage**: lidar cage, holds rubber pads + plate on top, holds D435 in front-face slot. 40 mm vertical face.
- **Rubber isolation pads (compressed)**: 0.905" (22.987 mm) between the underside of the lidar plate and the top of the upper 2040.

## URDF Stack-Up (lidar plate frame, +Z = up)

Measured 2026-04-25. Reference plane is the **top surface of the lidar plate**
(`Z = 0`); plate-frame X/Y axes match `cad/make_livox_mid360_plate.py`
(plate is 139.7 mm in X, 215.9 mm in Y, with Mid-360 body shifted to
plate Y = +25 mm to free room for the WT901 at Y = -60 mm).

```
Z =    0 mm            ┌──────────────────────────────────────┐  ← top of lidar plate (= sensor_plate)
                       │   Mid-360 (centered at plate Y=+25)  │
                       │   WT901    (centered at plate Y=-60) │
Z =   -7 mm            ├──────────────────────────────────────┤  ← bottom of lidar plate (7 mm thick)
                       ░░░░░░░░░░░ rubber pads (22.987 mm) ░░░░
Z =  -29.987 mm        ╔══════════════════════════════════════╗  ← top of UPPER 2040 (lidar cage)
                       ║                                      ║
                       ║   upper 2040, 40 mm vertical face    ║
              D435 ──→ ╠══╗                                   ║   D435 mounted on FRONT upper-2040 face
                       ║  ║   D435 top at Z = -16.002 mm      ║   (3D-printed slot mount in upper slot)
                       ║  ║   D435 vert. center Z ≈ -28.50 mm ║
Z =  -69.987 mm        ╚══╩═══════════════════════════════════╝  ← bottom of upper 2040
                       │  │  │                                │
                       │  │  │  2020 vertical posts           │   Jetson + USB hub live in this cavity
                       │  │  │  (97.683 mm tall, 4 corners)   │
                       │  │  │                                │
Z = -167.670 mm        ╔══════════════════════════════════════╗  ← top of LOWER 2040 (bottom rail)
                       ║                                      ║
                       ║   lower 2040, 40 mm vertical face    ║
Z = -207.670 mm        ╚══════════════════════════════════════╝  ← bottom of fixture (table contact, = base_link)
```

### D435 placement (relative to lidar plate frame)

- **Top of D435** sits 0.630" (16.002 mm) below the top of the lidar plate
  → top of D435 at `Z = -16.002 mm`.
- D435 housing height = 25 mm → vertical midpoint at `Z = -16.002 - 12.5 = -28.502 mm`.
- D435 is mounted in a 3D-printed bracket that slides into the **upper slot
  of the front 2040 extrusion** (the 40 mm-vertical face).
- **Mount source**: [Intel RealSense 435i Mount, 20mm 80/20 Extrusion (MakerWorld #1788451)](https://makerworld.com/en/models/1788451-intel-realsense-435i-mount-20mm-80-20-extrusion#profileId-1905852).
  Mount slides into a single 20 mm-wide extrusion slot via a t-nut and
  holds the D435 by its standard 1/4"-20 tripod thread on the camera's
  bottom face. Local copy of the printed file is `cad/RealSense435iMount.3mf`.
- **Lateral (X)**: D435 horizontal midpoint coincides with the lidar plate
  midpoint → D435 X = 0 in plate frame.
- **Fore/aft (Y)**: D435 mounts to the front 2040 perimeter bar. Camera
  protrudes forward from the front face of the 2040 by the depth of the
  printed mount + half the camera depth. **NEEDS MEASUREMENT** (see open
  questions below).

### Fast computation reference (so the URDF macro can re-derive)

```
plate_thickness          = 7.000 mm      (measured)
rubber_compressed        = 22.987 mm     (= 0.905")
extrusion_profile        = 2040 (40 mm vertical face)
d435_top_below_plate_top = 16.002 mm     (= 0.630")
d435_height              = 25 mm         (Intel datasheet)
d435_vert_center_z       = -(d435_top_below_plate_top + d435_height/2)
                         = -(16.002 + 12.5) = -28.502 mm  (relative to plate top)
top_of_2040_z            = -(plate_thickness + rubber_compressed)
                         = -(7 + 22.987) = -29.987 mm     (relative to plate top)
```

The D435 vertical center sits **1.485 mm above** the top of the 2040
extrusion — confirms the camera is held in the upper slot of the
40 mm-vertical face, just barely peeking above the extrusion rail.

## Resolved URDF Inputs (2026-04-25)

All measurements needed for `urdf/sensors_common.urdf.xacro` (Phase 1.7)
are now locked. Source: in-person measurement on the bench fixture.

### 1. 2040 perimeter — geometry

- Frame footprint **exactly matches** the lidar plate (139.7 × 215.9 mm).
- 2040 outer faces are **flush** with the plate edges on all four sides
  (no inset, no overhang).
- 40 mm dimension is vertical, 20 mm dimension is horizontal → 20 mm-wide
  perimeter rail, same convention as the 2020 plate Python in CAD.
- Front-bar outer face is at plate `Y = -(215.9 / 2) = -107.95 mm`
  (Mid-360 M12 cable exits at plate `+Y = +80`, so the D435-facing edge
  is `-Y`).
- **Two stacked 2040 cages** (upper = lidar cage, lower = bottom rail);
  each is 40 mm tall. The cages are linked by 2020 vertical posts that
  span 97.683 mm (the Jetson + USB-hub cavity). Total fixture height
  base→plate-top = `40 + 97.683 + 40 + 22.987 + 7 = 207.670 mm` (= 8.176").

### Troubleshooting — "Why are there two D435s in RViz?"

If `./start_perception.sh rviz:=true` shows what looks like a doubled
D435 — two clusters of axis triads at slightly different poses near the
front of the plate, with overlapping `d435_front_*` labels — the cause
is a TF parent-conflict between the URDF and the realsense2_camera node.

The realsense node's `camera_name` parameter prefixes the names of every
TF frame it publishes, but it does NOT tell the node which **parent**
to attach its root link to. By default the node publishes
`<camera_name>_link` as a **TF root** (no parent). The URDF then
publishes the same link as a child of `sensor_plate`, and tf2 sees two
parents — the result is two triads of axes at slightly different poses,
one from each publisher.

**Fix**: set `base_frame_id: '<camera_name>_link'` in the realsense node
parameters (`launch/d435.launch.py`). That tells the node "your root
link has an external parent — don't republish it as a root." After this,
`tf2_tools view_frames` shows a single chain
`sensor_plate → d435_front_link → d435_front_*_frame → d435_front_*_optical_frame`.

Diagnostic commands:

```bash
ros2 run tf2_tools view_frames                         # frames.pdf — look for d435_front_link's parent count
ros2 topic echo /tf_static --once | grep -A2 'child_frame_id: "d435_front_link"'
# Two matches = conflict; one match = fixed.
```

### 2. D435 forward offset

- D435 front glass protrudes **1.528" (38.811 mm)** forward of the front
  2040 outer face.
- D435 front glass plate-Y position = `-107.95 - 38.811 = -146.761 mm`.
- D435 horizontal X midpoint = plate X = 0 (centered).
- D435 vertical center Z (relative to plate top) = `-28.502 mm` (derived
  earlier from the 0.630" top-of-D435-below-plate-top measurement).
- **D435 pitch (nose-down)**: ≈20° by design of the MakerWorld
  #1788451 mount. Measured with a digital inclinometer on the rounded
  D435 housing top (2026-04-26): roughly 20°. Assumed exact 20° (round
  number per mount author intent; housing rounded top gives ±2°
  measurement uncertainty). Stored in URDF as
  `d435_front_rpy = (0, -0.349066, 0)` rad. Refine with a depth-image
  floor-plane fit if the RTABMap floor looks tilted.
- Convention shift for ROS REP-103 (`+X` forward, `+Y` left, `+Z` up):
  the **body frame** has `+X` pointing in the D435 direction, which is
  plate `-Y`. So `body → sensor_plate` is a 180° yaw rotation:
    body +X = plate -Y
    body +Y = plate +X
    body +Z = plate +Z
  In URDF terms, the `sensor_plate` link is published with a `yaw = π`
  joint origin relative to `base_link/body`, so all the plate-frame
  numbers above stay readable in CAD coordinates.

### 3. Mid-360 optical center

Accept the Livox Mid-360 datasheet figure: **optical center is 36.61 mm
above the bottom of the housing**. Mid-360 housing bottom rests directly
on the plate top (no shim — 2 locating pins index it), so:

```
livox_frame Z (relative to plate top) = +36.61 mm
livox_frame X = 0           (Mid-360 body centered on plate X axis)
livox_frame Y = +25 mm      (per make_livox_mid360_plate.py — body shifted +25 mm to free room for WT901)
```

If you want to verify by direct measurement instead of trusting the
datasheet: lay a straightedge across the top of the Mid-360 housing,
measure straightedge-to-plate-top with calipers (`= housing_total_height
≈ 65 mm`), then subtract the datasheet "top of housing to optical
center" offset (`≈ 28.4 mm`). The optical-center mark is etched on
some Mid-360 units near the M12 connector — if yours has it, calipers
to that mark is the most accurate option.

### 4. WitMotion WT901 IMU

Use housing geometric center as `imu_link` — accepted, no measurement
needed. WT901 is the backup IMU only (FAST-LIO2 uses the Mid-360's
onboard ICM40609); a few mm of error is acceptable.

```
imu_link X (plate frame) = 0
imu_link Y (plate frame) = -60 mm   (per make_livox_mid360_plate.py)
imu_link Z (plate frame) = +9 mm    (3 mm flange above plate + half of 12 mm housing-above-flange)
```

### 5. `base_link` for the bench fixture — pros/cons

The bench fixture has no chassis, so the choice is purely a convention
question. Three options and the trade-offs:

| Option | What it means | Pro | Con |
|--------|--------------|-----|-----|
| **(a) `base_link = sensor_plate`** (zero-offset alias of plate) | The rig has no "footprint" — base_link sits on the plate top, +X forward. | Simplest tree; one fewer joint. RViz fixed-frame "just works" pointing at the plate. | Breaks ROS convention slightly: most Nav2 / robot_localization configs expect `base_link` at the **footprint** (ground-contact level), not at sensor height. Reusable URDF macros that drive Nav2 may need `base_footprint` synthesized. |
| **(b) `base_link` at floor contact (table top)** | `base_link` at the very bottom of the lower 2040 = table top; `sensor_plate` is a child at +207.670 mm above it (lower 2040 + 2020 cavity + upper 2040 + rubber + plate = 8.176"). | Matches REP-105 ("base_link at the robot footprint"). Carries cleanly when you later mount on Go2 / mecanum. Nav2 / costmaps that assume `base_link Z = 0` at the ground will work without surgery. | Extra `base_link → sensor_plate` joint (~207.67 mm offset). Slightly more URDF. |
| **(c) Skip `base_link` for the fixture; only define on real platforms** | Fixture URDF has only `sensor_plate` + sensor children. Real-platform URDFs include `sensors_common.urdf.xacro` and add their own `base_link`. | Cleanest separation between sensor-rig geometry and platform geometry. | Some tools (`tf2_echo`, `robot_localization`) blow up if `base_link` is absent. Inconvenient for any tooling that defaults to `base_link`. |

**Recommendation: (b)**, implemented in `urdf/bench_fixture.urdf.xacro`.
It's only one extra static TF, and that TF is exactly the stack-up
above: `40 + 97.683 + 40 + 22.987 + 7 = 207.670 mm` from `base_link`
to `sensor_plate` top. Keeps ROS conventions intact for everything
downstream. You also don't have to revisit it when the rig moves to a
real robot — those URDFs will publish their own `base_link` and the
static-TF bridge to `body` lives in `slam.launch.py`'s
`PLATFORM_BRIDGES` dict (per PLAN.md §3.4).

If you want minimum URDF lines for the bench-only case, (a) is fine.
Either choice can be migrated to the other later — it's one joint.

---

## URDF Numerical Summary (copy into xacro)

All distances in meters. Frame `sensor_plate` is the lidar plate's top
surface, `+X` forward (= D435 direction), `+Y` left, `+Z` up.
Plate-frame CAD coordinates (used by `make_livox_mid360_plate.py`)
are rotated 180° in yaw relative to `sensor_plate` — see §2 above.

```yaml
# Plate / fixture
plate_thickness:        0.007    # 7 mm
rubber_compressed:      0.022987 # 0.905"
extrusion_2040_height:  0.040    # 40 mm vertical face
extrusion_2020_height:  0.020    # 20 mm vertical face (posts only here)
jetson_cavity_height:   0.097683 # 2020 vertical posts between the two 2040 cages
base_link_to_plate_top: 0.207670 # = 40 + 97.683 + 40 + 22.987 + 7 mm = 8.176" (option (b))

# Mid-360 — sensor_plate frame
livox_frame_xyz: [-0.025, 0.0, 0.03661]   # body-frame +X = plate -Y, so the
                                          # plate +25 mm Y offset becomes
                                          # body -0.025 m X
livox_frame_rpy: [0.0, 0.0, 0.0]

# WitMotion — sensor_plate frame
imu_link_xyz: [0.060, 0.0, 0.009]         # plate Y=-60 → body X=+0.060;
                                          # housing midpoint above plate ≈ 9 mm
imu_link_rpy: [0.0, 0.0, 0.0]             # WT901 X faces forward (= body +X),
                                          # Y faces left — confirmed 2026-04-25.

# D435 front — sensor_plate frame
# Position is the FRONT GLASS plane (realsense2_description's macro
# offsets internal optical frames from this reference).
# Pitch -0.349066 rad = -20° (nose-down by mount design).
d435_front_link_xyz: [0.146761, 0.0, -0.028502]
d435_front_link_rpy: [0.0, -0.349066, 0.0]
```

Vertical reference: `sensor_plate` is the **top** of the lidar plate.
D435 vertical center is below it (`Z = -0.0285 m`); Mid-360 optical
center is above it (`Z = +0.0366 m`).

## Table Setup (as of 2026-04-23)

- **Table type**: adjustable-height
- **Table height**: 38" (965.2 mm) from floor
- **Fixture position**: lengthwise in the middle of the table, 5" (127 mm) from the table edge
- **Total LiDAR optical-center height above floor (on table)**: 38" + 8.176" (fixture) + 1.441" (Mid-360 optical center above plate) = **47.617" (1209.3 mm)**
- **Total LiDAR optical-center height above floor (on floor)**: 8.176" + 1.441" = **9.617" (244.3 mm)**

## Derived LiDAR Geometry (no tilt)

Mid-360 vertical FOV: -7° to +52°

### On Table (LiDAR ~1.21 m above floor)

| Parameter | Value |
|-----------|-------|
| Nearest ground return radius | 1.209 / tan(7°) = **9.84 m** |
| Blind ring on floor | 0 to 9.84 m radius — no ground points inside this |
| Practical effect | In a typical room (<5 m wide), **zero ground returns** |

### On Floor (LiDAR ~0.244 m above floor)

| Parameter | Value |
|-----------|-------|
| Nearest ground return radius | 0.244 / tan(7°) = **1.99 m** |
| Blind ring on floor | 0 to 1.99 m radius |
| Practical effect | Ground returns start ~2 m out — viable in rooms >4 m |

## Testing Notes

- **2026-04-23**: On the living room floor, FAST-LIO2 did not drift badly for
  about 20 seconds before degrading. Likely cause: insufficient Z constraint
  (ground returns only beyond 2 m) or feature starvation as non-repetitive
  scan pattern hadn't filled in enough structure.
- **2026-04-23 (after blind 0.2→0.5 fix)**: FAST-LIO2 ran stable for 3+
  minutes on floor with no visible drift. Diagnostics confirmed 8197 ground
  returns (nearest 1.59m, median 3.19m) providing Z constraint, and zero
  fixture hits leaking past the blind filter. Root cause of original drift
  was fixture ghost points in the 0.2-0.5m band (-90°/-60° and +120°/+150°
  azimuths) corrupting the Kalman filter.
- No tilt mount on this fixture — full -7° to +52° vertical FOV aimed level.
- IMU time sync handled by Mid-360 internal ICM40609 (time_sync_en: false).
- `blind: 0.5` in config (was 0.2) — 2040 frame + battery/buck at 20-50cm from LiDAR.

## Why FAST-LIO2 Was Failing — Conceptual Explanation

FAST-LIO2 is an Extended Kalman Filter that fuses LiDAR points with IMU data.
Two things must hold for the filter to stay stable:

1. **Clean point inputs** — every point it receives must represent a real
   feature in the world, not rig-internal reflections.
2. **3D geometric constraint** — points must come from enough directions to
   pin down pose in all 6 DOF, especially Z (vertical).

The rig was failing both:

### Problem 1 — Fixture ghost points corrupting the filter

The 2040 frame, battery, buck converter, and cabling sit within 20-50 cm of
the LiDAR optical center. The Mid-360 hits them every rotation and reports
them as valid returns. With `blind: 0.2`, ~3000 points per 10 s in the
0.2-0.5 m band were flowing into FAST-LIO2. The filter sees them as features
that are *stationary relative to the LiDAR* — which looks identical to "the
robot isn't moving." This suppresses legitimate motion estimates and biases
the pose.

The diagnostic's azimuth correlation localized the leak to -90°/-60° and
+120°/+150° — matching the fixture geometry, proving they were self-hits
not room returns.

### Problem 2 — No ground plane = no Z anchor

Mid-360 minimum vertical angle is -7°. The floor-intersection radius is
`lidar_height / tan(7°)`:

| LiDAR height | Blind-ring radius | Floor returns available? |
|--------------|-------------------|--------------------------|
| 9.62" (0.244 m, fixture on floor)  | 1.99 m | Yes, in most rooms |
| 47.6" (1.21 m, fixture on table)   | 9.84 m | Rarely — typical rooms are <5 m |
| ~20" (~0.5 m, Go2 mount)           | ~4.1 m | Sometimes |
| ~24" (~0.6 m, mecanum mast)        | ~4.9 m | Sometimes |

Without ground returns, FAST-LIO2 loses its strongest Z constraint. IMU
accelerometer bias is never fully observable from side-walls alone, so it
integrates into a vertical drift that snowballs after ~20 seconds.

### The fix

Raised `blind` from 0.2 → 0.5 in `config/fast_lio_mid360.yaml`. Does two
things simultaneously:

- Eliminates ALL fixture ghost points (verified: 0 leak through post-fix)
- Costs zero useful data — nearest legitimate ground return on floor is
  2.07 m, far beyond 0.5 m

Result: 3+ minutes drift-free vs. 20 seconds before.

## Porting to Other Platforms (Go2, Mecanum, R2D2, RoboScout)

### What stays the same (do NOT re-tune)

- Mid-360 IMU extrinsics (`extrinsic_T: [-0.011, -0.02329, 0.04412]`) —
  factory-calibrated, baked into the sensor, mount-independent
- `time_sync_en: false` — ICM40609 is hardware-synced to LiDAR regardless
  of mount
- Voxel filter sizes, IMU noise covariances, scan_line — these are
  LiDAR-model properties, not platform properties

### What might change per platform

`blind` must be ≥ the largest radial distance from the LiDAR optical center
to any rigid part of the platform.

| Platform | What's near the LiDAR | Expected `blind` |
|----------|----------------------|------------------|
| 2040 fixture | Frame, battery, buck converter at 20-50 cm | **0.5 m** (validated) |
| Unitree Go2 | Body below mount, legs swinging in lower hemisphere | Likely **0.5-0.8 m**; legs show as time-varying azimuth clusters |
| Mecanum UGV | Chassis top, mast structure, wheels if mounted low | **0.6-1.0 m** depending on mount height |
| R2D2 / RoboScout | Dome/head structure, side panels | Unknown — run diagnostic after mount |

### Per-platform validation workflow

1. Mount the rig, measure LiDAR height above ground.
2. Run diagnostics with the current `blind` value:
   ```bash
   python3 scripts/lidar_diagnostics.py --ros-args \
       -p duration:=10.0 -p lidar_height:=<new_height> -p blind:=<current>
   ```
3. Check section 1 (self-hit detection):
   - No near-field azimuth clustering → `blind` is correct
   - Cluster in specific sectors → raise `blind` by 0.1 m, re-run until clean
4. Check section 2 (ground returns):
   - Ground returns present → FAST-LIO2 has Z anchor, good
   - No ground returns → room too small OR LiDAR mounted too high; move to
     larger space or lower the mount
5. Check section 5 (azimuth occlusion):
   - Large occluded sectors (>60° blocked by body) → expect permanent map
     shadows; acceptable for navigation but expected

### Body self-occlusion — platform-specific and unavoidable

Unlike fixture ghost points (fixable with `blind`), body occlusion is
permanent geometry:

- **Go2**: legs and back block large parts of the lower hemisphere. Leg
  positions change during gait — the azimuth occlusion map will be
  time-varying. FAST-LIO2 still works, map just has shadow sectors.
- **Mecanum**: chassis top may occlude below-horizon entirely. Mount the
  LiDAR on a mast to preserve ground returns.

### TL;DR

`blind: 0.5` is **geometry-specific**, not FAST-LIO2 tuning. It'll likely
work on Go2/mecanum too, but run the diagnostic once per platform to confirm.
If the near-field azimuth clusters reappear, raise `blind` until they vanish.
That's the whole procedure — no other tuning needed.
