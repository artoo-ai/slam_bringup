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
- **Fixture height (table to mounting plate)**: 9" (228.6 mm)
- **Mounting plate**: 5.5" x 8.5", 6 mm thick ASA
- **Mid-360 sits on top of the mounting plate** (no tilt, level)

## Table Setup (as of 2026-04-23)

- **Table type**: adjustable-height
- **Table height**: 38" (965.2 mm) from floor
- **Fixture position**: lengthwise in the middle of the table, 5" (127 mm) from the table edge
- **Total LiDAR height above floor (on table)**: 38" + 9" + plate thickness + Mid-360 base = ~48" (~1219 mm)
- **Total LiDAR height above floor (on floor)**: 9" + plate thickness + Mid-360 base = ~10" (~254 mm)

## Derived LiDAR Geometry (no tilt)

Mid-360 vertical FOV: -7° to +52°

### On Table (LiDAR ~1.2 m above floor)

| Parameter | Value |
|-----------|-------|
| Nearest ground return radius | 1.2 / tan(7°) = **9.78 m** |
| Blind ring on floor | 0 to 9.78 m radius — no ground points inside this |
| Practical effect | In a typical room (<5 m wide), **zero ground returns** |

### On Floor (LiDAR ~0.254 m above floor)

| Parameter | Value |
|-----------|-------|
| Nearest ground return radius | 0.254 / tan(7°) = **2.07 m** |
| Blind ring on floor | 0 to 2.07 m radius |
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
| 9" (0.254 m, fixture on floor)   | 2.07 m | Yes, in most rooms |
| 48" (1.2 m, fixture on table)    | 9.77 m | Rarely — typical rooms are <5 m |
| ~20" (~0.5 m, Go2 mount)         | ~4.1 m | Sometimes |
| ~24" (~0.6 m, mecanum mast)      | ~4.9 m | Sometimes |

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
