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
- `blind: 0.2` in config — 2040 frame tightest member is >20 cm from LiDAR head.
