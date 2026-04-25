"""
Generate Universal LiDAR (Hesai Pandar40 / Velodyne VLP-16) + WT901 IMU
Mounting Plate DXF.

Plate geometry:
- Simple rounded rectangle: 139.7 x 215.9 mm (5.5" x 8.5"), 6 mm PETG
- 5 mm corner radius (no cantilever tab)

2020 perimeter frame mount (6x M5 t-nut clearance):
- short bars (top/bottom): (0, +/-97.95) - 1 hole each
- long bars (left/right):  (+/-59.85, +/-60) - 2 holes each
- Slot centerlines assumed 10 mm in from main plate edge (2020 outer face
  flush with plate edge)

Either-or sensor mounting (sensor housings overlap on centerline).
Pandar40 has dual BCD patterns (0 and 180 deg) so it can be mounted in
either orientation — use the 180-deg pattern to route the cable away
from the IMU:

Pandar40 (centered at Y = +30):
- 1x M6 center clearance (phi 6.6) at (0, +30)
- 3x M6 BCD clearance (phi 6.6) at phi 98 mm BCD, angles 90/210/330 deg
- 3x M6 BCD clearance (phi 6.6) at phi 98 mm BCD, angles 270/30/150 deg
  (180-deg rotated orientation — cable exits +Y, away from IMU-A)
- 2x phi 4 alignment pins (phi 3.8 print for PETG) at (+/-44.45, +30)

VLP-16 (centered at Y = -35):
- 1x 1/4-20 center clearance (phi 6.75) at (0, -35)
- 2x 5/32" alignment pins (phi 3.8 print for PETG) at (+/-44.45, -35)

WitMotion WT901BLECL5.0 IMU — TWO positions on the base plate (use one at
a time, opposite side from the mounted lidar). Orientation matches the
Mid-360 plate: rotated 90 deg so long axis (51.5 mm) runs along plate Y,
X-arrow points forward (+Y). 42.8 mm slot spacing along Y.

IMU Position A (for Pandar40 config, center Y = -65):
- 2x phi 4.0 mm M3 brass-insert pilots at (0, -43.6) and (0, -86.4)
- Pandar40 at +Y, IMU at -Y

IMU Position B (for VLP-16 config, center Y = +70):
- 2x phi 4.0 mm M3 brass-insert pilots at (0, +48.6) and (0, +91.4)
- VLP-16 at -Y, IMU at +Y

Construction layer (reference, hide after Fusion import):
- Pandar40 phi 116 housing outline + phi 98 BCD circle
- VLP-16 phi 103.3 housing outline
- Pandar40 + VLP-16 center crosshairs
- WT901 housing outlines for both positions (51.5 x 36.1, long axis along Y)
"""
import ezdxf
from ezdxf.enums import TextEntityAlignment
import math

# --- Plate geometry ----------------------------------------------------------
PLATE_W = 139.70        # main plate X width (5.5")
PLATE_L = 215.90        # main plate Y length (8.5")
CORNER_R = 5.0          # corner fillet radius

# --- 2020 extrusion frame mounting ------------------------------------------
EXTRUSION_W = 20.0
EXTRUSION_INSET = EXTRUSION_W / 2   # slot centerline 10 mm in from plate edge
M5_CLR_R = 5.5 / 2      # M5 clearance hole
LONG_SIDE_HOLE_Y = 60.0

# --- Pandar40 mounting (offset Y = +30 mm from plate center) ----------------
PANDAR_CENTER_Y = 30.0
PANDAR_M6_CENTER_R = 6.6 / 2
PANDAR_BCD_R = 98.0 / 2
PANDAR_BCD_HOLE_R = 6.6 / 2
PANDAR_BCD_ANGLES = (90.0, 210.0, 330.0)
PANDAR_BCD_ANGLES_180 = (270.0, 30.0, 150.0)  # 180-deg rotated: cable exits +Y (away from IMU-A)
PANDAR_PIN_X = 44.45
PANDAR_PIN_PRINT_R = 3.8 / 2
PANDAR_BODY_R = 116.0 / 2

# --- VLP-16 mounting (offset Y = -35 mm from plate center) ------------------
VLP_CENTER_Y = -35.0
VLP_QTR20_R = 6.75 / 2
VLP_PIN_X = 44.45
VLP_PIN_PRINT_R = 3.8 / 2
VLP_BODY_R = 103.3 / 2

# --- WT901BLECL5.0 IMU (two on-plate positions, Mid-360 orientation) --------
# Rotated 90 deg: long axis (51.5 mm) along plate Y, X-arrow points +Y.
# Slot spacing 42.8 mm along Y. Holes centered on X = 0.
WT901_INSERT_R = 4.0 / 2        # phi 4.0 pilot for M3 brass insert (Ruthex RX-M3x5.7)
WT901_SLOT_Y_SPACING = 42.8     # slot-center to slot-center along Y
WT901_BODY_X = 36.1             # housing X (short axis, since rotated 90)
WT901_BODY_Y = 51.5             # housing Y (long axis)

# Position A: for Pandar40 config (IMU on -Y side, opposite Pandar at +Y)
WT901_A_CENTER_X = 0.0
WT901_A_CENTER_Y = -65.0

# Position B: for VLP-16 config (IMU on +Y side, opposite VLP at -Y)
WT901_B_CENTER_X = 0.0
WT901_B_CENTER_Y = 70.0

# --- Output ------------------------------------------------------------------
OUT_PATH = "/Users/rico/Documents/Robots/slam/cad/Universal_LiDAR_Mounting_Plate.dxf"

# --- Build DXF ---------------------------------------------------------------
doc = ezdxf.new("R2018", setup=True)
doc.header["$INSUNITS"] = 6
doc.header["$MEASUREMENT"] = 1

doc.layers.add("CONSTRUCTION", color=8)
doc.layers.add("LABELS", color=3)

msp = doc.modelspace()

# --- Plate outline (simple rounded rectangle, no tab) -----------------------
hx = PLATE_W / 2        # 69.85
hy = PLATE_L / 2        # 107.95
r = CORNER_R

msp.add_line((-hx + r, -hy), ( hx - r, -hy))                # bottom
msp.add_line(( hx,      -hy + r), ( hx,       hy - r))       # right
msp.add_line(( hx - r,   hy),     (-hx + r,   hy))           # top
msp.add_line((-hx,       hy - r), (-hx,      -hy + r))       # left

msp.add_arc(( hx - r,  hy - r), r, start_angle=0,   end_angle=90)
msp.add_arc((-hx + r,  hy - r), r, start_angle=90,  end_angle=180)
msp.add_arc((-hx + r, -hy + r), r, start_angle=180, end_angle=270)
msp.add_arc(( hx - r, -hy + r), r, start_angle=270, end_angle=360)

# --- 2020 frame mount holes (M5 t-nut clearance) ----------------------------
slot_x = hx - EXTRUSION_INSET   # +/-59.85
slot_y = hy - EXTRUSION_INSET   # +/-97.95
msp.add_circle(( 0,       slot_y), M5_CLR_R)   # top bar
msp.add_circle(( 0,      -slot_y), M5_CLR_R)   # bottom bar
for sy in (-1, 1):
    msp.add_circle(( slot_x, sy * LONG_SIDE_HOLE_Y), M5_CLR_R)  # right bar (2x)
    msp.add_circle((-slot_x, sy * LONG_SIDE_HOLE_Y), M5_CLR_R)  # left bar (2x)

# --- Pandar40 mounting pattern ----------------------------------------------
msp.add_circle((0, PANDAR_CENTER_Y), PANDAR_M6_CENTER_R)
for ang_deg in PANDAR_BCD_ANGLES:
    ang = math.radians(ang_deg)
    x = PANDAR_BCD_R * math.cos(ang)
    y = PANDAR_CENTER_Y + PANDAR_BCD_R * math.sin(ang)
    msp.add_circle((x, y), PANDAR_BCD_HOLE_R)
for ang_deg in PANDAR_BCD_ANGLES_180:
    ang = math.radians(ang_deg)
    x = PANDAR_BCD_R * math.cos(ang)
    y = PANDAR_CENTER_Y + PANDAR_BCD_R * math.sin(ang)
    msp.add_circle((x, y), PANDAR_BCD_HOLE_R)
msp.add_circle(( PANDAR_PIN_X, PANDAR_CENTER_Y), PANDAR_PIN_PRINT_R)
msp.add_circle((-PANDAR_PIN_X, PANDAR_CENTER_Y), PANDAR_PIN_PRINT_R)

# --- VLP-16 mounting pattern ------------------------------------------------
msp.add_circle((0, VLP_CENTER_Y), VLP_QTR20_R)
msp.add_circle(( VLP_PIN_X, VLP_CENTER_Y), VLP_PIN_PRINT_R)
msp.add_circle((-VLP_PIN_X, VLP_CENTER_Y), VLP_PIN_PRINT_R)

# --- WT901 IMU Position A (Pandar40 config, -Y side) -----------------------
for sy in (-1, 1):
    msp.add_circle(
        (WT901_A_CENTER_X, WT901_A_CENTER_Y + sy * WT901_SLOT_Y_SPACING / 2),
        WT901_INSERT_R,
    )

# --- WT901 IMU Position B (VLP-16 config, +Y side) -------------------------
for sy in (-1, 1):
    msp.add_circle(
        (WT901_B_CENTER_X, WT901_B_CENTER_Y + sy * WT901_SLOT_Y_SPACING / 2),
        WT901_INSERT_R,
    )

# --- Construction geometry (hide after import) ------------------------------
def rect(x0, y0, x1, y1, layer="CONSTRUCTION"):
    msp.add_line((x0, y0), (x1, y0), dxfattribs={"layer": layer})
    msp.add_line((x1, y0), (x1, y1), dxfattribs={"layer": layer})
    msp.add_line((x1, y1), (x0, y1), dxfattribs={"layer": layer})
    msp.add_line((x0, y1), (x0, y0), dxfattribs={"layer": layer})

# Pandar40 housing + BCD circle
msp.add_circle((0, PANDAR_CENTER_Y), PANDAR_BODY_R, dxfattribs={"layer": "CONSTRUCTION"})
msp.add_circle((0, PANDAR_CENTER_Y), PANDAR_BCD_R,  dxfattribs={"layer": "CONSTRUCTION"})

# VLP-16 housing
msp.add_circle((0, VLP_CENTER_Y), VLP_BODY_R, dxfattribs={"layer": "CONSTRUCTION"})

# WT901 housing outlines (rotated 90: X=36.1, Y=51.5)
wx = WT901_BODY_X / 2
wy = WT901_BODY_Y / 2
rect(WT901_A_CENTER_X - wx, WT901_A_CENTER_Y - wy,
     WT901_A_CENTER_X + wx, WT901_A_CENTER_Y + wy)
rect(WT901_B_CENTER_X - wx, WT901_B_CENTER_Y - wy,
     WT901_B_CENTER_X + wx, WT901_B_CENTER_Y + wy)

# Center crosshairs
def cross(cx, cy, size=10):
    msp.add_line((cx - size, cy), (cx + size, cy), dxfattribs={"layer": "CONSTRUCTION"})
    msp.add_line((cx, cy - size), (cx, cy + size), dxfattribs={"layer": "CONSTRUCTION"})
cross(0, PANDAR_CENTER_Y)
cross(0, VLP_CENTER_Y)
cross(WT901_A_CENTER_X, WT901_A_CENTER_Y)
cross(WT901_B_CENTER_X, WT901_B_CENTER_Y)

# --- Labels -----------------------------------------------------------------
def lbl(text, pos, height=4.0, align=TextEntityAlignment.LEFT):
    t = msp.add_text(text, dxfattribs={"layer": "LABELS", "height": height})
    t.set_placement(pos, align=align)

lbl("UNIVERSAL LIDAR + WT901 MOUNTING PLATE", (-hx + 20, hy - 8),  height=4.0)
lbl("Pandar40 + VLP-16 (either-or)  |  6 mm PETG", (-hx + 20, hy - 14), height=3.0)
lbl("M5 -> 2020 T-NUT (6x: 1 ea short, 2 ea long)", (-hx + 20, hy - 20), height=3.0)
lbl("PANDAR40 (Y=+30, dual BCD: 0/180 deg)",  (-hx + 5, PANDAR_CENTER_Y + 65), height=3.0)
lbl("VLP-16 (Y=-35)",    (-hx + 5, VLP_CENTER_Y - 60),    height=3.0)
lbl("IMU-A (Pandar cfg, Y=-65)", (25, WT901_A_CENTER_Y + 5), height=3.0)
lbl("42.8 mm slot, M3 insert, rotated 90, X-arrow +Y",
    (25, WT901_A_CENTER_Y - 2), height=2.5)
lbl("IMU-B (VLP cfg, Y=+70)", (25, WT901_B_CENTER_Y + 5), height=3.0)
lbl("42.8 mm slot, M3 insert, rotated 90, X-arrow +Y",
    (25, WT901_B_CENTER_Y - 2), height=2.5)

# --- Save -------------------------------------------------------------------
doc.saveas(OUT_PATH)
print(f"Saved: {OUT_PATH}")

# --- Summary ----------------------------------------------------------------
print("\n=== Plate Summary ===")
print(f"Plate: {PLATE_W} x {PLATE_L} mm, 6 mm PETG, R{CORNER_R} corners (no tab)")
print(f"2020 frame mounts: 6x M5 clearance:")
print(f"  short bars (top/bottom): (0, +/-{slot_y})")
print(f"  long bars (left/right):  (+/-{slot_x}, +/-{LONG_SIDE_HOLE_Y})")
print(f"Pandar40 (center Y={PANDAR_CENTER_Y:+}):")
print(f"  Center M6 + 3x M6 BCD ({PANDAR_BCD_R*2:.0f} mm) at "
      f"angles {PANDAR_BCD_ANGLES}")
print(f"  2x phi {PANDAR_PIN_PRINT_R*2:.2f} alignment pins at (+/-{PANDAR_PIN_X}, "
      f"{PANDAR_CENTER_Y:+})")
print(f"  3x M6 BCD (180-deg rotated) at angles {PANDAR_BCD_ANGLES_180} — "
      f"cable exits +Y, away from IMU-A")
print(f"VLP-16 (center Y={VLP_CENTER_Y:+}):")
print(f"  1x 1/4-20 center + 2x phi {VLP_PIN_PRINT_R*2:.2f} pins at "
      f"(+/-{VLP_PIN_X}, {VLP_CENTER_Y:+})")
imu_a_h1 = WT901_A_CENTER_Y - WT901_SLOT_Y_SPACING / 2
imu_a_h2 = WT901_A_CENTER_Y + WT901_SLOT_Y_SPACING / 2
imu_b_h1 = WT901_B_CENTER_Y - WT901_SLOT_Y_SPACING / 2
imu_b_h2 = WT901_B_CENTER_Y + WT901_SLOT_Y_SPACING / 2
print(f"WT901 IMU-A (Pandar40 config, center (0, {WT901_A_CENTER_Y:+}), "
      f"rotated 90, X-arrow +Y):")
print(f"  2x phi {WT901_INSERT_R*2:.2f} insert pilots at "
      f"(0, {imu_a_h1:+}) and (0, {imu_a_h2:+})")
print(f"WT901 IMU-B (VLP-16 config, center (0, {WT901_B_CENTER_Y:+}), "
      f"rotated 90, X-arrow +Y):")
print(f"  2x phi {WT901_INSERT_R*2:.2f} insert pilots at "
      f"(0, {imu_b_h1:+}) and (0, {imu_b_h2:+})")
