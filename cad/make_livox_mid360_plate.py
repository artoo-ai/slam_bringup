"""
Generate Livox Mid-360 + WitMotion WT901 Mounting Plate DXF.

Plate geometry:
- 139.7 x 215.9 mm (5.5" x 8.5"), 6 mm ASA
- 5 mm corner radius
- 6x M5 clearance holes for 2020 t-nuts:
    short bars (top/bottom): (0, +/-97.95)        - 1 hole each
    long bars (left/right):  (+/-59.85, +/-60)    - 2 holes each
  Slot centerlines are 10 mm inset from plate edges (2020 outer face flush
  with plate edge). Change M5_CLR_R to M4 (4.5/2) or M3 (3.4/2) if your
  t-nuts use a different thread. Change LONG_SIDE_HOLE_Y to adjust the
  long-bar hole spacing.

Mid-360 (sensor body shifted +25 mm in Y to free space for the WT901):
- Body centered at (0, +25)
- 4x M3 clearance holes at (+/-18, +1) and (+/-18, +49) — 36 x 48 mm pattern
  (long axis along plate Y; sensor mounted with M12 connector toward +Y)
- 2x locating pins at (+/-18, +25) — drawn at phi 3.0 mm nominal (ASA print)
  on the sensor's horizontal centerline, between the top and bottom M3 holes
- Cable pass-through: φ25 mm round at (0, +80) — 22.5 mm outside body edge (+57.5)
  for 90° M12 cable pointing down through the plate

WitMotion WT901BLECL5.0 (plastic enclosure, 2-screw slot mount on -Y side):
- Housing centered at (0, -60), long axis along plate X
- Housing: 51.5 (X) x 36.1 (Y) x 15 mm tall, with 3 mm flange at the base
- Slot ears stick out at X = +/-25.75; slot centers 42.8 mm apart at (+/-21.4, -60)
- Plate holes: phi 4.0 mm pilot for M3 brass heat-set inserts (e.g., Ruthex
  RX-M3x5.7) - pressed flush into plate top, M3 x 8 SHCS through the slot

Construction layer (reference only, hide/delete after Fusion import):
- Mid-360 body outline (65 x 65 square at (0, +25))
- M3 pattern rectangle (48 x 36 at (0, +25))
- WT901 housing outline (51.5 x 36.1 rectangle at (0, -60))
- Center crosshairs for both sensors
"""
import ezdxf
from ezdxf.enums import TextEntityAlignment

# --- Plate geometry ----------------------------------------------------------
PLATE_W = 139.70        # X width (5.5")
PLATE_L = 215.90        # Y length (8.5")
CORNER_R = 5.0          # corner fillet radius

# --- 2020 extrusion frame mounting ------------------------------------------
# Plate sits on top of a 2020 perimeter frame. Each bolt drops through the
# plate into a t-nut sliding in the slot of the 2020 bar directly below.
# Assumption: outer face of each 2020 bar is flush with the plate's outer
# edge -> slot centerline is 10 mm inset from the plate edge.
EXTRUSION_W = 20.0      # 2020 bar width (mm)
EXTRUSION_INSET = EXTRUSION_W / 2   # slot centerline 10 mm inset from plate edge
M5_CLR_R = 5.5 / 2      # M5 clearance hole (most common 2020 t-nut thread)
                        # Use 4.5/2 for M4 t-nuts, 3.4/2 for M3 t-nuts
LONG_SIDE_HOLE_Y = 60.0 # Y offset for the 2 holes on each long (left/right)
                        # bar - symmetric +/- around plate center, 120 mm apart

# --- Mid-360 mounting (sensor shifted forward to make room for IMU) ---------
# Pattern is 36 (X) x 48 (Y) - long axis runs along plate Y. Mount the sensor
# so the M12 connector still faces +Y (toward the cable pass-through).
MID360_CENTER_Y = 25.0  # sensor body centered at (0, +25)
M3_X = 36.0             # horizontal M3 spacing (short axis)
M3_Y = 48.0             # vertical M3 spacing (long axis, along plate Y)
M3_CLR_R = 3.4 / 2      # M3 clearance hole radius (phi 3.4)

# Locating pins - drawn as circles; user extrudes upward in Fusion 360.
# Mid-360 hole: phi 3.0 +0.1/-0 mm x 1.8 mm deep. ASA prints close to nominal,
# so draw at phi 3.0 (would draw at phi 2.8 for PETG to compensate for over-extrusion).
# Pins sit between the top and bottom M3 holes of each side column, on the
# sensor's horizontal centerline (Y = MID360_CENTER_Y).
LOC_PIN_R = 3.0 / 2

# Sensor body footprint (construction layer reference)
BODY_SIDE = 65.0

# --- Cable pass-through ------------------------------------------------------
# Mid-360 M12 connector exits one side of the body. With a 90deg cable pointing
# down, the cable drops through this hole. Position 22.5 mm beyond the body
# edge on +Y (body now ends at Y = +57.5 with the sensor centered at +25).
CABLE_HOLE_Y = 80.0
CABLE_HOLE_R = 25.0 / 2

# --- WitMotion WT901BLECL5.0 IMU (slot mount, -Y side) ----------------------
# ROTATED 90 deg so the WT901's printed X-arrow points along plate Y (forward).
# Long axis (51.5 mm) runs along plate Y; short axis (36.1 mm) along plate X.
# 2 plate holes for M3 brass heat-set inserts. M3 bolts pass through the
# housing's 2 slots (3.1 x 12 mm each) and thread into the inserts. The 12 mm
# slot length runs perpendicular to the IMU's long axis, giving +/-6 mm of
# slide adjustment in the plate's X direction.
WT901_CENTER_X = 0.0
WT901_CENTER_Y = -60.0
WT901_SLOT_Y_SPACING = 42.8     # slot-center to slot-center along Y (rotated 90)
WT901_INSERT_R = 4.0 / 2        # phi 4.0 pilot for M3 brass insert (Ruthex RX-M3x5.7);
                                # change to 4.7 mm for Voss-style larger-OD inserts
WT901_BODY_X = 36.1             # housing X (short axis, since rotated 90)
WT901_BODY_Y = 51.5             # housing Y (long axis incl. slot ears)
WT901_BODY_Z = 15.0              # housing height (3 mm flange + 12 mm dome)

# --- Output ------------------------------------------------------------------
OUT_PATH = "/Users/rico/Documents/Robots/slam/cad/Livox_Mid360_Mounting_Plate.dxf"

# --- Build DXF ---------------------------------------------------------------
doc = ezdxf.new("R2018", setup=True)

# Match the Universal plate's units header so the Fusion import workflow is
# identical (set Units -> Millimeters on import; otherwise the plate becomes
# 215,900 mm wide because Fusion reads INSUNITS=6 as meters).
doc.header["$INSUNITS"] = 6
doc.header["$MEASUREMENT"] = 1  # metric

doc.layers.add("CONSTRUCTION", color=8)  # gray
doc.layers.add("LABELS", color=3)        # green

msp = doc.modelspace()

# --- Plate outline (rounded rectangle, centered on origin) ------------------
hx = PLATE_W / 2
hy = PLATE_L / 2
r = CORNER_R

msp.add_line((-hx + r, -hy), (hx - r, -hy))
msp.add_line(( hx,      -hy + r), (hx,      hy - r))
msp.add_line(( hx - r,   hy), (-hx + r,  hy))
msp.add_line((-hx,       hy - r), (-hx,     -hy + r))

msp.add_arc(( hx - r,  hy - r), r, start_angle=0,   end_angle=90)
msp.add_arc((-hx + r,  hy - r), r, start_angle=90,  end_angle=180)
msp.add_arc((-hx + r, -hy + r), r, start_angle=180, end_angle=270)
msp.add_arc(( hx - r, -hy + r), r, start_angle=270, end_angle=360)

# --- 2020 frame mount holes (M5 t-nut clearance) ----------------------------
# 1 hole per short (top/bottom) bar, 2 holes per long (left/right) bar.
slot_x = hx - EXTRUSION_INSET   # right/left slot centerlines
slot_y = hy - EXTRUSION_INSET   # top/bottom slot centerlines
msp.add_circle(( 0,       slot_y), M5_CLR_R)   # top bar
msp.add_circle(( 0,      -slot_y), M5_CLR_R)   # bottom bar
for sy in (-1, 1):
    msp.add_circle(( slot_x, sy * LONG_SIDE_HOLE_Y), M5_CLR_R)  # right bar (2x)
    msp.add_circle((-slot_x, sy * LONG_SIDE_HOLE_Y), M5_CLR_R)  # left bar (2x)

# --- Mid-360 M3 clearance holes ---------------------------------------------
mx = M3_X / 2  # 24
my = M3_Y / 2  # 18
for sx in (-1, 1):
    for sy in (-1, 1):
        msp.add_circle((sx * mx, MID360_CENTER_Y + sy * my), M3_CLR_R)

# --- Mid-360 locating pins (drawn as circles; extrude in Fusion) ------------
# Pins on the sensor's horizontal centerline at the same X as the M3 columns.
msp.add_circle(( mx, MID360_CENTER_Y), LOC_PIN_R)  # (+18, +25)
msp.add_circle((-mx, MID360_CENTER_Y), LOC_PIN_R)  # (-18, +25)

# --- Cable pass-through -----------------------------------------------------
msp.add_circle((0, CABLE_HOLE_Y), CABLE_HOLE_R)

# --- WT901 IMU pilot holes (phi 4.0 for M3 brass insert) --------------------
for sy in (-1, 1):
    msp.add_circle(
        (WT901_CENTER_X, WT901_CENTER_Y + sy * WT901_SLOT_Y_SPACING / 2),
        WT901_INSERT_R,
    )

# --- Construction geometry (reference, hide/delete after import) ------------
def rect(x0, y0, x1, y1, layer="CONSTRUCTION"):
    msp.add_line((x0, y0), (x1, y0), dxfattribs={"layer": layer})
    msp.add_line((x1, y0), (x1, y1), dxfattribs={"layer": layer})
    msp.add_line((x1, y1), (x0, y1), dxfattribs={"layer": layer})
    msp.add_line((x0, y1), (x0, y0), dxfattribs={"layer": layer})

# Mid-360 body outline (65 x 65 square at sensor center)
bhs = BODY_SIDE / 2
rect(-bhs, MID360_CENTER_Y - bhs, bhs, MID360_CENTER_Y + bhs)

# M3 pattern rectangle (48 x 36 at sensor center)
rect(-mx, MID360_CENTER_Y - my, mx, MID360_CENTER_Y + my)

# WT901 housing outline (36.1 x 51.5 at IMU center, long axis along Y)
wx = WT901_BODY_X / 2
wy = WT901_BODY_Y / 2
rect(WT901_CENTER_X - wx, WT901_CENTER_Y - wy,
     WT901_CENTER_X + wx, WT901_CENTER_Y + wy)

# Mid-360 center crosshair
msp.add_line((-10, MID360_CENTER_Y), (10, MID360_CENTER_Y),
             dxfattribs={"layer": "CONSTRUCTION"})
msp.add_line((0, MID360_CENTER_Y - 10), (0, MID360_CENTER_Y + 10),
             dxfattribs={"layer": "CONSTRUCTION"})

# WT901 center crosshair
msp.add_line((WT901_CENTER_X - 10, WT901_CENTER_Y),
             (WT901_CENTER_X + 10, WT901_CENTER_Y),
             dxfattribs={"layer": "CONSTRUCTION"})
msp.add_line((WT901_CENTER_X, WT901_CENTER_Y - 10),
             (WT901_CENTER_X, WT901_CENTER_Y + 10),
             dxfattribs={"layer": "CONSTRUCTION"})

# --- Labels -----------------------------------------------------------------
def lbl(text, pos, height=4.0, align=TextEntityAlignment.LEFT):
    t = msp.add_text(text, dxfattribs={"layer": "LABELS", "height": height})
    t.set_placement(pos, align=align)

lbl("LIVOX MID-360 + WT901 MOUNTING PLATE", (-hx + 20, hy - 8),  height=4.0)
lbl("5.5 x 8.5 in  |  6 mm ASA",            (-hx + 20, hy - 14), height=3.0)

lbl("LIVOX MID-360",                  (35, MID360_CENTER_Y - 5))
lbl("36 x 48 M3 pattern (long axis +Y)", (35, MID360_CENTER_Y - 12), height=3.0)
lbl("CABLE PASS-THROUGH (M12 +Y)",    (-50, CABLE_HOLE_Y + 12), height=3.0)
lbl("WT901BLECL5.0 IMU",              (35, WT901_CENTER_Y))
lbl("42.8 mm slot mount, M3 insert (rotated 90, X-arrow fwd)",  (35, WT901_CENTER_Y - 7), height=3.0)
lbl("M5 -> 2020 T-NUT (6x: 1 ea short, 2 ea long)", (-hx + 20, hy - 25), height=3.0)

# --- Save -------------------------------------------------------------------
doc.saveas(OUT_PATH)
print(f"Saved: {OUT_PATH}")

# --- Summary ----------------------------------------------------------------
print("\n=== Plate Summary ===")
print(f"Plate: {PLATE_W} x {PLATE_L} mm, 6 mm ASA, R{CORNER_R} corners")
print(f"2020 frame mounts: 6x M5 clearance:")
print(f"  short bars (top/bottom): (0, +/-{slot_y})")
print(f"  long bars (left/right):  (+/-{slot_x}, +/-{LONG_SIDE_HOLE_Y})")
print(f"Mid-360 (centered at Y={MID360_CENTER_Y:+}):")
print(f"  M3 clearance holes: 4x phi {M3_CLR_R*2:.2f} mm at "
      f"(+/-{mx}, {MID360_CENTER_Y - my:+}) / (+/-{mx}, {MID360_CENTER_Y + my:+})")
print(f"  Locating pins: 2x phi {LOC_PIN_R*2:.2f} mm at "
      f"({-mx:+}, {MID360_CENTER_Y:+}) / ({+mx:+}, {MID360_CENTER_Y:+})")
print(f"  Cable hole: 1x phi {CABLE_HOLE_R*2:.2f} mm at (0, {CABLE_HOLE_Y:+})")
print(f"WT901 IMU (centered at ({WT901_CENTER_X:+}, {WT901_CENTER_Y:+}), rotated 90, X-arrow fwd):")
print(f"  Insert pilots: 2x phi {WT901_INSERT_R*2:.2f} mm at "
      f"({WT901_CENTER_X:+}, +/-{WT901_SLOT_Y_SPACING/2} offset from {WT901_CENTER_Y:+})")
