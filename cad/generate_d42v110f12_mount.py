#!/usr/bin/env python3
"""Generate a DXF mounting plate for the Pololu D42V110F12 on 2020 extrusion."""

import ezdxf

doc = ezdxf.new(dxfversion="R2013")
doc.units = ezdxf.units.MM
msp = doc.modelspace()

# --- Pololu D42V110F12 dimensions (from official drawing) ---
board_l = 43.2        # length (mm)
board_w = 31.8        # width (mm)
inner_l = 38.9        # main PCB body length
inner_w = 25.4        # main PCB body width
hole_dia = 3.2        # M2 heat-set insert hole diameter (insert: M2x4, OD 3.2mm)
hole_spacing_x = 38.9 # hole center-to-center horizontal
hole_spacing_y = 25.4 # hole center-to-center vertical

# --- Plate thickness (for Fusion 360 extrude) ---
plate_thickness = 7.0     # total plate thickness (mm)
insert_depth = 4.0        # M2 heat-set insert depth (mm)

# --- 2020 extrusion ---
extrusion_w = 20.0
m5_clearance = 5.5    # M5 clearance hole diameter

# --- Plate layout ---
# Board centered on plate, M5 holes flanking on each side along the length
# Plate width = board_width + margins for M5 holes
margin_side = 12.0    # margin each side for M5 holes
margin_end = 5.0      # margin top/bottom beyond board

plate_w = board_w + 2 * margin_side   # ~55.8 mm
plate_l = board_l + 2 * margin_end    # ~53.2 mm

# Round plate to nice numbers
plate_w = 56.0
plate_l = 54.0

# Origin at plate center
px = plate_l / 2
py = plate_w / 2

# --- Board position (centered on plate) ---
bx = px  # board center x
by = py  # board center y

# Board corner (bottom-left of board outline)
board_x0 = bx - board_l / 2
board_y0 = by - board_w / 2

# =====================================================================
# Layer setup
# =====================================================================
doc.layers.add("PLATE_OUTLINE", color=7)      # white
doc.layers.add("BOARD_OUTLINE", color=5)       # blue
doc.layers.add("BOARD_HOLES", color=3)         # green
doc.layers.add("M5_HOLES", color=1)            # red
doc.layers.add("DIMENSIONS", color=2)          # yellow
doc.layers.add("LABELS", color=4)              # cyan
doc.layers.add("EXTRUSION_REF", color=8)       # grey

# =====================================================================
# Plate outline (what you'll cut)
# =====================================================================
msp.add_lwpolyline(
    [
        (0, 0), (plate_l, 0), (plate_l, plate_w), (0, plate_w), (0, 0)
    ],
    dxfattribs={"layer": "PLATE_OUTLINE"},
)

# =====================================================================
# Board outline (reference — the PCB sits here)
# =====================================================================
msp.add_lwpolyline(
    [
        (board_x0, board_y0),
        (board_x0 + board_l, board_y0),
        (board_x0 + board_l, board_y0 + board_w),
        (board_x0, board_y0 + board_w),
        (board_x0, board_y0),
    ],
    dxfattribs={"layer": "BOARD_OUTLINE"},
)

# =====================================================================
# Board mounting holes (M2 heat-set insert, 3.2mm dia, 4mm deep) — 4 corners
# =====================================================================
# Hole positions relative to board bottom-left corner
# Bottom-left hole is at board edge inset: x=2.15, y=3.2
# But easier: holes are at (hole_spacing) centered on board
hole_cx = bx  # center of hole pattern
hole_cy = by

hole_positions = [
    (hole_cx - hole_spacing_x / 2, hole_cy - hole_spacing_y / 2),  # BL
    (hole_cx + hole_spacing_x / 2, hole_cy - hole_spacing_y / 2),  # BR
    (hole_cx - hole_spacing_x / 2, hole_cy + hole_spacing_y / 2),  # TL
    (hole_cx + hole_spacing_x / 2, hole_cy + hole_spacing_y / 2),  # TR
]

for hx, hy in hole_positions:
    msp.add_circle((hx, hy), radius=hole_dia / 2, dxfattribs={"layer": "BOARD_HOLES"})

# =====================================================================
# M5 mounting holes for 2020 extrusion
# 2020 T-slot center is 10mm from edge. Place M5 holes along both
# long sides of the plate, centered on the extrusion slot.
# Two holes per side, spaced to straddle the board.
# =====================================================================
m5_y_left = extrusion_w / 2                        # 10mm — T-slot center of bottom extrusion
m5_y_right = plate_w - extrusion_w / 2             # 46mm — T-slot center of top extrusion

# Vertical positions: two holes per side, evenly spaced
m5_x_positions = [plate_l / 3, 2 * plate_l / 3]

m5_holes = []
for mx in m5_x_positions:
    m5_holes.append((mx, m5_y_left))
    m5_holes.append((mx, m5_y_right))

for mx, my in m5_holes:
    msp.add_circle((mx, my), radius=m5_clearance / 2, dxfattribs={"layer": "M5_HOLES"})

# =====================================================================
# 2020 extrusion reference lines (where extrusion sits under the plate)
# =====================================================================
# Left extrusion
ext_y0_l = 0
ext_y1_l = extrusion_w
msp.add_line((0, ext_y0_l), (plate_l, ext_y0_l), dxfattribs={"layer": "EXTRUSION_REF"})
msp.add_line((0, ext_y1_l), (plate_l, ext_y1_l), dxfattribs={"layer": "EXTRUSION_REF"})

# Right extrusion
ext_y0_r = plate_w - extrusion_w
ext_y1_r = plate_w
msp.add_line((0, ext_y0_r), (plate_l, ext_y0_r), dxfattribs={"layer": "EXTRUSION_REF"})
msp.add_line((0, ext_y1_r), (plate_l, ext_y1_r), dxfattribs={"layer": "EXTRUSION_REF"})

# Center lines of extrusions (where T-slot is)
msp.add_line((0, extrusion_w / 2), (plate_l, extrusion_w / 2),
             dxfattribs={"layer": "EXTRUSION_REF"})
msp.add_line((0, plate_w - extrusion_w / 2), (plate_l, plate_w - extrusion_w / 2),
             dxfattribs={"layer": "EXTRUSION_REF"})

# =====================================================================
# Labels
# =====================================================================
msp.add_text(
    "D42V110F12",
    height=2.5,
    dxfattribs={"layer": "LABELS", "insert": (bx - 10, by - 1)},
)
msp.add_text(
    "M2 insert (3.2mm, 4mm deep) x4",
    height=1.5,
    dxfattribs={"layer": "LABELS", "insert": (hole_positions[0][0] + 3, hole_positions[0][1] - 3)},
)
msp.add_text(
    "M5 (5.5mm) x4",
    height=1.5,
    dxfattribs={"layer": "LABELS", "insert": (m5_holes[0][0] + 3, m5_holes[0][1] - 3)},
)
msp.add_text(
    f"Plate: {plate_l} x {plate_w} mm",
    height=1.5,
    dxfattribs={"layer": "LABELS", "insert": (2, plate_w + 3)},
)

# =====================================================================
# Save
# =====================================================================
doc.header['$EXTMIN'] = (0, 0, 0)
doc.header['$EXTMAX'] = (plate_l, plate_w, 0)

output = "/private/tmp/d42v110f12_2020_mount.dxf"
doc.saveas(output)
print(f"DXF saved to {output}")
print(f"  Plate:  {plate_l} x {plate_w} mm")
print(f"  Board:  {board_l} x {board_w} mm (centered)")
print(f"  Thickness: {plate_thickness}mm (insert depth: {insert_depth}mm)")
print(f"  Board holes: M2 insert (3.2mm dia, {insert_depth}mm deep) x4, spacing {hole_spacing_x} x {hole_spacing_y} mm")
print(f"  M5 holes: {m5_clearance}mm dia x4 for 2020 T-slot")
print(f"  M5 Y positions: {m5_y_left:.1f}mm and {m5_y_right:.1f}mm (extrusion centerlines: 10mm and {plate_w-10:.1f}mm)")
