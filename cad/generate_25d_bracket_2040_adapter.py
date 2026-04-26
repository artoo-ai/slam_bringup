#!/usr/bin/env python3
"""Generate a DXF adapter plate to mount a 25D motor bracket to 2040 extrusion.

Bracket: JIUWU 25mm DC Gearbox Motor Bracket (Amazon B07HHY2BJN)
Motor:   Pololu #4887 99:1 Metal Gearmotor 25Dx69L mm LP 12V
Extrusion: 2040 aluminum (40mm face)

The bracket mounting plate (26mm x 33mm) sits on the adapter plate.
The motor bolts to the bracket from above; the shaft passes through
the center hole in both the bracket and the adapter plate.

    TOP VIEW (bracket side up):

       front (motor side)
       y=0
        +------------------------------------------+
        |  (5mm front margin for bracket bend)     |
        |  +---- bracket plate (26x33) --------+  |
        |  |                                    |  |
        |  |  o(12,12)             o(28,12)    |  |  <- M3 heat-set
        |  |                                    |  |
        |  |          O(20,19)                 |  |  <- shaft clearance
        |  |                                    |  |
        |  |  o(12,26)             o(28,26)    |  |  <- M3 heat-set
        |  |                                    |  |
        |  +------------------------------------+  |
        |         (7mm gap)                        |
        |     O(10,45)             O(30,45)        |  <- M5 T-slot
        |         (13mm between rows)              |
        |     O(10,58)             O(30,58)        |  <- M5 T-slot
        |         (7mm edge margin)                |
        +------------------------------------------+
       y=65 (back)
"""

import ezdxf

doc = ezdxf.new(dxfversion="R2013")
doc.units = ezdxf.units.MM
msp = doc.modelspace()

# === Bracket mounting plate dimensions (from product image, right piece) ===
bracket_width = 26.0          # mm, plate width
bracket_depth = 33.0          # mm, plate height (runs along the 2040)
bracket_thickness = 1.5       # mm, material thickness
bracket_center_hole_dia = 10.0  # mm, center hole (motor shaft clearance)
bracket_center_y_from_bottom = 14.0  # mm, center hole position from bracket bottom edge

# 4 mounting holes in a rectangular pattern centered on the center hole
bracket_hole_h_spacing = 16.0  # mm, horizontal spacing (across 26mm bracket width)
bracket_hole_v_spacing = 14.0  # mm, vertical spacing (along 33mm bracket depth)

# === M3 heat-set insert ===
m3_heat_insert_dia = 4.0      # mm, hole diameter for M3 heat-set insert (OD ~4.0mm)

# === 2040 extrusion (40mm face up) ===
extrusion_width = 40.0        # mm, face width
extrusion_height = 20.0       # mm, profile height
tslot_center_1 = 10.0         # mm, first T-slot center from left edge
tslot_center_2 = 30.0         # mm, second T-slot center from left edge
m5_clearance = 5.5            # mm, M5 clearance hole diameter

# === Plate layout ===
plate_width = 40.0            # mm, matches 2040 face width
plate_thickness = 5.0         # mm, enough depth for M3 heat-set inserts

# Front margin: extra material so the bracket's 90° bend sits fully on the plate
front_margin = 5.0            # mm

# Bracket centered on plate width, inset from front edge by front_margin
bracket_x_offset = (plate_width - bracket_width) / 2   # 7mm from each side
bracket_y_offset = front_margin                          # bracket starts 5mm from front edge

# T-slot holes behind the bracket with generous spacing
tslot_gap = 7.0               # mm, gap between bracket back edge and first T-slot row
tslot_row_spacing = 13.0      # mm, between T-slot rows (M5 button head is 9.5mm dia)
tslot_edge_margin = 7.0       # mm, from last T-slot row to plate back edge

bracket_back_y = bracket_y_offset + bracket_depth                         # 38mm
tslot_row1_y = bracket_back_y + tslot_gap                                 # 45mm
tslot_row2_y = tslot_row1_y + tslot_row_spacing                          # 58mm
plate_depth = tslot_row2_y + tslot_edge_margin                           # 65mm

# === Hole positions on the adapter plate ===

# Motor shaft clearance hole (oversized for easy assembly)
shaft_clearance_dia = 12.0
shaft_x = bracket_x_offset + bracket_width / 2                            # 20mm
shaft_y = bracket_y_offset + bracket_center_y_from_bottom                  # 19mm

# 4x M3 heat-set insert holes — 16mm H x 14mm V rectangle centered on shaft hole
half_h = bracket_hole_h_spacing / 2  # 8mm (horizontal, across plate width)
half_v = bracket_hole_v_spacing / 2  # 7mm (vertical, along plate depth)
insert_holes = [
    (shaft_x - half_h, shaft_y - half_v),   # (12, 12)
    (shaft_x + half_h, shaft_y - half_v),   # (28, 12)
    (shaft_x - half_h, shaft_y + half_v),   # (12, 26)
    (shaft_x + half_h, shaft_y + half_v),   # (28, 26)
]

# 4x M5 T-slot holes — well behind the bracket footprint
tslot_holes = [
    (tslot_center_1, tslot_row1_y),   # (10, 45)
    (tslot_center_2, tslot_row1_y),   # (30, 45)
    (tslot_center_1, tslot_row2_y),   # (10, 58)
    (tslot_center_2, tslot_row2_y),   # (30, 58)
]

# === Layer setup ===
doc.layers.add("PLATE_OUTLINE", color=7)      # white — cut line
doc.layers.add("BRACKET_OUTLINE", color=5)    # blue — reference
doc.layers.add("SHAFT_HOLE", color=3)         # green — motor shaft clearance
doc.layers.add("HEAT_INSERT_HOLES", color=6)  # magenta — M3 heat-set inserts
doc.layers.add("M5_HOLES", color=1)           # red — T-slot bolts
doc.layers.add("LABELS", color=4)             # cyan
doc.layers.add("EXTRUSION_REF", color=8)      # grey
doc.layers.add("DIMENSIONS", color=2)         # yellow

# === Plate outline ===
msp.add_lwpolyline(
    [(0, 0), (plate_width, 0), (plate_width, plate_depth),
     (0, plate_depth), (0, 0)],
    dxfattribs={"layer": "PLATE_OUTLINE"},
)

# === Bracket plate outline (reference) ===
bx0 = bracket_x_offset
by0 = bracket_y_offset
msp.add_lwpolyline(
    [(bx0, by0), (bx0 + bracket_width, by0),
     (bx0 + bracket_width, by0 + bracket_depth),
     (bx0, by0 + bracket_depth), (bx0, by0)],
    dxfattribs={"layer": "BRACKET_OUTLINE"},
)

# === Motor shaft clearance hole ===
msp.add_circle(
    (shaft_x, shaft_y), radius=shaft_clearance_dia / 2,
    dxfattribs={"layer": "SHAFT_HOLE"},
)

# === M3 heat-set insert holes (4x, square pattern) ===
for ix, iy in insert_holes:
    msp.add_circle(
        (ix, iy), radius=m3_heat_insert_dia / 2,
        dxfattribs={"layer": "HEAT_INSERT_HOLES"},
    )

# === M5 T-slot holes (4x, behind bracket) ===
for tx, ty in tslot_holes:
    msp.add_circle(
        (tx, ty), radius=m5_clearance / 2,
        dxfattribs={"layer": "M5_HOLES"},
    )

# === 2040 extrusion reference ===
msp.add_line((0, 0), (0, plate_depth), dxfattribs={"layer": "EXTRUSION_REF"})
msp.add_line((extrusion_width, 0), (extrusion_width, plate_depth),
             dxfattribs={"layer": "EXTRUSION_REF"})
for tc in [tslot_center_1, tslot_center_2]:
    msp.add_line((tc, 0), (tc, plate_depth), dxfattribs={"layer": "EXTRUSION_REF"})

# === Labels ===
msp.add_text(
    f"Plate: {plate_width} x {plate_depth} x {plate_thickness}mm",
    height=1.5,
    dxfattribs={"layer": "LABELS", "insert": (1, plate_depth + 3)},
)
msp.add_text(
    f"Bracket: {bracket_width} x {bracket_depth}mm",
    height=1.2,
    dxfattribs={"layer": "LABELS", "insert": (bx0 + 1, by0 + bracket_depth + 1)},
)
msp.add_text(
    f"Shaft: {shaft_clearance_dia}mm dia",
    height=1.2,
    dxfattribs={"layer": "LABELS", "insert": (shaft_x + 7, shaft_y - 0.5)},
)
msp.add_text(
    f"M3 insert ({m3_heat_insert_dia}mm) x4, {bracket_hole_h_spacing}x{bracket_hole_v_spacing}mm",
    height=1.2,
    dxfattribs={"layer": "LABELS", "insert": (insert_holes[1][0] + 3, insert_holes[1][1] - 0.5)},
)
msp.add_text(
    f"M5 ({m5_clearance}mm) x4 T-slot",
    height=1.2,
    dxfattribs={"layer": "LABELS", "insert": (tslot_holes[0][0] + 4, tslot_holes[0][1] - 0.5)},
)

# === Dimension lines ===
dim_offset = -4
msp.add_line((0, dim_offset), (plate_width, dim_offset), dxfattribs={"layer": "DIMENSIONS"})
msp.add_line((0, dim_offset - 1), (0, dim_offset + 1), dxfattribs={"layer": "DIMENSIONS"})
msp.add_line((plate_width, dim_offset - 1), (plate_width, dim_offset + 1),
             dxfattribs={"layer": "DIMENSIONS"})
msp.add_text(f"{plate_width}", height=1.5,
             dxfattribs={"layer": "DIMENSIONS", "insert": (plate_width / 2 - 2, dim_offset - 3)})

dim_x = plate_width + 4
msp.add_line((dim_x, 0), (dim_x, plate_depth), dxfattribs={"layer": "DIMENSIONS"})
msp.add_line((dim_x - 1, 0), (dim_x + 1, 0), dxfattribs={"layer": "DIMENSIONS"})
msp.add_line((dim_x - 1, plate_depth), (dim_x + 1, plate_depth),
             dxfattribs={"layer": "DIMENSIONS"})
msp.add_text(f"{plate_depth}", height=1.5,
             dxfattribs={"layer": "DIMENSIONS", "insert": (dim_x + 2, plate_depth / 2 - 1)})

# === Save ===
doc.header["$EXTMIN"] = (-10, -10, 0)
doc.header["$EXTMAX"] = (plate_width + 20, plate_depth + 15, 0)

output_dxf = "/Users/rico/Documents/Robots/slam/cad/25d_bracket_2040_adapter.dxf"
doc.saveas(output_dxf)

print(f"DXF saved to {output_dxf}")
print(f"  Plate:          {plate_width} x {plate_depth} x {plate_thickness}mm")
print(f"  Front margin:   {front_margin}mm (supports bracket bend)")
print(f"  Bracket:        {bracket_width} x {bracket_depth}mm at y={bracket_y_offset}")
print(f"  Shaft hole:     {shaft_clearance_dia}mm dia at ({shaft_x}, {shaft_y})")
print(f"  Insert holes:   M3 ({m3_heat_insert_dia}mm) x4, {bracket_hole_h_spacing}mm H x {bracket_hole_v_spacing}mm V:")
for i, (ix, iy) in enumerate(insert_holes):
    print(f"                  [{i+1}] ({ix}, {iy})")
print(f"  T-slot holes:   M5 ({m5_clearance}mm) x4:")
for i, (tx, ty) in enumerate(tslot_holes):
    print(f"                  [{i+1}] ({tx}, {ty})")
print(f"  T-slot spacing: {tslot_row_spacing}mm between rows, {tslot_edge_margin}mm to edge")
print()
print(f"Verify bracket hole spacing with calipers.")
print(f"  Currently: {bracket_hole_h_spacing}mm horizontal (across width) x {bracket_hole_v_spacing}mm vertical (along depth)")
