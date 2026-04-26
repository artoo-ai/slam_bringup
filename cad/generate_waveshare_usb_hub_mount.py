#!/usr/bin/env python3
"""Generate a DXF mounting plate for the Waveshare USB3.2-Gen1-HUB-4U on 2040 extrusion."""

import ezdxf

doc = ezdxf.new(dxfversion="R2013")
doc.units = ezdxf.units.MM
msp = doc.modelspace()

# --- Waveshare USB3.2-Gen1-HUB-4U dimensions (from official drawing) ---
enclosure_l = 86.00      # total length with wings (mm)
enclosure_w = 47.80      # total width (mm)
body_l = 72.20           # main body length (without wings)
body_w = 44.80           # main body width
enclosure_h = 27.60      # enclosure height/depth

# --- Wing dimensions (measured) ---
wing_w = 0.284 * 25.4               # 7.2136mm wing width
wing_len = 1.780 * 25.4             # 45.212mm wing length

# --- Slot positions (measured from outside enclosure edges) ---
slot_inset_short = 0.225 * 25.4     # 5.715mm from short side edge (top/bottom)
slot_inset_long = 0.083 * 25.4      # 2.1082mm from long side edge (left/right)
slot_cc = 3.090 * 25.4              # 78.486mm center-to-center between left/right slots
slot_length = 10.0                  # slot length (mm), oriented vertically

# Derived slot geometry
slot_center_x = (enclosure_l - slot_cc) / 2   # 3.757mm from each side edge
slot_width = (slot_center_x - slot_inset_long) * 2  # ~3.298mm slot width
slot_center_y_bot = slot_inset_short + slot_length / 2  # 10.715mm from bottom
slot_center_y_top = enclosure_w - slot_inset_short - slot_length / 2  # 37.085mm from bottom

# Heat-set insert holes at slot centers
hub_hole_dia = 4.0       # M3 heat-set insert hole diameter (insert: M3x5.7, OD 4.0mm)

hub_hole_positions_on_enclosure = [
    (slot_center_x, slot_center_y_bot),                          # BL
    (enclosure_l - slot_center_x, slot_center_y_bot),            # BR
    (slot_center_x, slot_center_y_top),                          # TL
    (enclosure_l - slot_center_x, slot_center_y_top),            # TR
]

# --- Plate thickness (for Fusion 360 extrude) ---
plate_thickness = 7.0     # total plate thickness (mm)
insert_depth = 5.7        # M3 heat-set insert depth (mm)

# --- 2040 extrusion (single, 40mm face up) ---
extrusion_w = 40.0        # 40mm face up, T-slots at 10mm and 30mm from edge
extrusion_h = 20.0        # 20mm tall
m5_clearance = 5.5        # M5 clearance hole diameter

# --- Plate layout ---
margin_end = 20.0         # extended ends for M5 holes beyond hub
margin_side = 5.0

plate_l = 126.0
plate_w = 58.0

# --- Hub position (centered on plate) ---
hub_x0 = (plate_l - enclosure_l) / 2
hub_y0 = (plate_w - enclosure_w) / 2
hub_cx = plate_l / 2
hub_cy = plate_w / 2

# Body position on plate
body_x0 = hub_x0 + (enclosure_l - body_l) / 2
body_y0 = hub_y0 + (enclosure_w - body_w) / 2

# Wing positions on plate (centered vertically on enclosure)
wing_y_offset = (enclosure_w - wing_len) / 2

# --- 2040 extrusion position (centered under plate) ---
ext_overhang = (plate_w - extrusion_w) / 2    # 9mm overhang each side
tslot_1 = ext_overhang + 10.0                 # first T-slot center
tslot_2 = ext_overhang + 30.0                 # second T-slot center

# =====================================================================
# Layer setup
# =====================================================================
doc.layers.add("PLATE_OUTLINE", color=7)      # white
doc.layers.add("HUB_OUTLINE", color=5)        # blue  — 86mm enclosure
doc.layers.add("HUB_BODY", color=6)           # magenta — 72.20mm body
doc.layers.add("WINGS", color=14)             # dark grey — wing outlines
doc.layers.add("SLOTS", color=30)             # orange — slot rectangles
doc.layers.add("HUB_HOLES", color=3)          # green — insert holes
doc.layers.add("M5_HOLES", color=1)           # red
doc.layers.add("LABELS", color=4)             # cyan
doc.layers.add("EXTRUSION_REF", color=8)      # grey

# =====================================================================
# Plate outline
# =====================================================================
msp.add_lwpolyline(
    [(0, 0), (plate_l, 0), (plate_l, plate_w), (0, plate_w), (0, 0)],
    dxfattribs={"layer": "PLATE_OUTLINE"},
)

# =====================================================================
# Hub enclosure outline — 86.00 x 47.80mm (construction reference)
# =====================================================================
msp.add_lwpolyline(
    [
        (hub_x0, hub_y0),
        (hub_x0 + enclosure_l, hub_y0),
        (hub_x0 + enclosure_l, hub_y0 + enclosure_w),
        (hub_x0, hub_y0 + enclosure_w),
        (hub_x0, hub_y0),
    ],
    dxfattribs={"layer": "HUB_OUTLINE"},
)

# =====================================================================
# Hub body outline — 72.20 x 44.80mm (construction reference)
# =====================================================================
msp.add_lwpolyline(
    [
        (body_x0, body_y0),
        (body_x0 + body_l, body_y0),
        (body_x0 + body_l, body_y0 + body_w),
        (body_x0, body_y0 + body_w),
        (body_x0, body_y0),
    ],
    dxfattribs={"layer": "HUB_BODY"},
)

# =====================================================================
# Wing outlines — 4 wings at corners (7.214mm x 45.212mm each)
# =====================================================================
wing_y0 = hub_y0 + wing_y_offset
wing_y1 = wing_y0 + wing_len

# Left wing
lw_x0 = hub_x0
lw_x1 = hub_x0 + wing_w
msp.add_lwpolyline(
    [(lw_x0, wing_y0), (lw_x1, wing_y0), (lw_x1, wing_y1), (lw_x0, wing_y1), (lw_x0, wing_y0)],
    dxfattribs={"layer": "WINGS"},
)

# Right wing
rw_x0 = hub_x0 + enclosure_l - wing_w
rw_x1 = hub_x0 + enclosure_l
msp.add_lwpolyline(
    [(rw_x0, wing_y0), (rw_x1, wing_y0), (rw_x1, wing_y1), (rw_x0, wing_y1), (rw_x0, wing_y0)],
    dxfattribs={"layer": "WINGS"},
)

# =====================================================================
# Slot rectangles — 4 slots (slot_width x 10mm each)
# =====================================================================
slot_half_w = slot_width / 2
slot_half_l = slot_length / 2

slot_rects = [
    # BL slot
    (hub_x0 + slot_center_x - slot_half_w, hub_y0 + slot_center_y_bot - slot_half_l,
     hub_x0 + slot_center_x + slot_half_w, hub_y0 + slot_center_y_bot + slot_half_l),
    # BR slot
    (hub_x0 + enclosure_l - slot_center_x - slot_half_w, hub_y0 + slot_center_y_bot - slot_half_l,
     hub_x0 + enclosure_l - slot_center_x + slot_half_w, hub_y0 + slot_center_y_bot + slot_half_l),
    # TL slot
    (hub_x0 + slot_center_x - slot_half_w, hub_y0 + slot_center_y_top - slot_half_l,
     hub_x0 + slot_center_x + slot_half_w, hub_y0 + slot_center_y_top + slot_half_l),
    # TR slot
    (hub_x0 + enclosure_l - slot_center_x - slot_half_w, hub_y0 + slot_center_y_top - slot_half_l,
     hub_x0 + enclosure_l - slot_center_x + slot_half_w, hub_y0 + slot_center_y_top + slot_half_l),
]

for x0, y0, x1, y1 in slot_rects:
    msp.add_lwpolyline(
        [(x0, y0), (x1, y0), (x1, y1), (x0, y1), (x0, y0)],
        dxfattribs={"layer": "SLOTS"},
    )

# =====================================================================
# Hub mounting holes (M3 heat-set insert) — at slot centers
# =====================================================================
for fx, fy in hub_hole_positions_on_enclosure:
    abs_x = hub_x0 + fx
    abs_y = hub_y0 + fy
    msp.add_circle((abs_x, abs_y), radius=hub_hole_dia / 2,
                   dxfattribs={"layer": "HUB_HOLES"})

# =====================================================================
# M5 mounting holes for 2040 extrusion
# At plate ends, beyond hub footprint — on both T-slot lines
# =====================================================================
m5_x_margin = 10.0
m5_x_positions = [m5_x_margin, plate_l - m5_x_margin]

m5_holes = []
for mx in m5_x_positions:
    m5_holes.append((mx, tslot_1))
    m5_holes.append((mx, tslot_2))

for mx, my in m5_holes:
    msp.add_circle((mx, my), radius=m5_clearance / 2,
                   dxfattribs={"layer": "M5_HOLES"})

# =====================================================================
# 2040 extrusion reference lines (single 2040, 40mm face up, centered)
# =====================================================================
msp.add_line((0, ext_overhang), (plate_l, ext_overhang),
             dxfattribs={"layer": "EXTRUSION_REF"})
msp.add_line((0, ext_overhang + extrusion_w), (plate_l, ext_overhang + extrusion_w),
             dxfattribs={"layer": "EXTRUSION_REF"})
msp.add_line((0, tslot_1), (plate_l, tslot_1),
             dxfattribs={"layer": "EXTRUSION_REF"})
msp.add_line((0, tslot_2), (plate_l, tslot_2),
             dxfattribs={"layer": "EXTRUSION_REF"})

# =====================================================================
# Labels
# =====================================================================
msp.add_text(
    "USB3.2-Gen1-HUB-4U",
    height=2.5,
    dxfattribs={"layer": "LABELS", "insert": (hub_cx - 18, hub_cy - 1)},
)
msp.add_text(
    f"M3 insert ({hub_hole_dia}mm, {insert_depth}mm deep) x4",
    height=1.5,
    dxfattribs={"layer": "LABELS",
                "insert": (hub_x0 + hub_hole_positions_on_enclosure[0][0] + 3,
                           hub_y0 + hub_hole_positions_on_enclosure[0][1] - 3)},
)
msp.add_text(
    f"M5 ({m5_clearance}mm) x{len(m5_holes)}",
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

output = "/private/tmp/waveshare_usb_hub_2040_mount.dxf"
doc.saveas(output)
print(f"DXF saved to {output}")
print(f"  Plate:  {plate_l} x {plate_w} mm")
print(f"  Hub enclosure: {enclosure_l} x {enclosure_w} mm (centered)")
print(f"  Hub body: {body_l} x {body_w} mm")
print(f"  Wings: {wing_w:.3f} x {wing_len:.3f} mm (4 corners)")
print(f"  Slots: {slot_width:.3f} x {slot_length} mm (4 slots)")
print(f"  Slot centers: X={slot_center_x:.3f}mm from side, Y={slot_center_y_bot:.3f}mm / {slot_center_y_top:.3f}mm from bottom")
print(f"  Slot center-to-center (X): {slot_cc:.3f}mm ({slot_cc/25.4:.3f}\")")
print(f"  Thickness: {plate_thickness}mm (insert depth: {insert_depth}mm)")
print(f"  Hub holes: M3 insert ({hub_hole_dia}mm dia, {insert_depth}mm deep) x4")
print(f"  M5 holes: {m5_clearance}mm dia x{len(m5_holes)} for 2040 T-slot")
print(f"  2040 extrusion (40mm face): centered at Y={ext_overhang:.1f} to {ext_overhang + extrusion_w:.1f}mm")
print(f"  T-slot centers at Y={tslot_1:.1f}mm and Y={tslot_2:.1f}mm")
print(f"  M5 holes at X={m5_x_margin:.0f}mm and X={plate_l - m5_x_margin:.0f}mm (beyond hub)")
