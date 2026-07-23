# Clodagh Tracker Collar Case

Prototype 1 case design for the LilyGO T-Echo Lite tracker with GPS add-on, no e-paper display, 953450 battery, USB access, and 25 mm collar strap channels.

## File

- `clodagh_tracker_case.scad` - parametric OpenSCAD source.

## How To Export STL

1. Open `clodagh_tracker_case.scad` in OpenSCAD.
2. Set `part = "bottom";`
3. Press F6, then export STL as `clodagh_tracker_bottom.stl`.
4. Set `part = "lid";`
5. Press F6, then export STL as `clodagh_tracker_lid.stl`.

## Print Notes

- Print in PETG or ASA for outdoor/collar use.
- Use 0.2 mm layers for the first prototype.
- Use 3-4 walls and at least 30% infill.
- Print the bottom with strap channels on the build plate.
- Print the lid flat, outer face upward.

## Current Assumptions

- 953450 battery is treated as 50 x 34 x 9.5 mm.
- Stock LilyGO enclosure from `structure/总装.stl` measures about 41 x 41 x 14.5 mm.
- Case uses a stacked layout: battery below, board/GPS above.
- USB opening is intentionally oversized for first-fit testing.
- Screw holes are sized for small M2/M2.2 hardware.

Measure the first print before ruggedizing. The next revision should tune USB position, screw post height, and GPS/battery clearance from the real fitted parts.
