/*
  Clodagh T-Echo Lite dog tracker collar case, prototype 1.

  Design goals:
  - LilyGO T-Echo Lite with GPS add-on fitted.
  - No e-paper display opening.
  - 953450 LiPo cell inside the case. Typical size: 50 x 34 x 9.5 mm.
  - 25 mm collar strap channels underneath.
  - USB accessible through one side for charging/flashing.

  Source measurements used from this repo:
  - structure/总装.stl stock assembled shell bounding box: about 41.0 x 41.0 x 14.5 mm.
  - project/T-Echo-Lite_V1.0/T-Echo-Lite-Core_V1.0/kicad board core outline:
    about 20.9 x 27.7 mm, but the stock shell dimension is a better envelope reference.

  Print bottom and lid separately:
  - Set part = "bottom" for the base.
  - Set part = "lid" for the lid.
  - Set part = "preview" to see both next to each other.

  This is a first fitting model. Measure the printed fit before using it on the dog.
*/

part = "preview"; // "bottom", "lid", or "preview"
$fn = 48;

// Main tuning dimensions, in mm.
wall = 2.2;
corner_radius = 6;
case_x = 62;
case_y = 48;
bottom_h = 15.5;
lid_h = 8.0;
lip_h = 2.0;
lip_clearance = 0.30;

// Internal hardware envelopes.
board_x = 43.0;
board_y = 43.0;
board_clearance = 0.8;
board_z_clearance = 4.2;

battery_x = 52.0; // 50 mm cell plus padding.
battery_y = 36.0; // 34 mm cell plus padding.
battery_z = 10.5; // 9.5 mm cell plus padding.

gps_bump_x = 28;
gps_bump_y = 18;
gps_bump_z = 4.0;
gps_bump_y_offset = 7;

strap_width = 25;
strap_clearance = 1.2;
strap_slot_w = strap_width + 2 * strap_clearance;
strap_slot_h = 3.2;
strap_channel_y = 8.0;
strap_channel_z = 2.2;
strap_channel_x = case_x + 2;

usb_cut_w = 13;
usb_cut_h = 7;
usb_cut_z = 7.8;

screw_d = 2.4;       // Clearance for M2/M2.2 screw.
screw_head_d = 4.8;
post_d = 6.2;
post_x = case_x / 2 - 8;
post_y = case_y / 2 - 8;

module rounded_box(size, r)
{
    hull()
    {
        for (x = [-size[0] / 2 + r, size[0] / 2 - r])
            for (y = [-size[1] / 2 + r, size[1] / 2 - r])
                translate([x, y, 0])
                    cylinder(h = size[2], r = r);
    }
}

module screw_positions()
{
    for (x = [-post_x, post_x])
        for (y = [-post_y, post_y])
            translate([x, y, 0])
                children();
}

module strap_cutouts()
{
    for (y = [-strap_channel_y, strap_channel_y])
        translate([0, y, strap_channel_z])
            cube([strap_channel_x, strap_slot_w, strap_slot_h], center = true);
}

module usb_cutout()
{
    // Side opening centered on the long side. Enlarge after first fit if your plug is chunky.
    translate([case_x / 2 + 0.2, 0, usb_cut_z])
        cube([wall + 1.0, usb_cut_w, usb_cut_h], center = true);
}

module bottom_shell()
{
    difference()
    {
        union()
        {
            rounded_box([case_x, case_y, bottom_h], corner_radius);

            // Screw posts.
            screw_positions()
                cylinder(h = bottom_h - 1.0, d = post_d);

            // Board support rails over the battery pocket.
            translate([0, -case_y / 2 + 10, battery_z + 1.2])
                cube([board_x + 3, 2.2, 2.0], center = true);
            translate([0, case_y / 2 - 10, battery_z + 1.2])
                cube([board_x + 3, 2.2, 2.0], center = true);
        }

        // Main hollow cavity.
        translate([0, 0, wall])
            rounded_box([case_x - 2 * wall, case_y - 2 * wall, bottom_h], corner_radius - wall);

        // Battery pocket below the board.
        translate([0, 0, wall + battery_z / 2])
            cube([battery_x, battery_y, battery_z], center = true);

        // Board clearance pocket.
        translate([0, 0, wall + battery_z + board_z_clearance / 2])
            cube([board_x + board_clearance, board_y + board_clearance, board_z_clearance], center = true);

        // Collar strap channels pass across the underside.
        strap_cutouts();

        // USB access on right-hand side.
        usb_cutout();

        // Screw holes.
        screw_positions()
            translate([0, 0, -0.5])
                cylinder(h = bottom_h + 1.0, d = screw_d);
    }
}

module lid_shell()
{
    difference()
    {
        union()
        {
            rounded_box([case_x, case_y, lid_h], corner_radius);

            // Internal lip that drops into the bottom shell.
            translate([0, 0, -lip_h])
                difference()
                {
                    rounded_box([case_x - 2 * wall - lip_clearance, case_y - 2 * wall - lip_clearance, lip_h], corner_radius - wall);
                    translate([0, 0, -0.1])
                        rounded_box([case_x - 4 * wall, case_y - 4 * wall, lip_h + 0.2], max(1, corner_radius - 2 * wall));
                }
        }

        // Hollow lid interior and GPS module clearance.
        translate([0, 0, wall])
            rounded_box([case_x - 2 * wall, case_y - 2 * wall, lid_h], corner_radius - wall);

        translate([0, gps_bump_y_offset, wall + gps_bump_z / 2])
            cube([gps_bump_x, gps_bump_y, gps_bump_z], center = true);

        // Screw clearance/counterbore from the top.
        screw_positions()
        {
            translate([0, 0, -0.5])
                cylinder(h = lid_h + lip_h + 1.0, d = screw_d);
            translate([0, 0, lid_h - 2.4])
                cylinder(h = 3.0, d = screw_head_d);
        }

        // Matching USB opening through the lid wall edge.
        usb_cutout();
    }
}

module preview_hardware()
{
    color("royalblue", 0.35)
        translate([0, 0, wall + battery_z + 1.0])
            cube([41, 41, 1.6], center = true);

    color("silver", 0.35)
        translate([0, 0, wall + battery_z / 2])
            cube([50, 34, 9.5], center = true);

    color("darkgreen", 0.35)
        translate([0, gps_bump_y_offset, wall + battery_z + 4])
            cube([24, 15, 2], center = true);
}

if (part == "bottom")
{
    bottom_shell();
}
else if (part == "lid")
{
    lid_shell();
}
else
{
    bottom_shell();
    preview_hardware();
    translate([case_x + 12, 0, lid_h])
        rotate([180, 0, 0])
            lid_shell();
}
