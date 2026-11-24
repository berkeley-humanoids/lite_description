% scale(1000) import("waist.stl");

// Pelvis. Three capsules.

// The pelvis block.
translate([-0.01, 9.64, 91.66]) rotate([0, 90, 180])
    cylinder(r=69, h=39.03, center=true);

// Across both hip-pitch motors, on the hip-pitch axes, to the outer faces of the
// motors. At rest, this capsule overlaps both butt capsules, so physics.json
// excludes pelvis <-> hip_roll.
translate([0.01, 20, 0]) rotate([0, 90, 180])
    cylinder(r=52, h=144.49, center=true);

// The battery chest above the waist, tilted 6.45 deg back with its shell. 87% of the
// chest shell lies within 5 mm of this capsule, which reaches at most 18 mm past it.
translate([0, 11.2, 244.8]) rotate([6.45, 0, 0])
    cylinder(r=74, h=190, center=true);
