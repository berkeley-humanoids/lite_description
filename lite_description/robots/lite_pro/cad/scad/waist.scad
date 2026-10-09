% scale(1000) import("waist.stl");

// Pelvis. Two capsules.

// The pelvis block.
translate([-0.01, 9.64, 91.66]) rotate([0, 90, 180])
    cylinder(r=69, h=39.03, center=true);

// Across both hip-pitch motors, on the hip-pitch axes, to the outer faces of the
// motors. At rest, this capsule overlaps both butt capsules, so physics.json
// excludes pelvis <-> hip_roll.
translate([0.01, 20, 0]) rotate([0, 90, 180])
    cylinder(r=52, h=144.49, center=true);
