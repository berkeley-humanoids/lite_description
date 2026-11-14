% scale(1000) import("right_hip_roll.stl");

// Butt. One capsule flush with the rear face of the hip-roll shell. That face leans
// 24 deg from vertical. At rest, the capsule overlaps the pelvis capsule across the
// hip motors, so physics.json excludes pelvis <-> hip_roll.

translate([70, -32.85, -45.73]) rotate([0, 24, -90])
    cylinder(r=51, h=83.15, center=true);
