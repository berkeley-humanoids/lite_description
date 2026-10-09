% scale(1000) import("right_shoulder_yaw.stl");

// Upper arm. One capsule from the shoulder outline to the elbow.

translate([195.82, 0, 0]) rotate([0, 90, -180])
    cylinder(r=35, h=188.35, center=true);
