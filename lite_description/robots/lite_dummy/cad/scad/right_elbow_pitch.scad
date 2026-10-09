% scale(1000) import("right_elbow_pitch.stl");

// Forearm. One capsule from the elbow to the wrist. It also covers the wrist-yaw
// and wrist-roll links.

translate([359.84, 0, 0]) rotate([0, 90, 0])
    cylinder(r=32, h=139.69, center=true);
