% scale(1000) import("left_knee_pitch.stl");

// Shank. One capsule from the knee to the ankle. It also covers the ankle-yaw link.

translate([-70, -25, -481.9]) rotate([0, 180, 0])
    cylinder(r=44, h=303.79, center=true);
