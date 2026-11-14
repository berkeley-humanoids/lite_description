% scale(1000) import("left_hip_yaw.stl");

// Thigh. One capsule from the hip-yaw axis to the knee axis. It leans 3.95 deg from
// the hip-yaw axis, so that the knee end does not extend past the front of the knee.

translate([-70, -5, -257.5]) rotate([0, 3.95, 90])
    cylinder(r=55, h=145.34, center=true);
