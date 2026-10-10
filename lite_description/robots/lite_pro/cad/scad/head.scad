% scale(1000) import("head.stl");

// Head. One capsule parallel to the z axis of the Head part. It is fitted to the
// head shell. Then it is moved 10 mm forward, its radius is increased by 5 mm, and
// its bottom end is moved 10 mm down. This covers the cheeks and the jaw.

translate([0, -9.75, -0.9])
    cylinder(r=97, h=40.37, center=true);
