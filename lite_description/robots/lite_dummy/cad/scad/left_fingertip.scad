% scale(1000) import("left_fingertip.stl");

// Hand. Two capsules.

// The palm.
translate([-555.91, 5.21, 3.15]) rotate([0, 84.56, 178.81])
    cylinder(r=35, h=103.75, center=true);

// The finger that extends past the palm: one thin capsule.
translate([-680.41, -7.04, 22.51]) rotate([0, 91.48, 179.46])
    cylinder(r=5, h=56.04, center=true);
