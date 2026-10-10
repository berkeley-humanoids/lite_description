% scale(1000) import("chest.stl");

// Torso. Two spheres on the torso axis, r74 below and r58 above. The radii are
// limited so that the upper arms can hang down without contact.

translate([-0.27, 30.35, 167.36]) sphere(r=74);
translate([0.03, 3.87, 278.24]) sphere(r=58);
