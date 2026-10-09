% scale(1000) import("chest.stl");

// Torso. Two spheres on the torso axis, r74 below and r58 above. The radii are
// limited so that the upper arms can hang down without contact.

translate([-0.05, 18.24, 237.11]) sphere(r=74);
translate([0.03, -6.78, 348.33]) sphere(r=58);
