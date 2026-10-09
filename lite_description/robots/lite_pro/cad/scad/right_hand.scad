% scale(1000) import("right_hand.stl");

// Hand. Three capsules.

// The palm and the straight fingers: two capsules side by side.
translate([548.3, -17.28, 352.08]) rotate([0, 89.48, 176.81])
    cylinder(r=17.58, h=127.82, center=true);
translate([549.38, 11.2, 351.35]) rotate([0, 89.48, 176.81])
    cylinder(r=17.58, h=128.82, center=true);

// The thumb: one thin capsule.
translate([541.5, 56.51, 350.09]) rotate([0, 90.34, -134.96])
    cylinder(r=5, h=64.96, center=true);
