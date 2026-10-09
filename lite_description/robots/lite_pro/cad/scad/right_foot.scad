% scale(1000) import("right_foot.stl");

// Shoe. Seven r8 capsules across the sole, from the heel to the toe. The five
// capsules under the flat sole are flush with it, 65 mm below the foot origin. The
// last two follow the toe spring.
// The centres of the round ends are 4 mm inside the sole outline. Thus each end
// extends 4 mm past the outline, not the full 8 mm radius.
// sole_capsule() runs along x, from the end centre at x0 to the end centre at x1.
// The MJCF stage changes each cylinder into a capsule.
module sole_capsule(x0, x1, y, z) {
    translate([(x0 + x1) / 2, y, z]) rotate([0, 90, 0]) cylinder(r=8, h=x1 - x0, center=true);
}

sole_capsule(   62.00,    88.00,  -76.00,  -767.00);
sole_capsule(   46.00,   105.00,  -42.30,  -767.00);
sole_capsule(   47.00,   105.00,   -8.70,  -767.00);
sole_capsule(   43.00,   106.00,   25.00,  -767.00);
sole_capsule(   34.00,   110.00,   58.70,  -767.00);
sole_capsule(   32.00,   106.00,   92.30,  -762.60);
sole_capsule(   46.00,    86.00,  126.00,  -748.90);
