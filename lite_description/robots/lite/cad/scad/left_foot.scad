% scale(1000) import("left_foot.stl");

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

sole_capsule(  -88.00,   -62.00,  -81.00,  -767.00);
sole_capsule( -105.00,   -46.00,  -47.30,  -767.00);
sole_capsule( -105.00,   -47.00,  -13.70,  -767.00);
sole_capsule( -106.00,   -43.00,   20.00,  -767.00);
sole_capsule( -110.00,   -34.00,   53.70,  -767.00);
sole_capsule( -106.00,   -32.00,   87.30,  -762.60);
sole_capsule(  -86.00,   -46.00,  121.00,  -748.90);
