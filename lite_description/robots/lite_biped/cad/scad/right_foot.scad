% scale(1000) import("right_foot.stl");

// flat sole -- these 8 carry the standing load (support 97.97 cm, 89.9%)
translate([ 74.60, -79.98, -767.00]) sphere(r=8);
translate([109.41, -45.90, -767.00]) sphere(r=8);
translate([ 41.96, -44.42, -767.00]) sphere(r=8);
translate([112.09,   7.64, -767.00]) sphere(r=8);
translate([ 35.24,   8.77, -767.00]) sphere(r=8);
translate([114.32,  61.20, -767.00]) sphere(r=8);
translate([ 28.52,  61.96, -767.00]) sphere(r=8);
translate([ 70.47,  71.73, -767.00]) sphere(r=8);

// toe upturn -- clear the ground at neutral, engage as the foot pitches forward
translate([104.38, 104.89, -758.91]) sphere(r=8);
translate([ 30.68, 105.97, -758.53]) sphere(r=8);
translate([ 65.35, 128.74, -747.13]) sphere(r=8);
