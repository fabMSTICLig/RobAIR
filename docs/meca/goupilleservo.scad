
difference()
{
    cylinder (h = 25, r=15, center = true, $fn=100);
    cylinder (h = 25, r=4, center = true, $fn=100);
    translate ([0,0,4.5])rotate([90,0,0]) cylinder (h = 100, r=2.1, center = true, $fn=100);
    
    translate ([8,0,-5])cylinder (h = 10, r=0.5, center = true, $fn=10);
    translate ([-8,0,-5])cylinder (h = 10, r=0.5, center = true, $fn=10);
    translate ([0,7,-5])cylinder (h = 10, r=0.5, center = true, $fn=10);
    translate ([0,-7,-5])cylinder (h = 10, r=0.5, center = true, $fn=10);
}