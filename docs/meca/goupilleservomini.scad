
difference()
{
    cylinder (h = 20, r=15, center = true, $fn=100);
    cylinder (h = 20, r=3.5, center = true, $fn=100);
    translate ([0,0,3.5])rotate([90,0,0]) cylinder (h = 100, r=2.1, center = true, $fn=100);
    
    translate ([10,0,-5])cylinder (h = 10, r=0.75, center = true, $fn=10);
    rotate([0,0,90])translate ([10,0,-5])cylinder (h = 10, r=0.75, center = true, $fn=10);
    rotate([0,0,180])translate ([10,0,-5])cylinder (h = 10, r=0.75, center = true, $fn=10);
    rotate([0,0,270])translate ([10,0,-5])cylinder (h = 10, r=0.75, center = true, $fn=10);
}