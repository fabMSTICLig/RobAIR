
difference()
{
    cylinder (h = 20, r=15, center = true, $fn=100);
    cylinder (h = 20, r=3.5, center = true, $fn=100);
    translate ([0,0,5])rotate([90,0,0]) cylinder (h = 100, r=2.1, center = true, $fn=100);
    
    translate ([6.5,0,-5])cylinder (h = 10, r=0.5, center = true, $fn=100);
    rotate([0,0,90])translate ([6.5,0,-5])cylinder (h = 10, r=0.5, center = true, $fn=100);
    rotate([0,0,180])translate ([6.5,0,-5])cylinder (h = 10, r=0.5, center = true, $fn=100);
    rotate([0,0,270])translate ([6.5,0,-5])cylinder (h = 10, r=0.5, center = true, $fn=100);
}