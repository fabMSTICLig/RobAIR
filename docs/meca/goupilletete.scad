

difference()
{
translate ([0,11.25,0]) cube([45,22.5,16], true);
translate ([0,0,3])rotate([90,0,0]) cylinder (h = 100, r=2.1, center = true, $fn=100);
cylinder (h = 16, r=15.1, center = true, $fn=100);
    
cube([45,25,16], true);
    
    for (a = [ 0 : 90 : 270 ])   rotate([0,0,a]) translate ([17,17,0]) cylinder (h = 100, r=2.25, center = true, $fn=100);
}