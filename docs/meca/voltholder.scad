$fn=20;
difference()
{
    cube([10,33,15],true);
    translate([0,0,-15/2])cylinder(h=15,r=5.5/2);
    translate([-10/2,33/2-3,0])rotate([0,90,0])cylinder(h=10,r=3/2);
    translate([-10/2,-(33/2-3),0])rotate([0,90,0])cylinder(h=10,r=3/2);
}
