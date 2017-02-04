difference(){
    
    union(){
        cylinder (h = 19, r=4, center = false, $fn=100);
        translate([0,0,18])cylinder (h = 5, r1=6, r2=3, center = false, $fn=100);
    }
    cylinder (h = 23, r=1.6, center = false, $fn=20);
}