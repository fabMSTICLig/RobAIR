
/*Servo 12mm
* Minus Medium 5mm, pot 3mm, 3mm mousse, 1mm pla = 12
* Reste 12-12=0 mm
* 
* Medium 5 mm, pvc 1 mm = 6 mm
*
* 
* hole center 8 mm
* 
* 6+8-0=14mm
*
*/

difference()
{
translate ([0,11.25,0]) cube([45,22.5,16], true);
translate ([0,0,0])rotate([90,0,0]) cylinder (h = 100, r=2.1, center = true, $fn=100);
cylinder (h = 16, r=15.1, center = true, $fn=100);
    
cube([45,25,16], true);
    
    for (a = [ 0 : 90 : 270 ])   rotate([0,0,a]) translate ([17,17,0]) cylinder (h = 100, r=2.25, center = true, $fn=100);
}