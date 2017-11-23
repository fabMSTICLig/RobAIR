#include <ros.h>
#include <Robair.h>

ros::NodeHandle nh;
Robair robair(nh);

void setup()
{
	while(!Serial);
	nh.getHardware()->setBaud(9600);
	nh.initNode();

	robair.begin();


	pinMode(13,OUTPUT);
}

void loop()
{
	// ROBOT MOVEMENT
	robair.spinOnce();
	nh.spinOnce();
	delay(50);
}
