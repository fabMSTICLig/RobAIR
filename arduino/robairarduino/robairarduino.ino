
// ROS INCLUDE
#include <ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Byte.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <robairmain/MotorsCmd.h>
//ARDUINO InCUDE
#include <Servo.h>
#include <Adafruit_NeoPixel.h>
#include <Eyes.h>
#include <md49.h>
#include <Papierlogik.h>
#include <Robair.h>


//ROS VARIABLE
ros::NodeHandle nh;


Robair robair(nh);

//////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////ARDUINO SETUP////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////
void setup() {
  while(!Serial);
  nh.getHardware()->setBaud(57600);
  nh.initNode();
  
  robair.begin();


  pinMode(13,OUTPUT);
}

void loop() {
  // ROBOT MOVEMENT
  robair.spinOnce();
  nh.spinOnce();
  delay(50);
}
