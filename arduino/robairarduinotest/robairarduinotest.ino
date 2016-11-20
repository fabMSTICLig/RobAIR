

// ROS INCLUDE
#include <ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Byte.h>
#include <std_msgs/Bool.h>
#include <robairmain/MotorsCmd.h>
//ARDUINO InCUDE
#include <Servo.h>
#include <Adafruit_NeoPixel.h>
#include <Eyes.h>


//ROS VARIABLE
ros::NodeHandle nh;
std_msgs::Int32 battery_msg;
ros::Publisher battery_pub("battery_level",&battery_msg);

//MOTORS
int cmd_speedL = 0;  //current_speed used by motors
int cmd_speedR = 0;
int cmd_msg_speedL = 0; //current command send by user
int cmd_msg_speedR = 0;

Servo servoL;
Servo servoR;

void cmdmotorsCb(const robairmain::MotorsCmd& command_msg) {  //CALLBACK FUNCTION
  digitalWrite(13, HIGH-digitalRead(13));

  cmd_msg_speedL = command_msg.speedL;
  cmd_msg_speedR = command_msg.speedR;

  //digitalWrite(13,val);
  //val =!val;

    battery_msg.data = 24;
    battery_pub.publish( &battery_msg );
}

void stop_motors(){
  cmd_msg_speedL = 0;
  cmd_msg_speedR = 0;
}

int val=0;

void speed_control(float coef_smoothness){
  //low filter for smooth acceleration
  cmd_speedL = cmd_speedL*coef_smoothness+(1-coef_smoothness)*cmd_msg_speedL;
  cmd_speedR = cmd_speedR*coef_smoothness+(1-coef_smoothness)*cmd_msg_speedR;

  servoL.write(map(cmd_speedL, -100, 100, 0, 179));
  servoR.write(map(cmd_speedR, -100, 100, 0, 179));

}

ros::Subscriber<robairmain::MotorsCmd> sub_cmdmotor("cmdmotors", &cmdmotorsCb);

//BATTERY

unsigned long last_timestamp_battery = 0;

void check_battery(unsigned int delay_check) {
  if( millis() > last_timestamp_battery) {
    battery_msg.data = 24;
    battery_pub.publish( &battery_msg );
    last_timestamp_battery = millis() + delay_check;
  }
}

////////////EYES/////////////
/*
Eyes eyes(7);
std_msgs::UInt8 eyes_msg;
ros::Publisher eyes_pub("eyes",&eyes_msg);

void setEyes(int id)
{
  eyes.setMatrice(id);
  eyes_msg.data = 24;
  eyes_pub.publish( &eyes_msg );
}

void cmdEyesCb(const std_msgs::UInt8& eyes_msg) {  //CALLBACK FUNCTION
  setEyes(eyes_msg.data);
}

ros::Subscriber<std_msgs::UInt8> sub_cmdeyes("cmdeyes", &cmdEyesCb);
*/
//////////////////HEAD/////////////////////////////////


Servo servoHead;

std_msgs::Int8 head_msg;
ros::Publisher head_pub("head",&head_msg);

void setHead(int degree)
{
  servoHead.write(map(degree,-90,90,0,180));
  head_msg.data=degree;
  head_pub.publish( &head_msg );
}

void cmdHeadCb(const std_msgs::Int8& head_msg) {  //CALLBACK FUNCTION
  setHead(head_msg.data);
}

ros::Subscriber<std_msgs::Int8> sub_cmdhead("cmdhead", &cmdHeadCb);

//////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////ARDUINO SETUP////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////
void setup() {
  while(!Serial);
  nh.getHardware()->setBaud(57600);
  nh.initNode();
  nh.subscribe(sub_cmdmotor);
  nh.advertise(battery_pub);
  /*nh.advertise(eyes_pub);
  nh.subscribe(sub_cmdeyes);*/
  nh.advertise(head_pub);
  nh.subscribe(sub_cmdhead);
  nh.spinOnce();

  //eyes.begin();

  servoL.attach(6);
  servoR.attach(5);
  servoHead.attach(9);

  servoL.write(0);
  servoR.write(0);
  servoHead.write(90);

  pinMode(13,OUTPUT);
  //eyes.setMatrice(EYESSTRAIGHT);
}

void loop() {
  // ROBOT MOVEMENT
  check_battery(5000);
  speed_control(0.8);
  nh.spinOnce();
  delay(100);
}
