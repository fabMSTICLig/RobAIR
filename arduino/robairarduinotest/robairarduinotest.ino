// ROS INCLUDE
#include <ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Byte.h> 
#include <std_msgs/Bool.h>
#include <robairmain/MotorsCmd.h>
//ARDUINO InCUDE
#include <Servo.h> 



//ROS VARIABLE
ros::NodeHandle nh;
std_msgs::Int32 battery_msg;
ros::Publisher battery_pub("battery_level",&battery_msg);
std_msgs::Bool social_msg;
ros::Publisher social_pub("social_touch",&social_msg);

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

bool social= false;
bool now= false;

void check_social() {
  now = digitalRead(2);
  if( social != now) {
    social_msg.data = !now;
    digitalWrite(13,now);
    social_pub.publish( &social_msg );
    social = now;
  }
}






//////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////ARDUINO SETUP////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////// 
void setup() {   
  while(!Serial);    
  nh.getHardware()->setBaud(57600);
  nh.initNode();
  nh.subscribe(sub_cmdmotor);
  nh.advertise(battery_pub);
  nh.advertise(social_pub);
  nh.spinOnce();

  
  
  servoL.attach(6);
  servoR.attach(5);
  
  servoL.write(0);
  servoR.write(0);
  
  pinMode(13,OUTPUT);
  pinMode(2,INPUT);
  digitalWrite(2,HIGH);
}

void loop() {
  // ROBOT MOVEMENT
  check_battery(5000);
  speed_control(0.8);
  check_social();
  nh.spinOnce();
  delay(100);
}
