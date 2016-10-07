//maj 27 decembre 2015

#include <ros.h>
#include <md49.h>
#include <Papierlogik.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Byte.h> 
#include <std_msgs/Bool.h> 

#include <robairmain/MotorsCmd.h>
// END ROSSERIAL INCLUDES

MD49 md49(Serial1);
ros::NodeHandle nh;


std_msgs::Byte battery_msg;

//Motor global variable
int cmd_speedL = 0;  //current_speed used by motors
int cmd_speedR = 0;

int cmd_msg_speedL = 0; //current command send by user
int cmd_msg_speedR = 0;


//Obstacle or Panic event

ros::Publisher battery_pub("battery_level",&battery_msg);


unsigned int last_timestamp_battery = 0;

void check_battery(unsigned int delay_check) {
  if( millis() > last_timestamp_battery) {
    battery_msg.data = md49.getVolt();
    battery_pub.publish( &battery_msg );
    last_timestamp_battery = millis() + delay_check;
  }
}


void stop_motors(){
  cmd_msg_speedL = 0;
  cmd_msg_speedR = 0;
  md49.setSpeed1(0);
  md49.setSpeed2(0); 
}



///////////////////////ROS FUNC///////////////

void cmdmotorsCb(const robairmain::MotorsCmd& command_msg) {  //CALLBACK FUNCTION
  digitalWrite(13, HIGH-digitalRead(13));
  
  cmd_msg_speedL = command_msg.speedL;
  cmd_msg_speedR = command_msg.speedR;
  
}

void speed_control(float coef_smoothness){
  //low filter for smooth acceleration
  cmd_speedL = cmd_speedL*coef_smoothness+(1-coef_smoothness)*cmd_msg_speedL;
  cmd_speedR = cmd_speedR*coef_smoothness+(1-coef_smoothness)*cmd_msg_speedR;
  
  md49.setSpeed1(cmd_speedL);
  md49.setSpeed2(cmd_speedR);
}

//ros::Subscriber<std_msgs::Int8> sub_cmdondo("cmdondo", &cmdondoCb);
ros::Subscriber<robairmain::MotorsCmd> sub_cmdmotor("cmdmotors", &cmdmotorsCb);


//////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////ARDUINO SETUP////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////// 
void setup() {   
  while(!Serial);    
  nh.getHardware()->setBaud(57600);
  nh.initNode();
  nh.subscribe(sub_cmdmotor);
  nh.advertise(battery_pub);
  nh.spinOnce();
  
pinMode(13, OUTPUT);  
  md49.init(9600);
  Serial2.begin(9600);
  md49.setMode(MD49_MODE1);
  
  // modification accelération
  md49.setAccel(3);
  //Serial.begin(9600);
}

void loop() {
  // ROBOT MOVEMENT
  check_battery(5000);
  speed_control(0.8);
  nh.spinOnce();
  delay(100);
}
