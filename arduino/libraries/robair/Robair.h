#ifndef ROBAIR_H
#define ROBAIR_H


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

class Robair{
private:


  //ROS VARIABLE
  ros::NodeHandle & nh;

  std_msgs::String log_msg;
  ros::Publisher log_pub;

  //MOTORS
  int cmd_speedL = 0;  //current_speed used by motors
  int cmd_speedR = 0;
  int cmd_msg_speedL = 0; //current command send by user
  int cmd_msg_speedR = 0;

  #ifdef USESERVO
  Servo servoL;
  Servo servoR;
  #else
  MD49 md49;
  const uint8_t RML=2;
  const uint8_t RMR=3;
  const uint8_t RTRESH=0;
  #endif

  ros::Subscriber<robairmain::MotorsCmd,Robair> sub_cmdmotor;

  void cmdmotorsCb(const robairmain::MotorsCmd& command_msg);
  void stop_motors();
  void speed_control(float coef_smoothness);

  //BATTERY

  std_msgs::Int32 battery_msg;
  ros::Publisher battery_pub;
  unsigned long last_timestamp_battery = 0;
  void check_battery(unsigned int delay_check) ;

  ////////////EYES/////////////

  Eyes eyes;
  std_msgs::UInt8 eyes_msg;
  ros::Publisher eyes_pub;
  ros::Subscriber<std_msgs::UInt8,Robair> sub_cmdeyes;
  void setEyes(int id);
  void cmdEyesCb(const std_msgs::UInt8& eyes_msg) ;


  //////////////////HEAD/////////////////////////////////


  Servo servoHead;
  int cmd_msg_head = 0;
  int cmd_head=0;
  std_msgs::Int8 head_msg;
  ros::Publisher head_pub;

  ros::Subscriber<std_msgs::Int8,Robair> sub_cmdhead;

  void setHead(int degree);
  void cmdHeadCb(const std_msgs::Int8& head_msg);



public:
Robair(ros::NodeHandle & nh);

void begin();
void spin();

};


#endif
