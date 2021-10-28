#ifndef ROBAIR_H
#define ROBAIR_H


// ROS
#include <ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <robairmain/MotorsCmd.h>
#include <robairmain/MotorsInfo.h>
#include <robairmain/EyesMat.h>

// Arduino libraries
#include <Adafruit_TiCoServo.h>
#include <Adafruit_NeoPixel.h>
#include <Eyes.h>
#include <md49.h>


class Robair {
private:
	ros::NodeHandle & nh;

	std_msgs::String log_msg;
	ros::Publisher log_pub;


	const uint8_t PIN_OFF = 7;
	// ========================  MOTORS  ========================
	const uint8_t PIN_RMD49 = 5;
	int cmd_speedL = 0;  //current_speed used by motors
	int cmd_speedR = 0;
	int cmd_msg_speedL = 0; //current command send by user
	int cmd_msg_speedR = 0;
	unsigned long last_cmdvel = 0;
	MD49 md49;

	ros::Subscriber<geometry_msgs::Twist, Robair> sub_cmdvel;

	robairmain::MotorsInfo motors_msg;
	ros::Publisher motors_pub;

	void cmdvelCb(const geometry_msgs::Twist& command_msg);
	void stop_motors(void);
	void speed_control(void);

	void powerMD49(bool on);


	// ========================  BATTERY  =======================
	const uint16_t SENSE_MIN = 870;
	const uint8_t PIN_SENSE = A0;
	std_msgs::Int32 battery_msg;
	ros::Publisher battery_pub;
	unsigned long last_timestamp_battery = 0;
	void check_battery(unsigned int delay_check);


	// =========================  EYES  =========================
	const uint8_t PIN_EYES = 4;
	Eyes eyes;
	std_msgs::UInt8 eyes_msg;
	ros::Publisher eyes_pub;
	ros::Subscriber<std_msgs::UInt8,Robair> sub_cmdeyes;
	ros::Subscriber<robairmain::EyesMat,Robair> sub_eyesmat;
	void setEyes(int id);
	void cmdEyesCb(const std_msgs::UInt8 &eyes_msg) ;
	void eyesMatCb(const robairmain::EyesMat &mat_msg) ;


	// =========================  HEAD  =========================
	const uint8_t PIN_HEAD = 3;
	Adafruit_TiCoServo servoHead;
	int cmd_msg_head = 0;
	int cmd_head = 0;

	int head_inc = 1;
	std_msgs::Int8 head_msg;
	ros::Publisher head_pub;

	ros::Subscriber<std_msgs::Int8, Robair> sub_cmdhead;

	void setHead(int degree);
	void cmdHeadCb(const std_msgs::Int8 &head_msg);


	// ========================  REBOOT  ========================
	ros::Subscriber<std_msgs::UInt8,Robair> sub_reboot;
	void rebootCb(const std_msgs::UInt8 &reboot_msg);


	// ==========================  ARU  =========================
	const uint8_t PIN_ARU = 2;
	std_msgs::Bool aru_msg;
	ros::Publisher aru_pub;
	uint32_t timeoutARU = 0;
	uint32_t timeoutARUDelay = 5000;

	boolean aru = false;

	void checkStop(void);
	
	

	// ========================  SERIE DEBUG  ========================
	
	void remote_control();

public:
	Robair(ros::NodeHandle &nh);

	void begin();
	void spinOnce();

	void log(String str);
};


#endif
