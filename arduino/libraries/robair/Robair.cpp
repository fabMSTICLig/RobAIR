#include "Robair.h"

#include <limits.h>

#define ONEK 1024

#define MOVE_TIMEOUT 1000

//TODO: use params instead
#define ENTRAX 0.40
#define MPS_TO_PERCENT_A 90.0
#define MPS_TO_PERCENT_B  8.0

Robair::Robair(ros::NodeHandle &nh) :
	nh(nh),
	battery_pub("battery_level", &battery_msg),
	log_pub("log", &log_msg),
	md49(Serial1),
	sub_cmdvel("cmd_vel", &Robair::cmdvelCb, this),
	motors_pub("motors_info", &motors_msg),
	eyes_pub("eyes", &eyes_msg),
	eyes(PIN_EYES),
	sub_cmdeyes("cmdeyes", &Robair::cmdEyesCb, this),
	sub_eyesmat("eyesmat", &Robair::eyesMatCb, this),
	head_pub("head", &head_msg),
	sub_cmdhead("cmdhead", &Robair::cmdHeadCb, this),
	sub_reboot("reboot", &Robair::rebootCb, this),
	aru_pub("aru", &aru_msg)
{
}


// ========================  MOTORS  ========================

void Robair::powerMD49(bool on)
{
	digitalWrite(PIN_RMD49, on ? HIGH : LOW);
	delay(100);
	if (on) {
		md49.setAccel(2);
		md49.resetEncoder();
	}
}

void Robair::cmdvelCb(const geometry_msgs::Twist& command_msg)
{
	if (!aru) {
		// Compute requested speed

		double angular_comp = command_msg.angular.z * (ENTRAX / 2.0);
		double mps_speedL = command_msg.linear.x - angular_comp,
		       mps_speedR = command_msg.linear.x + angular_comp;


		// Convert to percents

		if (mps_speedL > 0)
			cmd_msg_speedL = mps_speedL
				* MPS_TO_PERCENT_A + MPS_TO_PERCENT_B;
		else if (mps_speedL < 0)
			cmd_msg_speedL = mps_speedL
				* MPS_TO_PERCENT_A - MPS_TO_PERCENT_B;
		else
			cmd_msg_speedL = 0;

		if (mps_speedR > 0)
			cmd_msg_speedR = mps_speedR
				* MPS_TO_PERCENT_A + MPS_TO_PERCENT_B;
		else if (mps_speedR < 0)
			cmd_msg_speedR = mps_speedR
				* MPS_TO_PERCENT_A - MPS_TO_PERCENT_B;
		else
			cmd_msg_speedR = 0;


		// Clamp values

		if(cmd_msg_speedL < -100)
			cmd_msg_speedL = -100;
		else if(cmd_msg_speedL > 100)
			cmd_msg_speedL = 100;

		if(cmd_msg_speedR < -100)
			cmd_msg_speedR = -100;
		else if(cmd_msg_speedR > 100)
			cmd_msg_speedR = 100;

		last_cmdvel = millis();
	}
}

void Robair::stop_motors()
{
	cmd_msg_speedL = 0;
	cmd_msg_speedR = 0;
}

void Robair::speed_control()
{
	if (aru
	    || last_cmdvel + MOVE_TIMEOUT < millis()) {
		cmd_speedL=0;
		cmd_speedR=0;
	} else {
		cmd_speedL = cmd_msg_speedL;
		cmd_speedR = cmd_msg_speedR;
	}

	uint8_t cml =map(cmd_speedL, -100, 100, 0, 255);
	uint8_t cmr =map(cmd_speedR, -100, 100, 0, 255);

	if(cml>125 && cml<131) cml=128;
	if(cmr>125 && cmr<131) cmr=128;

	md49.setSpeed1(cml);
	md49.setSpeed2(cmr);


	int encs[2];
	md49.getEncoders(encs);

	motors_msg.speedL = cmd_speedL;
	motors_msg.speedR = cmd_speedR;
	motors_msg.countL = encs[0];
	motors_msg.countR = encs[1];

	motors_pub.publish(&motors_msg);
}


// ========================  BATTERY  =======================

void Robair::check_battery(unsigned int delay_check)
{
	if (millis() > last_timestamp_battery) {
		uint16_t val=0;
		for(int i=0;i<10;i++)
		{
			val+=analogRead(PIN_SENSE);
		}
		if(val/10<SENSE_MIN) digitalWrite(PIN_OFF, HIGH);

		battery_msg.data = md49.getVolt();
		//battery_msg.data = analogRead(PIN_SENSE);
		battery_pub.publish(&battery_msg);

		last_timestamp_battery = millis() + delay_check;
	}

}


// =========================  EYES  =========================

void Robair::setEyes(int id)
{
	eyes.setMatrice(id);
	eyes_msg.data = id;
	eyes_pub.publish(&eyes_msg);
}

void Robair::cmdEyesCb(const std_msgs::UInt8 &eyes_msg)
{
	setEyes(eyes_msg.data);
}

void Robair::eyesMatCb(const robairmain::EyesMat &mat_msg) {
	//FIXME: doesn't work, messages are too long for serial interface
	//eyes.setMatrice(&mat_msg.mat);
}


// =========================  HEAD  =========================

void Robair::setHead(int degree)
{
	if (degree != cmd_head) {
		cmd_head = degree;
		head_msg.data = cmd_head;
		head_pub.publish(&head_msg);
	}

	servoHead.write(map(cmd_head, -90, 90, 0, 180));
}

void Robair::cmdHeadCb(const std_msgs::Int8 &head_msg)
{
	cmd_msg_head = head_msg.data;
}

void Robair::rebootCb(const std_msgs::UInt8 &reboot_msg)
{
	eyes.setMatrice(EYESSTRAIGHT);
	setHead(0);
	stop_motors();
}


void Robair::checkStop()
{
	boolean oldaru;
	float volts;

	oldaru = aru;
	
	if (digitalRead(PIN_ARU) == LOW) {
		aru = true;
		timeoutARU = millis() + timeoutARUDelay;
		setEyes(EYESSTOP);
		stop_motors();
		powerMD49(false);
	} else if (aru && timeoutARU < millis()) {
		aru = false;
		setEyes(EYESSTRAIGHT);
		powerMD49(true);
	}

	if (oldaru != aru) {
		aru_msg.data = aru;
		aru_pub.publish(&aru_msg);
	}
}


// ==========================  LOG  =========================

void Robair::log(String str)
{
	log_msg.data = str.c_str();
	log_pub.publish(&log_msg);
}


// =========================  SETUP  ========================

void Robair::begin()
{
	pinMode(PIN_OFF, OUTPUT);
	digitalWrite(PIN_OFF, LOW);
	
	delay(500);

	uint16_t val=0;
	for(int i=0;i<10;i++)
	{
		val+=analogRead(PIN_SENSE);
	}
	if(val/10<SENSE_MIN) digitalWrite(PIN_OFF, HIGH);
	
	pinMode(PIN_RMD49, OUTPUT);
	digitalWrite(PIN_RMD49, HIGH);

	// Serial debug
	Serial3.begin(9600);

	eyes.begin();

	md49.init(9600);
	powerMD49(true);


	servoHead.attach(PIN_HEAD);
	servoHead.write(90);

	pinMode(PIN_ARU, INPUT_PULLUP);

	eyes.setMatrice(EYESSTRAIGHT);
	
	nh.subscribe(sub_cmdvel);
	nh.advertise(motors_pub);
	nh.advertise(log_pub);
	nh.advertise(battery_pub);
	nh.advertise(eyes_pub);
	nh.subscribe(sub_cmdeyes);
	nh.subscribe(sub_eyesmat);
	nh.advertise(head_pub);
	nh.subscribe(sub_cmdhead);
	nh.advertise(aru_pub);
	nh.spinOnce();

}

String debugbuffer;
geometry_msgs::Twist command_msg_debug;

void Robair::remote_control()
{
	static int bytes_read = 0;
	static union { uint8_t i[4]; float f; } linear, angular;

	while (Serial3.available()) {
		if (bytes_read < 4) {
			if (Serial3.read() == 0xff)
				bytes_read++;
			else
				bytes_read = 0;
		} else if (bytes_read < 8) {
			linear.i[bytes_read-4] = (uint8_t)Serial3.read();
			bytes_read++;
		} else if (bytes_read < 12) {
			angular.i[bytes_read-8] = (uint8_t)Serial3.read();
			bytes_read++;
		} else {
			geometry_msgs::Twist msg;
			if (!isnan(linear.f) && !isnan(angular.f)) {
				msg.linear.x = linear.f;
				msg.angular.z = angular.f;
				cmdvelCb(msg);
			}
			bytes_read = 0;
		}
	}
}

// =========================  LOOP  =========================

void Robair::spinOnce()
{
	checkStop();

	int val = 0;
	val = cmd_msg_head - cmd_head;
	if (abs(val) > head_inc)
		val = head_inc * ((val < 0) ? -1 : 1);
	setHead(cmd_head + val);
        
	check_battery(5000);
	remote_control();
	speed_control();
}
