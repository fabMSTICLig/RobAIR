#include "Robair.h"

#define ONEK 1024

//TODO: use params instead
#define ENTRAX 0.40
#define MPS_TO_PERCENT_A 90.0
#define MPS_TO_PERCENT_B 8.0

Robair::Robair(ros::NodeHandle & nh):nh(nh),
battery_pub("battery_level",&battery_msg),
log_pub("log",&log_msg),
md49(Serial1),
sub_cmdmotor("cmdmotors", &Robair::cmdmotorsCb, this),
sub_cmdvel("cmd_vel", &Robair::cmdvelCb, this),
motors_pub("motors_info", &motors_msg),
eyes_pub("eyes",&eyes_msg),
eyes(PIN_EYES),
sub_cmdeyes("cmdeyes", &Robair::cmdEyesCb, this),
sub_eyesmat("eyesmat", &Robair::eyesMatCb, this),
head_pub("head",&head_msg),
sub_cmdhead("cmdhead", &Robair::cmdHeadCb, this),
sub_reboot("reboot", &Robair::rebootCb, this),
aru_pub("aru", &aru_msg),
bumperRear_pub("bumper_rear", &bumperRear_msg),
bumperFront_pub("bumper_front", &bumperFront_msg),
touchLeft_pub("touch_left", &touchLeft_msg),
touchRight_pub("touch_right", &touchRight_msg),
sub_loadParams("load_params", &Robair::loadParamsCb, this)
{

}


void Robair::powerMD49(bool on)
{
  digitalWrite(PIN_RMD49,on ? HIGH : LOW);
  delay(100);
  if(on)
  {
  	md49.setMode(MD49_MODE1);
  	md49.setAccel(3);
    md49.resetEncoder();
  }
}

void Robair::cmdmotorsCb(const robairmain::MotorsCmd& command_msg) {  //CALLBACK FUNCTION

  if(!aru)
  {
    smooth = true;
    cmd_msg_speedL = command_msg.speedL;
    cmd_msg_speedR = command_msg.speedR;
  }
}

void Robair::cmdvelCb(const geometry_msgs::Twist& command_msg)
{
  if(!aru) {
    smooth = false;

    // Compute requested speed

    double angular_comp = command_msg.angular.z * (ENTRAX / 2.0);
    double mps_speedL = command_msg.linear.x + angular_comp,
           mps_speedR = command_msg.linear.x - angular_comp;


    // Convert to percents

    if (mps_speedL > 0)
      cmd_msg_speedL = mps_speedL * MPS_TO_PERCENT_A + MPS_TO_PERCENT_B;
    else if (mps_speedL < 0)
      cmd_msg_speedL = mps_speedL * MPS_TO_PERCENT_A - MPS_TO_PERCENT_B;
    else
      cmd_msg_speedL = 0;
    if (mps_speedR > 0)
      cmd_msg_speedR = mps_speedR * MPS_TO_PERCENT_A + MPS_TO_PERCENT_B;
    else if (mps_speedR < 0)
      cmd_msg_speedR = mps_speedR * MPS_TO_PERCENT_A - MPS_TO_PERCENT_B;
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
  }
}

void Robair::stop_motors(){
  cmd_msg_speedL = 0;
  cmd_msg_speedR = 0;
}

void Robair::speed_control(){
  if(aru ||
      (cmd_msg_speedR > 0 && cmd_msg_speedL > 0 && bumperFront) ||
      (cmd_msg_speedR < 0 && cmd_msg_speedL < 0 && bumperRear) ) {
    cmd_speedL=0;
    cmd_speedR=0;
  } else if(smooth) {
    //low filter for smooth acceleration
    cmd_speedL = (float)(cmd_speedL) * coef_smoothness
      + (1.0-coef_smoothness) * (float)(cmd_msg_speedL);
    cmd_speedR = (float)(cmd_speedR)*coef_smoothness
      + (1.0-coef_smoothness) * (float)(cmd_msg_speedR);

  } else {
    cmd_speedL = cmd_msg_speedL;
    cmd_speedR = cmd_msg_speedR;
  }
  md49.setSpeed1(map(cmd_speedR, -100, 100, -127, 127));
  md49.setSpeed2(map(cmd_speedL, -100, 100, -127, 127));

  int encs[2];
  motors_msg.speedL = cmd_speedL;
  motors_msg.speedR = cmd_speedR;
  md49.getEncoders(encs);
  motors_msg.countL = encs[0];
  motors_msg.countR = encs[1];
  motors_pub.publish(&motors_msg);
}


void Robair::check_battery(unsigned int delay_check) {
  if( millis() > last_timestamp_battery) {

    battery_msg.data = md49.getVolt();

    battery_pub.publish( &battery_msg );
    last_timestamp_battery = millis() + delay_check;
  }
}

////////////EYES/////////////

void Robair::setEyes(int id)
{
  eyes.setMatrice(id);
  eyes_msg.data = id;
  eyes_pub.publish( &eyes_msg );
}

void Robair::cmdEyesCb(const std_msgs::UInt8& eyes_msg) {  //CALLBACK FUNCTION
  setEyes(eyes_msg.data);
}
void Robair::eyesMatCb(const robairmain::EyesMat& mat_msg) {  //CALLBACK FUNCTION
    //log(String((int)mat_msg.mat_length)+"MAT");
    //eyes.setMatrice(&mat_msg.mat);
}


//////////////////HEAD/////////////////////////////////

void Robair::setHead(int degree)
{
  if(degree!=cmd_head)
  {
    cmd_head=degree;
	  head_msg.data=cmd_head;
	  head_pub.publish( &head_msg );
  }
  servoHead.write(map(cmd_head,-90,90,0,180));
}

void Robair::cmdHeadCb(const std_msgs::Int8& head_msg) {  //CALLBACK FUNCTION
  cmd_msg_head=head_msg.data;
}

void Robair::rebootCb(const std_msgs::UInt8& reboot_msg)
{
	eyes.setMatrice(EYESSTRAIGHT);
	setHead(0);
	stop_motors();
}


void Robair::checkStop(){

  boolean oldbf,oldbr,oldaru;

  oldbf=bumperFront;
  oldbr=bumperRear;
  oldaru=aru;
  bumperFront = !papBumperFront.detect_contact((float)(analogRead(PIN_BUMPER_FRONT))/ONEK,bumperFTresh);
  bumperRear = !papBumperRear.detect_contact((float)(analogRead(PIN_BUMPER_REAR))/ONEK,bumperRTresh);

  if(oldbf!=bumperFront)
  {
    bumperFront_msg.data = bumperFront;
    bumperFront_pub.publish( &bumperFront_msg );
  }
  if(oldbr!=bumperRear)
  {
    bumperRear_msg.data = bumperRear;
    bumperRear_pub.publish( &bumperRear_msg );
  }

	if(digitalRead(PIN_ARU)==HIGH)
	{
		aru=true;
		timeoutARU=millis()+timeoutARUDelay;
    setEyes(EYESSTOP);
    stop_motors();
    powerMD49(false);
	}
	else if(aru && timeoutARU<millis())
	{
		aru=false;
    setEyes(EYESSTRAIGHT);
    powerMD49(true);
	}

  if(oldaru!=aru)
  {
    aru_msg.data = aru;
    aru_pub.publish( &aru_msg );
  }

}
//////////////PARAMS//////////////////////////////////
void Robair::loadParamsCb(const std_msgs::Empty& msgemp){

    int ibuff;
    nh.getParam("/bumpFTresh", &bumperFTresh);
    nh.getParam("/bumpRTresh", &bumperRTresh);
    nh.getParam("/touchLTresh", &touchLeftTresh);
    nh.getParam("/touchRTresh", &touchRightTresh);
    nh.getParam("/coefSmoothness", &coef_smoothness);
    nh.getParam("/aruDelay", &ibuff);
    timeoutARUDelay=ibuff;

}

////////////////////////////////////////////////TOUCH///////////////////////////////

void Robair::checkTouch()
{
      boolean oldtl,oldtr;

      oldtl=touchLeft;
      oldtr=touchRight;
      touchLeft = !papTouchLeft.detect_contact((float)(analogRead(PIN_TOUCH_LEFT))/ONEK,touchLeftTresh);
      touchRight = !papTouchRight.detect_contact((float)(analogRead(PIN_TOUCH_RIGHT))/ONEK,touchRightTresh);

      if(oldtl!=touchLeft)
      {
        touchLeft_msg.data = touchLeft;
        touchLeft_pub.publish( &touchLeft_msg );
      }
      if(oldtr!=touchRight)
      {
        touchRight_msg.data = touchRight;
        touchRight_pub.publish( &touchRight_msg );
      }
}

////////////////////////////LOG/////////////////////

void Robair::log(String str)
{

    log_msg.data = str.c_str();
    log_pub.publish(&log_msg);
}

//////////////////////////////ARDUINO////////////////
void Robair::begin()
{

  pinMode(PIN_RMD49,OUTPUT);
  digitalWrite(PIN_RMD49,HIGH);

	eyes.begin();

	pinMode(PIN_RMD49,OUTPUT);
	md49.init(9600);
  powerMD49(true);


	servoHead.attach(PIN_HEAD);
	servoHead.write(90);

	pinMode(PIN_ARU,INPUT);

	eyes.setMatrice(EYESSTRAIGHT);

  bumperFront=false;
  bumperRear=false;
  papBumperFront.init(float(analogRead(PIN_BUMPER_FRONT))/ONEK);
  papBumperRear.init(float(analogRead(PIN_BUMPER_REAR))/ONEK);


  touchLeft=false;
  touchLeft=false;
  papTouchLeft.init(float(analogRead(PIN_TOUCH_LEFT))/ONEK);
  papTouchRight.init(float(analogRead(PIN_TOUCH_RIGHT))/ONEK);

	nh.subscribe(sub_cmdmotor);
	nh.subscribe(sub_cmdvel);
	nh.advertise(motors_pub);
	nh.advertise(log_pub);
	nh.advertise(battery_pub);
	nh.advertise(eyes_pub);
	nh.subscribe(sub_cmdeyes);
	nh.subscribe(sub_eyesmat);
	nh.advertise(head_pub);
	nh.subscribe(sub_cmdhead);
	nh.advertise(bumperRear_pub);
	nh.advertise(bumperFront_pub);
	nh.advertise(touchLeft_pub);
	nh.advertise(touchRight_pub);
	nh.advertise(aru_pub);
	nh.subscribe(sub_loadParams);
	nh.spinOnce();


  int ibuff;
  nh.getParam("/bumpFTresh", &bumperFTresh);
  nh.getParam("/bumpRTresh", &bumperRTresh);
  nh.getParam("/touchLTresh", &touchLeftTresh);
  nh.getParam("/touchRTresh", &touchRightTresh);
  nh.getParam("/coefSmoothness", &coef_smoothness);
  nh.getParam("/aruDelay", &ibuff);
  timeoutARUDelay=ibuff;

}
void Robair::spinOnce()
{

  checkStop();
  checkTouch();
	int val=0;
	val=cmd_msg_head - cmd_head ;
	if(abs(val)>head_inc)
	{
		val= head_inc * ((val < 0) ? -1 :1);
	}
	setHead(cmd_head+val);

	check_battery(5000);
	speed_control();

}
