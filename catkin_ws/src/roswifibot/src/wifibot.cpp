#include "libwifibot.h"
#include "wifibot.h"
#include "ros/ros.h"

#define TWOPI (M_PI * 2)

Wifibot::Wifibot(const ros::NodeHandle& nh)
  : _nh (nh)
  , _updated(false)
  , _speedLeft(0.0)
  , _speedRight(0.0)
{
  // Parameters handler
  ros::NodeHandle pn("~");

  // get base frame parameter
  std::string frameBase;
  if (!pn.getParam("base_frame", frameBase))
    _frameBase = "base_frame";
  else
    _frameBase = frameBase;
  
  // get entrax parameter
  double entrax;
  if (!pn.getParam("entrax", entrax))
    _entrax = 0.30;
  else
    _entrax = entrax;
  
  // Create and configure the driver
  _pDriver = new wifibot::Driver(nh);
  _pDriver->setTicsPerMeter(5312.0); // FIXME Adapt this value according your wheels size

  // Save initial position
  wifibot::driverData st = _pDriver->readData();
  _odometryLeftLast = st.odometryLeft;
  _odometryRightLast = st.odometryRight;

  _position.x = 0;
  _position.y = 0;
  _position.th = 0;

  // Create topics
  _pubOdometry = _nh.advertise<nav_msgs::Odometry>("odom", 10);

  _subSpeeds = _nh.subscribe("cmd_vel", 1, &Wifibot::velocityCallback, this);
  _change_odometry = _nh.subscribe("change_odometry", 1, &Wifibot::change_odometryCallback, this);

  ros::Rate r(100);
  
  _timeCurrent = ros::Time::now();
  _timeLast = ros::Time::now();

  while (ros::ok())
    {
      ros::spinOnce();
      update();
      r.sleep();
    }
}

Wifibot::~Wifibot()
{
  delete _pDriver;
}

double Wifibot::getSpeedLinear(double left, double right)
{
  return (right + left) / 2.0;
}

double Wifibot::getSpeedAngular(double left, double right)
{
  return (right - left) / _entrax;
}

void Wifibot::computeOdometry(double left, double right)
{
  double dleft = left - _odometryLeftLast;
  double dright = right - _odometryRightLast;
  double distance = getSpeedLinear(dleft, dright);

  ROS_INFO("leftlast : %f, rightlast : %f", _odometryLeftLast, _odometryRightLast);

  /*if ( distance < 0.5 ) {*/
    _position.th += getSpeedAngular(dleft, dright);
    _position.th -= (float)((int)(_position.th / TWOPI)) * TWOPI;
  
    _position.x += distance * cos(_position.th);
    _position.y += distance * sin(_position.th);

    _odometryLeftLast = left;
    _odometryRightLast = right;
    /*}
  else
    ROS_INFO("odometry pb");*/

}

void Wifibot::velocityCallback(const geometry_msgs::TwistConstPtr& vel)
{
  //ROS_INFO("input : %0.3f | %0.3f", vel->linear.x, vel->angular.z);

  _speedLeft = vel->linear.x - (vel->angular.z * (_entrax / 2.0));
  _speedRight =  vel->linear.x + (vel->angular.z * (_entrax / 2.0));
  _updated = true;
}

void Wifibot::change_odometryCallback(const geometry_msgs::Point::ConstPtr& o) {
// added by O. Aycard

    // Save initial position
    wifibot::driverData st = _pDriver->readData();
    _odometryLeftLast = st.odometryLeft;
    _odometryRightLast = st.odometryRight;

     //change position and orientation
    _position.x = o->x;
    _position.y = o->y;
    _position.th = o->z;

}

void Wifibot::update()
{
  // get data from driver
  wifibot::driverData st = _pDriver->readData();
  
  _timeCurrent = ros::Time::now();

  // compute position
  computeOdometry(st.odometryLeft, st.odometryRight);
  
  //TRANSFORM we'll publish the transform over tf
  _odomTf.header.stamp = _timeCurrent;
  _odomTf.header.frame_id = "/odom";
  _odomTf.child_frame_id = _frameBase;
  
  _odomTf.transform.translation.x = _position.x;
  _odomTf.transform.translation.y = _position.y;
  _odomTf.transform.translation.z = 0.0;
  _odomTf.transform.rotation = 
    tf::createQuaternionMsgFromYaw(_position.th);
  
  //send the transform
  _odomBroadcast.sendTransform(_odomTf);
  
  //TOPIC, we'll publish the odometry message over ROS
  nav_msgs::Odometry odometryTopic;
  odometryTopic.header.stamp = _timeCurrent;
  odometryTopic.header.frame_id = "/odom";
  
  //set the position
  odometryTopic.pose.pose.position.x = _position.x;
  odometryTopic.pose.pose.position.y = _position.y;
  odometryTopic.pose.pose.position.z = 0.0;
  odometryTopic.pose.pose.orientation = 
    tf::createQuaternionMsgFromYaw(_position.th);
    
  //set the velocity
  odometryTopic.child_frame_id = _frameBase;
  odometryTopic.twist.twist.linear.x = getSpeedLinear(
    st.speedFrontLeft, st.speedFrontRight);
  odometryTopic.twist.twist.linear.y = 0.0;
  odometryTopic.twist.twist.angular.z = getSpeedAngular(
    st.speedFrontLeft, st.speedFrontRight);

  /*
  ROS_INFO("lin:%0.3f ang:%0.3f", 
	   odometryTopic.twist.twist.linear.x,
	   odometryTopic.twist.twist.angular.z);
  */

  //publish the message
  _pubOdometry.publish(odometryTopic);
  
  // Update last time
  _timeLast = _timeCurrent;
}

int main (int argc, char **argv)
{
  ros::init(argc, argv, "wifibot_base");
  Wifibot _self(ros::NodeHandle("")); 
  
  return 0;
}

