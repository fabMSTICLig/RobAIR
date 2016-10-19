#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/UInt64.h"

#include <sstream>
  ros::Publisher pong_pub;
  ros::Subscriber ping_sub;


void pingCallback(const std_msgs::UInt64::ConstPtr& msg)
{
  std_msgs::UInt64 msgout;
  msgout.data=msg->data;
  pong_pub.publish(msgout);
}

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "proxy");

  ros::NodeHandle n;
  
  pong_pub = n.advertise<std_msgs::UInt64>("pong", 1000);
  ping_sub = n.subscribe("ping", 1000, pingCallback);

  ros::spin();

  return 0;
}

