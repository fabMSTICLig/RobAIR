#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/UInt64.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Empty.h"

#include <sstream>
ros::Publisher pong_pub;
ros::Publisher load_params_pub;
ros::Subscriber ping_sub;
ros::Subscriber reboot_sub;


void pingCallback(const std_msgs::UInt64::ConstPtr& msg)
{
        std_msgs::UInt64 msgout;
        msgout.data=msg->data;
        pong_pub.publish(msgout);
}
void rebootCallback(const std_msgs::Int8::ConstPtr& msg)
{

  pid_t child_pid;
  child_pid = fork ();
  if (child_pid == 0)
  {
        //ros::Duration(2.0).sleep();
        system("robair restart");
        ros::shutdown();
  }

}

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
        ros::init(argc, argv, "proxy");

        ros::NodeHandle n;
        load_params_pub = n.advertise<std_msgs::Empty>("load_params", 1);
        pong_pub = n.advertise<std_msgs::UInt64>("pong", 1);
        ping_sub = n.subscribe("ping", 1, pingCallback);
        reboot_sub = n.subscribe("reboot", 1000, rebootCallback);

        ros::spinOnce();
        
        ros::Duration(2.0).sleep();
        std_msgs::Empty emp;
        load_params_pub.publish(emp);
        ros::spin();

        return 0;
}
