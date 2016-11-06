#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/UInt64.h"
#include "std_msgs/Empty.h"

#include <sstream>
ros::Publisher pong_pub;
ros::Subscriber ping_sub;
ros::Subscriber reboot_sub;


void pingCallback(const std_msgs::UInt64::ConstPtr& msg)
{
        std_msgs::UInt64 msgout;
        msgout.data=msg->data;
        pong_pub.publish(msgout);
}
void rebootCallback(const std_msgs::Empty::ConstPtr& msg)
{

  pid_t child_pid;
  child_pid = fork ();
  if (child_pid == 0)
  {
        system("robairreboot.bash");
        ros::shutdown();
  }
   //execl("/home/robair/Rob-AIR/scripts/robairreboot.bash","robairreboot.bash",NULL);*/

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
        reboot_sub = n.subscribe("reboot", 1000, rebootCallback);

        ros::spin();

        return 0;
}
