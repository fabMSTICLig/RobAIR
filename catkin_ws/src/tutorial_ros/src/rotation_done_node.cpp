#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include <cmath>
#include <tf/transform_datatypes.h>
#include "geometry_msgs/Point.h"

#define security_distance 0.5

class rotation_done_node {
private:

    ros::NodeHandle n;

    // communication with odometry
    ros::Subscriber sub_odometry;

    float rotation_done;
    float initial_orientation;// to store the initial orientation ie, before starting the pid for rotation control
    float current_orientation;// to store the current orientation: this information is provided by the odometer

    bool init_odom;
    bool first;

public:

rotation_done_node() {

    // communication with odometry
    sub_odometry = n.subscribe("odom", 1, &rotation_done_node::odomCallback, this);

    init_odom = false;
    first = true;
    rotation_done = 0;

    //INFINITE LOOP TO COLLECT LASER DATA AND PROCESS THEM
    ros::Rate r(10);// this node will run at 10hz
    while (ros::ok()) {
        ros::spinOnce();//each callback is called once to collect new data: laser + robot_moving
        update();//processing of data
        r.sleep();//we wait if the processing (ie, callback+update) has taken less than 0.1s (ie, 10 hz)
    }

}

//UPDATE: main processing
/*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
void update() {

    if ( init_odom )
    {

//        if ( first )
        // initial_orientation = ...;
        first = false;

        //rotation_done = ...;

        //do not forget that rotation_done must always be between -M_PI and +M_PI
        if ( rotation_done > M_PI )
        {
            ROS_WARN("rotation_done > 180 degrees: %f degrees -> %f degrees", rotation_done*180/M_PI, (rotation_done-2*M_PI)*180/M_PI);
            rotation_done -= 2*M_PI;
        }
        else
            if ( rotation_done < -M_PI )
            {
                ROS_WARN("rotation_done < -180 degrees: %f degrees -> %f degrees", rotation_done*180/M_PI, (rotation_done+2*M_PI)*180/M_PI);
                rotation_done += 2*M_PI;
            }

        ROS_INFO("current_orientation: %f, initial_orientation: %f, rotation_done: %f", current_orientation*180/M_PI, initial_orientation*180/M_PI, rotation_done*180/M_PI);

    }

}// update

//CALLBACKS
/*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
void odomCallback(const nav_msgs::Odometry::ConstPtr& o) {

    init_odom = true;
    current_orientation = tf::getYaw(o->pose.pose.orientation);

}

};

int main(int argc, char **argv){

    ros::init(argc, argv, "rotation_done_node");

    rotation_done_node bsObject;

    ros::spin();

    return 0;
}
