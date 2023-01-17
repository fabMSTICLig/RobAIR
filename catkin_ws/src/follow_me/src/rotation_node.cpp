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

#define rotation_error_threshold 0.2//radians

#define kpr 0.5
#define kir 0
#define kdr 0

class rotation_node {
private:

    ros::NodeHandle n;

    // communication with detection_node or datmo_node
    ros::Subscriber sub_goal_to_reach;

    // communication with odometry
    ros::Subscriber sub_odometry;

    // communication with cmd_vel to send command to the mobile robot
    ros::Publisher pub_cmd_vel;

    geometry_msgs::Point goal_to_reach;
    bool new_goal_to_reach;//to check if a new /goal_to_reach is available or not

    //pid for rotation
    float rotation_to_do, rotation_done;
    float error_rotation;//error in rotation
    bool cond_rotation;//boolean to check if we still have to rotate or not
    float initial_orientation;// to store the initial orientation ie, before starting the pid for rotation control
    float current_orientation;// to store the current orientation: this information is provided by the odometer
    float error_integral_rotation;
    float error_previous_rotation;
    float rotation_speed;

    float translation_to_do;

    bool init_odom;

public:

rotation_node() {

    // communication with cmd_vel to command the mobile robot
    pub_cmd_vel = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);

    // communication with odometry
    sub_odometry = n.subscribe("odom", 1, &rotation_node::odomCallback, this);

    // communication with datmo
    sub_goal_to_reach = n.subscribe("goal_to_reach", 1, &rotation_node::goal_to_reachCallback, this);

    new_goal_to_reach = false;
    init_odom = false;   

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

        // we receive a new /goal_to_reach
        if ( new_goal_to_reach )
            init_rotation();

        //we are performing a rotation
        if ( cond_rotation )
        {
            compute_rotation();
            move_robot();
        }                

    }

}// update

void init_rotation()
{
    new_goal_to_reach = false;
    ROS_INFO("processing the /goal_to_reach received at (%f, %f)", goal_to_reach.x, goal_to_reach.y);

    // we have a rotation and a translation to perform
    // we compute the /translation_to_do
    translation_to_do = sqrt( ( goal_to_reach.x * goal_to_reach.x ) + ( goal_to_reach.y * goal_to_reach.y ) );

    if ( translation_to_do )
    {

        //we compute the /rotation_to_do
        rotation_to_do = acos( goal_to_reach.x / translation_to_do );

        if ( goal_to_reach.y < 0 )
            rotation_to_do *=-1;

        //we initialize the pid for the control of rotation
        // initial_orientation = ;
        error_integral_rotation = 0;
        error_previous_rotation = 0;

        ROS_INFO("rotation_to_do: %f, translation_to_do: %f", rotation_to_do*180/M_PI, translation_to_do);
        cond_rotation = true;

    }
    else
        ROS_WARN("translation_to_do is equal to 0");

}// init_rotation

void compute_rotation()
{

    ROS_INFO("current_orientation: %f, initial_orientation: %f", current_orientation*180/M_PI, initial_orientation*180/M_PI);
    //rotation_done = ...;

    //do not forget that rotation_done must always be between -M_PI and +M_PI

    //error_rotation = ...;
    ROS_INFO("rotation_to_do: %f, rotation_done: %f, error_rotation: %f", rotation_to_do*180/M_PI, rotation_done*180/M_PI, error_rotation*180/M_PI);

    //cond_rotation = ...; cond_rotation is used to control if we stop or not the pid

    rotation_speed = 0;
    if ( cond_rotation )
    {
        //Implementation of a PID controller for rotation_to_do;

        //float error_derivation_rotation = ...;
        error_previous_rotation = error_rotation;
        //ROS_INFO("error_derivation_rotation: %f", error_derivation_rotation);

        //error_integral_rotation = ...;
        //ROS_INFO("error_integral_rotation: %f", error_integral_rotation);

        //control of rotation with a PID controller
        //rotation_speed = ...;
        ROS_INFO("rotation_speed: %f", rotation_speed*180/M_PI);
    }
    else
        ROS_WARN("pid for rotation will stop");

}//compute_rotation

void move_robot()
{

    geometry_msgs::Twist twist;
    twist.linear.x = 0;
    twist.linear.y = 0;
    twist.linear.z = 0;

    twist.angular.x = 0;
    twist.angular.y = 0;
    twist.angular.z = rotation_speed;

    pub_cmd_vel.publish(twist);

}// move_robot

//CALLBACKS
/*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
void odomCallback(const nav_msgs::Odometry::ConstPtr& o) {

    init_odom = true;
    current_orientation = tf::getYaw(o->pose.pose.orientation);

}

void goal_to_reachCallback(const geometry_msgs::Point::ConstPtr& g) {
// process the goal received from moving_persons detector

    new_goal_to_reach = true;
    goal_to_reach = *g;

}

// Distance between two points
float distancePoints(geometry_msgs::Point pa, geometry_msgs::Point pb) {

    return sqrt(pow((pa.x-pb.x),2.0) + pow((pa.y-pb.y),2.0));

}

};

int main(int argc, char **argv){

    ros::init(argc, argv, "rotation_node");

    ROS_INFO("(rotation_node) waiting for a /goal_to_reach");
    rotation_node bsObject;

    ros::spin();

    return 0;
}
