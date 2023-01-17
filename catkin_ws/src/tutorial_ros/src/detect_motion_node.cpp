#include "ros/ros.h"
#include "ros/time.h"
#include "sensor_msgs/LaserScan.h"
#include "visualization_msgs/Marker.h"
#include "geometry_msgs/Point.h"
#include "std_msgs/ColorRGBA.h"
#include <cmath>
#include "std_msgs/Bool.h"

using namespace std;

//used for detection of motion
#define detection_threshold 0.2 //threshold for motion detection

class detect_motion_node {

private:
    ros::NodeHandle n;

    ros::Subscriber sub_scan;
    ros::Subscriber sub_robot_moving;
    ros::Publisher pub_detect_motion_marker;

    // to store, process and display laserdata
    int nb_beams;
    float range_min, range_max;
    float angle_min, angle_max, angle_inc;
    float r[1000], theta[1000];
    geometry_msgs::Point current_scan[1000];

    //to perform detection of motion
    bool init_robot;
    bool stored_background;
    float background[1000];
    bool dynamic[1000];
    bool current_robot_moving;
    bool previous_robot_moving;

    // GRAPHICAL DISPLAY
    int nb_pts;
    geometry_msgs::Point display[1000];
    std_msgs::ColorRGBA colors[1000];

    bool init_laser;//to check if new data of laser is available or not
    bool first;

public:

detect_motion_node() {

    sub_scan = n.subscribe("scan", 1, &detect_motion_node::scanCallback, this);
    sub_robot_moving = n.subscribe("robot_moving", 1, &detect_motion_node::robot_movingCallback, this);

    pub_detect_motion_marker = n.advertise<visualization_msgs::Marker>("detect_motion_marker", 1); // Preparing a topic to publish our results. This will be used by the visualization tool rviz

    init_laser = false;
    init_robot = false;
    previous_robot_moving = true;

    //INFINTE LOOP TO COLLECT LASER DATA AND PROCESS THEM
    ros::Rate r(10);// this node will run at 10hz
    while (ros::ok()) {
        ros::spinOnce();//each callback is called once to collect new data: laser + robot_moving
        update();//processing of data
        r.sleep();//we wait if the processing (ie, callback+update) has taken less than 0.1s (ie, 10 hz)
    }

}

//UPDATE: main processing of laser data and robot_moving
/*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
void update() {

    // we wait for new data of the laser and of the robot_moving_node to perform laser processing
    if ( init_laser && init_robot ) {

        ROS_INFO("\n");
        ROS_INFO("New data of laser received");
        ROS_INFO("New data of robot_moving received");

        nb_pts = 0;
        if ( !current_robot_moving ) {
            //if the robot is not moving then we can perform moving person detection
            //DO NOT FORGET to store the background but when ???
            ROS_INFO("robot is not moving");
        }
        else
        {
            // IMPOSSIBLE TO DETECT MOTIONS because the base is moving
            // what is the value of dynamic table for each hit of the laser ?
            ROS_INFO("robot is moving");
        }
        previous_robot_moving = current_robot_moving;

        //graphical display of the results
        populateMarkerTopic();

    }
    else
        if ( !init_robot )
        {
            ROS_WARN("waiting for robot_moving_node");
            ROS_WARN("launch: rosrun follow_me robot_moving_node");
        }

}// update

// DETECTION OF MOTION
/*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
void store_background() {
// store for each hit of the laser its range r in the background table

    ROS_INFO("storing background");

    /*for (int loop=0; loop<nb_beams; loop++)
        background[loop] = ...;*/

    ROS_INFO("background stored");

}//store_background

void reset_motion() {
// for each hit, compare the current range with the background to detect motion

    ROS_INFO("reset motion");
    for (int loop=0 ; loop<nb_beams; loop++ )
        dynamic[loop] = false;

    ROS_INFO("reset_motion done");

}//reset_motion

void detect_motion() {

    ROS_INFO("detecting motion");

    /*for (int loop=0; loop<nb_beams; loop++ )
      {//loop over all the hits
        if the difference between ( the background and the current value r ) is higher than "detection_threshold"
        then
             dynamic[loop] = true;//the current hit is dynamic
        else
            dynamic[loop] = false;//else its static

    if ( dynamic[loop] ) {

        ROS_INFO("hit[%i](%f, %f) is dynamic", loop, current_scan[loop].x, current_scan[loop].y);

        //display in blue of hits that are dynamic
        display[nb_pts] = current_scan[loop];

        colors[nb_pts].r = 0;
        colors[nb_pts].g = 0;
        colors[nb_pts].b = 1;
        colors[nb_pts].a = 1.0;

        nb_pts++;
    }
    }*/
    ROS_INFO("motion detected");

}//detect_motion

//CALLBACKS
/*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {

    init_laser = true;
    // store the important data related to laserscanner
    range_min = scan->range_min;
    range_max = scan->range_max;
    angle_min = scan->angle_min;
    angle_max = scan->angle_max;
    angle_inc = scan->angle_increment;
    nb_beams = ((-1 * angle_min) + angle_max)/angle_inc;

    // store the range and the coordinates in cartesian framework of each hit
    float beam_angle = angle_min;
    for ( int loop=0 ; loop < nb_beams; loop++, beam_angle += angle_inc ) {
        if ( ( scan->ranges[loop] < range_max ) && ( scan->ranges[loop] > range_min ) )
            r[loop] = scan->ranges[loop];
        else
            r[loop] = range_max;
        theta[loop] = beam_angle;

        //transform the scan in cartesian framework
        current_scan[loop].x = r[loop] * cos(beam_angle);
        current_scan[loop].y = r[loop] * sin(beam_angle);
        current_scan[loop].z = 0.0;
    }

}//scanCallback

void robot_movingCallback(const std_msgs::Bool::ConstPtr& state) {

    init_robot = true;
    current_robot_moving = state->data;

}//robot_movingCallback

// Draw the field of view and other references
void populateMarkerReference() {

    visualization_msgs::Marker references;

    references.header.frame_id = "laser";
    references.header.stamp = ros::Time::now();
    references.ns = "example";
    references.id = 1;
    references.type = visualization_msgs::Marker::LINE_STRIP;
    references.action = visualization_msgs::Marker::ADD;
    references.pose.orientation.w = 1;

    references.scale.x = 0.02;

    references.color.r = 1.0f;
    references.color.g = 1.0f;
    references.color.b = 1.0f;
    references.color.a = 1.0;
    geometry_msgs::Point v;

    v.x =  0.02 * cos(-2.356194);
    v.y =  0.02 * sin(-2.356194);
    v.z = 0.0;
    references.points.push_back(v);

    v.x =  5.6 * cos(-2.356194);
    v.y =  5.6 * sin(-2.356194);
    v.z = 0.0;
    references.points.push_back(v);

    float beam_angle = -2.356194 + 0.006136;
    // first and last beam are already included
    for (int i=0 ; i< 723; i++, beam_angle += 0.006136){
        v.x =  5.6 * cos(beam_angle);
        v.y =  5.6 * sin(beam_angle);
        v.z = 0.0;
        references.points.push_back(v);
    }

    v.x =  5.6 * cos(2.092350);
    v.y =  5.6 * sin(2.092350);
    v.z = 0.0;
    references.points.push_back(v);

    v.x =  0.02 * cos(2.092350);
    v.y =  0.02 * sin(2.092350);
    v.z = 0.0;
    references.points.push_back(v);

    pub_detect_motion_marker.publish(references);

}

void populateMarkerTopic(){

    visualization_msgs::Marker marker;

    marker.header.frame_id = "laser";
    marker.header.stamp = ros::Time::now();
    marker.ns = "example";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::POINTS;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.orientation.w = 1;

    marker.scale.x = 0.05;
    marker.scale.y = 0.05;

    marker.color.a = 1.0;

    //ROS_INFO("%i points to display", nb_pts);
    for (int loop = 0; loop < nb_pts; loop++) {
            geometry_msgs::Point p;
            std_msgs::ColorRGBA c;

            p.x = display[loop].x;
            p.y = display[loop].y;
            p.z = display[loop].z;

            c.r = colors[loop].r;
            c.g = colors[loop].g;
            c.b = colors[loop].b;
            c.a = colors[loop].a;

            //ROS_INFO("(%f, %f, %f) with rgba (%f, %f, %f, %f)", p.x, p.y, p.z, c.r, c.g, c.b, c.a);
            marker.points.push_back(p);
            marker.colors.push_back(c);
        }

    pub_detect_motion_marker.publish(marker);
    populateMarkerReference();

}

};

int main(int argc, char **argv){

    ros::init(argc, argv, "detect_motion_node");

    detect_motion_node bsObject;

    ros::spin();

    return 0;
}
