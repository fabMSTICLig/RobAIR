// Signal handling
#include <signal.h>

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "visualization_msgs/Marker.h"
#include "geometry_msgs/Point.h"
#include "std_msgs/ColorRGBA.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int32.h"
#include <cmath>
#include "nav_msgs/Odometry.h"
#include <tf/transform_datatypes.h>
#include "std_srvs/Empty.h"

#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"
#include "message_filters/subscriber.h"
#include "tf/message_filter.h"

//BY DEFAULT the behavior of the robot is very slow: low translation_speed_max and high safety_distance
int locked = 0;//boolean to lock the robot on the current goal to reach. If its locked on the current goal to reach, it wont accept new goal before reaching the current one

bool obstacle_detection = true;//boolean to decide if we detect obstacles (and stop) or not: true by default
float safety_distance = 0.5;//safety distance between the mobile robot and the closest obstacle in meter

float translation_speed_max = 2;// in m/s
float translation_acceleration = 0.05;// in m/s2
float translation_deceleration = 0.2;// in m/s2

float rotation_acceleration = 0.1;// in radian/s2
float rotation_deceleration = 0.1;// in radian/s2

float translation_error = 0.2;// in m
float rotation_error = 0.3;// in radian

float kpr = 1;
float kir = 0;
float kdr = 0;

float kpt = 0.5;
float kit = 0;
float kdt = 0;

using namespace std;

class action {
private:

    ros::NodeHandle n;

     // communication with cmd_vel to send command to the mobile robot
    ros::Publisher pub_cmd_vel;

    // communication with odometry
    ros::Subscriber sub_odometry;
    geometry_msgs::Point position;
    float orientation;

    int cond_rotation;
    int cond_translation;
    float current_translation_speed;
    float translation_integral;

    float current_rotation_speed;
    float error_integral;
    float error_previous;
    int init_odom;

    // communication with person_detector or person_tracker
    ros::Publisher pub_goal_reached;
    ros::Subscriber sub_goal_to_reach;
    geometry_msgs::Point goal;
    geometry_msgs::Point origin;
    int init_goal;

    // Communication with obstacle_detection
    ros::Subscriber sub_obstacle_detection;
    int obstacle_detected;
    geometry_msgs::Point closest_obstacle;
    int init_obstacle;

    //  geometry_msgs::Point previous_goal;

public:

action() {

    // Communication with obstacle_detection
    sub_obstacle_detection = n.subscribe("closest_obstacle", 1, &action::closest_obstacleCallback, this);

    // communication with cmd_vel
    pub_cmd_vel = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);

    // communication with odometry
    sub_odometry = n.subscribe("odom", 1, &action::odomCallback, this);

    // communication with person_detector or person_tracker
    sub_goal_to_reach = n.subscribe("goal_to_reach", 1, &action::goal_to_reachCallback, this);
    pub_goal_reached = n.advertise<geometry_msgs::Point>("goal_reached", 1);

    cond_rotation = 0;
    cond_translation = 0;
    obstacle_detected = 0;
    current_translation_speed = 0;
    current_rotation_speed = 0;

    error_integral = 0;
    error_previous = 0;

    translation_integral = 0;

    init_goal = 0;
    init_obstacle = 0;
    init_odom = 0;

    ros::Rate r(10);//this node is updated at 10hz

    while (ros::ok()) {
        ros::spinOnce();
        update();
        r.sleep();
    }

}//action

void update() {

    //ROS_INFO("init_obstacle: %i", init_obstacle);
    //ROS_INFO("init_odom: %i", init_odom);
    //ROS_INFO("init_goal: %i", init_goal);
    if ( ( init_obstacle ) && ( init_odom ) && ( init_goal ) && ( cond_rotation || cond_translation ) ) {

        //init_odom = false;

        geometry_msgs::Twist twist;
        twist.linear.x = 0;
        twist.linear.y = 0;
        twist.linear.z = 0;

        twist.angular.x = 0;
        twist.angular.y = 0;
        twist.angular.z = 0;

 /*       if ( new_goal )
            pub_cmd_vel.publish(twist);*/

        //we compute the translation_to_do
        geometry_msgs::Point translation_vector_to_do;
        translation_vector_to_do.x = goal.x-position.x;
        translation_vector_to_do.y = goal.y-position.y;
        float translation_to_do = sqrt( ( translation_vector_to_do.x * translation_vector_to_do.x ) + ( translation_vector_to_do.y * translation_vector_to_do.y ) );
        ROS_INFO("(action_node) current_position: (%f, %f) -> goal: (%f, %f): %f", position.x, position.y, goal.x, goal.y, translation_to_do);

        //we compute the orientation_to_reach
        float orientation_to_reach = acos( translation_vector_to_do.x / translation_to_do );
        if ( translation_vector_to_do.y < 0 )
            orientation_to_reach *=-1;        

       float error = orientation_to_reach - orientation;
        if ( error > M_PI ) {
            error -= 2*M_PI;
            ROS_WARN("(action node) error > 180 degrees: %f", error*180/M_PI);
        }
        else
            if ( error < -M_PI ) {
                error += 2*M_PI;
                ROS_WARN("(action node) error < -180 degrees: %f", error*180/M_PI);
            }

        //computation of target_rotation_speed
        cond_rotation = ( fabs(error) > rotation_error );

        float target_rotation_speed = 0;
        if ( cond_rotation && cond_translation ) {
            //Implementation of a PID controller for rotation_to_do;

            float kp_rot = kpr;
            if ( ( fabs(error) < 0.6 ) && ( translation_to_do < 1 ) )
                kp_rot /= 2;

            float error_derivation = error-error_previous;
            error_previous = error;
            //ROS_INFO("error_derivaion: %f", error_derivation);
            //pub_cmd_vel.publish(twist);

            error_integral += error;
            //ROS_INFO("error_integral: %f", error_integral);
            //pub_cmd_vel.publish(twist);

            //control of rotation with a PID controller
            target_rotation_speed = kp_rot * error + kir * error_integral + kdr * error_derivation;
        }

        //accelerate or decelerate to fit the current_rotation_speed to the target_rotation_speed
        if ( target_rotation_speed > current_rotation_speed )
            if ( target_rotation_speed > current_rotation_speed + rotation_acceleration )
                current_rotation_speed += rotation_acceleration;
            else
                current_rotation_speed = target_rotation_speed;
        else
            if ( target_rotation_speed < current_rotation_speed )
                if ( target_rotation_speed < current_rotation_speed - rotation_deceleration )
                    current_rotation_speed -= rotation_deceleration;
                else
                    current_rotation_speed = target_rotation_speed;

        //computation of target_translation_speed and current_translation_speed
        cond_translation = ( translation_to_do > translation_error ) /*|| ( ( obstacle_detection ) && ( translation_to_do > safety_distance ) )*/;

        float target_translation_speed = 0;
        ROS_INFO("(action_node) cond_translation: %i, cond_rotation: %i", cond_translation, cond_rotation);
        if ( ( cond_translation ) && ( ( !obstacle_detection ) || ( !obstacle_detected ) ) /*&& ( fabs(error) < M_PI/4 )*/ ) {
            //control of translation according to the control of rotation iff there is no obstacle in the safety area
            target_translation_speed = translation_to_do;
            if ( closest_obstacle.x < target_translation_speed )
                target_translation_speed = closest_obstacle.x;

            float kp_trans = kpt;
            if ( target_translation_speed > 1 )
                kp_trans += .25;
            if ( target_translation_speed > 2 )
                kp_trans += .25;

            target_translation_speed *= kp_trans;

            target_translation_speed -= fabs(target_rotation_speed);

            if ( target_translation_speed > translation_speed_max )
                target_translation_speed = translation_speed_max;

            if ( target_translation_speed < 0 )
                target_translation_speed = 0;
        }

        // we are to going to stop the translation for safety reason
        if ( target_translation_speed == 0 )
            current_translation_speed = 0;
        //accelerate or decelerate to fit the current_rotation_speed to the target_rotation_speed
        else
            if ( target_translation_speed > current_translation_speed )
                if ( target_translation_speed > current_translation_speed + translation_acceleration )
                    current_translation_speed += translation_acceleration;
                else
                    current_translation_speed = target_translation_speed;
            else
                if ( target_translation_speed < current_translation_speed )
                    if ( target_translation_speed < current_translation_speed - translation_deceleration )
                        current_translation_speed -= translation_deceleration;
                    else
                        current_translation_speed = target_translation_speed;

        //pub_cmd_vel.publish(twist);
        //getchar();
        twist.linear.x = current_translation_speed;
        twist.angular.z = current_rotation_speed;

        ROS_INFO("(action node) current_translation_speed = %f m/s, target_translation_speed = %f m/s", current_translation_speed, target_translation_speed);
        ROS_INFO("(action node) current_rotation_speed = %f degrees/s, target_rotation_speed = %f degrees/s\n", current_rotation_speed*180/M_PI, target_rotation_speed*180/M_PI);

        //getchar();
       /* if ( new_goal )
            getchar();
        new_goal = 0;*/
        pub_cmd_vel.publish(twist);

        if ( !cond_translation ) {
            //the action is done so we send the goal_reached to the detector/tracker node
            geometry_msgs::Point msg_goal_reached;

            msg_goal_reached.x = position.x-origin.x;;
            msg_goal_reached.y = position.y-origin.y;
            msg_goal_reached.z = 0;

            ROS_INFO("(action_node) /goal_reached (%f, %f)", msg_goal_reached.x, msg_goal_reached.y);
            pub_goal_reached.publish(msg_goal_reached);

            ROS_INFO(" ");
            ROS_INFO("(action_node) waiting for a new /goal_to_reach");
        }
    }

}//update

void closest_obstacleCallback(const geometry_msgs::Point::ConstPtr& co) {

    init_obstacle = 1;
    if ( obstacle_detection ) {
        closest_obstacle.x = co->x;
        closest_obstacle.y = co->y;
        obstacle_detected = ( closest_obstacle.x < safety_distance );
        if ( obstacle_detected )
            ROS_WARN("closest_obstacle (%f, %f)", closest_obstacle.x, closest_obstacle.y);
    }

}//closest_obstacleCallback

void odomCallback(const nav_msgs::Odometry::ConstPtr& o) {

    init_odom = 1;
    position.x = o->pose.pose.position.x;
    position.y = o->pose.pose.position.y;
    orientation = tf::getYaw(o->pose.pose.orientation);

}//odomCallback

void goal_to_reachCallback(const geometry_msgs::Point::ConstPtr& g) {
// process /goal_to_reach received from the person tracker    

    init_goal = 1;
    ROS_INFO("new goal");
    if ( ( !cond_translation || !locked ) && ( init_odom ) ) {//we accept a new goal if the current /goal_to_reach is reached or if there is no wait_ack

        cond_translation = 0;
        if ( g->x || g->y ) {

            cond_translation = true;
            ROS_INFO("\n(action_node) processing /goal_to_reach at (%f, %f)", g->x, g->y);
            ROS_INFO("(action_node) origin: %f, %f, %f)", position.x, position.y, orientation*180/M_PI);
            //ROS_INFO("frame_odom: %s", odom_frame);
            //ROS_INFO(odom_frame);

            origin.x = position.x;
            origin.y = position.y;

            //we compute the position of the goal in the /odom frame "by hand": it only works if the goal is in the laser frame.
            goal.x = position.x;
            goal.y = position.y;

            //we compute the translation_to_do
            float translation_to_do = sqrt( ( g->x * g->x ) + ( g->y * g->y ) );

            //we compute the rotation_to_do
            float rotation_to_do = acos( g->x / translation_to_do );
            if ( g->y < 0 )
                rotation_to_do *=-1;

            ROS_INFO("(action_node) translation_to_do: %f, rotation_to_do: %f", translation_to_do, rotation_to_do*180/M_PI);

            goal.x += translation_to_do * cos(rotation_to_do+orientation);
            goal.y += translation_to_do * sin(rotation_to_do+orientation);
            ROS_INFO("\n(action_node) /goal (without tf method) at (%f, %f)", goal.x, goal.y);

        }
    }

}//goal_to_reachCallback

/*void goal_to_reachCallback(const geometry_msgs::PointStamped::ConstPtr& g) {
// process /goal_to_reach received from the person tracker

    init_goal = 1;
    ROS_INFO("new goal");
    if ( ( !cond_translation || !locked ) && ( init_odom ) ) {//we accept a new goal if the current /goal_to_reach is reached or if there is no wait_ack

        cond_translation = 0;
        if ( g->point.x || g->point.y ) {

            ROS_INFO("\n(action_node) processing /goal_to_reach at (%f, %f)", g->point.x, g->point.y);
            ROS_INFO("(action_node) origin: %f, %f, %f)", position.x, position.y, orientation*180/M_PI);
            //ROS_INFO("frame_odom: %s", odom_frame);
            //ROS_INFO(odom_frame);

            origin.x = position.x;
            origin.y = position.y;

            //we compute the position of the goal in the /odom frame "by hand": it only works if the goal is in the laser frame.
            goal.x = position.x;
            goal.y = position.y;

            //we compute the translation_to_do
            float translation_to_do = sqrt( ( g->point.x * g->point.x ) + ( g->point.y * g->point.y ) );

            //we compute the rotation_to_do
            float rotation_to_do = acos( g->point.x / translation_to_do );
            if ( g->point.y < 0 )
                rotation_to_do *=-1;

            ROS_INFO("(action_node) translation_to_do: %f, rotation_to_do: %f", translation_to_do, rotation_to_do*180/M_PI);

            goal.x += translation_to_do * cos(rotation_to_do+orientation);
            goal.y += translation_to_do * sin(rotation_to_do+orientation);
            ROS_INFO("\n(action_node) /goal (without tf method) at (%f, %f)", goal.x, goal.y);

            geometry_msgs::PointStamped odom_goal;
            tf::TransformListener tf_;

            int tf_not_available = 1;
            while( tf_not_available ) {
                tf_not_available = 0;
                try {
                    tf_.transformPoint("/odom", *g, odom_goal);
                }
                catch(tf::TransformException& e) {
                    tf_not_available = 1;
                }
            }

            ROS_INFO("\n(action_node) /goal (with tf method) at (%f, %f)", odom_goal.point.x, odom_goal.point.y);

            //To check if the transform is ok or not: it only works when the goal is in the /laser frame
            if ( ( odom_goal.point.x*.95 < goal.x ) && ( odom_goal.point.x*1.05 > goal.x ) && ( odom_goal.point.y*.95 < goal.y ) && ( odom_goal.point.y*1.05 > goal.y ) )
                cond_translation = 1;
            else {
                ROS_WARN("tf not ok");
                exit(1);
            }
        }
    }

}//goal_to_reachCallback*/

float distancePoints(geometry_msgs::Point pa, geometry_msgs::Point pb) {
// Distance between two points

    return sqrt(pow((pa.x-pb.x),2.0) + pow((pa.y-pb.y),2.0));

}

};

int main(int argc, char **argv) {

    ros::init(argc, argv, "action");
    ros::NodeHandle n;

    if ( ros::param::get("/action_node/locked", obstacle_detection) )
        if ( locked )
            ROS_INFO("locked: true");
        else
            ROS_INFO("locked: false");

    if ( ros::param::get("/action_node/obstacle_detection", obstacle_detection) )
        if ( obstacle_detection )
            ROS_INFO("obstacle_detection: true");
        else
            ROS_INFO("obstacle_detection: false");

    if (ros::param::get("/action_node/safety_distance", safety_distance) )
        ROS_INFO("safety_distance: %f", safety_distance);

    if ( ros::param::get("/action_node/translation_speed_max", translation_speed_max) )
        ROS_INFO("translation_speed_max: %f", translation_speed_max);

    if ( ros::param::get("/action_node/translation_acceleration", translation_acceleration) )
        ROS_INFO("translation_acceleration: %f", translation_acceleration);

    if ( ros::param::get("/action_node/translation_deceleration", translation_deceleration) )
        ROS_INFO("translation_deceleration: %f", translation_deceleration);

    if ( ros::param::get("/action_node/rotation_acceleration", rotation_acceleration) )
        ROS_INFO("rotation_acceleration: %f", rotation_acceleration);

    if ( ros::param::get("/action_node/rotation_deceleration", rotation_deceleration) )
        ROS_INFO("rotation_decceleration: %f", rotation_deceleration);

    if ( ros::param::get("/action_node/translation_error", translation_error) )
        ROS_INFO("translation_error: %f", translation_error);

    if ( ros::param::get("/action_node/rotation_error", rotation_error) )
        ROS_INFO("rotation_error: %f", rotation_error);

    if ( ros::param::get("/action_node/kpt", kpt) )
        ROS_INFO("kpt: %f", kpt);

    if ( ros::param::get("/action_node/kit", kit) )
        ROS_INFO("kit: %f", kit);

    if ( ros::param::get("/action_node/kdt", kdt) )
        ROS_INFO("kdt: %f", kdt);

    ROS_INFO("(action_node) waiting for a new /goal_to_reach");

    action bsObject;
    ros::spin();

    return 0;

}


