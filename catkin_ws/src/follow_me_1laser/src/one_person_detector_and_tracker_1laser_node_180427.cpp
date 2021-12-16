// person detector using 2 lidar data
// written by O. Aycard

#include "ros/ros.h"
#include "ros/time.h"
#include "sensor_msgs/LaserScan.h"
#include "visualization_msgs/Marker.h"
#include "geometry_msgs/Point.h"
#include "std_msgs/ColorRGBA.h"
#include <cmath>
#include "nav_msgs/Odometry.h"
#include <tf/transform_datatypes.h>
#include "std_msgs/Int32.h"
#include "std_msgs/Bool.h"

#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"
#include "message_filters/subscriber.h"
#include "tf/message_filter.h"

#define unassociated -1
#define left 0
#define right 1

#define detection_threshold 0.2 //threshold for motion detection
#define dynamic_threshold 75 //to decide if a cluster is static or dynamic

//threshold for clustering
#define cluster_threshold 0.2

//used for detection of leg
#define leg_size_min 0.1
#define leg_size_max 0.3
#define legs_distance_max 0.7

//used for frequency
#define frequency_init 5
#define frequency_max 25

//used for uncertainty of leg
#define uncertainty_min_leg 0.5
#define uncertainty_max_leg 1
#define uncertainty_inc_leg 0.1

using namespace std;

class person_detector_tracker {

private:
    ros::NodeHandle n;
    ros::Subscriber sub_scan;    

    ros::Subscriber sub_robot_moving;

    ros::Publisher pub_person_detector_tracker_marker;

    ros::Publisher pub_stamped_person_tracker;
    ros::Publisher pub_person_tracker;

    // communication with odometry
    ros::Subscriber sub_odometry;

    // to store, process and display both laserdata
    int nb_beams;
    float range_min, range_max;
    float angle_min, angle_max, angle_inc;
    float range[1000];
    geometry_msgs::Point current_scan[1000];
    bool new_laser;
    geometry_msgs::Point transform_laser;

    //to perform detection of motion
    bool stored_background;
    float background[1000];
    bool dynamic[1000];
    bool new_robot;
    bool current_robot_moving;

    //to perform clustering
    int nb_cluster;// number of cluster
    int cluster[1000]; //to store for each hit, the cluster it belongs to
    float cluster_size[1000];// to store the size of each cluster
    geometry_msgs::Point cluster_middle[1000];// to store the middle of each cluster
    float cluster_dynamic[1000];// to store the percentage of the cluster that is dynamic
    int cluster_start[1000], cluster_end[1000];
    bool local_minimum[1000];

    //to perform detection of legs and to store them
    int nb_legs_detected;
    geometry_msgs::Point leg_detected[1000];
    int leg_cluster[1000];//to store the cluster corresponding to a leg
    bool leg_dynamic[1000];//to know if a leg is dynamic or not

    //to perform tracking
    bool person_tracked;
    geometry_msgs::Point left_tracked, right_tracked;
    geometry_msgs::Point left_tracked2, right_tracked2;
    float left_uncertainty, right_uncertainty;
    int frequency;
    int best_left, best_right;
    geometry_msgs::Point left_right;

    // to perform ego_motion_compensation
    geometry_msgs::Point left_odom, right_odom;
    geometry_msgs::Point left_odom2, right_odom2;
    geometry_msgs::Point position;
    float orientation;
    bool new_odom;

    // GRAPHICAL DISPLAY
    int nb_pts;
    geometry_msgs::Point display[1000];
    std_msgs::ColorRGBA colors[1000];

    bool data_from_bag;
    bool laser_display;
    bool robot_display;
    bool first;

public:

person_detector_tracker() {

    pub_person_detector_tracker_marker = n.advertise<visualization_msgs::Marker>("one_person_detector_tracker", 1); // Preparing a topic to publish our results. This will be used by the visualization tool rviz

    sub_scan = n.subscribe("scan", 1, &person_detector_tracker::scanCallback, this);

    sub_robot_moving = n.subscribe("robot_moving", 1, &person_detector_tracker::robot_movingCallback, this);

    // communication with odometry
    sub_odometry = n.subscribe("odom", 1, &person_detector_tracker::odomCallback, this);

    // communication with decision/control
    //pub_stamped_person_tracker = n.advertise<geometry_msgs::PointStamped>("goal_to_reach", 1);     // Preparing a topic to publish the goal to reach.
    pub_person_tracker = n.advertise<geometry_msgs::Point>("goal_to_reach", 1);     // Preparing a topic to publish the goal to reach.

    new_laser = false;
    new_robot = false;
    stored_background = false;
    new_odom = false;

    person_tracked = false;
    data_from_bag = true;

    laser_display = false;
    robot_display = false;

    ros::Rate r(10);

    while (ros::ok()) {
        ros::spinOnce();
        update();
        r.sleep();
    }

}

//UPDATE
/*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
void update() {

    //ROS_INFO("(%i, %i, %i)", new_laser, new_robot, new_odom);
    if ( new_laser && new_robot & new_odom ) {
        if ( data_from_bag ) {
            new_laser = false;
            new_robot = false;
        }

        laser_display = false;
        robot_display = false;
        nb_pts = 0;

        ROS_INFO(" ");
        if ( person_tracked )
            first = false;
        if ( first && data_from_bag )
            current_robot_moving = false;

        //if the robot is not moving then we can perform moving persons detection
        ROS_INFO("detecting a moving person");
        if ( !current_robot_moving ) {
            ROS_INFO("robot is not moving");
            // if the robot was moving previously and now it is not moving now then we store the background
            if ( !stored_background ) {
                store_background();
                stored_background = true;
            }
        }
        else {
            ROS_INFO("robot is moving");
            stored_background = false;
        }

        detect_motion();
        perform_clustering();
        search_local_minimum();
        detect_legs();

        if ( !person_tracked )
            detect_moving_persons();
        else {
            //detect_2legs();
            track_person();
        }

        populateMarkerTopic();
    }
    else {
        if ( !laser_display ) {
            laser_display = true;
            ROS_INFO("wait for /scan");
        }
        if ( !robot_display ) {
            robot_display = true;
            ROS_INFO("wait for /robot_moving");
        }
    }

}// update

// DETECT MOTION FOR BOTH LASER
/*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
void store_background() {
// store all the hits of the laser in the background table

    ROS_INFO("store_background");

    for (int loop=0; loop<nb_beams; loop++)
        background[loop] = range[loop];

}//init_background

void display_background() {

    ROS_INFO("display background");
    float beam_angle = angle_min;
    for ( int loop=0 ; loop < nb_beams; loop++, beam_angle += angle_inc ) {
        display[nb_pts].x = background[loop] * cos(beam_angle);
        display[nb_pts].y = background[loop] * sin(beam_angle);
        display[nb_pts].z = 0;

        colors[nb_pts].r = 1;
        colors[nb_pts].g = 1;
        colors[nb_pts].b = 1;
        colors[nb_pts].a = 1.0;

        nb_pts++;
    }

}//display_background

void detect_motion() {
// for each hit, compare the current range with the background to detect motion

    ROS_INFO("detect_motion");
    for (int loop=0 ; loop<nb_beams; loop++ )
        if ( ( !current_robot_moving ) &&
             //( fabs(background[loop]-range[loop]) > detection_threshold ) ) {
             ( ( ( background[loop] - range[loop] ) > detection_threshold ) || //we r getting closer to the robot
             ( ( ( range[loop] - background[loop] ) > detection_threshold ) && ( ( range[loop] - background[loop] ) < 2*detection_threshold ) ) ) ) {
            dynamic[loop] = true;
      /*      display[nb_pts].x = current_scan[loop].x;
            display[nb_pts].y = current_scan[loop].y;
            display[nb_pts].z = 0;

            colors[nb_pts].r = 1;
            colors[nb_pts].g = 1;
            colors[nb_pts].b = 1;
            colors[nb_pts].a = 1.0;

            nb_pts++;*/
        }
        else
            dynamic[loop] = false;

    ROS_INFO("%i points are dynamic", nb_pts);
    //populateMarkerTopic();
    //getchar();

}//detect_motion

// CLUSTERING FOR BOTH LASER
/*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
void perform_clustering() {
//store in the table cluster, the cluster of each hit of the laser
//if the distance between the previous hit of the laser and the current one is higher than a threshold
//else we start a new cluster

    ROS_INFO("performing clustering for laser");

    nb_cluster = 0;

    cluster_start[0] = 0;// the first hit is the start of the first cluster
    cluster[0] = 0;// the first hit belongs to the first cluster
    int nb_dynamic = 0;// to count the number of hits of the current cluster that are dynamic

    for( int loop=1; loop<nb_beams; loop++ )
        if ( distancePoints(current_scan[loop-1], current_scan[loop]) < cluster_threshold ) {
            cluster[loop] = nb_cluster;
            if ( dynamic[loop] )
                nb_dynamic++;
        }
        else {

            int current_cluster = nb_cluster;//easier to read
            cluster_end[current_cluster] = loop-1;

            int current_start = cluster_start[current_cluster];
            int current_end = cluster_end[current_cluster];
            cluster_dynamic[current_cluster] = nb_dynamic*100 / (current_end-current_start+1);
            cluster_size[current_cluster] = distancePoints(current_scan[current_start], current_scan[current_end]);
            cluster_middle[current_cluster].x = (current_scan[current_start].x+current_scan[current_end].x)/2;
            cluster_middle[current_cluster].y = (current_scan[current_start].y+current_scan[current_end].y)/2;
            cluster_middle[current_cluster].z = (current_scan[current_start].z+current_scan[current_end].z)/2;

            ROS_INFO("cluster[%i](%f, %f): [%i](%f, %f) -> [%i](%f, %f), size: %f, dynamic: %i, %f", current_cluster,
                                                                                 cluster_middle[current_cluster].x,
                                                                                 cluster_middle[current_cluster].y,
                                                                                 current_start,
                                                                                 current_scan[current_start].x,
                                                                                 current_scan[current_start].y,
                                                                                 current_end,
                                                                                 current_scan[current_end].x,
                                                                                 current_scan[current_end].y,
                                                                                 cluster_size[current_cluster],
                                                                                 nb_dynamic,
                                                                                 cluster_dynamic[current_cluster]);

            /*//display of the current cluster
            nb_pts = 0;
            for(int loop2=current_start; loop2<=current_end; loop2++) {
                // clusters are white
                display[nb_pts].x = current_scan[loop2].x;
                display[nb_pts].y = current_scan[loop2].y;
                display[nb_pts].z = current_scan[loop2].z;

                colors[nb_pts].r = 1;
                colors[nb_pts].g = 1;
                colors[nb_pts].b = 1;
                colors[nb_pts].a = 1.0;

                nb_pts++;
            }
            populateMarkerTopic();
            getchar();*/

            nb_dynamic = 0;
            nb_cluster++;
            current_cluster++;

            cluster_start[current_cluster] = loop;
            cluster[loop] = current_cluster;
            if ( dynamic[loop] )
                nb_dynamic++;
        }

    int current_cluster = nb_cluster;//easier to read
    int current_start = cluster_start[current_cluster];
    cluster_end[current_cluster] = nb_beams-1;
    int current_end = cluster_end[current_cluster];
    cluster_dynamic[current_cluster] = nb_dynamic*100 / (current_end-current_start+1);
    cluster_size[current_cluster] = distancePoints(current_scan[current_start], current_scan[current_end]);
    cluster_middle[current_cluster].x = (current_scan[current_start].x+current_scan[current_end].x)/2;
    cluster_middle[current_cluster].y = (current_scan[current_start].y+current_scan[current_end].y)/2;
    cluster_middle[current_cluster].z = (current_scan[current_start].z+current_scan[current_end].z)/2;

    ROS_INFO("cluster[%i](%f, %f): [%i](%f, %f) -> [%i](%f, %f), size: %f, dynamic: %i, %f", current_cluster,
                                                                         cluster_middle[current_cluster].x,
                                                                         cluster_middle[current_cluster].y,
                                                                         current_start,
                                                                         current_scan[current_start].x,
                                                                         current_scan[current_start].y,
                                                                         current_end,
                                                                         current_scan[current_end].x,
                                                                         current_scan[current_end].y,
                                                                         cluster_size[current_cluster],
                                                                         nb_dynamic,
                                                                         cluster_dynamic[current_cluster]);

    /*//display of the current cluster
    nb_pts = 0;
    for(int loop2=cluster_start[current_cluster]; loop2<=cluster_end[current_cluster]; loop2++) {
        // clusters are white
        display[nb_pts].x = current_scan[loop2].x;
        display[nb_pts].y = current_scan[loop2].y;
        display[nb_pts].z = current_scan[loop2].z;

        colors[nb_pts].r = 1;
        colors[nb_pts].g = 1;
        colors[nb_pts].b = 1;
        colors[nb_pts].a = 1.0;

        nb_pts++;
    }
    populateMarkerTopic();
    getchar();*/

    nb_cluster++;

}//perfor_clustering

void search_local_minimum() {

    geometry_msgs::Point origin;
    origin.x = 0;
    origin.y = 0;

    for(int loop=0; loop<nb_cluster; loop++) {
        if ( loop == 0 )
            local_minimum[loop] = ( distancePoints(origin, current_scan[cluster_end[loop]]) < distancePoints(origin, current_scan[cluster_start[loop+1]]) );
        else
            if ( loop == nb_cluster-1 )
                local_minimum[loop] = ( distancePoints(origin, current_scan[cluster_start[loop]]) < distancePoints(origin, current_scan[cluster_end[loop-1]]) );
            else
                local_minimum[loop] = ( distancePoints(origin, current_scan[cluster_end[loop]]) < distancePoints(origin, current_scan[cluster_start[loop+1]]) ) &&
                                             ( distancePoints(origin, current_scan[cluster_start[loop]]) < distancePoints(origin, current_scan[cluster_end[loop-1]]) );
    }

}//local_minimum

void detect_legs() {

    ROS_INFO("detect_legs with laser");
    nb_legs_detected = 0;

    for (int loop=0; loop<nb_cluster; loop++)
        if ( ( cluster_size[loop]<leg_size_max ) && ( cluster_size[loop]>leg_size_min ) && ( local_minimum[loop] ) ) {
            leg_detected[nb_legs_detected].x = cluster_middle[loop].x;
            leg_detected[nb_legs_detected].y = cluster_middle[loop].y;
            leg_detected[nb_legs_detected].z = cluster_middle[loop].z;
            leg_dynamic[nb_legs_detected] = ( cluster_dynamic[loop] > dynamic_threshold );
            float dist_max = 0;

            if ( leg_dynamic[nb_legs_detected] )
                ROS_INFO("moving leg found: %i -> cluster = %i, (%f, %f), size: %f, dynamic: %f, local_minimum: %i", nb_legs_detected,
                                                                                                                     loop,
                                                                                                                     leg_detected[nb_legs_detected].x,
                                                                                                                     leg_detected[nb_legs_detected].y,
                                                                                                                     cluster_size[loop],
                                                                                                                     cluster_dynamic[loop],
                                                                                                                     local_minimum[loop]);
            else
                ROS_INFO("static leg found: %i -> cluster = %i, (%f, %f), size: %f, dynamic: %f, local_minimum: %i", nb_legs_detected,
                                                                                                                     loop,
                                                                                                                     leg_detected[nb_legs_detected].x,
                                                                                                                     leg_detected[nb_legs_detected].y,
                                                                                                                     cluster_size[loop],
                                                                                                                     cluster_dynamic[loop],
                                                                                                                     local_minimum[loop]);
            leg_cluster[nb_legs_detected] = loop;
            if ( leg_dynamic[nb_legs_detected] || person_tracked )
                for(int loop2=0; loop2<nb_beams; loop2++)
                    if ( cluster[loop2] == loop ) {

                        // to compute the compacity of the leg: NOT USED AT THE MOMENT
                        if ( cluster[loop2+1] == loop ) {
                            float dist_current = distancePoints(current_scan[loop2], current_scan[loop2+1]);
                            if ( dist_current > dist_max )
                                dist_max = dist_current;
                        }

                        // legs are white
                        display[nb_pts].x = current_scan[loop2].x;
                        display[nb_pts].y = current_scan[loop2].y;
                        display[nb_pts].z = current_scan[loop2].z;

                        colors[nb_pts].r = 1;
                        colors[nb_pts].g = 1;
                        colors[nb_pts].b = 1;
                        colors[nb_pts].a = 1.0;

                        nb_pts++;
                    }
            nb_legs_detected++;
        }

    if ( nb_legs_detected ) {
        ROS_INFO("%d legs have been detected.\n", nb_legs_detected);
    }

}//detect_legs

// FUSION
/*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
void detect_moving_persons() {

    ROS_INFO("detect_moving_persons");

    for (int loop_left_leg=0; loop_left_leg<nb_legs_detected; loop_left_leg++)
        if ( leg_dynamic[loop_left_leg] )
            for (int loop_right_leg=loop_left_leg+1; loop_right_leg<nb_legs_detected; loop_right_leg++)
                if ( leg_dynamic[loop_right_leg] ) {
                    float two_legs_distance = distancePoints(leg_detected[loop_left_leg], leg_detected[loop_right_leg]);
                    bool person_detected = ( two_legs_distance<legs_distance_max );
                    if ( person_detected && !person_tracked )
                        initialize_tracking(loop_left_leg, loop_right_leg);
                }

}//detect_persons

void initialize_tracking(int l, int r) {

    ROS_INFO("initialize_tracking");

    person_tracked = true;
    frequency = frequency_init;

    //initialization of left leg
    left_tracked.x = leg_detected[l].x;
    left_tracked.y = leg_detected[l].y;
    left_tracked.z = 0;

    left_uncertainty = uncertainty_min_leg;

    //initialization of right leg
    right_tracked.x = leg_detected[r].x;
    right_tracked.y = leg_detected[r].y;
    right_tracked.z = 0;

    right_uncertainty = uncertainty_min_leg;

    left_right.x = right_tracked.x-left_tracked.x;
    left_right.y = right_tracked.y-left_tracked.y;

    ROS_INFO("frequency: %i", frequency);
    ROS_INFO("left (%f, %f), %f", left_tracked.x, left_tracked.y, left_uncertainty);
    ROS_INFO("right (%f, %f), %f", right_tracked.x, right_tracked.y, right_uncertainty);
    ROS_INFO("left_right: (%f, %f)", left_right.x, left_right.y);

    ROS_INFO("person has been detected");

    geometry_msgs::Point person_tracked;
    person_tracked.x = ( left_tracked.x + right_tracked.x ) / 2;
    person_tracked.y = ( left_tracked.y + right_tracked.y ) / 2;

    display[nb_pts].x = person_tracked.x;
    display[nb_pts].y = person_tracked.y;
    display[nb_pts].z = 0;

    //detection is green
    colors[nb_pts].r = 0.0f;
    colors[nb_pts].g = 1.0f;
    colors[nb_pts].b = 0.0f;
    colors[nb_pts].a = 1.0;
    nb_pts++;

    geometry_msgs::Point origin;
    origin.x = 0;
    origin.y = 0;

    ROS_INFO("odom = (%f, %f, %f)", position.x, position.y, orientation);

    geometry_msgs::PointStamped stamped_left_tracked;
    stamped_left_tracked.header.frame_id = "/laser";
    stamped_left_tracked.header.stamp = ros::Time(0);
    stamped_left_tracked.point = left_tracked;

    geometry_msgs::PointStamped stamped_left_odom;
    tf::TransformListener tf_;

    bool tf_not_available = false;
    while( !tf_not_available ) {
        tf_not_available = 0;
        try {
            tf_.transformPoint("/odom", stamped_left_tracked, stamped_left_odom);
        }
        catch(tf::TransformException& e) {
            tf_not_available = true;
        }
    }
    left_odom = stamped_left_odom.point;
    ROS_INFO("left_odom = (%f, %f)", left_odom.x, left_odom.y);

    float left_distance = distancePoints(origin, left_tracked);
    float left_angle = acos(left_tracked.x / left_distance);
    if ( left_tracked.y < 0 )
        left_angle *= -1;
    left_odom2.x = position.x + left_distance*cos(orientation+left_angle);
    left_odom2.y = position.y + left_distance*sin(orientation+left_angle);
    ROS_INFO("left_odom2 = (%f, %f) -> (%f, %f)", left_distance, left_angle*180/M_PI, left_odom2.x, left_odom2.y);

    geometry_msgs::PointStamped stamped_right_odom;

    geometry_msgs::PointStamped stamped_right_tracked;
    stamped_right_tracked.header.frame_id = "/laser";
    stamped_right_tracked.header.stamp = ros::Time(0);
    stamped_right_tracked.point = right_tracked;

    tf_not_available = false;
    while( !tf_not_available ) {
        tf_not_available = 0;
        try {
            tf_.transformPoint("/odom", stamped_right_tracked, stamped_right_odom);
        }
        catch(tf::TransformException& e) {
            tf_not_available = true;
        }
    }
    right_odom = stamped_right_odom.point;
    ROS_INFO("right_odom = (%f, %f)", right_odom.x, right_odom.y);

    float right_distance = distancePoints(origin, right_tracked);
    float right_angle = acos(right_tracked.x / right_distance);
    if ( right_tracked.y < 0 )
        right_angle *= -1;
    right_odom2.x = position.x + right_distance*cos(orientation+right_angle);
    right_odom2.y = position.y + right_distance*sin(orientation+right_angle);
    ROS_INFO("right_odom2 = (%f, %f) -> (%f, %f)", right_distance, right_angle*180/M_PI, right_odom2.x, right_odom2.y);

    //pub_stamped_person_tracker.publish(stamped_person_tracked);
    pub_person_tracker.publish(person_tracked);

}//initialize_tracking

void track_person() {

    predict_person();
    associate_person();
    estimate_person();

}//track_person

void predict_person() {

    ROS_INFO("predict_person");
    geometry_msgs::Point origin;
    origin.x = 0;
    origin.y = 0;

    ROS_INFO("odom = (%f, %f, %f)", position.x, position.y, orientation*180/M_PI);

    ROS_INFO("left_odom = (%f, %f)", left_odom.x, left_odom.y);

    tf::TransformListener tf_;
    geometry_msgs::PointStamped stamped_left_tracked;

    geometry_msgs::PointStamped stamped_left_odom;
    stamped_left_odom.header.frame_id = "/odom";
    stamped_left_odom.header.stamp = ros::Time(0);
    stamped_left_odom.point = left_odom;

    bool tf_not_available = false;
    while( !tf_not_available ) {
        tf_not_available = 0;
        try {
            tf_.transformPoint("/laser", stamped_left_odom, stamped_left_tracked);
        }
        catch(tf::TransformException& e) {
            tf_not_available = true;
        }
    }
    left_tracked = stamped_left_tracked.point;
    ROS_INFO("left_tracked = (%f, %f)", left_tracked.x, left_tracked.y);

    left_odom.x -= position.x;
    left_odom.y -= position.y;
    float left_distance = distancePoints(origin, left_odom);
    float left_angle = acos(left_odom.x / left_distance);
    if ( left_odom.y < 0 )
        left_angle *= -1;
    left_tracked2.x = left_distance*cos(left_angle-orientation);
    left_tracked2.y = left_distance*sin(left_angle-orientation);
    ROS_INFO("left_tracked2 = (%f, %f) -> (%f, %f)", left_distance, left_angle*180/M_PI, left_tracked2.x, left_tracked2.y);

    display[nb_pts] = left_tracked;

    //prediction is red
    colors[nb_pts].r = 0.0f;
    colors[nb_pts].g = 0.0f;
    colors[nb_pts].b = 1.0f;
    colors[nb_pts].a = 1.0;
    nb_pts++;

    ROS_INFO("right_odom = (%f, %f)", right_odom.x, right_odom.y);

    geometry_msgs::PointStamped stamped_right_tracked;

    geometry_msgs::PointStamped stamped_right_odom;
    stamped_right_odom.header.frame_id = "/odom";
    stamped_right_odom.header.stamp = ros::Time(0);
    stamped_right_odom.point = right_odom;

    tf_not_available = false;
    while( !tf_not_available ) {
        tf_not_available = 0;
        try {
            tf_.transformPoint("/laser", stamped_right_odom, stamped_right_tracked);
        }
        catch(tf::TransformException& e) {
            tf_not_available = true;
        }
    }
    right_tracked = stamped_right_tracked.point;
    ROS_INFO("right_tracked = (%f, %f)", right_tracked.x, right_tracked.y);

    right_odom.x -= position.x;
    right_odom.y -= position.y;
    float right_distance = distancePoints(origin, right_odom);
    float right_angle = acos(right_tracked.x / right_distance);
    if ( right_tracked.y < 0 )
        right_angle *= -1;
    right_tracked2.x = position.x + right_distance*cos(right_angle-orientation);
    right_tracked2.y = position.y + right_distance*sin(right_angle-orientation);
    ROS_INFO("right_tracked2 = (%f, %f) -> (%f, %f)", right_distance, right_angle*180/M_PI, right_tracked2.x, right_tracked2.y);

    display[nb_pts] = right_tracked;

    //prediction is red
    colors[nb_pts].r = 0.0f;
    colors[nb_pts].g = 0.0f;
    colors[nb_pts].b = 1.0f;
    colors[nb_pts].a = 1.0;
    nb_pts++;

    ROS_INFO("prediction done\n");

}

void associate_person() {

    ROS_INFO("associate_person");
    int best_associated = 0;
    best_left = unassociated;
    best_right = unassociated;

    float best_uncertainty = left_uncertainty+right_uncertainty;

    //2 legs apart
    for (int loop_left=unassociated; loop_left<nb_legs_detected; loop_left++) {
        bool left_associated = true;
        if ( loop_left != unassociated )
            left_associated = ( distancePoints(leg_detected[loop_left], left_tracked) < left_uncertainty );

            if ( left_associated )
                for (int loop_right=unassociated; loop_right<nb_legs_detected; loop_right++) {
                    bool right_associated = true;
                    if ( loop_right != unassociated )
                        right_associated = ( distancePoints(leg_detected[loop_right], right_tracked) < right_uncertainty );

                    bool check_two_legs_distance = true;
                    if ( ( loop_left != unassociated ) && ( loop_right != unassociated ) && ( loop_left != loop_right ) )
                        check_two_legs_distance = distancePoints(leg_detected[loop_left], leg_detected[loop_right]) < legs_distance_max;

                    if ( ( left_associated || right_associated ) && check_two_legs_distance && ( ( loop_left != loop_right ) || ( ( loop_left == unassociated ) && ( loop_right == unassociated ) ) ) ) {
                        int current_associated = 0;
                        float current_uncertainty = 0;
                        //ROS_INFO("best_associated: %i, best_uncertainty: %f", best_associated, best_uncertainty);

                        //ROS_INFO("current_uncertainty: %f", current_uncertainty);
                        if ( left_associated && loop_left != unassociated ) {
                            current_associated++;
                            current_uncertainty += distancePoints(leg_detected[loop_left], left_tracked);
                        }
                        else
                            current_uncertainty += left_uncertainty;

                        //ROS_INFO("current_uncertainty: %f", current_uncertainty);
                        if ( right_associated && loop_right != unassociated ) {
                            current_associated++;
                            current_uncertainty += distancePoints(leg_detected[loop_right], right_tracked);
                        }
                        else
                            current_uncertainty += right_uncertainty;

                        if ( current_associated >= best_associated ) {
                            if ( ( current_associated > best_associated ) || ( ( current_associated == best_associated ) && ( best_uncertainty > current_uncertainty ) ) ) {
                                best_uncertainty = current_uncertainty;
                                best_associated = current_associated;
                                best_left = loop_left;
                                best_right = loop_right;
                                ROS_INFO("best_associated: %i, best_uncertainty: %f, best_left: %i, best_right: %i", best_associated, best_uncertainty, best_left, best_right);
                            }
                        }
                    }
                }
    }

    //2 legs together
    //if ( ( best_left == unassociated ) && ( best_right == unassociated ) )
        for (int loop_legs=0; loop_legs<nb_cluster; loop_legs++) {
            bool legs_associated = ( cluster_size[loop_legs]>leg_size_max ) &&
                                   ( cluster_size[loop_legs]<2*leg_size_max ) &&
                                   ( distancePoints(cluster_middle[loop_legs], left_tracked) < left_uncertainty ) &&
                                   ( distancePoints(cluster_middle[loop_legs], right_tracked) < right_uncertainty );

            if ( legs_associated ) {
                int current_associated = 2;
                float current_uncertainty= distancePoints(cluster_middle[loop_legs], left_tracked) + distancePoints(cluster_middle[loop_legs], right_tracked);

                //ROS_INFO("current_uncertainty: %f", current_uncertainty);
                if ( current_associated >= best_associated )

                    if ( ( current_associated > best_associated ) || ( ( current_associated == best_associated ) && ( best_uncertainty > current_uncertainty ) ) ) {
                        best_uncertainty = current_uncertainty;
                        best_associated = current_associated;
                        best_left = loop_legs;
                        best_right = loop_legs;
                        ROS_INFO("2legs: best_associated: %i, best_uncertainty: %f, best_left: %i, best_right: %i", best_associated, best_uncertainty, best_left, best_right);
                    }
        }
    }

    //nb_pts = 0;
    if ( ( best_left != unassociated ) && ( best_left != best_right ) ) {
        ROS_INFO("left is associated with %i at (%f, %f)", best_left, leg_detected[best_left].x, leg_detected[best_left].y);

        //left_leg is white
        for ( int loop_hit=0; loop_hit<nb_beams;loop_hit++)
            if ( leg_cluster[best_left] == cluster[loop_hit] ) {

                display[nb_pts] = current_scan[loop_hit];
                colors[nb_pts].r = 1.0f;
                colors[nb_pts].g = 1.0f;
                colors[nb_pts].b = 1.0f;
                colors[nb_pts].a = 1.0;
                nb_pts++;
            }
    }
    else
        if ( best_left == unassociated )
            ROS_INFO("left is unassociated");

    if ( ( best_right != unassociated ) && ( best_left != best_right ) ) {
        ROS_INFO("right is associated with %i at (%f, %f)", best_right, leg_detected[best_right].x, leg_detected[best_right].y);        

        //right_leg is white
        for ( int loop_hit=0; loop_hit<nb_beams;loop_hit++)
            if ( leg_cluster[best_right] == cluster[loop_hit] ) {

                display[nb_pts] = current_scan[loop_hit];
                colors[nb_pts].r = 1.0f;
                colors[nb_pts].g = 1.0f;
                colors[nb_pts].b = 1.0f;
                colors[nb_pts].a = 1.0;
                nb_pts++;
            }
    }
    else
        if ( best_right == unassociated )
            ROS_INFO("right is unassociated");

    if ( ( best_left != unassociated ) && ( best_left == best_right ) ) {
        ROS_INFO("left and right are associated with cluster[%i] at (%f, %f)", best_left, cluster_middle[best_left].x, cluster_middle[best_left].y);

        //left_leg is white
        for ( int loop_hit=0; loop_hit<nb_beams;loop_hit++)
            if ( best_left == cluster[loop_hit] ) {

                display[nb_pts] = current_scan[loop_hit];
                colors[nb_pts].r = 1.0f;
                colors[nb_pts].g = 1.0f;
                colors[nb_pts].b = 1.0f;
                colors[nb_pts].a = 1.0;
                nb_pts++;
            }
    }

    populateMarkerTopic();
    ROS_INFO("association done\n");
    //getchar();

}//associate_person

void estimate_person() {

    ROS_INFO("estimate_person");

    if ( ( best_left != unassociated ) && ( best_right != unassociated ) ) {
        if ( frequency < frequency_max )
            frequency++;

        if ( best_left != best_right ) {
            left_tracked = leg_detected[best_left];
            left_uncertainty = uncertainty_min_leg;

            right_tracked = leg_detected[best_right];
            right_uncertainty = uncertainty_min_leg;

            left_right.x = right_tracked.x-left_tracked.x;
            left_right.y = right_tracked.y-left_tracked.y;
        }
        else {
            left_tracked = cluster_middle[best_left];
            left_uncertainty = uncertainty_min_leg;

            right_tracked = cluster_middle[best_right];
            right_uncertainty = uncertainty_min_leg;

        }
    }

    if ( ( best_left != unassociated ) && ( best_right == unassociated ) ) {
        if ( frequency < frequency_max )
            frequency++;
        left_tracked = leg_detected[best_left];
        left_uncertainty = uncertainty_min_leg;

        right_tracked.x = left_tracked.x+left_right.x;
        right_tracked.y = left_tracked.y+left_right.y;
        if ( right_uncertainty < uncertainty_max_leg )
            right_uncertainty += uncertainty_inc_leg;
    }

    if ( ( best_left == unassociated ) && ( best_right != unassociated ) ) {
        if ( frequency < frequency_max )
            frequency++;
        right_tracked = leg_detected[best_right];
        right_uncertainty = uncertainty_min_leg;

        left_tracked.x = right_tracked.x-left_right.x;
        left_tracked.y = right_tracked.y-left_right.y;
        if ( left_uncertainty < uncertainty_max_leg )
            left_uncertainty += uncertainty_inc_leg;
    }

    if ( ( best_left == unassociated ) && ( best_right == unassociated ) ) {
        frequency -= 2;

        left_uncertainty += 2*uncertainty_inc_leg;
        if ( left_uncertainty > uncertainty_max_leg )
            left_uncertainty = uncertainty_max_leg;

        right_uncertainty += 2*uncertainty_inc_leg;
        if ( right_uncertainty > uncertainty_max_leg )
            right_uncertainty = uncertainty_max_leg;

    }

    if ( frequency >= 0 ) {
        ROS_INFO("frequency: %i", frequency);
        ROS_INFO("left (%f, %f), %f", left_tracked.x, left_tracked.y, left_uncertainty);
        ROS_INFO("right (%f, %f), %f", right_tracked.x, right_tracked.y, right_uncertainty);
        ROS_INFO("left_right: (%f, %f)", left_right.x, left_right.y);
        //getchar();

        geometry_msgs::Point position_tracked;
        position_tracked.x = ( left_tracked.x + right_tracked.x ) / 2;
        position_tracked.y = ( left_tracked.y + right_tracked.y ) / 2;

        pub_person_tracker.publish(position_tracked);

        display[nb_pts] = left_tracked;
        colors[nb_pts].r = 1.0f;
        colors[nb_pts].g = 1.0f;
        colors[nb_pts].b = 0.0f;
        colors[nb_pts].a = 1.0;
        nb_pts++;

       display[nb_pts] = right_tracked;
       colors[nb_pts].r = 1.0f;
       colors[nb_pts].g = 1.0f;
       colors[nb_pts].b = 0.0f;
       colors[nb_pts].a = 1.0;
       nb_pts++;

       //detection is green
       display[nb_pts] = position_tracked;
       colors[nb_pts].r = 0.0f;
       colors[nb_pts].g = 1.0f;
       colors[nb_pts].b = 0.0f;
       colors[nb_pts].a = 1.0;
       nb_pts++;

       //pub_stamped_person_tracker.publish(stamped_person_tracked);
       populateMarkerTopic();
       //getchar();

       geometry_msgs::Point origin;
       origin.x = 0;
       origin.y = 0;

       ROS_INFO("odom = (%f, %f, %f)", position.x, position.y, orientation*180/M_PI);

       geometry_msgs::PointStamped stamped_left_tracked;
       stamped_left_tracked.header.frame_id = "/laser";
       stamped_left_tracked.header.stamp = ros::Time(0);
       stamped_left_tracked.point = left_tracked;

       geometry_msgs::PointStamped stamped_left_odom;
       tf::TransformListener tf_;

       bool tf_not_available = false;
       while( !tf_not_available ) {
           tf_not_available = 0;
           try {
               tf_.transformPoint("/odom", stamped_left_tracked, stamped_left_odom);
           }
           catch(tf::TransformException& e) {
               tf_not_available = true;
           }
       }
       left_odom = stamped_left_odom.point;
       ROS_INFO("left_odom = (%f, %f)", left_odom.x, left_odom.y);

       display[nb_pts] = left_odom;

       //prediction is red
       colors[nb_pts].r = 1.0f;
       colors[nb_pts].g = 0.0f;
       colors[nb_pts].b = 0.0f;
       colors[nb_pts].a = 1.0;
       nb_pts++;

       float left_distance = distancePoints(origin, left_tracked);
       float left_angle = acos(left_tracked.x / left_distance);
       if ( left_tracked.y < 0 )
           left_angle *= -1;
       left_odom2.x = position.x + left_distance*cos(orientation+left_angle);
       left_odom2.y = position.y + left_distance*sin(orientation+left_angle);
       ROS_INFO("left_odom2 = (%f, %f) -> (%f, %f)", left_distance, left_angle*180/M_PI, left_odom2.x, left_odom2.y);

       geometry_msgs::PointStamped stamped_right_odom;

       geometry_msgs::PointStamped stamped_right_tracked;
       stamped_right_tracked.header.frame_id = "/laser";
       stamped_right_tracked.header.stamp = ros::Time(0);
       stamped_right_tracked.point = right_tracked;

       tf_not_available = false;
       while( !tf_not_available ) {
           tf_not_available = 0;
           try {
               tf_.transformPoint("/odom", stamped_right_tracked, stamped_right_odom);
           }
           catch(tf::TransformException& e) {
               tf_not_available = true;
           }
       }
       right_odom = stamped_right_odom.point;
       ROS_INFO("right_odom = (%f, %f)", right_odom.x, right_odom.y);

       display[nb_pts] = right_odom;

       //prediction is red
       colors[nb_pts].r = 1.0f;
       colors[nb_pts].g = 0.0f;
       colors[nb_pts].b = 0.0f;
       colors[nb_pts].a = 1.0;
       nb_pts++;

       float right_distance = distancePoints(origin, right_tracked);
       float right_angle = acos(right_tracked.x / right_distance);
       if ( right_tracked.y < 0 )
           right_angle *= -1;
       right_odom2.x = position.x + right_distance*cos(orientation+right_angle);
       right_odom2.y = position.y + right_distance*sin(orientation+right_angle);
       ROS_INFO("right_odom2 = (%f, %f) -> (%f, %f)", right_distance, right_angle*180/M_PI, right_odom2.x, right_odom2.y);

    }

    person_tracked = frequency > 0;
    ROS_INFO("estimate person done\n");

}

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {

    new_laser = true;

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
            range[loop] = scan->ranges[loop];
        else
            range[loop] = range_max;

        //transform the scan in cartesian framewrok
        current_scan[loop].x = range[loop] * cos(beam_angle);
        current_scan[loop].y = range[loop] * sin(beam_angle);
        current_scan[loop].z = 0.0;
        //ROS_INFO("laser[%i]: (%f, %f) -> (%f, %f)", loop, range[loop], beam_angle*180/M_PI, current_scan[loop].x, current_scan[loop].y);

    }

}//scanCallback

void robot_movingCallback(const std_msgs::Bool::ConstPtr& state) {

    new_robot = true;
    current_robot_moving = state->data;

}//robot_movingCallback

void odomCallback(const nav_msgs::Odometry::ConstPtr& o) {

    new_odom = true;
    position.x = o->pose.pose.position.x;
    position.y = o->pose.pose.position.y;
    orientation = tf::getYaw(o->pose.pose.orientation);

}//odomCallback

// Distance between two points
float distancePoints(geometry_msgs::Point pa, geometry_msgs::Point pb) {

    return sqrt(pow((pa.x-pb.x),2.0) + pow((pa.y-pb.y),2.0));

}

// Draw the field of view and other references
void populateMarkerReference() {

    visualization_msgs::Marker references;

    references.header.frame_id = "laser";
    references.header.stamp = ros::Time::now();
    references.ns = "one_person_detector_tracker";
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

    pub_person_detector_tracker_marker.publish(references);

}

void populateMarkerTopic(){

    visualization_msgs::Marker marker;

    marker.header.frame_id = "laser";
    marker.header.stamp = ros::Time::now();
    marker.ns = "one_person_detector_tracker";
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

    pub_person_detector_tracker_marker.publish(marker);
    populateMarkerReference();

}

};

int main(int argc, char **argv){

    ros::init(argc, argv, "person_detector_tracker");

    ROS_INFO("waiting for activation of person detector and tracker");
    person_detector_tracker bsObject;

    ros::spin();

    return 0;
}
