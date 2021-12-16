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

#define detection_threshold 0.2 //threshold for motion detection
#define dynamic_threshold 75 //to decide if a cluster is static or dynamic

//threshold for clustering
#define cluster_threshold 0.2

//used for detection of leg
#define leg_size_min 0.05
#define leg_size_max 0.25
#define legs_distance_min 0.1
#define legs_distance_max 0.7

//used for frequency
#define frequency_init 5
#define frequency_max 25

//used for uncertainty of leg
#define uncertainty_min_leg 0.5
#define uncertainty_max_leg 1
#define uncertainty_inc_leg 0.05

#define unassociated -1
#define leg 0

using namespace std;

class perception {

private:
    ros::NodeHandle n;

    ros::Subscriber sub_scan;
    ros::Subscriber sub_robot_moving;

    ros::Publisher pub_perception;
    ros::Publisher pub_perception_marker;

    // to store, process and display both laserdata
    bool init_laser;
    int nb_beams;
    float range_min, range_max;
    float angle_min, angle_max, angle_inc;
    float r[1000];
    float theta[1000];
    geometry_msgs::Point current_scan[1000];
    geometry_msgs::Point previous_scan[1000];
    bool current_valid[1000];
    bool previous_valid[1000];
    int current_label[1000];
    int previous_label[1000];

    //to perform detection of motion
    bool init_robot;
    bool stored_background;
    float background[1000];
    bool dynamic[1000];
    bool current_robot_moving;
    bool previous_robot_moving;

    //to perform clustering
    int nb_cluster;// number of cluster
    int cluster[1000]; //to store for each hit, the cluster it belongs to
    float cluster_size[1000];// to store the size of each cluster
    geometry_msgs::Point cluster_middle[1000];// to store the middle of each cluster
    int cluster_dynamic[1000];// to store the percentage of the cluster that is dynamic
    int cluster_start[1000], cluster_end[1000];

    //to perform detection of legs and to store them
    int nb_legs_detected;
    geometry_msgs::Point leg_detected[1000];
    int leg_cluster[1000];//to store the cluster corresponding to a leg
    bool leg_dynamic[1000];//to know if a leg is dynamic or not

    //to perform detection of a moving person and store it
    int nb_persons_detected;
    geometry_msgs::Point person_detected[1000];
    int leg1[1000], leg2[1000];
    bool person_dynamic[1000];

    //to perform tracking of the moving person
    bool moving_person_detected;
    geometry_msgs::Point moving_person_tracked;
    float uncertainty;
    int frequency;

    // GRAPHICAL DISPLAY
    int nb_pts;
    geometry_msgs::Point display[1000];
    std_msgs::ColorRGBA colors[1000];

public:

perception() {

    sub_scan = n.subscribe("scan", 1, &perception::scanCallback, this);
    sub_robot_moving = n.subscribe("robot_moving", 1, &perception::robot_movingCallback, this);

    // communication with action
    pub_perception = n.advertise<geometry_msgs::Point>("goal_to_reach", 1);     // Preparing a topic to publish the goal to reach.

    pub_perception_marker = n.advertise<visualization_msgs::Marker>("perception", 1); // Preparing a topic to publish our results. This will be used by the visualization tool rviz

    init_laser = false;
    init_robot = false;
    previous_robot_moving = true;

    moving_person_detected = false;

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

    // we wait for new data of the laser and of the robot_moving_node to perform laser processing
    if ( init_laser && init_robot ) {

        init_laser = false;
        ROS_INFO("\n");
        ROS_INFO("New data of laser received");
        ROS_INFO("New data of robot_moving received");        

        nb_pts = 0;
        //if the robot is not moving then we can perform moving person detection
        if ( !current_robot_moving ) {
            ROS_INFO("robot is not moving");

            // if the robot was moving previously and now it is not moving then we store the background
            if ( previous_robot_moving && !current_robot_moving ) {
                store_background();
                ROS_INFO("background_stored");
                //getchar();*/
            }
        }
        else
            ROS_INFO("robot is moving");

        previous_robot_moving = current_robot_moving;

        if ( !moving_person_detected )
        {
            //we search for moving person in 4 steps
            detect_motion();//to classify each hit of the laser as dynamic or not
            perform_clustering_with_valid();//to perform clustering
            detect_legs();//to detect moving legs using cluster
            detect_persons();//to detect moving_person using moving legs detected
            detect_moving_persons();
            copy_tables();
        }
        else
        {
            track_moving_person(true);
            copy_tables();
        }

        //graphical display of the results
        populateMarkerTopic();        
    }
    else
    {
//        if ( !init_laser )
//            ROS_INFO("waiting for laser data");
        if ( !init_robot )
            ROS_INFO("waiting for robot_moving_node");
    }

}// update

// DETECT MOTION FOR LASER DATA
/*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
void store_background() {
// store all the hits of the laser in the background table

    ROS_INFO("storing background");

    for (int loop=0; loop<nb_beams; loop++)
        background[loop] = r[loop];

    ROS_INFO("background stored");

}//store_background

void detect_motion() {
// for each hit, compare the current range with the background to detect motion

    ROS_INFO("detecting motion");
    for (int loop=0 ; loop<nb_beams; loop++ )
        if ( ( !current_robot_moving ) &&
             //( fabs(background[loop][laser]-range[loop][laser]) > detection_threshold ) ) {
             ( ( ( background[loop] - r[loop] ) > detection_threshold ) || //we r getting closer to the robot
             ( ( ( r[loop] - background[loop] ) > detection_threshold ) && ( ( r[loop] - background[loop] ) < 2*detection_threshold ) ) ) )
        {
            dynamic[loop] = true;

            display[nb_pts] = current_scan[loop];

            colors[nb_pts].r = 0;
            colors[nb_pts].g = 0;
            colors[nb_pts].b = 1;
            colors[nb_pts].a = 1.0;

            nb_pts++;
        }
        else
            dynamic[loop] = false;

    ROS_INFO("%i points are dynamic", nb_pts);
    //populateMarkerTopic();
    //getchar();

    ROS_INFO("motion detected");

}//detect_motion

// CLUSTERING FOR LASER DATA
/*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
void perform_clustering() {
//store in the table cluster, the cluster of each hit of the laser
//if the distance between the previous hit of the laser and the current one is higher than a threshold
//else we start a new cluster

    ROS_INFO("performing clustering");

    nb_cluster = 0;

    cluster_start[0] = 0;// the first hit is the start of the first cluster
    cluster[0] = 0;// the first hit belongs to the first cluster
    int nb_dynamic = 0;// to count the number of hits of the current cluster that are dynamic

    //graphical display of the start of the current cluster in green
    display[nb_pts] = current_scan[cluster_start[nb_cluster]];

    colors[nb_pts].r = 0;
    colors[nb_pts].g = 1;
    colors[nb_pts].b = 0;
    colors[nb_pts].a = 1.0;
    nb_pts++;

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

            ROS_INFO("cluster[%i](%f, %f): [%i](%f, %f) -> [%i](%f, %f), size: %f, dynamic: %i, %i", current_cluster,
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

            //graphical display of the end of the current cluster in red
            display[nb_pts] = current_scan[cluster_end[nb_cluster]];

            colors[nb_pts].r = 1;
            colors[nb_pts].g = 0;
            colors[nb_pts].b = 0;
            colors[nb_pts].a = 1.0;
            nb_pts++;

            nb_dynamic = 0;
            nb_cluster++;
            current_cluster++;

            cluster_start[current_cluster] = loop;
            cluster[loop] = current_cluster;
            if ( dynamic[loop] )
                nb_dynamic++;

            //graphical display of the start of the current cluster in green
            display[nb_pts] = current_scan[cluster_start[nb_cluster]];

            colors[nb_pts].r = 0;
            colors[nb_pts].g = 1;
            colors[nb_pts].b = 0;
            colors[nb_pts].a = 1.0;
            nb_pts++;

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

    ROS_INFO("cluster[%i](%f, %f): [%i](%f, %f) -> [%i](%f, %f), size: %f, dynamic: %i, %i", current_cluster,
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

    //graphical display of the end of the current cluster in red
    display[nb_pts] = current_scan[cluster_end[nb_cluster]];

    colors[nb_pts].r = 1;
    colors[nb_pts].g = 0;
    colors[nb_pts].b = 0;
    colors[nb_pts].a = 1.0;
    nb_pts++;

    nb_cluster++;

    ROS_INFO("clustering performed");

}//perform_clustering

void perform_clustering_with_valid() {
//store in the table cluster, the cluster of each hit of the laser
//if the distance between the previous hit of the laser and the current one is higher than a threshold
//else we start a new cluster

    ROS_INFO("performing clustering with valid");

    nb_cluster = 0;

    int nb_dynamic;// to count the number of hits of the current cluster that are dynamic
    bool first = true;

    for( int loop=1; loop<nb_beams; loop++ ) {
        if ( !first )
        {
            if ( ( distancePoints(current_scan[loop-1], current_scan[loop]) < cluster_threshold ) &&
                 ( current_valid[loop-1] ) && ( current_valid[loop] ) )
            {
                cluster[loop] = nb_cluster;
                if ( dynamic[loop] )
                    nb_dynamic++;
            }
            else
            {
                cluster[loop] = nb_cluster;
                if ( dynamic[loop] )
                    nb_dynamic++;

                cluster_end[nb_cluster] = loop-1;

                //graphical display of the end of the current cluster in red
                display[nb_pts] = current_scan[cluster_end[nb_cluster]];

                colors[nb_pts].r = 1;
                colors[nb_pts].g = 0;
                colors[nb_pts].b = 0;
                colors[nb_pts].a = 1.0;
                nb_pts++;

                int current_start = cluster_start[nb_cluster];
                int current_end = cluster_end[nb_cluster];
                cluster_dynamic[nb_cluster] = nb_dynamic*100 / (current_end-current_start+1);
                cluster_size[nb_cluster] = distancePoints(current_scan[current_start], current_scan[current_end]);
                cluster_middle[nb_cluster].x = (current_scan[current_start].x+current_scan[current_end].x)/2;
                cluster_middle[nb_cluster].y = (current_scan[current_start].y+current_scan[current_end].y)/2;
                cluster_middle[nb_cluster].z = 0;//(current_scan[current_start][laser].z+current_scan[current_end][laser].z)/2;

                ROS_INFO("cluster[%i](%f, %f): [%i](%f, %f) -> [%i](%f, %f), size: %f, dynamic: %i, %i",
                         nb_cluster,
                         cluster_middle[nb_cluster].x,
                         cluster_middle[nb_cluster].y,
                         current_start,
                         current_scan[current_start].x,
                         current_scan[current_start].y,
                         current_end,
                         current_scan[current_end].x,
                         current_scan[current_end].y,
                         cluster_size[nb_cluster],
                         nb_dynamic,
                         cluster_dynamic[nb_cluster]);

                    nb_cluster++;
                    //current_cluster++;

                    first = true;
            }
        }

        if ( first && current_valid[loop] ) {
            nb_dynamic = 0;
            cluster_start[nb_cluster] = loop;
            cluster[loop] = nb_cluster;
            if ( dynamic[loop] )
                nb_dynamic++;
            first = false;

            //graphical display of the end of the current cluster in red
            display[nb_pts] = current_scan[cluster_start[nb_cluster]];

            colors[nb_pts].r = 0;
            colors[nb_pts].g = 1;
            colors[nb_pts].b = 0;
            colors[nb_pts].a = 1.0;
            nb_pts++;
//                ROS_INFO("cluster_start[%i] %i", current_cluster, loop);
        }
    }

    if ( !first ) {

        int current_cluster = nb_cluster;//easier to read
        int current_start = cluster_start[nb_cluster];
        cluster_end[nb_cluster] = nb_beams-1;
        int current_end = cluster_end[nb_cluster];
        cluster_dynamic[nb_cluster] = nb_dynamic*100 / (current_end-current_start+1);
        cluster_size[nb_cluster] = distancePoints(current_scan[current_start], current_scan[current_end]);
        cluster_middle[nb_cluster].x = (current_scan[current_start].x+current_scan[current_end].x)/2;
        cluster_middle[nb_cluster].y = (current_scan[current_start].y+current_scan[current_end].y)/2;
        cluster_middle[nb_cluster].z = 0;

        ROS_INFO("cluster[%i](%f, %f): [%i](%f, %f) -> [%i](%f, %f), size: %f, dynamic: %i, %i",
                 nb_cluster,
                 cluster_middle[nb_cluster].x,
                 cluster_middle[nb_cluster].y,
                 current_start,
                 current_scan[current_start].x,
                 current_scan[current_start].y,
                 current_end,
                 current_scan[current_end].x,
                 current_scan[current_end].y,
                 cluster_size[current_cluster],
                 nb_dynamic,
                 cluster_dynamic[current_cluster]);

        nb_cluster++;

    }

    populateMarkerTopic();
    //getchar();

}//perform_clustering_with_valid

void detect_legs() {

    ROS_INFO("detecting legs");
    nb_legs_detected = 0;

    for (int loop=0; loop<nb_cluster; loop++)
        if ( ( cluster_size[loop]<leg_size_max ) && ( cluster_size[loop]>leg_size_min ) /*&& ( local_minimum[loop] )*/ ) {
            leg_cluster[nb_legs_detected] = loop;
            leg_detected[nb_legs_detected] = cluster_middle[loop];
            leg_dynamic[nb_legs_detected] = ( cluster_dynamic[loop] > dynamic_threshold ) /*&& ( cluster_size[loop]>leg_size_min ) && ( local_minimum[loop] )*/;

            if ( leg_dynamic[nb_legs_detected] )
            {
                ROS_INFO("moving leg found: %i -> cluster = %i, (%f, %f), size: %f, dynamic: %i", nb_legs_detected,
                                                                                                  loop,
                                                                                                  leg_detected[nb_legs_detected].x,
                                                                                                  leg_detected[nb_legs_detected].y,
                                                                                                  cluster_size[loop],
                                                                                                  cluster_dynamic[loop]);
                for(int loop2=0; loop2<nb_beams; loop2++)
                    if ( cluster[loop2] == loop && current_valid[loop2] ) {

                        // moving legs are yellow
                        display[nb_pts] = current_scan[loop2];

                        colors[nb_pts].r = 1;
                        colors[nb_pts].g = 1;
                        colors[nb_pts].b = 0;
                        colors[nb_pts].a = 1.0;

                        nb_pts++;
                    }
                //populateMarkerTopic();
                //getchar();
            }
            else
            {
                ROS_INFO("static leg found: %i -> cluster = %i, (%f, %f), size: %f, dynamic: %i", nb_legs_detected,
                                                                                                  loop,
                                                                                                  leg_detected[nb_legs_detected].x,
                                                                                                  leg_detected[nb_legs_detected].y,
                                                                                                  cluster_size[loop],
                                                                                                  cluster_dynamic[loop]);
                for(int loop2=0; loop2<nb_beams; loop2++)
                    if ( cluster[loop2] == loop && current_valid[loop2] ) {

                        // static legs are white
                        display[nb_pts] = current_scan[loop2];

                        colors[nb_pts].r = 1;
                        colors[nb_pts].g = 1;
                        colors[nb_pts].b = 1;
                        colors[nb_pts].a = 1.0;

                        nb_pts++;
                    }
            }

            nb_legs_detected++;
        }

    if ( nb_legs_detected ) {
        ROS_INFO("%d legs have been detected.\n", nb_legs_detected);
    }
    ROS_INFO("legs detected");

}//detect_legs

void detect_persons() {

    ROS_INFO("detecting persons");
    nb_persons_detected = 0;

    for (int loop_leg1=0; loop_leg1<nb_legs_detected; loop_leg1++)
        for (int loop_leg2=loop_leg1+1; loop_leg2<nb_legs_detected; loop_leg2++)
            if ( distancePoints(leg_detected[loop_leg1], leg_detected[loop_leg2]) < legs_distance_max )
            {
                person_detected[nb_persons_detected].x = ( leg_detected[loop_leg1].x + leg_detected[loop_leg2].x ) / 2;
                person_detected[nb_persons_detected].y = ( leg_detected[loop_leg1].y + leg_detected[loop_leg2].y ) / 2;
                person_detected[nb_persons_detected].z = ( leg_detected[loop_leg1].z + leg_detected[loop_leg2].z ) / 2;

                person_dynamic[nb_persons_detected] = leg_dynamic[loop_leg1] && leg_dynamic[loop_leg2];
                leg1[nb_persons_detected] = leg_cluster[loop_leg1];
                leg2[nb_persons_detected] = leg_cluster[loop_leg2];

                if ( person_dynamic[nb_persons_detected] )
                {
                    ROS_INFO("moving person detected: leg[%i]+leg[%i] -> (%f, %f)", loop_leg1,
                                                                                    loop_leg2,
                                                                                    person_detected[nb_persons_detected].x,
                                                                                    person_detected[nb_persons_detected].y);

                    // a detected person is green
                    display[nb_pts] = person_detected[nb_persons_detected];

                    colors[nb_pts].r = 0;
                    colors[nb_pts].g = 1;
                    colors[nb_pts].b = 0;
                    colors[nb_pts].a = 1.0;

                    nb_pts++;
                }
                else
                    ROS_INFO("static person detected: leg[%i]+leg[%i] -> (%f, %f)", loop_leg1,
                                                                                    loop_leg2,
                                                                                    person_detected[nb_persons_detected].x,
                                                                                    person_detected[nb_persons_detected].y);

                nb_persons_detected++;
            }

    if ( nb_persons_detected ) {
        ROS_INFO("%d persons have been detected.", nb_persons_detected);
        //getchar();
    }
    ROS_INFO("persons detected");

}//detect_persons

void detect_moving_persons() {

    ROS_INFO("detecting moving persons");

    for (int loop=0; loop<nb_persons_detected; loop++)
        if ( person_dynamic[loop] && !moving_person_detected )
        {

            int nb = 0;
            for(int loop_hit=0; loop_hit<nb_beams; loop_hit++)
                if ( ( cluster[loop_hit] == leg1[loop] || cluster[loop_hit] == leg2[loop] ) /*&& ( current_valid[loop_hit] )*/ )
                {
                    current_label[loop_hit] = leg;
                    nb++;
                }
                else
                    current_label[loop_hit] = unassociated;
            ROS_INFO("%i", nb);

            if ( !moving_person_detected )
            {
                moving_person_detected = true;
                moving_person_tracked = person_detected[loop];
                pub_perception.publish(moving_person_tracked);

                frequency = frequency_init;
                uncertainty = uncertainty_min_leg;
            }

        }

    ROS_INFO("moving persons detected");

}//detect_moving_persons

void track_moving_person(bool d) {

    ROS_INFO("tracking moving person");

    geometry_msgs::Point low_level;

    low_level.x = 0;
    low_level.y = 0;

    nb_pts = 0;
    int last = 0;

    for( int current_loop=0; current_loop<nb_beams; current_loop++ )
        if ( current_valid[current_loop] )
        {

            int indice_min = 0;
            float dist_min = uncertainty;
            int label_min = unassociated;

            for( int previous_loop=0; previous_loop<nb_beams; previous_loop++ )
                if ( previous_valid[previous_loop] )
                {
                    float current_dist = distancePoints(current_scan[current_loop], previous_scan[previous_loop]);

                    if ( current_dist < dist_min ) {
                        dist_min = current_dist;
                        indice_min = previous_loop;
                        label_min = previous_label[previous_loop];
                    }
                }

            if ( d && label_min == leg /*&& last+1<current_loop*/ )
            {
                ROS_INFO("[%i, %f, %f] associated with [%i, %f ,%f] = %f => leg", current_loop, current_scan[current_loop].x, current_scan[current_loop].y, indice_min, previous_scan[indice_min].x, previous_scan[indice_min].y, dist_min);
                last = current_loop;
            }
            if ( d && label_min != leg )
                ROS_INFO("[%i, %f, %f] associated with [%i, %f ,%f] = %f => no leg", current_loop, current_scan[current_loop].x, current_scan[current_loop].y, indice_min, previous_scan[indice_min].x, previous_scan[indice_min].y, dist_min);
            current_label[current_loop] = label_min;

            if ( current_label[current_loop] != unassociated ) {
                display[nb_pts] = current_scan[current_loop];

                colors[nb_pts].r = 1.0f;
                colors[nb_pts].g = 1.0f;
                colors[nb_pts].b = 1.0f;
                colors[nb_pts].a = 1.0;

                low_level.x += current_scan[current_loop].x;
                low_level.y += current_scan[current_loop].y;
                nb_pts++;
            }
        }

        if ( nb_pts>0 )
        {

            low_level.x /= nb_pts;
            low_level.y /= nb_pts;

            display[nb_pts] = low_level;

            colors[nb_pts].r = 0.0f;
            colors[nb_pts].g = 1.0f;
            colors[nb_pts].b = 0.0f;
            colors[nb_pts].a = 1.0;

            nb_pts++;

            if ( frequency < frequency_max )
                frequency++;
            uncertainty = uncertainty_min_leg;

            moving_person_tracked = low_level;
            ROS_INFO("low level detection");
            ROS_INFO("moving_person_tracked: (%f, %f, %i)", moving_person_tracked.x, moving_person_tracked.y, frequency);
            pub_perception.publish(moving_person_tracked);

        }
        else
        {

            detect_motion();
            perform_clustering_with_valid();
            detect_legs();
            detect_persons();

            ROS_INFO("tracking");
            bool associated = false;
            float distance_min = uncertainty;
            int index_min;

            for( int loop_detection=0; loop_detection<nb_persons_detected; loop_detection++ )
            {
                float current_dist = distancePoints(moving_person_tracked, person_detected[loop_detection]);
                ROS_INFO("distance with [%i] = %f", loop_detection, current_dist);
                if (  current_dist < distance_min ) {
                    index_min = loop_detection;
                    distance_min = current_dist;
                    associated = true;
                    ROS_INFO("track associated with %i", loop_detection);
                }
            }

            if ( associated ) {
                moving_person_tracked = person_detected[index_min];
                if ( frequency < frequency_max )
                    frequency++;
                uncertainty = uncertainty_min_leg;
                pub_perception.publish(moving_person_tracked);

                nb_pts = 0;
                int nb = 0;
                for(int loop_hit=0; loop_hit<nb_beams; loop_hit++)
                    if ( ( cluster[loop_hit] == leg1[index_min] || cluster[loop_hit] == leg2[index_min] ) && ( current_valid[loop_hit] ) )
                    {
                        current_label[loop_hit] = leg;
                        nb++;
                    }
                    else
                        current_label[loop_hit] = unassociated;
                ROS_INFO("%i", nb);
                populateMarkerTopic();

            }
            else {
                frequency--;
                if ( uncertainty < uncertainty_max_leg )
                    uncertainty += uncertainty_inc_leg;

                for(int loop_hit=0; loop_hit<nb_beams; loop_hit++)
                        current_label[loop_hit] = unassociated;
            }
            ROS_INFO("moving_person_tracked: (%f, %f, %i, %f):", moving_person_tracked.x, moving_person_tracked.y, frequency, uncertainty);
        }

        moving_person_detected = frequency > 0;
        populateMarkerTopic();
    /*    ROS_INFO("press enter to continue");
        getchar();*/
        ROS_INFO("moving person tracked done");

}

void copy_tables() {

    for( int current_loop=0; current_loop<nb_beams; current_loop++ )
    {

        previous_scan[current_loop] = current_scan[current_loop];
        previous_label[current_loop] = current_label[current_loop];
        previous_valid[current_loop] = current_valid[current_loop];

    }

}

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
        current_valid[loop] = ( r[loop] < range_max );

        //transform the scan in cartesian framewrok
        current_scan[loop].x = r[loop] * cos(beam_angle);
        current_scan[loop].y = r[loop] * sin(beam_angle);
        current_scan[loop].z = 0.0;
        //ROS_INFO("laser[%i]: (%f, %f) -> (%f, %f)", loop, range[loop], beam_angle*180/M_PI, current_scan[loop].x, current_scan[loop].y);

    }

}//scanCallback

void robot_movingCallback(const std_msgs::Bool::ConstPtr& state) {

    init_robot = true;
    current_robot_moving = state->data;

//    ROS_INFO("New data of robot_moving received");

    /*if ( current_robot_moving )
        ROS_INFO("robot is moving");
    else
        ROS_INFO("robot is not moving");*/

}//robot_movingCallback

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

    pub_perception_marker.publish(references);

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

    pub_perception_marker.publish(marker);
    populateMarkerReference();

}

};

int main(int argc, char **argv){

    ros::init(argc, argv, "perception");

    ROS_INFO("waiting for activation of perception");
    perception bsObject;

    ros::spin();

    return 0;
}
