#include "ros/ros.h"
#include "ros/time.h"
#include "sensor_msgs/LaserScan.h"
#include "visualization_msgs/Marker.h"
#include "geometry_msgs/Point.h"
#include "std_msgs/ColorRGBA.h"
#include <cmath>
#include "std_msgs/Bool.h"

//used for clustering
#define cluster_threshold 0.2 //threshold for clustering

using namespace std;

class perform_clustering_node {

private:
    ros::NodeHandle n;

    ros::Subscriber sub_scan;
    ros::Publisher pub_perform_clustering_marker;

    // to store, process and display laserdata
    int nb_beams;
    float range_min, range_max;
    float angle_min, angle_max, angle_inc;
    float r[1000], theta[1000];
    geometry_msgs::Point current_scan[1000];

    // GRAPHICAL DISPLAY
    int nb_pts;
    geometry_msgs::Point display[1000];
    std_msgs::ColorRGBA colors[1000];

    bool init_laser;//to check if new data of laser is available or not

public:

perform_clustering_node() {

    sub_scan = n.subscribe("scan", 1, &perform_clustering_node::scanCallback, this);

    pub_perform_clustering_marker = n.advertise<visualization_msgs::Marker>("perform_clustering_marker", 1); // Preparing a topic to publish our results. This will be used by the visualization tool rviz

    init_laser = false;

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
    if ( init_laser ) {
        nb_pts = 0;

        ROS_INFO("\n");
        ROS_INFO("New data of laser received");

        perform_clustering();//to perform clustering

        //graphical display of the results
        populateMarkerTopic();

    }

}// update

// CLUSTERING
/*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
void perform_clustering() {
//store in the table cluster, the cluster of each hit of the laser
//if the distance between the previous hit and the current one is higher than "cluster_threshold"
//then we end the current cluster with the previous hit and start a new cluster with the current hit

    ROS_INFO("performing clustering");

 /*   int nb_cluster = 0;//to count the number of cluster

    int start = 0; //to store the index of the start of current cluster
    int end;//to store the index of the end of the current cluster

    //graphical display of the start of the current cluster in green
    display[nb_pts] = current_scan[start];

    colors[nb_pts].r = 0;
    colors[nb_pts].g = 1;
    colors[nb_pts].b = 0;
    colors[nb_pts].a = 1.0;
    nb_pts++;

    for( int loop=1; loop<nb_beams; loop++ )//loop over all the hits
        if euclidian DISTANCE between the previous hit and the current one is higher than "cluster_threshold"
        {//the previous hit is the end of the current cluster
            geometry_msgs::Point middle;

            //we end the current cluster
            end = ...;

            //graphical display of the end of the current cluster in red
            display[nb_pts] = current_scan[end];

            colors[nb_pts].r = 1;
            colors[nb_pts].g = 0;
            colors[nb_pts].b = 0;
            colors[nb_pts].a = 1.0;
            nb_pts++;

            middle = ...; // compute the middle of the cluster

            //textual display
            ROS_INFO("cluster[%i] (%f, %f): hit[%i](%f, %f) -> hit[%i](%f, %f)", nb_cluster, middle.x, middle.y, start, current_scan[start].x, current_scan[start].y, end, current_scan[end].x, current_scan[end].y);

            //graphical display of the middle of the current cluster in blue
            display[nb_pts] = middle;

            colors[nb_pts].r = 0;
            colors[nb_pts].g = 0;
            colors[nb_pts].b = 1;
            colors[nb_pts].a = 1.0;
            nb_pts++;

            //the current hit is the start of the next cluster
            //we start the next cluster
            start = ...;

            //graphical display of the start of the current cluster in green
            display[nb_pts] = current_scan[start];

            colors[nb_pts].r = 0;
            colors[nb_pts].g = 1;
            colors[nb_pts].b = 0;
            colors[nb_pts].a = 1.0;
            nb_pts++;

        }*/

    //Dont forget to update and display the last cluster

    ROS_INFO("clustering performed");

}//perfor_clustering

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

// Distance between two points
float distancePoints(geometry_msgs::Point pa, geometry_msgs::Point pb) {

    return sqrt(pow((pa.x-pb.x),2.0) + pow((pa.y-pb.y),2.0));

}

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

    pub_perform_clustering_marker.publish(references);

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

    pub_perform_clustering_marker.publish(marker);
    populateMarkerReference();

}

};

int main(int argc, char **argv){

    ros::init(argc, argv, "perform_clustering_node");

    perform_clustering_node bsObject;

    ros::spin();

    return 0;
}
