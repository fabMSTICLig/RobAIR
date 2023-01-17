//localization using lidar data
// written by O. Aycard

#include "ros/ros.h"
#include "ros/time.h"
#include "sensor_msgs/LaserScan.h"
#include "visualization_msgs/Marker.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
#include "std_msgs/ColorRGBA.h"
#include <cmath>
#include "nav_msgs/Odometry.h"
#include <tf/transform_datatypes.h>
#include "std_msgs/Float32.h"
#include "nav_msgs/GetMap.h"
#include "nav_msgs/SetMap.h"

using namespace std;

#define angle_resolution 5//in degrees
#define distance_to_travel 1
#define angle_to_travel 20

#define uncertainty 0.05

class sensor_model_node {

private:
    ros::NodeHandle n;

    ros::Subscriber sub_scan;
    ros::Subscriber sub_position;
    ros::Publisher pub_sensor_model_marker;

    // to store, process and display laserdata
    bool init_laser;
    int nb_beams;
    float range_min, range_max;
    float angle_min, angle_max, angle_inc;
    float r[1000], theta[1000];
    geometry_msgs::Point current_scan[1000];

    //to store the map
    nav_msgs::GetMap::Response resp;
    geometry_msgs::Point min, max;
    float cell_size;
    int width_max;
    int height_max;

    // GRAPHICAL DISPLAY
    int nb_pts;
    geometry_msgs::Point display[1000];
    std_msgs::ColorRGBA colors[1000];

    //to store the initial_position of the mobile robot
    bool init_position, new_position;
    geometry_msgs::Point initial_position;
    float initial_orientation;

public:

sensor_model_node() {

    sub_scan = n.subscribe("scan", 1, &sensor_model_node::scanCallback, this);
    sub_position = n.subscribe("initialpose", 1, &sensor_model_node::positionCallback, this);

    pub_sensor_model_marker = n.advertise<visualization_msgs::Marker>("sensor_model_marker", 1); // Preparing a topic to publish our results. This will be used by the visualization tool rviz

    // get map via RPC
    nav_msgs::GetMap::Request  req;
    ROS_INFO("Requesting the map...");
    while(!ros::service::call("static_map", req, resp)) {
      ROS_WARN("Request for map failed; trying again...");
      ros::Duration d(0.5);
      d.sleep();
    }

    init_laser = false;
    init_position = false;
    new_position = false;

    width_max = resp.map.info.width;
    height_max = resp.map.info.height;
    cell_size = resp.map.info.resolution;
    min.x = resp.map.info.origin.position.x;
    min.y = resp.map.info.origin.position.y;
    max.x = min.x + width_max*cell_size;
    max.y = min.y + height_max*cell_size;

    ROS_INFO("map loaded");
    ROS_INFO("Map: (%f, %f) -> (%f, %f) with size: %f",min.x, min.y, max.x, max.y, cell_size);
    ROS_INFO("wait for initial pose");

    //INFINTE LOOP TO COLLECT LASER DATA AND PROCESS THEM
    ros::Rate r(10);// this node will work at 10hz
    while (ros::ok()) {
        ros::spinOnce();//each callback is called once
        update();
        r.sleep();//we wait if the processing (ie, callback+update) has taken less than 0.1s (ie, 10 hz)
    }
}

//UPDATE: main processing of laser data
/*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
void update() {

    if ( init_laser && init_position && new_position )
    {

        new_position = false;
        int score_current = sensor_model(initial_position.x, initial_position.y, initial_orientation);
        ROS_INFO("initial_position(%f, %f, %f): score = %i", initial_position.x, initial_position.y, initial_orientation*180/M_PI, score_current);
        populateMarkerTopic();

    }
    else
        if ( !init_laser || !init_position )
            ROS_INFO("waiting for laser data or initialization of position");

}// update

int sensor_model(float x, float y, float o)
{
//compute the score of the position (x, y, o)

    nb_pts = 0;
    // we add the current hit to the hits to display
    display[nb_pts].x = x;
    display[nb_pts].y = y;
    display[nb_pts].z = 0;

    colors[nb_pts].r = 0;
    colors[nb_pts].g = 0;
    colors[nb_pts].b = 1;
    colors[nb_pts].a = 1.0;
    nb_pts++;

    // we add the current hit to the hits to display
    display[nb_pts].x = x+cos(o);
    display[nb_pts].y = y+sin(o);
    display[nb_pts].z = 0;

    colors[nb_pts].r = 1;
    colors[nb_pts].g = 1;
    colors[nb_pts].b = 0;
    colors[nb_pts].a = 1.0;
    nb_pts++;

    //loop over the hits of the laser
    int score_current = 0;
    float beam_angle = angle_min;
    for (int loop=0 ; loop < nb_beams; loop++, beam_angle += angle_inc)
    {
        //for each hit of the laser, we compute its position in the map and check if it is occupied or not

        geometry_msgs::Point hit;
        //hit.x = ...;
        //hit.y = ..;

        // we add the current hit to the hits to display
        display[nb_pts] = hit;

        bool cell_occupied = false;
        //loop over the positions surronding the current hit of the laser and test if one of this cell is occupied
        for(float loop_x=hit.x-uncertainty; loop_x<=hit.x+uncertainty;loop_x += uncertainty)
            for(float loop_y=hit.y-uncertainty; loop_y<=hit.y+uncertainty;loop_y += uncertainty)
                //test if the current hit of the laser corresponds to an occupied cell
                cell_occupied = cell_occupied || ( cell_value(loop_x, loop_y) == 100 );

        if ( cell_occupied )
        {
            score_current++;

            // when matching is ok: the hit of the laser is green
            colors[nb_pts].r = 0;
            colors[nb_pts].g = 1;
            colors[nb_pts].b = 0;
            colors[nb_pts].a = 1.0;
        }
        //the current hit of the laser corresponds to a free cell
        else {
            // when matching is not ok: the hit of the laser is red
            colors[nb_pts].r = 1;
            colors[nb_pts].g = 0;
            colors[nb_pts].b = 0;
            colors[nb_pts].a = 1.0;
        }
        nb_pts++;
    }

return(score_current);

}

int cell_value(float x, float y) {
//returns the value of the cell corresponding to the position (x, y) in the map
//returns 100 if cell(x, y) is occupied, 0 if cell(x, y) is free

    if ( ( min.x <= x ) && ( x <= max.x ) && ( min.y <= y ) && ( y <= max.y ) ) {
        float x_cell = (x-min.x)/cell_size;
        float y_cell = (y-min.y)/cell_size;
        int x_int = x_cell;
        int y_int = y_cell;
        //ROS_INFO("cell[%f = %d][%f = %d] = %d", x_cell, x_int, y_cell, y_int, map[x_int][y_int]);
        return(resp.map.data[width_max*y_int+x_int]);
    }
    else
        return(-1);

}

//CALLBACK
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

void positionCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& p) {

    init_position = true;
    new_position = true;

    initial_position.x = p->pose.pose.position.x;
    initial_position.y = p->pose.pose.position.y;
    initial_orientation = tf::getYaw(p->pose.pose.orientation);

}

//GRAPHICAL_DISPLAY
/*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
void populateMarkerTopic(){

    visualization_msgs::Marker marker;

    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "example";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::POINTS;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.orientation.w = 1;

    marker.scale.x = 0.05;
    marker.scale.y = 0.05;

    marker.color.a = 1.0;

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

    pub_sensor_model_marker.publish(marker);

}

// Distance between two points
float distancePoints(geometry_msgs::Point pa, geometry_msgs::Point pb) {

    return sqrt(pow((pa.x-pb.x),2.0) + pow((pa.y-pb.y),2.0));

}

};


int main(int argc, char **argv){

    ros::init(argc, argv, "compute_score_node");

    sensor_model_node bsObject;

    ros::spin();

    return 0;
}


