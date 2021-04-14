#include "visualize_workspace.h"
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <boost/foreach.hpp>
#include <ros/node_handle.h>
#include <ros/package.h>
#include <armadillo>

#define foreach BOOST_FOREACH

const std::vector<int> FRONT_HEMISPHERE = {0, INT_MAX, INT_MIN, INT_MAX, INT_MIN, INT_MAX}; //TODO in play programm möglich machen, dass schnitt gepublisht wird, irgendwie clever filtern möglich machen, ggf. x,y,z ranges erlauben
const std::vector<int> REAR_HEMISPHERE = {INT_MIN, 0, INT_MIN, INT_MAX, INT_MIN, INT_MAX};
const std::vector<int> LEFT_HEMISPHERE = {INT_MIN, INT_MAX, 0, INT_MAX, INT_MIN, INT_MAX};
const std::vector<int> RIGHT_HEMISPHERE = {INT_MIN, INT_MAX, INT_MIN, 0, INT_MIN, INT_MAX};
const std::vector<int> UPPER_HEMISPHERE = {INT_MIN, INT_MAX, INT_MIN, INT_MAX, 0, INT_MAX};
const std::vector<int> LOWER_HEMISPHERE = {INT_MIN, INT_MAX, INT_MIN, INT_MAX, INT_MIN, 0};
const std::vector<int> FULL = {INT_MIN, INT_MAX, INT_MIN, INT_MAX, INT_MIN, INT_MAX};




//TODO frame mit dem map erstellt wurde muss da sein?! ist das gut?
int main(int argc, char **argv) {
    ros::init(argc, argv, "simple_reachability");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    ros::Rate r(1);
    ros::Publisher marker_pub = node_handle.advertise<visualization_msgs::Marker>("visualization_marker", 10);

    rosbag::Bag bag;
    std::string path = ros::package::getPath("simple-reachability");
    bag.open(path + "/bags/test.bag");  // BagMode is Read by default //TODO Bagname and path as param

    visualization_msgs::Marker points;

    for (rosbag::MessageInstance const m: rosbag::View(bag)) {
        visualization_msgs::Marker::ConstPtr marker = m.instantiate<visualization_msgs::Marker>();
        if (marker != nullptr) {
            points.header.frame_id = marker->header.frame_id;
            points.header.stamp = marker->header.stamp;
            points.ns = marker->ns;
            points.action = marker->action;
            points.id = marker->id;
            points.type = marker->type;
            points.scale.x = marker->scale.x;
            points.scale.y = marker->scale.y;
            std::vector<int> range; // x_min, x_max, y_min, y_max, z_min, z_max TODO as param für schnitte

            range = FULL;
            for (int i = 0; i < marker->points.size(); i++) {
                geometry_msgs::Point point = marker->points.at(i);
                if (point.x >= range.at(0) and point.x <= range.at(1) and point.y >= range.at(2) and point.y <= range.at(3) and point.z >= range.at(4) and point.z <= range.at(5)) { //if marker is in requested range TODO einige werte sind nicht exakt 0 , irgendwo sinnvoll numerische Fehler wegrunden
                    points.points.push_back(point);
                    points.colors.push_back(marker->colors.at(i));
                }
            }
        }
    }

    bag.close();
    ROS_INFO_STREAM("Bag loaded successfully");

    spinner.start();

    while (ros::ok()) {
        marker_pub.publish(points);
        r.sleep();
    }

    ros::shutdown();
    return 0;
}
