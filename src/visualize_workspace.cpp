#include "visualize_workspace.h"
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <boost/foreach.hpp>
#include <ros/node_handle.h>
#include <ros/package.h>

#define foreach BOOST_FOREACH

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
        visualization_msgs::Marker::ConstPtr i = m.instantiate<visualization_msgs::Marker>();
        if (i != nullptr) {
            points.header.frame_id = i->header.frame_id;
            points.header.stamp = i->header.stamp;
            points.ns = i->ns;
            points.action = i->action;
            points.id = i->id;
            points.type = i->type;
            points.scale.x = i->scale.x;
            points.scale.y = i->scale.y;
            points.points = i->points;
            points.colors = i->colors;
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