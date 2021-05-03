#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <std_msgs/Int32.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <boost/foreach.hpp>
#include <ros/node_handle.h>
#include <ros/package.h>

std::vector<std::vector<int>> visualization_modes;

int main(int argc, char **argv) {
    ros::init(argc, argv, "simple_reachability");
    ros::NodeHandle node_handle("~");
    ros::AsyncSpinner spinner(1);

    // Read param rate
    int rate;
    node_handle.param("publish_rate", rate, 1);
    ros::Rate r(rate);

    // Read param mode
    int mode;
    node_handle.param("visualization_mode", mode, 0);
    if (mode > 6 or mode < 0) mode = 0;

    ros::Publisher marker_pub = node_handle.advertise<visualization_msgs::Marker>("visualization_marker", 10);

    rosbag::Bag bag;
    std::string path = ros::package::getPath("simple-reachability");

    std::string file_name;
    if (!node_handle.getParam("file_name", file_name)) {
        ROS_ERROR_STREAM("You must provide a file name in the visualization.yaml configuration.");
        ros::shutdown();
    }

    bag.open(path + "/bags/" + file_name);  // BagMode is Read by default

    visualization_msgs::Marker points;

    //Cuts which may be useful
    const std::vector<int> FULL = {INT_MIN, INT_MAX, INT_MIN, INT_MAX, INT_MIN, INT_MAX};
    const std::vector<int> FRONT_HEMISPHERE = {0, INT_MAX, INT_MIN, INT_MAX, INT_MIN, INT_MAX};
    const std::vector<int> REAR_HEMISPHERE = {INT_MIN, 0, INT_MIN, INT_MAX, INT_MIN, INT_MAX};
    const std::vector<int> LEFT_HEMISPHERE = {INT_MIN, INT_MAX, 0, INT_MAX, INT_MIN, INT_MAX};
    const std::vector<int> RIGHT_HEMISPHERE = {INT_MIN, INT_MAX, INT_MIN, 0, INT_MIN, INT_MAX};
    const std::vector<int> UPPER_HEMISPHERE = {INT_MIN, INT_MAX, INT_MIN, INT_MAX, 0, INT_MAX};
    const std::vector<int> LOWER_HEMISPHERE = {INT_MIN, INT_MAX, INT_MIN, INT_MAX, INT_MIN, 0};

    visualization_modes.push_back(FULL);
    visualization_modes.push_back(FRONT_HEMISPHERE);
    visualization_modes.push_back(REAR_HEMISPHERE);
    visualization_modes.push_back(LEFT_HEMISPHERE);
    visualization_modes.push_back(RIGHT_HEMISPHERE);
    visualization_modes.push_back(UPPER_HEMISPHERE);
    visualization_modes.push_back(LOWER_HEMISPHERE);

    for (rosbag::MessageInstance const m: rosbag::View(bag)) {
        visualization_msgs::Marker::ConstPtr marker = m.instantiate<visualization_msgs::Marker>();
        ROS_INFO_STREAM("Visualizing " << marker->points.size() << " Markers");
        if (marker != nullptr) {
            points.header.frame_id = marker->header.frame_id;
            points.header.stamp = marker->header.stamp;
            points.ns = marker->ns;
            points.action = marker->action;
            points.id = marker->id;
            points.type = marker->type;
            node_handle.param<double>("scale", points.scale.x, marker->scale.x);
            points.scale.y = points.scale.x;

            int visualization_mode;
            node_handle.param<int>("visualization_mode", visualization_mode, 0);
            std::vector<int> range = visualization_modes[visualization_mode];

            range = visualization_modes[mode];

            for (int i = 0; i < marker->points.size(); i++) {
                geometry_msgs::Point point = marker->points.at(i);
                if (point.x >= range.at(0) and point.x <= range.at(1) and point.y >= range.at(2) and
                    point.y <= range.at(3) and point.z >= range.at(4) and point.z <= range.at(
                        5)) { //if marker is in requested range
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
