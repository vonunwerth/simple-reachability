#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <std_msgs/Int32.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <boost/foreach.hpp>
#include <ros/node_handle.h>
#include <ros/package.h>

/**
 * Do some stuff with a bag file, change lines 47-49 do do your own changes
 * @param argc
 * @param argv
 * @return
 */
int main(int argc, char **argv) {
    ros::init(argc, argv, "ros_bag_editor");
    ros::NodeHandle node_handle("~"); // Allow access to private ROS parameters

    rosbag::Bag bag;
    std::string path = ros::package::getPath("simple_reachability");
    std::string file_name;
    std::cout << "Please enter a bag file name to edit. (from outgoing from the simple_reachability/bags folder)\n";
    std::cin >> file_name;
    bag.open(path + "/bags/" + file_name);  // BagMode is Read by default

    visualization_msgs::Marker points;

    for (rosbag::MessageInstance const m: rosbag::View(bag)) {
        visualization_msgs::Marker::ConstPtr marker = m.instantiate<visualization_msgs::Marker>();
        ROS_INFO_STREAM("Loading " << marker->points.size() << " Markers");
        if (marker != nullptr) {
            points.header.frame_id = marker->header.frame_id;
            points.header.stamp = marker->header.stamp;
            points.ns = marker->ns;
            points.action = marker->action;
            points.id = marker->id;
            points.type = marker->type;
            points.scale.x = 0.05;
            points.scale.y = 0.05;

            for (int i = 0; i < marker->points.size(); i++) {
                geometry_msgs::Point point = marker->points.at(i);
                point.z = -0.45;
                points.points.push_back(point);
                points.colors.push_back(marker->colors.at(i));
            }
        }
    }

    bag.close();
    ROS_INFO_STREAM("Bag loaded successfully");


    rosbag::Bag saveBag;
    std::string filename = file_name + "_remastered.bag";
    saveBag.open(path + "/bags/" + filename, rosbag::bagmode::Write); // Save bag in the bags folder of the package
    saveBag.write("/visualization_marker", ros::Time::now(), points);
    saveBag.close();

    ROS_INFO_STREAM("Saving " << points.points.size() << " Markers");
    ROS_INFO_STREAM("Bag rewritten to " + filename);

    ros::shutdown();
    return 0;
}
