#include <rosbag/bag.h>
#include <ros/ros.h>
#include <rosbag/view.h>
#include <visualization_msgs/Marker.h>
#include "simple_reachability/CLCORegion.h"
#include "simple_reachability/CLCOResult.h"
#include "Region.cpp"

int main(int argc, char **argv) {
    ros::init(argc, argv, "calculate_constant_level_constant_orientation_workspace");
    ros::NodeHandle node_handle("~"); // Allow access to private ROS parameters

    ros::Publisher marker_pub = node_handle.advertise<visualization_msgs::Marker>("visualization_marker", 10);

    rosbag::Bag bag;
    std::string path = ros::package::getPath("simple_reachability");
    std::string file_name = "clco_0_05.bag";
    bag.open(path + "/bags/" + file_name);  // BagMode is Read by default

    visualization_msgs::Marker marker;
    marker.header.frame_id = "ur10_base_link";
    marker.header.stamp = ros::Time(0);
    marker.ns = "points";
    marker.action = visualization_msgs::Marker::ADD;
    marker.id = 0;
    marker.type = visualization_msgs::Marker::POINTS;
    double resolution;
    node_handle.param<double>("resolution", resolution, 0.05);
    ROS_INFO_STREAM("Workspace resolution: " << resolution);
    marker.scale.x = resolution;
    marker.scale.y = resolution;

    std::vector<Region> region_list;
    for (rosbag::MessageInstance const m: rosbag::View(bag)) {
        simple_reachability::CLCOResult::ConstPtr region_msg = m.instantiate<simple_reachability::CLCOResult>();

        int id_counter = 0;
        for (const simple_reachability::CLCORegion &region : region_msg->regions) {
            Region r(region.initial_pose, region.reachable_poses, id_counter);
            region_list.push_back(r);
            id_counter++;
        }
    }

    ROS_INFO("Found %zu regions.", region_list.size());

    std::vector<Region> initial_region_list = region_list;
    for (Region region: region_list) {
        ROS_INFO("--- Region %d ---", region.id);
        for (Region region2 : initial_region_list) {// TODO nur ab da wo Schleife 1 noch nicht ist --> über die id: id < ...
            if (region.id != region2.id) {
                if (region2.contains_any_of(region.reachable_poses)) {
                    ROS_INFO("Merging regions %d with %d", region.id, region2.id);
                    region.merge_region(region2);
                    region_list.erase(std::remove(region_list.begin(), region_list.end(), region2), region_list.end());
                    ROS_INFO("Region list size: %zu", region_list.size());
                }
            }
        }
    }

    for (const Region &region : region_list) {
        std_msgs::ColorRGBA color;
        color.r = rand() % 255;
        color.g = rand() % 255;
        color.b = rand() % 255;
        color.a = 1.0;
        for (geometry_msgs::Pose pose : region.reachable_poses) {
            marker.points.push_back(pose.position);
            marker.colors.push_back(color);
        }
    }

    ROS_INFO("Finished.");
    bag.close();

    rosbag::Bag saveBag;
    std::string filename = "region_analysis.bag";
    saveBag.open(path + "/bags/" + filename, rosbag::bagmode::Write); // Save bag in the bags folder of the package
    saveBag.write("/visualization_marker", ros::Time::now(), marker);
    saveBag.close();

}