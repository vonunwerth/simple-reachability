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

    rosbag::Bag bag;
    std::string path = ros::package::getPath("simple_reachability");
    std::string file_name = "clco_0_05.bag";
    bag.open(path + "/bags/" + file_name);  // BagMode is Read by default

    visualization_msgs::Marker marker;
    visualization_msgs::Marker marker_initial_poses;
    marker.header.frame_id = "ur10_base_link";
    marker_initial_poses.header.frame_id = "ur10_base_link";
    marker.header.stamp = ros::Time(0);
    marker_initial_poses.header.stamp = ros::Time(0);
    marker.ns = "points";
    marker_initial_poses.ns = "points";
    marker.action = visualization_msgs::Marker::ADD;
    marker_initial_poses.action = visualization_msgs::Marker::ADD;
    marker.id = 0;
    marker_initial_poses.id = 1;
    marker.type = visualization_msgs::Marker::POINTS;
    marker_initial_poses.type = visualization_msgs::Marker::SPHERE_LIST;
    marker.scale.x = 0.02; //TODO depending on resolution
    marker.scale.y = 0.02;
    marker_initial_poses.scale.x = 0.02;
    marker_initial_poses.scale.y = 0.02;
    marker_initial_poses.scale.z = 0.02;

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

    int max = 0;
    Region best = Region(0, 0);
    for (Region region: region_list) {
        if (region.reachable_poses.size() > max) {
            max = region.reachable_poses.size();
            best = region;
        }
    }

    std_msgs::ColorRGBA BLACK;
    BLACK.r = 0;
    BLACK.g = 0;
    BLACK.b = 0;
    BLACK.a = 1.0;

    std::vector<Region> region_list_best;
    region_list_best.push_back(best);

    ROS_INFO("%zu", best.reachable_poses.size());
    for (const Region& region : region_list_best) {
        std_msgs::ColorRGBA color;
        color.r = rand() % 255;
        color.g = rand() % 255;
        color.b = rand() % 255;
        color.a = 1.0;
        for (geometry_msgs::Pose pose : region.reachable_poses) {
            marker.points.push_back(pose.position);
            if (Region::equal_position(pose.position, region.initial_pose.position))
                marker.colors.push_back(BLACK);
            else
                marker.colors.push_back(color);
        }
        ROS_INFO("%zu", marker.points.size());
    }

    ROS_INFO("Finished.");
    bag.close();

    rosbag::Bag saveBag;
    std::string filename = "best_region_visualizer.bag";
    saveBag.open(path + "/bags/" + filename, rosbag::bagmode::Write); // Save bag in the bags folder of the package
    saveBag.write("/visualization_marker", ros::Time::now(), marker);
    saveBag.close();
}