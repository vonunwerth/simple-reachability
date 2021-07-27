#include <simple_reachability/CLCORegion.h>
#include <vector>
#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/Marker.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include "Region.cpp"

/**
 * Saves a bag for each individual region in a region list
 * @param regions Region list
 * @param path Path to save the bags
 */
void saveIndividualBagFiles(const std::vector<simple_reachability::CLCORegion> regions, const std::string &path) {
    // Writing the individual regions as bag files for easy visualization
    int counter = 0;
    std_msgs::ColorRGBA BLACK;
    BLACK.r = 0;
    BLACK.g = 0;
    BLACK.b = 0;
    BLACK.a = 1.0;

    for (const simple_reachability::CLCORegion &region : regions) {
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
        marker.scale.x = 0.05; //TODO depending on resolution
        marker.scale.y = 0.05;
        marker_initial_poses.scale.x = 0.05;
        marker_initial_poses.scale.y = 0.05;
        marker_initial_poses.scale.z = 0.05;
        ROS_INFO("Writing bag for Region %d with %zu poses.", counter,
                 region.reachable_poses.size());
        //Save results in markers
        std_msgs::ColorRGBA color;
        color.r = rand() % 255;
        color.g = rand() % 255;
        color.b = rand() % 255;
        color.a = 1;
        for (geometry_msgs::Pose p : region.reachable_poses) {
            marker.points.push_back(p.position);
            marker.colors.push_back(color);
        }
        marker.points.push_back(region.initial_pose.position);
        marker.colors.push_back(BLACK);

        ROS_INFO("Finished.");

        rosbag::Bag saveBag;
        std::string filename = "master_region_visualizer" + std::to_string(counter) + ".bag";
        saveBag.open(path + filename,
                     rosbag::bagmode::Write); // Save bag in the bags folder of the package
        saveBag.write("/visualization_marker", ros::Time::now(), marker);
        saveBag.close();

    }
}

/**
 * Converts Regions to CLCORegions and calls the proper method
 * @param regions Regionlist
 * @param path Path to save the individual bags
 */
void saveIndividualBagFiles(const std::vector<Region> &regions, const std::string &path) {
    std::vector<simple_reachability::CLCORegion> clco_regions;
    for (Region r : regions) {
        simple_reachability::CLCORegion clco_r;
        clco_r.reachable_poses = r.reachable_poses;
        clco_r.initial_pose = r.initial_pose;
        clco_r.count = r.reachable_poses.size();
        clco_regions.push_back(clco_r);
    }
    saveIndividualBagFiles(clco_regions, path);
}