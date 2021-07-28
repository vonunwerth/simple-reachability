#include <vector>
#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/Marker.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include "simple_reachability/CLCORegion.h"
#include "simple_reachability/CLCOResult.h"
#include "Region.cpp"
#include <rosbag/view.h>

/**
 * Converts Regions to CLCORegions and calls the proper method
 * @param regions Regionlist
 * @param path Path to save the individual bags
 */
void saveIndividualBagFiles(const std::vector<Region> &regions, const std::string &path, double resolution) {
// Writing the individual regions as bag files for easy visualization
    int counter = 0;
    std_msgs::ColorRGBA BLACK;
    BLACK.r = 0;
    BLACK.g = 0;
    BLACK.b = 0;
    BLACK.a = 1.0;

    for (const Region& region : regions) {
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
        marker.scale.x = resolution; //TODO depending on resolution
        marker.scale.y = resolution;
        marker_initial_poses.scale.x = resolution;
        marker_initial_poses.scale.y = resolution;
        marker_initial_poses.scale.z = resolution;
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
        std::string filename = "master_region_visualizer" + std::to_string(counter) + ".bag"; //TODO increase counter?!!!!!!!!!!!! Hier weiter 27.07.2021 20:52
        saveBag.open(path + filename,
                     rosbag::bagmode::Write); // Save bag in the bags folder of the package
        saveBag.write("/visualization_marker", ros::Time::now(), marker);
        saveBag.close();
        counter++;

    }
}

void saveIndividualBagFiles(const std::vector<Region> &regions, const std::string &path) {
    saveIndividualBagFiles(regions, path, 0.05);
}

void saveIndividualBagFiles(const std::vector<simple_reachability::CLCORegion> &regions, const std::string &path, double resolution) {
    std::vector<Region> new_regions;
    for (const simple_reachability::CLCORegion& region: regions) {
        Region r;
        r.reachable_poses = region.reachable_poses;
        r.initial_pose = region.initial_pose;
        r.id = region.id;
        new_regions.push_back(r);
    }
    saveIndividualBagFiles(new_regions, path, resolution);
}

void saveIndividualBagFiles(const std::vector<simple_reachability::CLCORegion> &regions, const std::string &path) {
    saveIndividualBagFiles(regions, path, 0.05);
}

int extract_regions() {
    rosbag::Bag bag;
    std::string path = ros::package::getPath("simple_reachability");
    std::string file_name = "clco/clco.bag";
    bag.open(path + "/bags/" + file_name);  // BagMode is Read by default

    std::vector<Region> region_list;
    for (rosbag::MessageInstance const m: rosbag::View(bag)) {
        simple_reachability::CLCOResult::ConstPtr region_msg = m.instantiate<simple_reachability::CLCOResult>();

        for (const simple_reachability::CLCORegion &region : region_msg->regions) {
            Region r(region.initial_pose, region.reachable_poses, region.id);
            region_list.push_back(r);
        }
    }

    saveIndividualBagFiles(region_list, path + "/bags/clco/singles/");
}

