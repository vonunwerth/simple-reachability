#include <rosbag/bag.h>
#include <ros/ros.h>
#include <rosbag/view.h>
#include <visualization_msgs/Marker.h>
#include "simple_reachability/CLCORegion.h"
#include "simple_reachability/CLCOResult.h"
#include "Region.cpp"

Region master_region;

int region_with_initial_pose(geometry_msgs::Pose pose, const std::vector<Region> &region_list) {
    for (const Region &r : region_list) {
        if (Region::equal_position(r.initial_pose.position, pose.position)) {
            return r.id;
        }
    }
    return -1; // Region with that initial pose not found
}

/**
 * If there is a way from p to all poses in the master_region and a way from all poses in the master region to p fill the master_region
 * @param p
 * @param region_list
 */
void paths(const ReachablePose& p, std::vector<Region> region_list) {
    int p_to_all_count = 0;
    for (const ReachablePose& goal : master_region.reachable_poses) {
        if (region_list[region_with_initial_pose(p.pose, region_list)].contains(goal.pose)) {
            p_to_all_count++;
        }
    }
    bool p_to_all = false;
    if (p_to_all_count == master_region.reachable_poses.size()) p_to_all = true;

    int all_to_p_count = 0;
    for (const ReachablePose& start : master_region.reachable_poses) {
        if (region_list[region_with_initial_pose(start.pose, region_list)].contains(p.pose)) {
            all_to_p_count++;
        }
    }
    bool all_to_p = false;
    if (all_to_p_count == master_region.reachable_poses.size()) all_to_p = true;

    if (all_to_p and p_to_all_count) {
        master_region.reachable_poses.push_back(p);
    }
}

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

    std_msgs::ColorRGBA BLACK;
    BLACK.r = 0;
    BLACK.g = 0;
    BLACK.b = 0;
    BLACK.a = 1.0;

    Region region1 = region_list[0];
//    for (Region region : region_list) {
//
//    }


    master_region.initial_pose = region1.initial_pose;
    ReachablePose rp(region1.calculate_neighbours(region1.initial_pose), region1.initial_pose);
    master_region.reachable_poses.push_back(rp);
    paths(rp, region_list);

    for (ReachablePose p : region1.reachable_poses) { // Es reicht aus, alle Posen in region1 zu checken und nicht rekursiv von der initialpose alle nachbarn zu checken, da auf keinen Fall mehr dazu kommen als bereits in region1 sind --> alle außerhalb von region1 sind von der initialpose von region1 nicht erreichbar! Über Nachbarn wäre effizienter, aber egal
        paths(p, region_list);
    }

    Region rr = master_region;
    ROS_INFO("Finished.");
    bag.close();

    rosbag::Bag saveBag;
    std::string filename = "best_region_visualizer.bag";
    saveBag.open(path + "/bags/" + filename, rosbag::bagmode::Write); // Save bag in the bags folder of the package
    saveBag.write("/visualization_marker", ros::Time::now(), marker);
    saveBag.close();
}
