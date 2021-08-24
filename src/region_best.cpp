#include <rosbag/bag.h>
#include <ros/ros.h>
#include <rosbag/view.h>
#include <visualization_msgs/Marker.h>
#include <random>
#include "simple_reachability/CLCORegion.h"
#include "simple_reachability/CLCOResult.h"
#include "utils.cpp"

std::vector<Region> master_regions;


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
void paths(const geometry_msgs::Pose &p, std::vector<Region> region_list, Region &master_region) {
    int p_to_all_count = 0;
    int region_number = region_with_initial_pose(p, region_list);
    for (const geometry_msgs::Pose &goal : master_region.reachable_poses) {
        if (region_number > 0) {
            if (region_list[region_number].contains(goal)) {
                p_to_all_count++;
            } else {
                ROS_WARN("Goal not found: %d does not contain %f|%f|%f", region_number, goal.position.x, goal.position.y, goal.position.z);
            }
        }
    }
    bool p_to_all = (p_to_all_count == master_region.reachable_poses.size());

    int all_to_p_count = 0;
    for (const geometry_msgs::Pose &start : master_region.reachable_poses) {
        if (region_number > 0) {
            int start_number = region_with_initial_pose(start, region_list);
            if (region_list[start_number].contains(p)) {
                all_to_p_count++;
            } else {
                ROS_WARN("P not found: %d does not contain %f|%f|%f", start_number, p.position.x, p.position.y, p.position.z);
            }
        }
    }
    bool all_to_p = false;
    if (all_to_p_count == master_region.reachable_poses.size()) all_to_p = true;

    if (all_to_p and p_to_all) {
        master_region.reachable_poses.push_back(p);
    }
    else {
        ROS_ERROR("Not valid: %f|%f|%f with id: %d", p.position.x, p.position.y, p.position.z, region_number);
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "calculate_constant_level_constant_orientation_workspace");
    ros::NodeHandle node_handle("~"); // Allow access to private ROS parameters

    rosbag::Bag bag;
    std::string path = ros::package::getPath("simple_reachability");
    std::string file_name = "clco/clco.bag"; // TODO als param
    bag.open(path + "/bags/" + file_name);  // BagMode is Read by default

    std::vector<Region> region_list;
    for (rosbag::MessageInstance const m: rosbag::View(bag)) {
        simple_reachability::CLCOResult::ConstPtr region_msg = m.instantiate<simple_reachability::CLCOResult>();

        int id_counter = 0;
        std::vector<simple_reachability::CLCORegion> rr =  region_msg->regions;
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


    for (Region region : region_list) { //region_list_debug for one region only
        ROS_INFO("Calculating master region %d of %zu", region.id, region_list.size() );
        Region master_region;
        master_region.id = region.id;
        master_region.initial_pose = region.initial_pose;
        master_region.reachable_poses.push_back(region.initial_pose);


        std::vector<int> numbers;
        for (int i = 0; i < region.reachable_poses.size(); i++) {
            numbers.push_back(i);
        }
        auto rng = std::default_random_engine {};
        std::shuffle(std::begin(numbers), std::end(numbers), rng);

        std::vector<geometry_msgs::Pose> shuffled_poses;
        for (int num : numbers) {
            shuffled_poses.push_back(region.reachable_poses[num]);
        }
        region.reachable_poses = shuffled_poses;


        for (geometry_msgs::Pose p : region.reachable_poses) {
            paths(p, region_list, master_region);
        }
        master_regions.push_back(master_region);
    }

    //TODO comment this out if not needed
    std::vector<Region> cut_region_list = master_regions;
    float y_max = -0.40; // configure for your robot
    for (const Region &region : master_regions) {
        if (region.initial_pose.position.y <= y_max) {
            std::vector<geometry_msgs::Pose> new_re_poses;
            for (geometry_msgs::Pose pose : region.reachable_poses) {
                if (pose.position.y <= y_max) {
                    new_re_poses.push_back(pose);
                }
            }
            Region r(region.initial_pose, new_re_poses, region.id);
            cut_region_list.push_back(r);
        }
    }
    ROS_INFO("Reduced to %zu regions", cut_region_list.size());


    for (const Region &r : cut_region_list) {
        ROS_INFO("%d : %zu", r.id, r.reachable_poses.size());
    }

    int max = 0;
    int master_id = -1;
    for (const Region &mr : cut_region_list) {
        if (mr.reachable_poses.size() > max) {
            max = mr.reachable_poses.size();
            master_id = mr.id;
        }
    }

    ROS_INFO("Max Master Region %d holds %zu reachable poses", master_regions[master_id].id,
             master_regions[master_id].reachable_poses.size());

    saveIndividualBagFiles(cut_region_list, path + "/bags/clco/single_clco_regions/", 0.05);

    ros::shutdown();
    return 0;
}
