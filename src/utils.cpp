#include <vector>
#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/Marker.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include "simple_reachability/CLCORegion.h"
#include "simple_reachability/CLCOResult.h"
#include "Region.cpp"
#include <rosbag/view.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/kinematics_metrics/kinematics_metrics.h>
#include <rosbag/bag.h>
#include <cstdio>

bool homing(moveit::planning_interface::MoveGroupInterface &move_group, std::string planning_group) {
    ROS_INFO("Homing...");
    moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
    std::vector<double> joint_group_positions;
    const moveit::core::JointModelGroup *joint_model_group =
            move_group.getCurrentState()->getJointModelGroup(planning_group);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
    joint_group_positions[0] = 0;
    joint_group_positions[1] = 0;
    joint_group_positions[2] = 0;
    joint_group_positions[3] = 0;
    joint_group_positions[4] = 0;
    joint_group_positions[5] = 0;
    move_group.setJointValueTarget(joint_group_positions);
    bool success = (move_group.plan(my_plan) ==
                    moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (success) {
        move_group.move();
        ROS_INFO("%s", "Reached home");
        return true;
    } else {
        ROS_ERROR("%s", "Can't reach home");
        ros::shutdown();
        return false;
    }
}


/**
 * Move arm constrained
 * @param node_handle  ROS Node handle
 * @param x x
 * @param y x
 * @param z z
 * @param move Move or just plan
 * @return Motion possible
 */
bool
move_arm_constrained(const ros::NodeHandle &node_handle, moveit::planning_interface::MoveGroupInterface &move_group,
                     double x, double y, double z, bool move) {

    geometry_msgs::Pose p2 = move_group.getCurrentPose().pose;
    geometry_msgs::Pose p = move_group.getCurrentPose("ur10_base_link").pose;
    geometry_msgs::Pose initial_pose;
    initial_pose.orientation.w = 0.707;
    initial_pose.orientation.x = -0.707;
    initial_pose.position.x = p2.position.x - p.position.x;
    initial_pose.position.y = p2.position.y - p.position.y;
    initial_pose.position.z = p2.position.z - p.position.z;

    geometry_msgs::Pose target_pose;
    target_pose.orientation.w = 0.707;
    target_pose.orientation.x = -0.707;
    target_pose.orientation.y = 0;
    target_pose.orientation.z = 0;
    target_pose.position.x = x;
    target_pose.position.y = y;
    target_pose.position.z = z;

    std::vector<geometry_msgs::Pose> waypoints = {initial_pose, target_pose};
    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 2.0;
    const double eef_step = 0.01;
    double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

    ROS_INFO("Moving to pose %f|%f|%f", x, y, z); //Unterscheiden zwischen bewegen und planen bei der Ausgabe je nach move param
    if (fraction == 1) {
        if (move) {
            move_group.execute(trajectory);
            ROS_INFO("Moved constrained to pose %f|%f|%f", x, y, z);
            move_group.stop();
            move_group.clearPoseTargets();
            return true;
        } else {
            ROS_INFO("Motion possible");
            return true;
        }
    } else {
        ROS_WARN("Cant move to target pose %f|%f|%f. Only %f percent of the path can be executed.", x, y, z,
                  (fraction * 100));
        return false;
    }
}


bool move_arm(const ros::NodeHandle &node_handle, moveit::planning_interface::MoveGroupInterface &move_group, double x,
              double y, double z, bool move) {
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    geometry_msgs::Pose initial_pose;
    initial_pose.orientation.w = 0.707; //TODO this movegroup, ... as param
    initial_pose.orientation.x = -0.707;
    initial_pose.orientation.y = 0;
    initial_pose.orientation.z = 0;
    initial_pose.position.x = x;
    initial_pose.position.y = y;
    initial_pose.position.z = z;
    move_group.setPoseTarget(initial_pose);

    geometry_msgs::Pose p2 = move_group.getCurrentPose().pose;
    geometry_msgs::Pose p = move_group.getCurrentPose("ur10_base_link").pose;

    ROS_INFO("Moving to pose %f|%f|%f", x, y, z);
    bool success = (move_group.plan(my_plan) ==
                    moveit::planning_interface::MoveItErrorCode::SUCCESS); // Plans a motion to a target_pose, Hint: Choose your IK Plugin in the moveit_config - IKFast could be really useful here
    if (success) {
        if (move) {
            move_group.move();
            move_group.stop();
            move_group.clearPoseTargets();
            ROS_INFO("Moved to pose %f|%f|%f", x, y, z);
            for (int i = 0; i < 6; i++) {
                ROS_INFO("Joint %d : %f", i, move_group.getCurrentJointValues()[i]);
            }
            return true;
        } else {
            ROS_INFO("Motion possible");
            return true;
        }
    } else {
        ROS_FATAL("Cant move to pose %f|%f|%f", x, y, z);
        return false;
    }
}

bool move_arm(const ros::NodeHandle &node_handle, moveit::planning_interface::MoveGroupInterface &move_group, double x,
              double y, double z) {
    return move_arm(node_handle, move_group, x, y, z, true);
}

bool
move_arm_constrained(const ros::NodeHandle &node_handle, moveit::planning_interface::MoveGroupInterface &move_group,
                     double x, double y, double z) {
    return move_arm_constrained(node_handle, move_group, x, y, z, true);
}

bool move_arm(const ros::NodeHandle &node_handle, moveit::planning_interface::MoveGroupInterface &move_group,
              geometry_msgs::Pose p) {
    return move_arm(node_handle, move_group, p.position.x, p.position.y, p.position.z); //TODO orientation
}

bool move_arm_constrained(const ros::NodeHandle &node_handle, moveit::planning_interface::MoveGroupInterface &move_group,
                     geometry_msgs::Pose p) {
    return move_arm_constrained(node_handle, move_group, p.position.x, p.position.y, p.position.z);
}

bool move_arm(const ros::NodeHandle &node_handle, moveit::planning_interface::MoveGroupInterface &move_group,
              geometry_msgs::Pose p, bool move) {
    return move_arm(node_handle, move_group, p.position.x, p.position.y, p.position.z, move); //TODO orientation
}

bool move_arm_constrained(const ros::NodeHandle &node_handle, moveit::planning_interface::MoveGroupInterface &move_group,
                          geometry_msgs::Pose p, bool move) {
    return move_arm_constrained(node_handle, move_group, p.position.x, p.position.y, p.position.z, move);
}


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

    for (const Region &region : regions) {
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
        std::string filename = "master_region_visualizer" + std::to_string(counter) +
                               ".bag"; //TODO increase counter?!!!!!!!!!!!! Hier weiter 27.07.2021 20:52
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

void saveIndividualBagFiles(const std::vector<simple_reachability::CLCORegion> &regions, const std::string &path,
                            double resolution) {
    std::vector<Region> new_regions;
    for (const simple_reachability::CLCORegion &region: regions) {
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
    return 0;
}

