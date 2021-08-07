#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/kinematics_metrics/kinematics_metrics.h>
#include <tf/tf.h>
#include <rosbag/bag.h>
#include <cmath>
#include <rosbag/view.h>
#include <cstdio>
#include <unordered_map>
#include "simple_reachability/CLCOResult.h"
#include "simple_reachability/CLCORegion.h"
#include "utils.cpp"

/**
 * Calculates the CLCO regions
 * @param argc Not used
 * @param argv Not used
 * @return 0 on success
 */
int main(int argc, char **argv) {
    ros::init(argc, argv, "calculate_constant_level_constant_orientation_regions");
    ros::NodeHandle node_handle("~"); // Allow access to private ROS parameters

    // Load the planning group parameter from the configuration
    std::string planning_group;
    if (node_handle.getParam("planning_group", planning_group)) ROS_INFO_STREAM("Planning group: " << planning_group);
    else {
        planning_group = "manipulator"; // Default value
        ROS_INFO_STREAM(
                "Param \"planning group\" not provided. simple_reachability will use default planning group \"manipulator\".");
    }

    // Load the base_link parameter from the configuration
    std::string base_link;
    node_handle.param<std::string>("manipulator_base_link", base_link, "ur10_base_link");

    //Initializing robot arm configuration
    moveit::planning_interface::MoveGroupInterface move_group(planning_group);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    const robot_model::RobotModelPtr &kinematic_model = robot_model_loader.getModel();
    ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());
    robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
    moveit::core::RobotState robot_state(robot_model_loader.getModel());

    kinematic_state->setToDefaultValues();
    move_group.setGoalOrientationTolerance(0.001); // Set an orienation tolerance
    //Set reference frame for planning to the ur base link
    move_group.setPoseReferenceFrame(base_link);

    ROS_INFO("%s", "Calculating regions");

    // Load the resolution parameter from the configuration
    double resolution; // Resolution in meter
    node_handle.param<double>("resolution", resolution, 0.05);
    ROS_INFO_STREAM("Workspace resolution: " << resolution);

    // Load the resolution parameter from the configuration
    double resolution_initials; // Resolution in meter
    node_handle.param<double>("resolution_initial_poses", resolution_initials, 0.05);
    ROS_INFO_STREAM("Initial pose resolution: " << resolution_initials);


    // Load the radius parameter from the configuration file
    double radius;
    node_handle.param("manipulator_reach", radius, 1.30);

    // Loads a specified detail region from the configuration file
    double x_min, x_max, y_min, y_max, z_min, z_max;
    node_handle.param<double>("x_min", x_min, -radius);
    node_handle.param<double>("x_max", x_max, radius);
    node_handle.param<double>("y_min", y_min, -radius);
    node_handle.param<double>("y_max", y_max, radius);
    node_handle.param<double>("z_min", z_min, -radius);
    node_handle.param<double>("z_max", z_max, radius);

    if ((x_min > x_max) or (y_min > y_max) or (z_min > z_max)) {
        ROS_ERROR_STREAM("The minimum values must be lower/equal than the max values for a region");
    }

    if ((radius < resolution) or
        (std::abs(x_min - x_max) < resolution and std::abs(y_min - y_max) < resolution and std::abs(z_min - z_max) <
                                                                                           resolution)) { // Only if a complete calculation should be done check if resolution is set to a proper value
        ROS_ERROR_STREAM(
                "The volume of the given region is lower than the resolution. This will result in only a single marker in the origin. Check your resolution and manipulator_reach parameters.");
    }

    // Load the end-effector orientation parameters from the configuration
    geometry_msgs::Pose pose;
    tf::Vector3 z_axis(0, 0, 1);
    tf::Vector3 x_axis(1, 0, 0);
    double orientation_tolerance;
    //node_handle.getParam("ee_orientation_x", pose.orientation.x);
    //node_handle.getParam("ee_orientation_y", pose.orientation.y);
    //node_handle.getParam("ee_orientation_z", pose.orientation.z);
    //node_handle.getParam("ee_orientation_w", pose.orientation.w);
    pose.orientation.x = 0.707; //Used for target poses and initial poses
    pose.orientation.y = 0;
    pose.orientation.z = 0;
    pose.orientation.w = -0.707;

    node_handle.param<double>("orientation_tolerance", orientation_tolerance, 0.01);
    ROS_INFO_STREAM(
            "End-Effector Rotation set to (x,y,z,w): (" << pose.orientation.x << ", " << pose.orientation.y << ", "
                                                        << pose.orientation.z << ", " << pose.orientation.w << ")");


    // Steps of calculation and the current step
    unsigned long steps = 0;
    unsigned long step_counter = 0;

    ros::AsyncSpinner spinner(12);
    spinner.start();

    // Generate target poses, this also checks if a partial result already contains a pose
    ROS_INFO_STREAM(
            "Generating target pose list to check if the robot can reach that poses in the next step.");
    tf::Point position;

    // Calculate all target poses on the plane
    std::vector<geometry_msgs::Pose> target_poses; // List for all pose to go to from each initial point
    std::vector<geometry_msgs::Point> temp_positions; // Must be used because Pose has no ==

    y_max = -0.4;//TODO DEBUG

    //Calculate target poses
    ROS_INFO("min/max values %f, %f, %f, %f, %f, %f", x_min, y_min, z_min, x_max, y_max, z_max);
    for (double z = z_min; z <= z_max; z += resolution) { //TODO als Methode auslagern für inital und targets
        for (double x = x_min; x <= x_max; x += resolution) {
            for (double y = y_min; y <= y_max; y += resolution) {
                //pose.orientation.w = 0.707;
                //pose.orientation.x = -0.707;
                position.setX(x);
                position.setY(y);
                position.setZ(z);

                // Only a workspace region is to be calculated
                tf::pointTFToMsg(position, pose.position);
                if ((position.length() <=
                     radius) and
                    (std::find(temp_positions.begin(), temp_positions.end(), pose.position) ==
                     temp_positions.end())) { // Only if they are in the sphere anyways, the pose should be added to the target_poses
                    target_poses.push_back(pose);
                    temp_positions.push_back(pose.position);
                }
            }
            steps = target_poses.size();
            ROS_INFO("Currently in queue: %lu steps: %zu targets and %d initials.", steps, target_poses.size(), 0);
        }
    }

    //TODO as method with resolution parameter
    //Calculate all initial poses on the plane using their own resolution
    std::vector<geometry_msgs::Pose> initial_poses;
    temp_positions.clear();
    for (double z = z_min; z <= z_max; z += resolution_initials) {
        for (double x = x_min; x <= x_max; x += resolution_initials) {
            for (double y = y_min; y <= y_max; y += resolution_initials) {
                //pose.orientation.w = 0.707;
                //pose.orientation.x = -0.707;
                position.setX(x);
                position.setY(y);
                position.setZ(z);

                // Only a workspace region is to be calculated
                tf::pointTFToMsg(position, pose.position);
                if ((position.length() <=
                     radius) and
                    (std::find(temp_positions.begin(), temp_positions.end(), pose.position) ==
                     temp_positions.end())) { // Only if they are in the sphere anyways, the pose should be added to the target_poses
                    initial_poses.push_back(pose);
                    temp_positions.push_back(pose.position);
                }
            }
            steps = target_poses.size() * initial_poses.size();
            ROS_INFO("Currently in queue: %lu steps: %zu targets and %zu initials.", steps, target_poses.size(),
                     initial_poses.size());
        }
    }

    steps = target_poses.size() * initial_poses.size();
    ros::Time begin = ros::Time::now();
    ros::Duration elapsed = ros::Time::now() - begin;


    simple_reachability::CLCOResult result;
    std::vector<geometry_msgs::Pose> region_poses; //Count "green" points for each initial pose
    std::vector<int> region_counts;

    int region_id = 0;
    for (geometry_msgs::Pose initial_pose : initial_poses) {

        //HOMING
        homing(move_group, planning_group);

        simple_reachability::CLCORegion r;
        r.id = region_id;
        region_id++; //TODO regionen mit gleicher id in regions_list??
        int result_count = 0;
        r.initial_pose = initial_pose;

        if (!move_arm(node_handle, move_group, initial_pose)) {
            ROS_WARN("Skipping %ld steps.", target_poses.size());
            step_counter += target_poses.size();
            ROS_INFO_STREAM("Completed " << step_counter << " of " << steps);
            elapsed += ros::Time::now() - begin;
            begin = ros::Time::now();
            int estimated_time = (elapsed.toSec() / step_counter) * (steps - step_counter);
            int minutes = estimated_time / 60;
            int hours = minutes / 60;
            ROS_INFO("Time to complete: %02d:%02d:%02d", int(hours), int(minutes % 60), int(estimated_time % 60));
            continue; //Continue with next initial pose
        }

        for (geometry_msgs::Pose target_pose : target_poses) {
            if (ros::ok()) {
                if (move_arm_constrained(node_handle, move_group, target_pose, false)) { //TODO ggf. hier mit mehr Farben arbeiten
                    result_count++;
                    r.reachable_poses.push_back(target_pose);
                }
                step_counter++;
                ROS_INFO_STREAM("Completed " << step_counter << " of " << steps);

                elapsed += ros::Time::now() - begin;
                begin = ros::Time::now();
                int estimated_time = (elapsed.toSec() / step_counter) * (steps - step_counter);
                int minutes = estimated_time / 60;
                int hours = minutes / 60;
                ROS_INFO("Time to complete: %02d:%02d:%02d", int(hours), int(minutes % 60), int(estimated_time % 60));
            }
        }
        region_poses.push_back(initial_pose);
        region_counts.push_back(result_count);
        result.regions.push_back(r);
    }


    int counter = 0;
    for (geometry_msgs::Pose region : region_poses) {
        ROS_INFO("Init: %f|%f|%f --> %d", region.position.x, region.position.y, region.position.z,
                 region_counts[counter]);
        counter++;
    }

    rosbag::Bag bag;

    std::string filename = "clco_fox.bag";
    std::string path = ros::package::getPath("simple_reachability");
    bag.open(path + "/bags/clco/" + filename, rosbag::bagmode::Write); // Save bag in the bags folder of the package
    bag.write("/clco_results", ros::Time::now(), result);
    bag.close();

    path = path + "/bags/clco/singles_fox/";
    saveIndividualBagFiles(result.regions, path, resolution);

    ros::shutdown();
    return 0;
}