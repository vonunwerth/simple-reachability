
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/kinematics_metrics/kinematics_metrics.h>
#include <tf/tf.h>
#include <rosbag/bag.h>
#include <cmath>
#include <csignal>
#include <rosbag/view.h>
#include <cstdio>
#include <unordered_map>
#include "simple_reachability/CLCOResult.h"
#include "simple_reachability/CLCORegion.h"

std_msgs::ColorRGBA newColor(std_msgs::ColorRGBA rgba);

/**
 * Calculates the workspace of a robot
 * @param argc Not used
 * @param argv Not used
 * @return 0 on success
 */
int main(int argc, char **argv) {
    ros::init(argc, argv, "calculate_constant_level_constant_orientation_workspace");
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

    // Load the resolution parameter from the configuration
    double resolution; // Resolution in meter
    node_handle.param<double>("resolution", resolution, 0.6);
    ROS_INFO_STREAM("Workspace resolution: " << resolution);

    // Load the resolution parameter from the configuration
    double resolution_initials; // Resolution in meter
    node_handle.param<double>("resolution_initials", resolution_initials, 0.6);
    ROS_INFO_STREAM("Workspace resolution: " << resolution_initials);


    // Load the radius parameter from the configuration file
    double radius;
    node_handle.param("manipulator_reach", radius, 1.30);

    // Loads a specified detail region from the configuration file
    double x_min, x_max, y_min, y_max, z_min, z_max;
    node_handle.param<double>("x_min", x_min, -radius);
    node_handle.param<double>("x_max", x_max, radius);
    node_handle.param<double>("y_min", y_min, -radius); //TODO fix all default values in the end
    node_handle.param<double>("y_max", y_max, radius);
    node_handle.param<double>("z_min", z_min, -radius); //TODO check if max param is more than min
    node_handle.param<double>("z_max", z_max, radius);


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

    ros::AsyncSpinner spinner(1);
    spinner.start();
    moveit::planning_interface::MoveGroupInterface move_group(planning_group);
    //TODO move_group.setEndEffector() ?
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    const robot_model::RobotModelPtr &kinematic_model = robot_model_loader.getModel();
    ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());
    robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
    moveit::core::RobotState robot_state(robot_model_loader.getModel());
    kinematic_state->setToDefaultValues();

    //Set reference frame for planning to the ur base link
    move_group.setPoseReferenceFrame(
            base_link);

    // Generate target poses, this also checks if a partial result already contains a pose
    ROS_INFO_STREAM(
            "Generating target pose list to check if the robot can reach that poses in the next step. This could take a minute if the node loads a previously generated partial result.");
    tf::Point position;

    // Calculate all target poses on the plane
    std::vector <geometry_msgs::Pose> target_poses; // List for all pose to go to from each initial point
    std::vector <geometry_msgs::Point> temp_positions; // Must be used because Pose has no ==
    for (double z = z_min; z <= z_max; z += resolution) {
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
    std::vector <geometry_msgs::Pose> initial_poses;
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
            ROS_INFO("Currently in queue: %lu steps: %zu targets and %d initials.", steps, target_poses.size(), 0);
        }
    }

    steps = target_poses.size() * initial_poses.size();
    ros::Time begin = ros::Time::now();
    ros::Duration elapsed = ros::Time::now() - begin;

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    visualization_msgs::Marker points;
    points.header.frame_id = base_link;
    points.header.stamp = ros::Time(0);
    points.ns = "points";
    points.action = visualization_msgs::Marker::ADD;
    points.id = 0;
    points.type = visualization_msgs::Marker::POINTS;
    double scale;
    node_handle.param<double>("scale", scale, 0);
    if (scale == 0) {
        points.scale.x = resolution / 5;
        points.scale.y = resolution / 5;
    } else {
        points.scale.x = scale;
        points.scale.y = scale;
    }

    simple_reachability::CLCOResult result;
    for (geometry_msgs::Pose initial_pose : initial_poses) {
        //TODO Variable die entscheidet ob einzelne beste Region (linear im cart. space) oder alle Regionen (mit verschmelzungen) berechnet werden sollen
        simple_reachability::CLCORegion r;
        int result_count = 0;
        r.initial_pose = initial_pose;
        r.count = 0;
        move_group.setPoseTarget(initial_pose);
        bool success = (move_group.plan(my_plan) ==
                        moveit::planning_interface::MoveItErrorCode::SUCCESS); // Plans a motion to a target_pose, Hint: Choose your IK Plugin in the moveit_config - IKFast could be really useful here
        if (success) {
            move_group.move();
            ROS_INFO("Reached initial pose: %f|%f|%f", initial_pose.position.x, initial_pose.position.y,
                     initial_pose.position.z);
        } else {
            ROS_ERROR("Can not go to initial pose: %f|%f|%f", initial_pose.position.x, initial_pose.position.y,
                      initial_pose.position.z);
            ROS_WARN("Skipping %ld steps.", target_poses.size());
            step_counter += target_poses.size();
            ROS_INFO_STREAM("Completed " << step_counter << " of " << steps);
            continue; //Continue with next initial pose
        }

        for (geometry_msgs::Pose target_pose : target_poses) {
            if (ros::ok()) {
                std::vector <geometry_msgs::Pose> waypoints = {initial_pose, target_pose};
                moveit_msgs::RobotTrajectory trajectory;
                const double jump_threshold = 2.0;
                const double eef_step = 0.01;
                double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
                ROS_ERROR_STREAM(fraction); //TODO ggf. Stellen, an denen Probleme auftreten schwarz markieren
                if (fraction == 1.0) { //TODO ggf. hier mit mehr Farben arbeiten
                    ROS_INFO("Plan found for Pose: %f %f %f", target_pose.position.x, target_pose.position.y,
                             target_pose.position.z);
                    result_count++;
                    r.count++;
                    r.reachable_poses.push_back(target_pose);

                } else {
                    ROS_WARN("No Plan found for Pose: %f %f %f", target_pose.position.x, target_pose.position.y,
                             target_pose.position.z);
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

        bool already_in = false;
        for (simple_reachability::CLCORegion region : result.regions) {
            if (std::find(region.reachable_poses.begin(), region.reachable_poses.end(), initial_pose) !=
                region.reachable_poses.end()) { // If initial pose not already in
                // Dann verschmelze die hier berechneten Ergebnisse mit den bestehenden der Region
                already_in = true;
                std::vector<geometry_msgs::Pose> combinedReachablePoses = region.reachable_poses;
                for (geometry_msgs::Pose p : r.reachable_poses) {
                    if (std::find(combinedReachablePoses.begin(), combinedReachablePoses.end(), p) == combinedReachablePoses.end()) combinedReachablePoses.push_back(p);
                }
                region.reachable_poses = combinedReachablePoses;
            }
        }
        if (!already_in) result.regions.push_back(r); //TODO nur einfügen, wenn initial pose noch keine reachable pose einer anderen initial_pose ist

    }

    std_msgs::ColorRGBA color;
    color.r = false;
    color.g = true;
    color.b = false;
    color.a =    1.0f;
    for (simple_reachability::CLCORegion clco : result.regions) {
        for (geometry_msgs::Pose p : clco.reachable_poses) {
            points.points.push_back(p.position);
            points.colors.push_back(color);
        }
        color = newColor(color);
    }

    rosbag::Bag bag;
    std::string filename = "clco.bag";
    std::string path = ros::package::getPath("simple_reachability");
    bag.open(path + "/bags/" + filename, rosbag::bagmode::Write); // Save bag in the bags folder of the package
    bag.write("/clco_results", ros::Time::now(), result);
    bag.close();

    filename = "clco_marker.bag";
    bag.open(path + "/bags/" + filename, rosbag::bagmode::Write); // Save bag in the bags folder of the package
    bag.write("/clco_results", ros::Time::now(), points);
    bag.close();

    ros::shutdown();
    return 0;
}

std_msgs::ColorRGBA newColor(std_msgs::ColorRGBA color) {
    std_msgs::ColorRGBA newColor;
    newColor.r = (double)rand()/RAND_MAX;
    newColor.g = (double)rand()/RAND_MAX;
    newColor.b = (double)rand()/RAND_MAX;
    return newColor;
}
