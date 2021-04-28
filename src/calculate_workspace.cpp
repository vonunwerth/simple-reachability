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

/**
 * Filename of the generate output file
 */
std::string filename;

/**
 * Calculation result data structure
 */
visualization_msgs::Marker points;

/**
 * Saves the workspace calculation result as a ROS bag
 * @param complete Calculation completed or just partially
 */
void saveROSBag(bool complete) {
    rosbag::Bag bag;
    std::string path = ros::package::getPath("simple-reachability");
    if (!complete) { // if computation is incomplete generate .partial workspace bag
        points.points.pop_back(); //Remove incompletely calculated last point which has been be added silently because moveit and the out stream lost its connection to the master
        points.colors.pop_back();
        filename.append(".partial");
        ROS_INFO_STREAM(
                "Partial Workspace (" << points.points.size() << " calculated poses) calculation will be written to: "
                                      << path << "/bags/" << filename);
    } else {
        ROS_INFO_STREAM("Workspace will be saved in: " << path << "/bags/" << filename);
        std::remove((path + "/bags/" + filename + ".partial").c_str()); // Remove the partial files after completion
    }
    bag.open(path + "/bags/" + filename, rosbag::bagmode::Write); // Save bag in the bags folder of the package
    bag.write("/visualization_marker", ros::Time::now(),
              points);
    bag.close();
}

/**
 * Save progress if node is interrupted while calculating
 * @param sig Interrupt
 */
void shutdownNode(int sig) {
    ROS_INFO_STREAM("Shutting down node with signal " << sig << ". Will save current calculation.");
    saveROSBag(false);
    //std::exit(25);
    ros::shutdown();
}

/**
 * Calculates the workspace of a robot
 * @param argc Not used
 * @param argv Not used
 * @return 0 on success
 */
int main(int argc, char **argv) {
    ros::init(argc, argv, "simple_reachability");
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
    node_handle.param<std::string>("manipulator_base_link", base_link, "base_link");

    // Load the resolution parameter from the configuration
    double resolution; // Resolution in meter
    node_handle.param<double>("resolution", resolution, 0.05);
    ROS_INFO_STREAM("Workspace resolution: " << resolution);

    // Load the file_name parameter from the configuration
    node_handle.param("file_name", filename,
                      (planning_group + "_" + base_link + "_" + std::to_string(resolution) + ".bag"));
    std::string path = ros::package::getPath("simple-reachability");

    // Load the end-effector orientation parameters from the configuration
    geometry_msgs::Pose pose;
    tf::Vector3 z_axis(0, 0, 1);
    tf::Vector3 x_axis(1, 0, 0);
    node_handle.getParam("ee_orientation_x", pose.orientation.x);
    node_handle.getParam("ee_orientation_y", pose.orientation.y);
    node_handle.getParam("ee_orientation_z", pose.orientation.z);
    node_handle.getParam("ee_orientation_w", pose.orientation.w);
    ROS_INFO_STREAM(
            "End-Effector Rotation set to (x,y,z,w): (" << pose.orientation.x << ", " << pose.orientation.y << ", "
                                                        << pose.orientation.z << ", " << pose.orientation.w << ")");

    // Generate the basic result of the calculation with default values
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

    // Steps of calculation and the current step
    unsigned long steps = 0;
    unsigned long step_counter = 0;

    //Look for a .partial result for the calculation to execute
    rosbag::Bag bag;
    ROS_INFO_STREAM("Try to find a partial workspace computation: " << filename << ".partial");
    try {
        bag.open(path + "/bags/" + filename + ".partial");  // BagMode is Read by default
        ROS_INFO_STREAM("Partial computation found. Will continue...");
        for (rosbag::MessageInstance const m: rosbag::View(bag)) {
            visualization_msgs::Marker::ConstPtr marker = m.instantiate<visualization_msgs::Marker>();
            if (marker != nullptr) { // Overwrite configuration with loaded config to do not mix something up
                points.header.frame_id = marker->header.frame_id;
                points.header.stamp = marker->header.stamp;
                points.ns = marker->ns;
                points.action = marker->action;
                points.id = marker->id;
                points.type = marker->type;
                points.scale.x = marker->scale.x;
                points.scale.y = points.scale.x;
                points.points = marker->points;
                points.colors = marker->colors;
                ROS_INFO_STREAM(
                        "Found " << points.points.size() << " calculated points in the partial calculated workspace.");
                steps += marker->points.size();
                step_counter += marker->points.size();
            }
        }
    } catch (rosbag::BagException &b) {
        ROS_INFO_STREAM("No partial computation found. Starting a new workspace calculation...");
    }

    // Allow to stop the calculation by CTRL + C, this will save the partial generated result
    signal(SIGINT, shutdownNode);

    ros::AsyncSpinner spinner(1);
    spinner.start();
    moveit::planning_interface::MoveGroupInterface move_group(planning_group);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    const robot_model::RobotModelPtr &kinematic_model = robot_model_loader.getModel();
    ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());
    robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
    moveit::core::RobotState robot_state(robot_model_loader.getModel());
    kinematic_state->setToDefaultValues();

    // Structure to store all poses which need to be tested
    std::vector <geometry_msgs::Pose> target_poses;

    //Set reference frame for planning to the ur base link
    move_group.setPoseReferenceFrame(
            base_link);

    double radius;
    node_handle.param("manipulator_reach", radius, 0.1);
    if (radius < resolution) {
        ROS_ERROR_STREAM(
                "The given sphere radius is lower than the resolution. This will result in only a single marker in the origin. Check your resolution and manipulator_reach parameters.");
    }


    // Generate target poses, this also checks if a partial result already contains a pose
    ROS_INFO_STREAM(
            "Generating target pose list to check if the robot can reach that poses in the next step. This could take a minute if the node loads a previously generated partial result.");
    tf::Point position;
    for (double z = 0; z <= radius; z += resolution) {
        for (double x = 0; x <= radius; x += resolution) {
            for (double y = 0; y <= radius; y += resolution) {
                //pose.orientation.w = 0.707;
                //pose.orientation.x = -0.707;
                position.setX(x);
                position.setY(y);
                position.setZ(z);
                for (int x_rot = 0; x_rot < 2; x_rot++) {
                    for (int z_rot = 1; z_rot < 5; z_rot++) {
                        if (position.length() <= radius) {
                            tf::pointTFToMsg(position, pose.position);
                            bool already_calculated = false;
                            if (std::find(target_poses.begin(), target_poses.end(), pose) == target_poses.end()) {// If the pose (especially a rotatetd one is not already in target_poses)
                                if (std::find(points.points.begin(), points.points.end(), pose.position) != points.points.end()) {
                                    ROS_DEBUG_STREAM("Found result for " << pose.position
                                                                         << " already in partial calculation result. Skipping.");
                                    already_calculated = true;
                                }
                            } else {
                                already_calculated = true;
                            }
                            if (!already_calculated) target_poses.push_back(pose);
                        }
                        position = position.rotate(z_axis, z_rot * M_PI_2);
                    }
                    position = position.rotate(x_axis, M_PI);
                }
            }
        }
    }
    steps += target_poses.size();

    // Marker for success (green) and failure (red)
    std_msgs::ColorRGBA green;
    green.g = 1.0f;
    green.a = 1.0f;
    std_msgs::ColorRGBA red;
    red.a = 1.0f;
    red.r = 1.0f;

    // Prepare remaining time calculation
    ros::Time begin = ros::Time::now();
    ros::Duration elapsed = ros::Time::now() - begin;

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    for (geometry_msgs::Pose &target_pose : target_poses) { // For all target poses do ...
        if (ros::ok()) { // Check if shutdown is requested, otherwise connection to move it and out stream would be lost and all markers will generated with "unsuccessful" state
            move_group.setPoseTarget(target_pose);
            bool success = (move_group.plan(my_plan) ==
                            moveit::planning_interface::MoveItErrorCode::SUCCESS); // Plans a motion to a target_pose, Hint: Choose your IK Plugin in the moveit_config - IKFast could be really useful here

            geometry_msgs::Point target;
            target = target_pose.position;

            points.points.push_back(target); // Pushed pose to the output data structure (marker list)
            if (success) {
                points.colors.push_back(green); //Plan found
                //            move_group.move();
            } else {
                points.colors.push_back(red); //If no plan is found MoveIt! will provide a warning in ROS_WARN
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
    saveROSBag(true); // Save complete result and delete .partial
    ros::shutdown();
    return 0;
}