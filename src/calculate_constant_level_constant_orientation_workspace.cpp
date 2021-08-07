
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
    std::string path = ros::package::getPath("simple_reachability");
    if (!complete) { // if computation is incomplete generate .partial workspace bag
        ROS_DEBUG_STREAM("Aborted at: \n" << points.points[points.points.size() - 1] << "\n Posecount: "
                                          << points.points.size());

        int index = 0;
        for (geometry_msgs::Point position : points.points) {
            ROS_DEBUG_STREAM("\nPosition: " << position << "\n" << "Color: " << points.colors[index]);
            index++;
        }
        filename.append(".partial");
        ROS_INFO_STREAM(
                "Partial Workspace (" << points.points.size() << " calculated poses) calculation will be written to: "
                                      << path << "/bags/" << filename);
    } else {
        ROS_INFO_STREAM("Workspace will be saved in: " << path << "/bags/" << filename);
        std::remove((path + "/bags/" + filename + ".partial").c_str()); // Remove the partial files after completion
    }
    visualization_msgs::Marker tmp_p = points;
    bag.open(path + "/bags/" + filename, rosbag::bagmode::Write); // Save bag in the bags folder of the package
    bag.write("/visualization_marker", ros::Time::now(),
              points);
    bag.close();
    ROS_INFO_STREAM(
            "Partial Workspace (" << points.points.size() << " calculated poses) calculation will be written to: "
                                  << path << "/bags/" << filename);

}

/**
 * Save progress if node is interrupted while calculating
 * @param sig Interrupt
 */
void shutdownNode(int sig) {
    ROS_INFO_STREAM("Shutting down node with signal " << sig << ". Will save current calculation.");
    saveROSBag(false);
    std::exit(25);
}

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
    node_handle.param<double>("resolution", resolution, 0.01);
    ROS_INFO_STREAM("Workspace resolution: " << resolution);

    // Load the file_name parameter from the configuration
    node_handle.param("file_name", filename,
                      ("constant_level_constant_orientation_" + planning_group + "_" + base_link + "_" + std::to_string(resolution) + ".bag"));
    std::string path = ros::package::getPath("simple_reachability");

    // Load the radius parameter from the configuration file
    double radius;
    node_handle.param("manipulator_reach", radius, 1.30);

    // Loads a specified detail region from the configuration file
    bool calculate_full = !(node_handle.hasParam("x_min") or node_handle.hasParam("x_max") or
                            node_handle.hasParam("y_min") or node_handle.hasParam("y_max") or
                            node_handle.hasParam("z_min") or node_handle.hasParam("z_max"));
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

    double initial_x, initial_y, initial_z;
    node_handle.param<double>("initial_x", initial_x, 0);
    node_handle.param<double>("initial_y", initial_y, 0);
    node_handle.param<double>("initial_z", initial_z, 0);

    if ((calculate_full and radius < resolution) or
        (std::abs(x_min - x_max) < resolution and std::abs(y_min - y_max) < resolution and std::abs(z_min - z_max) <
                                                                                           resolution)) { // Only if a complete calculation should be done check if resolution is set to a proper value
        ROS_ERROR_STREAM(
                "The volume of the given region is lower than the resolution. This will result in only a single marker in the origin. Check your resolution and manipulator_reach parameters.");
    }

    // Loads the complete_region parameter from the configuration
    bool complete_region; //TODO either complete_region or availability of all x-y-z ranges on the param server
    node_handle.param<bool>("complete_region", complete_region, false);

    // Load the end-effector orientation parameters from the configuration
    geometry_msgs::Pose pose;
    tf::Vector3 z_axis(0, 0, 1);
    tf::Vector3 x_axis(1, 0, 0);
    double orientation_tolerance;
    node_handle.getParam("ee_orientation_x", pose.orientation.x);
    node_handle.getParam("ee_orientation_y", pose.orientation.y);
    node_handle.getParam("ee_orientation_z", pose.orientation.z);
    node_handle.getParam("ee_orientation_w", pose.orientation.w);
//    pose.orientation.x = 0.707;
//    pose.orientation.y = 0;
//    pose.orientation.z = 0;
//    pose.orientation.w = -0.707;

    node_handle.param<double>("orientation_tolerance", orientation_tolerance, 0.01);
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
    //TODO move_group.setEndEffector() ?
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    const robot_model::RobotModelPtr &kinematic_model = robot_model_loader.getModel();
    ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());
    robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
    moveit::core::RobotState robot_state(robot_model_loader.getModel());
    kinematic_state->setToDefaultValues();

    // Structure to store all poses which need to be tested
    std::vector<geometry_msgs::Pose> target_poses;

    //Set reference frame for planning to the ur base link
    move_group.setPoseReferenceFrame(
            base_link);

    // Generate target poses, this also checks if a partial result already contains a pose
    ROS_INFO_STREAM(
            "Generating target pose list to check if the robot can reach that poses in the next step. This could take a minute if the node loads a previously generated partial result.");
    tf::Point position;
    for (double z = z_min; z <= z_max; z += resolution) {
        for (double x = x_min; x <= x_max; x += resolution) {
            for (double y = y_min; y <= y_max; y += resolution) {
                //pose.orientation.w = 0.707;
                //pose.orientation.x = -0.707;
                position.setX(x);
                position.setY(y);
                position.setZ(z);

                if (calculate_full) { // If a full workspace is to be calculated
                    for (int x_rot = 0; x_rot < 2; x_rot++) {
                        for (int z_rot = 1; z_rot <
                                            5; z_rot++) {
                            if (position.length() <=
                                radius) { // First check if a full calculation should be done, then check if position is in the possible sphere defined by the arms reach
                                bool already_calculated = false;
                                tf::pointTFToMsg(position, pose.position);
                                if (std::find(target_poses.begin(), target_poses.end(), pose) ==
                                    target_poses.end()) {// If the pose (especially a rotated one) is not already in target_poses)
                                    if (std::find(points.points.begin(), points.points.end(), pose.position) !=
                                        points.points.end()) { // If pose is loaded from a partial calculation
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
                } else { // Only a workspace region is to be calculated
                    tf::pointTFToMsg(position, pose.position);
                    if ((complete_region or position.length() <=
                                            radius) and
                        (std::find(points.points.begin(), points.points.end(), pose.position) ==
                         points.points.end())) { // Only if either all points should be calculated (complete_region) or they are in the sphere anyways, the pose should be added to the target_poses
                        target_poses.push_back(pose);
                    }
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

    geometry_msgs::Pose initial_pose;
    initial_pose.orientation = pose.orientation;
    initial_pose.position.x = initial_x;
    initial_pose.position.y = initial_y;
    initial_pose.position.z = initial_z;
    move_group.setPoseTarget(initial_pose);
    bool success = (move_group.plan(my_plan) ==
                    moveit::planning_interface::MoveItErrorCode::SUCCESS); // Plans a motion to a target_pose, Hint: Choose your IK Plugin in the moveit_config - IKFast could be really useful here
    if (success) {
        move_group.move();
        ROS_INFO_STREAM("Reached initial pose.");
    } else {
        ROS_ERROR_STREAM("Can not go to initial pose.");
        shutdownNode(25);
    }


    for (geometry_msgs::Pose target_pose : target_poses) {
        if (ros::ok()) {
            std::vector<geometry_msgs::Pose> waypoints = {initial_pose, target_pose};
            moveit_msgs::RobotTrajectory trajectory;
            const double jump_threshold = 2.0;
            const double eef_step = 0.01;
            double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
            ROS_ERROR_STREAM(fraction); //TODO ggf. Stellen, an denen Probleme auftreten schwarz markieren
            if (fraction == 1.0) { //TODO ggf. hier mit mehr Farben arbeiten
                ROS_INFO("Plan found for Pose: %f %f %f", target_pose.position.x, target_pose.position.y,
                         target_pose.position.z);
                points.colors.push_back(green);

            } else {
                ROS_WARN("No Plan found for Pose: %f %f %f", target_pose.position.x, target_pose.position.y,
                         target_pose.position.z);
                points.colors.push_back(red);
            }
            points.points.push_back(target_pose.position);
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