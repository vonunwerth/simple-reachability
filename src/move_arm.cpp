
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/kinematics_metrics/kinematics_metrics.h>
#include <rosbag/bag.h>
#include <cstdio>

int main(int argc, char **argv) { //TODO move arm and move arm constrained as method
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

    //Set reference frame for planning to the ur base link
    move_group.setPoseReferenceFrame(
            base_link);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    move_group.setGoalOrientationTolerance(0.001); // Set an orienation tolerance

    geometry_msgs::Pose initial_pose;
    initial_pose.orientation.w = 0.707;
    initial_pose.orientation.x = -0.707;
    initial_pose.orientation.y = 0;
    initial_pose.orientation.z = 0;
    initial_pose.position.x = 0.4;//move_group.getCurrentPose().pose.position.x;
    initial_pose.position.y = -0.8;//move_group.getCurrentPose().pose.position.y;
    initial_pose.position.z = -0.35;//move_group.getCurrentPose().pose.position.z;
    move_group.setPoseTarget(initial_pose);

    std_msgs::ColorRGBA BLACK;
    BLACK.r = 200;
    BLACK.g = 200;
    BLACK.b = 200;
    BLACK.a = 1.0;
    visualization_msgs::Marker marker;
    marker.header.frame_id = "ur10_base_link";
    marker.header.stamp = ros::Time(0);
    marker.ns = "points";
    marker.action = visualization_msgs::Marker::ADD;
    marker.id = 0;
    marker.type = visualization_msgs::Marker::POINTS;

    double resolution;
    node_handle.param<double>("resolution", resolution, 0.01);
    ROS_INFO_STREAM("Workspace resolution: " << resolution);

    marker.scale.x = resolution;
    marker.scale.y = resolution;

    marker.points.push_back(initial_pose.position);
    marker.colors.push_back(BLACK);

    ros::Publisher marker_pub = node_handle.advertise<visualization_msgs::Marker>("visualization_marker", 1);
//    moveit_msgs::OrientationConstraint ocm;
//    ocm.link_name = "ur10_wrist_3_link";
//    ocm.header.frame_id = "ur10_base_link";
//    ocm.orientation.w = 0.707;
//    ocm.orientation.x = -0.707;
//    ocm.absolute_x_axis_tolerance = 2.0;
//    ocm.absolute_y_axis_tolerance = 2.0;
//    ocm.absolute_z_axis_tolerance = 2.0;
//    ocm.weight = 1.0;
//    moveit_msgs::Constraints test_constraints;
//    test_constraints.orientation_constraints.push_back(ocm);
//    move_group.setPathConstraints(test_constraints);

    geometry_msgs::Pose p2 = move_group.getCurrentPose().pose;
    geometry_msgs::Pose p = move_group.getCurrentPose("ur10_base_link").pose;

    ROS_INFO_STREAM("Moving to initial pose");
    bool success = (move_group.plan(my_plan) ==
                    moveit::planning_interface::MoveItErrorCode::SUCCESS); // Plans a motion to a target_pose, Hint: Choose your IK Plugin in the moveit_config - IKFast could be really useful here
    if (success)
        move_group.move();
    else
        ROS_FATAL_STREAM("cant move to initial pose");

    ROS_INFO_STREAM("Moved to initial pose");
    /*geometry_msgs::Pose start_pose = move_group.getCurrentPose().pose;
    //const robot_state::JointModelGroup *joint_model_group = start_state.getJointModelGroup(move_group.getName());
    //start_state.setFromIK(joint_model_group, start_pose);
    robot_state::RobotState start_state(*move_group.getCurrentState());
    move_group.setStartState(start_state);*/

    for (int i = 0; i < 6; i++) {
        ROS_INFO("Joint %d : %f", i, move_group.getCurrentJointValues()[i]);
    }

    move_group.stop();
    move_group.clearPoseTargets();

    ros::Rate r(25);

    while (ros::ok()) {
        marker_pub.publish(marker);
        //ROS_INFO("Publishing marker...");
        r.sleep();
    }

    ros::shutdown();
    return 0;
}