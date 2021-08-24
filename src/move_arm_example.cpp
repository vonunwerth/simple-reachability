#include "utils.cpp"
#include <moveit/kinematics_metrics/kinematics_metrics.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "calculate_constant_level_constant_orientation_workspace");
    ros::NodeHandle node_handle("~");

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

    ros::AsyncSpinner spinner(1);
    spinner.start();

    move_arm(node_handle, move_group, 0.4, -0.8, -0.35);
    while (ros::ok()) {
        move_arm_constrained(node_handle, move_group, 0.7, -0.5, -0.35);
        move_arm_constrained(node_handle, move_group, -0.4, -0.6, -0.35);
        move_arm_constrained(node_handle, move_group, -0.5, -0.8, -0.35);
        move_arm_constrained(node_handle, move_group, 0.4, -0.8, -0.35);
    }

    ros::shutdown();
    return 0;
}