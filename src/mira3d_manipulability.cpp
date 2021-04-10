#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/kinematics_metrics/kinematics_metrics.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "move_group_interface_tutorial");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    static const std::string PLANNING_GROUP = "manipulator";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    const robot_state::JointModelGroup *joint_model_group =
            move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
    ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());
    robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
    moveit::core::RobotState robot_state(robot_model_loader.getModel());
    kinematic_state->setToDefaultValues();

    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("panda_link0");

    geometry_msgs::Pose target_pose1;
    target_pose1.orientation.w = 1.0;
    target_pose1.position.x = 0.1;
    target_pose1.position.y = -0.6;
    target_pose1.position.z = 0;
    move_group.setPoseTarget(target_pose1);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    move_group.move();

    ros::Publisher marker_pub = node_handle.advertise<visualization_msgs::Marker>("visualization_marker", 10);
    ros::Rate r(30);

    visualization_msgs::Marker points;
    points.header.frame_id = "odom";
    points.header.stamp = ros::Time::now();
    points.ns = "points";
    points.action = visualization_msgs::Marker::ADD;
    points.id = 0;
    points.type = visualization_msgs::Marker::POINTS;
    points.scale.x = 0.02;
    points.scale.y = 0.02;
    points.color.g = 1.0f;
    points.color.a = 1.0f;
    geometry_msgs::Point p;
    p.x = 0.1;
    p.y = -0.6;
    p.z = 0.3;
    points.points.push_back(p);


    while (ros::ok()) {
        marker_pub.publish(points);
        r.sleep();

        Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0);

        Eigen::MatrixXd jacobian;
//        kinematic_state->getJointPositions()
        kinematic_state->getJacobian(joint_model_group,
                                     kinematic_state->getLinkModel(joint_model_group->getLinkModelNames().back()),
                                     reference_point_position, jacobian);
        ROS_INFO_STREAM("Jacobian: \n" << jacobian << "\n");


        double manipulability_index = 1.0;

        jacobian *= jacobian.transpose();
        double determinant = jacobian.determinant();
        manipulability_index = sqrt(determinant);
        ROS_INFO_STREAM("Jacobian times Transposed: \n" << jacobian << "\n");
        ROS_INFO_STREAM("Determinant: " << determinant);
        ROS_INFO_STREAM("Manipulability index: " << manipulability_index);
    }

    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

    ros::shutdown();
    return 0;
}
