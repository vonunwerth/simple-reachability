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

    std::vector<geometry_msgs::Pose> target_poses;
    geometry_msgs::Pose pose;

    //Set reference frame for planning to the ur base link
    move_group.setPoseReferenceFrame("ur10_base_link");
    //TODO calculate reach --> ggf ausprobieren, wie weit Roboter reichen kann in x,y,z Richtung
    // Generate poses to check TODO resolution as param
    double resolution = 0.3;
    int max_x = 3;
    int max_y = 3;
    int max_z = 3;
    int steps = (2 * max_x + 1) * (2 * max_y + 1) * (2 * max_z + 1);

    for (int x = -max_x; x <= max_x; x++) {
        for (int y = -max_y; y <= max_y; y++) {
            for (int z = -max_z; z <= max_z; z++) {
                pose.orientation.w = 1.0; //TODO Give orientation in launch file --> if none provided calculate for all orientations like reuleaux
                pose.position.x = x / 10.0; //TODO measure time between two plannings and calculate estimated end
                pose.position.y = y / 10.0;
                pose.position.z = z / 10.0;
                target_poses.push_back(pose);
            }
        }
    }

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    visualization_msgs::Marker points;
    points.header.frame_id = "ur10_base_link";
    points.header.stamp = ros::Time::now();
    points.ns = "points";
    points.action = visualization_msgs::Marker::ADD;
    points.id = 0;
    points.type = visualization_msgs::Marker::POINTS;
    points.scale.x = 0.02;
    points.scale.y = 0.02;

    std_msgs::ColorRGBA green;
    green.g = 1.0f;
    green.a = 1.0f;
    std_msgs::ColorRGBA red;
    red.a = 1.0f;
    red.r = 1.0f;

    int step_counter = 0;
    for (geometry_msgs::Pose &target_pose : target_poses) {
        move_group.setPoseTarget(target_pose);
        bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

        geometry_msgs::Point target;
        target = target_pose.position;
        points.points.push_back(target);

        if (success) {
            ROS_INFO_STREAM(
                    "Plan found for - x: " << target_pose.position.x << " y: " << target_pose.position.y << " z: "
                                           << target_pose.position.z);
            points.colors.push_back(green);
        } else {
            points.colors.push_back(red);
        }
        step_counter++;
        ROS_INFO_STREAM("Completed " << step_counter << " of " << steps);
    }

    ros::Publisher marker_pub = node_handle.advertise<visualization_msgs::Marker>("visualization_marker", 10);
    ros::Rate r(30);

    while (ros::ok()) {
        marker_pub.publish(points);
        r.sleep();
    }

    ros::shutdown();
    return 0;
}
