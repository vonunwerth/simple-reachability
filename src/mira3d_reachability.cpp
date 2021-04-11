#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/kinematics_metrics/kinematics_metrics.h>
#include <tf/tf.h>

bool inCircle(tf::Point pose, int radius);

int main(int argc, char **argv) {
    ros::init(argc, argv, "simple_reachability");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(4);
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

    //Set reference frame for planning to the ur base link
    move_group.setPoseReferenceFrame("ur10_base_link");
    //TODO calculate reach --> ggf ausprobieren, wie weit Roboter reichen kann in x,y,z Richtung
    // Generate poses to check TODO resolution as param
    double resolution = 0.3;

    int radius = 13;

    //TODO nur einmal rechnen dann 3mal um 90° gedreht hinzufügen
    ROS_INFO_STREAM("Sphere discretization");
    geometry_msgs::Pose pose;
    tf::Vector3 z_axis(0, 0, 1);
    tf::Vector3 x_axis(1, 0, 0);
    tf::Point position;
    for (int z = 0; z <= radius; z++) {
        for (int x = 0; x <= radius; x++) {
            for (int y = 0; y <= radius; y++) {
                pose.orientation.w = 1.0;
                position.setX(x / 10.0);
                position.setY(y / 10.0);
                position.setZ(z / 10.0);
                for (int x_rot = 0; x_rot < 4; x_rot++) {
                    position.rotate(z_axis, x_rot * 90);
                    if (inCircle(position, radius)) {
                        //TODO achsen am Anfang einzeln einfügen, sodass hier niccht auf doppelte geprüft werden muss
                        tf::pointTFToMsg(position, pose.position);
                        target_poses.push_back(pose);
                    }
                }
            }
        }
    }

    unsigned long steps = target_poses.size();

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    visualization_msgs::Marker points;

    std_msgs::ColorRGBA green;
    green.g = 1.0f;
    green.a = 1.0f;
    std_msgs::ColorRGBA red;
    red.a = 1.0f;
    red.r = 1.0f;

    unsigned long step_counter = 0;

    ros::Time begin = ros::Time::now();
    ros::Duration elapsed = ros::Time::now() - begin;

    for (geometry_msgs::Pose &target_pose : target_poses) {
        move_group.setPoseTarget(target_pose);
        bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

        geometry_msgs::Point target;
        target = target_pose.position;
        points.points.push_back(target);

        if (success) {
            points.colors.push_back(green); //Plan found
        } else {
            points.colors.push_back(red); //If no plan is found moveit will provide a warning in ROS_WARN
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

    //TODO speichern des erstellten workspaces in einem schicken format --> ggf rosbag? hdf5 wie reuleaux?
    points.header.frame_id = "ur10_base_link";
    points.header.stamp = ros::Time::now();
    points.ns = "points";
    points.action = visualization_msgs::Marker::ADD;
    points.id = 0;
    points.type = visualization_msgs::Marker::POINTS;
    points.scale.x = 0.02;
    points.scale.y = 0.02;

    ros::Publisher marker_pub = node_handle.advertise<visualization_msgs::Marker>("visualization_marker", 10);
    ros::Rate r(1);

    while (ros::ok()) {
        marker_pub.publish(points);
        r.sleep();
    }

    ros::shutdown();
    return 0;
}

bool inCircle(tf::Point p, int radius) {
    ROS_INFO_STREAM("Position: " << p << " : " << p.length());
    return p.length() <= radius / 10.0; // If the length is shorter than the radius the point is in the sphere
}
