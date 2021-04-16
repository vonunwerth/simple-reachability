#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/kinematics_metrics/kinematics_metrics.h>
#include <tf/tf.h>
#include <rosbag/bag.h>
#include <cmath>

int main(int argc, char **argv) {
    ros::init(argc, argv, "simple_reachability");
    ros::NodeHandle node_handle("~");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    std::string planning_group;
    if (node_handle.getParam("planning_group", planning_group)) ROS_INFO_STREAM("Planning group: " << planning_group);
    else {
        planning_group = "manipulator"; // Default value
        ROS_INFO_STREAM(
                "Param \"planning group\" not provided. simple_reachability will use default planning group \"manipulator\".");
    }

    double resolution; // Resolution in meter
    node_handle.param<double>("resolution", resolution, 0.1);
    ROS_INFO_STREAM("Workspace resolution: " << resolution);

    moveit::planning_interface::MoveGroupInterface move_group(planning_group);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    const robot_state::JointModelGroup *joint_model_group =
            move_group.getCurrentState()->getJointModelGroup(planning_group);

    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
    ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());
    robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
    moveit::core::RobotState robot_state(robot_model_loader.getModel());
    kinematic_state->setToDefaultValues();

    std::vector<geometry_msgs::Pose> target_poses;

    //Set reference frame for planning to the ur base link
    std::string base_link;
    node_handle.param<std::string>("manipulator_base_link", base_link, "base_link");
    move_group.setPoseReferenceFrame(base_link); //TODO check which link is the base link automaticcally move_group.getLinkNames()[0]??
    //TODO base_link manuell setzen ermöglicht sperren von joints hinter diesem link und anzeige von Workspace nur unter Nutzung der "vorderen" Gelenke (nach dem angegebenen Link)
    //TODO calculate reach --> ggf ausprobieren, wie weit Roboter reichen kann in x,y,z Richtung

    double radius = 0.2;
    //TODO param um custom bereiche auszuwählen, die untersucht werden sollen, nicht nur sphere --> von bestimmtem frame aus xmin-xmax, ... angeben möglich

    ROS_INFO_STREAM("Sphere discretization");
    geometry_msgs::Pose pose;
    tf::Vector3 z_axis(0, 0, 1);
    tf::Vector3 x_axis(1, 0, 0);
    tf::Point position;
    for (double z = 0; z <= radius; z += resolution) { //TODO Define minimum distance (perhaps mm) and bring to int?
        for (double x = 0; x <= radius; x += resolution) {
            for (double y = 0; y <= radius; y += resolution) {
                //pose.orientation.w = 0.707; //TODO Orientation as param or free
                //pose.orientation.x = -0.707;
                position.setX(x);
                position.setY(y);
                position.setZ(z);
                for (int x_rot = 0; x_rot < 2; x_rot++) { //TODO schleifen irgendwie hübscher
                    for (int z_rot = 1; z_rot < 5; z_rot++) {
                        if (position.length() <= radius) { //TODO einmal überprüfen reicht, gedrehte können ohne Prüfung hinzugefügt werden
                            //TODO achsen am Anfang einzeln einfügen, sodass hier niccht auf doppelte geprüft werden muss -> auf achsen ist x,y oder z == 0!
                            tf::pointTFToMsg(position, pose.position);
                            target_poses.push_back(pose);
                        }
                        position = position.rotate(z_axis, z_rot * M_PI_2);
                    }
                    position = position.rotate(x_axis, M_PI);
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
        bool success = (move_group.plan(my_plan) ==
                        moveit::planning_interface::MoveItErrorCode::SUCCESS); //TODO wie geht das ohne moveit? direkt über IK? IKFast?

        geometry_msgs::Point target;
        target = target_pose.position;
        points.points.push_back(target);

        if (success) {
            points.colors.push_back(green); //Plan found
//            move_group.move();
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

    std::string visualization_topic = "visualization_marker"; //TODo find out current node name
    ROS_INFO_STREAM("Workspace calculation completed. Will publish visualization on: " << visualization_topic);

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

    ros::Publisher marker_pub = node_handle.advertise<visualization_msgs::Marker>(visualization_topic, 1);
    int rate;
    node_handle.param<int>("publish_rate", rate, 1);
    ros::Rate r(rate);

    rosbag::Bag bag;
    std::string timecode = std::to_string(ros::Time::now().toSec());
    std::string path = ros::package::getPath("simple-reachability");
    std::string filename;
    node_handle.param("file_name", filename, (planning_group + "_" + base_link + std::to_string(resolution) + timecode + ".bag"));
    ROS_INFO_STREAM("Workspace written to: " << path << "/bags/" << filename);
    bag.open(path + "/bags/" + filename, rosbag::bagmode::Write);
    bag.write("/visualization_marker", ros::Time::now(),
              points);
    bag.close();
    while (ros::ok()) {
        marker_pub.publish(points);
        r.sleep();
    }

    ros::shutdown();
    return 0;
}