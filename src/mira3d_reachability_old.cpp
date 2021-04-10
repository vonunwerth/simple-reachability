#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/kinematics_metrics/kinematics_metrics.h>

bool metrics(const robot_state::RobotState &state,
             const robot_model::JointModelGroup *joint_model_group,
             double &manipulability_index) {
// state.getJacobian() only works for chain groups.
    if (!joint_model_group->isChain()) {
        return false;
    }
    Eigen::MatrixXd jacobian = state.getJacobian(joint_model_group);
// Get joint limits penalty
    //TODO
    bool translation = false;
    double penalty = 0.0;//TODO getJointLimitsPenalty(state, joint_model_group);
    if (translation) {
        if (jacobian.cols() < 6) {
            Eigen::JacobiSVD<Eigen::MatrixXd> svdsolver(jacobian.topLeftCorner(3, jacobian.cols()));
            Eigen::MatrixXd singular_values = svdsolver.singularValues();
            manipulability_index = 1.0;
            for (
                    unsigned int i = 0;
                    i < singular_values.rows(); ++i) {
                //ROS_INFO_STREAM("moveit.kin_metrics: Singular value: %d %f", i,singular_values(i,0));
                manipulability_index *= singular_values(i, 0);
            }
// Get manipulability index
            manipulability_index = penalty * manipulability_index;
        } else {
            Eigen::MatrixXd jacobian_2 = jacobian.topLeftCorner(3, jacobian.cols());
            Eigen::MatrixXd matrix = jacobian_2 * jacobian_2.transpose();
// Get manipulability index
            manipulability_index = penalty * sqrt(matrix.determinant());
        }
    } else {
        if (jacobian.cols() < 6) {
            Eigen::JacobiSVD<Eigen::MatrixXd> svdsolver(jacobian);
            Eigen::MatrixXd singular_values = svdsolver.singularValues();
            manipulability_index = 1.0;
            for (
                    unsigned int i = 0;
                    i < singular_values.rows(); ++i) {
                //logDebug("moveit.kin_metrics: Singular value: %d %f", i,
                singular_values(i, 0);
                manipulability_index *=
                        singular_values(i, 0);
            }
// Get manipulability index
            manipulability_index = penalty * manipulability_index;
        } else {
            Eigen::MatrixXd matrix = jacobian * jacobian.transpose();
// Get manipulability index
            manipulability_index = penalty * sqrt(matrix.determinant());
        }
    }
    return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "move_group_interface_tutorial");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // BEGIN_TUTORIAL
    //
    // Setup
    // ^^^^^
    //
    // MoveIt operates on sets of joints called "planning groups" and stores them in an object called
    // the `JointModelGroup`. Throughout MoveIt the terms "planning group" and "joint model group"
    // are used interchangably.
    static const std::string PLANNING_GROUP = "manipulator";

    // The :move_group_interface:`MoveGroup` class can be easily
    // setup using just the name of the planning group you would like to control and plan for.
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

    // We will use the :planning_scene_interface:`PlanningSceneInterface`
    // class to add and remove collision objects in our "virtual world" scene
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // Raw pointers are frequently used to refer to the planning group for improved performance.
    const robot_state::JointModelGroup *joint_model_group =
            move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
    ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());
    robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
    moveit::core::RobotState robot_state(robot_model_loader.getModel());
    kinematic_state->setToDefaultValues();

    // Visualization
    // ^^^^^^^^^^^^^
    //
    // The package MoveItVisualTools provides many capabilties for visualizing objects, robots,
    // and trajectories in RViz as well as debugging tools such as step-by-step introspection of a script
    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("panda_link0");

    // Planning to a Pose goal
    // ^^^^^^^^^^^^^^^^^^^^^^^
    // We can plan a motion for this group to a desired pose for the
    // end-effector.
    geometry_msgs::Pose target_pose1;
    target_pose1.orientation.w = 1.0;
    target_pose1.position.x = 0.1;
    target_pose1.position.y = -0.6;
    target_pose1.position.z = 0;
    move_group.setPoseTarget(target_pose1);

    // Now, we call the planner to compute the plan and visualize it.
    // Note that we are just planning, not asking move_group
    // to actually move the robot.
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
        kinematic_state->getJacobian(joint_model_group,
                                     kinematic_state->getLinkModel(joint_model_group->getLinkModelNames().back()),
                                     reference_point_position, jacobian);
        ROS_INFO_STREAM("Jacobian: \n" << jacobian << "\n");

        kinematics_metrics::KinematicsMetrics km = kinematics_metrics::KinematicsMetrics(robot_model_loader.getModel());

        double penalty = km.getJointLimitsPenalty(robot_state, joint_model_group);
        double manipulability_index = 1.0;
        Eigen::JacobiSVD<Eigen::MatrixXd> svdsolver(jacobian.topLeftCorner(3, jacobian.cols()));
        Eigen::MatrixXd singular_values = svdsolver.singularValues();
        for (unsigned int i = 0; i < singular_values.rows(); ++i) {
            //logDebug("moveit.kin_metrics: Singular value: %d %f", i, singular_values(i, 0));
            manipulability_index *= singular_values(i, 0);
        }
        manipulability_index = penalty * manipulability_index;

//        double manipulability_index = 0.25;
        const std::string group = "manipulator";

        const robot_model::JointModelGroup *joint_model_group_2 = kinematic_model->getJointModelGroup(group);
        if (joint_model_group_2)
            ROS_INFO_STREAM("Joint model group found: " << "Found");
        else
            ROS_INFO_STREAM("Joint model group found: " << "Not Found");
//        bool b = metrics(robot_state, joint_model_group_2, manipulability_index);
//        bool b = km.getManipulabilityIndex(robot_state, joint_model_group_2, manipulability_index);
//        bool b = km.getManipulabilityIndex()
//        ROS_INFO_STREAM("Joint model group found: " << b);
        ROS_INFO_STREAM("Manipulability index: " << manipulability_index);
    }

    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

    ros::shutdown();
    return 0;
}
