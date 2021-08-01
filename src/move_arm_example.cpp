#include "utils.cpp"
#include <moveit/kinematics_metrics/kinematics_metrics.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "calculate_constant_level_constant_orientation_workspace");
    ros::NodeHandle node_handle("~");
    //TODO goal ggf hier visualisieren
    ros::AsyncSpinner spinner(1);
    spinner.start();

    move_arm(node_handle, 0.2, -0.7, -0.35);
    while (ros::ok()) {
        move_arm_constrained(node_handle, 0.5, -0.7, -0.35);
        move_arm_constrained(node_handle, 0.5, -0.8, -0.35);
        move_arm_constrained(node_handle, 0.4, -0.8, -0.35);
        move_arm_constrained(node_handle, 0.4, -0.7, -0.35);
    }

    ros::shutdown();
    return 0;
}