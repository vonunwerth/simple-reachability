#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <utility>

class ReachablePose {
public:
    geometry_msgs::Pose pose;
    std::vector<geometry_msgs::Pose> neighbor_poses;
    float resolution = 0.05;

    ReachablePose() {

    }

    /**
     * Reachable Pose
     * @param neighbours List of neighbour poses
     * @param pose Pose
     */
    ReachablePose(std::vector<geometry_msgs::Pose> neighbours, geometry_msgs::Pose pose) {
        this->pose = pose;
        this->neighbor_poses = std::move(neighbours);
    }

    bool operator==(const ReachablePose& p) const {
        //return (Region::equal_position(this->pose.position, p.pose.position));
        return false;
    }
};
