#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <utility>

class Region {
public:
    std::vector<geometry_msgs::Pose> reachable_poses;
    geometry_msgs::Pose initial_pose;
    int id;

    /**
     *
     * @param i Initial pose
     * @param r  Reachable poses
     * @param d id
     */
    Region(geometry_msgs::Pose i, std::vector<geometry_msgs::Pose> r, int d);

    bool equal_position(geometry_msgs::Point pos, geometry_msgs::Point item);

    bool contains_any_of(const std::vector<geometry_msgs::Pose> &items);

    void merge_region(Region region_2);

    bool contains(geometry_msgs::Pose pose);

};
