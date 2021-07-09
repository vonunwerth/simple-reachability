#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

#include <utility>

class Region {
public:
    std::vector<geometry_msgs::Pose> reachable_poses;
    geometry_msgs::Pose initial_pose;
    int id;

    Region(int a, int b) {

    }

    Region(geometry_msgs::Pose i, std::vector<geometry_msgs::Pose> r, int d) {
        reachable_poses = std::move(r);
        initial_pose = i;
        id = d;
    }

    static bool equal_position(geometry_msgs::Point pos, geometry_msgs::Point item) {
        float res_t = 0.0001;
        bool result;
        result = (abs(pos.x - item.x) <= res_t) and (
                abs(pos.y - item.y) <= res_t) and (abs(
                pos.z - item.z) <= res_t);
        if (result) {
            int x = 4;
        }
        return result;
    }

    bool contains_any_of(const std::vector<geometry_msgs::Pose> &items) {
        for (geometry_msgs::Pose pose : this->reachable_poses) {
            for (geometry_msgs::Pose item : items) {
                if (this->equal_position(pose.position, item.position)) return true;
            }
        }
        return false;
    }

    void merge_region(const Region &region_2) {
        for (geometry_msgs::Pose pose : region_2.reachable_poses) {
            if (!this->contains(pose)) {
                this->reachable_poses.push_back(pose);
            }
        }
    }

    bool contains(geometry_msgs::Pose pose) {
        for (geometry_msgs::Pose r_pose : this->reachable_poses) {
            if (this->equal_position(pose.position, r_pose.position)) return true;
        }
        return false;
    }

    bool operator==(const Region r) const {
        return this->id == r.id;
    }

};
