#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

/**
 * Represents a region
 */
class Region {
public:
    /**
     * vector of all reachable poses containing the initial pose
     */
    std::vector<geometry_msgs::Pose> reachable_poses;
    geometry_msgs::Pose initial_pose;
    int id;

    /**
     * Empty constructor
     */
    Region() {
        id = -1;
    }

    /**
     * Constructor for a new region
     * @param i pose
     * @param r pose list
     * @param d id
     */
    Region(geometry_msgs::Pose i, const std::vector<geometry_msgs::Pose>& r, int d) {
        reachable_poses = r;
        initial_pose = i;
        id = d;
    }

    /**
     * Does two points have the same position, numeric error safe
     * @param pos Point 1
     * @param item Point 2
     * @return Same Position?
     */
    static bool equal_position(geometry_msgs::Point pos, geometry_msgs::Point item) {
        float res_t = 0.01;
        return (abs(pos.x - item.x) <= res_t) and (
                abs(pos.y - item.y) <= res_t) and (abs(
                pos.z - item.z) <= res_t);
    }

    /**
     * Does the region contain any of the given elements
     * @param items Poses
     * @return Contains the elements?
     */
    bool contains_any_of(const std::vector<geometry_msgs::Pose> &items) {
        for (geometry_msgs::Pose pose : this->reachable_poses) {
            for (geometry_msgs::Pose item : items) {
                if (Region::equal_position(pose.position, item.position)) return true;
            }
        }
        return false;
    }

    /**
     * Merge regions
     * @param region_2 region 2
     */
    void merge_region(const Region &region_2) {
        for (geometry_msgs::Pose pose : region_2.reachable_poses) {
            if (!this->contains(pose)) {
                this->reachable_poses.push_back(pose);
            }
        }
    }

    /**
     * Does the region contain a pose
     * @param pose Pose
     * @return In Region?
     */
    bool contains(geometry_msgs::Pose pose) {
        for (geometry_msgs::Pose r_pose : this->reachable_poses) {
            if (Region::equal_position(pose.position, r_pose.position)) return true;
        }
        return false;
    }

    /**
     * Compares to regions by id
     * @param r id
     * @return same id?
     */
    bool operator==(const Region& r) const {
        return this->id == r.id;
    }

};
