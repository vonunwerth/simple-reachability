#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include "ReachablePose.cpp"

class Region {
private:
    std::vector<geometry_msgs::Pose> reachables; //TODO losweden alle methoden auf ReachablePose Verendung umbauen
    const float RESOLUTION = 0.05; //TODO from rosparam

public:
    /**
     * vector of all reachable poses containing the initial pose
     */
    std::vector<ReachablePose> reachable_poses;
    geometry_msgs::Pose initial_pose;
    int id;

    Region() {

    }

    std::vector<geometry_msgs::Pose> calculate_neighbours(geometry_msgs::Pose pose) { //TODO Debuggen --> fast immer nur zwei Neighbour Posen aus irgendeinem Grund
        std::vector<geometry_msgs::Pose> neighbours;
        pose.position.x += this->RESOLUTION;
        if (this->contains(pose)) {
            neighbours.push_back(pose);
        }
        pose.position.x -= this->RESOLUTION; //TODO: Verändert Änderung von Pose auch Eintrag von vector?
        pose.position.y += this->RESOLUTION;
        if (this->contains(pose)) {
            neighbours.push_back(pose);
        }
        pose.position.y -= this->RESOLUTION;
        pose.position.x -= this->RESOLUTION;
        if (this->contains(pose)) {
            neighbours.push_back(pose);
        }
        pose.position.x += this->RESOLUTION;
        pose.position.y -= this->RESOLUTION;
        if (this->contains(pose)) {
            neighbours.push_back(pose);
        }
        ROS_INFO("count:%zu", neighbours.size());
        return neighbours;
    }

    Region(geometry_msgs::Pose i, const std::vector<geometry_msgs::Pose>& r, int d) {
        this->reachables = r;
        ReachablePose rp(calculate_neighbours(i), i);
        //this->initial_pose = rp;
        reachable_poses.push_back(rp);
        for (geometry_msgs::Pose p : r) {
            ReachablePose r_p(calculate_neighbours(p), p);
            reachable_poses.push_back(rp);
        }
        id = d;
    }

    static bool equal_position(geometry_msgs::Point pos, geometry_msgs::Point item) {
        float res_t = 0.01;
        return (abs(pos.x - item.x) <= res_t) and (
                abs(pos.y - item.y) <= res_t) and (abs(
                pos.z - item.z) <= res_t);
    }

    bool contains_any_of(const std::vector<geometry_msgs::Pose> &items) {
        for (geometry_msgs::Pose pose : this->reachables) {
            for (geometry_msgs::Pose item : items) {
                if (Region::equal_position(pose.position, item.position)) return true;
            }
        }
        return false;
    }

    void merge_region(const Region &region_2) {
        for (geometry_msgs::Pose pose : region_2.reachables) {
            if (!this->contains(pose)) {
                this->reachables.push_back(pose);
            }
        }
    }

    bool contains(geometry_msgs::Pose pose) {
        for (geometry_msgs::Pose r_pose : this->reachables) {
            if (Region::equal_position(pose.position, r_pose.position)) return true;
        }
        return false;
    }

    bool operator==(const Region& r) const {
        return this->id == r.id;
    }

};
