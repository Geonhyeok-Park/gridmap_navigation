#include <global_planner_ros/dijkstraSearch.hpp>

#include <ros/ros.h>
#include <nav_msgs/Path.h>

class DijkstraPathRosConverter
{
private:
public:
    DijkstraPathRosConverter() = default;
    ~DijkstraPathRosConverter() = default;

    void toRosMsg(const std::vector<Position> &data, nav_msgs::Path &msg)
    {
        msg.header.frame_id = "map";
        msg.header.stamp = ros::Time::now();

        for (const auto &pose : data)
        {
            geometry_msgs::PoseStamped poseMsg;
            poseMsg.pose.position.x = pose.x();
            poseMsg.pose.position.y = pose.y();
            msg.poses.push_back(poseMsg);
        }
    }
};