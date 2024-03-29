#include <gridmap_navigation/dijkstra_search.h>

#include <ros/ros.h>
#include <nav_msgs/Path.h>

class DijkstraPathRosConverter
{
private:
public:
    DijkstraPathRosConverter() = default;
    ~DijkstraPathRosConverter() = default;

    void toRosMsg(const std::vector<grid_map::Position> &data, nav_msgs::Path &msg)
    {
        msg.poses.clear();
        
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