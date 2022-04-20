#include <gridmap_navigation/global_planner_ros.h>

using namespace grid_map;

void GlobalPlannerRos::updateSensorMap(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{
    updateSensorMap(cloud, OCCUPIED);
}

void GlobalPlannerRos::updateSensorMap(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, double cost)
{
    map_["obstacle_laser_raw"].setZero();
    map_["obstacle_laser"].setZero();

    auto &laser_raw = map_["obstacle_laser_raw"];
    auto &laser = map_["obstacle_laser"];
    for (const auto &p : cloud->points)
    {
        Position point(p.x, p.y);
        Index point_index;
        if (!map_.getIndex(point, point_index))
            continue;

        laser_raw(point_index(0), point_index(1)) = cost;

        Index search_size(inflation_size_, inflation_size_);
        Index search_region(2 * inflation_size_ + 1, 2 * inflation_size_ + 1);
        SubmapIterator search_iter(map_, point_index - search_size, search_region);
        for (search_iter; !search_iter.isPastEnd(); ++search_iter)
        {
            Index search_index = *search_iter;
            laser(search_index(0), search_index(1)) = cost;
        }
    }
}

void GlobalPlannerRos::updateRobotPosition(const ros::Time &time)
{
    tf_.getTF(time);
    position_robot_(0) = tf_.BaseToMap.translation.x;
    position_robot_(1) = tf_.BaseToMap.translation.y;
}

void GlobalPlannerRos::updateGoalPosition(const geometry_msgs::Pose &goal_pose)
{
    position_goal_(0) = goal_pose.position.x;
    position_goal_(1) = goal_pose.position.y;
}

void GlobalPlannerRos::publishMap(const GridMap &gridmap, const ros::Publisher &publisher)
{
    if (publisher.getNumSubscribers() < 1)
        return;

    grid_map_msgs::GridMap msg;
    GridMapRosConverter::toMessage(gridmap, msg);
    publisher.publish(msg);
}

void GlobalPlannerRos::publishSubmap(const grid_map::GridMap &gridmap, const ros::Publisher &publisher, const grid_map::Length &length)
{
    bool get_submap;
    auto submap = gridmap.getSubmap(position_robot_, length, get_submap);
    if (!get_submap)
        ROS_WARN("Could not get submap to publish.");
    else
    {
        publishMap(submap, publisher);
    }
}