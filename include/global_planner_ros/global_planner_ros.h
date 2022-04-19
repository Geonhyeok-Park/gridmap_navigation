#ifndef GRIDMAP_navigation_GLOBAL_NAV_PLANNER_HPP
#define GRIDMAP_navigation_GLOBAL_NAV_PLANNER_HPP

#include <global_planner_ros/tf_manager.h>
#include <global_planner_ros/map_converter_ros.h>
#include <global_planner_ros/dijkstra_search_ros.h>
#include <global_planner_ros/elastic_bands_ros.h>

// grid map ROS
#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/GridMapRosConverter.hpp>

// ROS msgs
#include <nav_msgs/GetMap.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h>

// pcl
#include <pcl/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include <queue>

#include <chrono>
#define duration(a) std::chrono::duration_cast<std::chrono::microseconds>(a).count()
typedef std::chrono::high_resolution_clock clk;


class GlobalNavPlannerRos
{
    const float OCCUPIED = 100;
    const float FREE = 0;

private:

public:
    GlobalNavPlannerRos();

    virtual ~GlobalNavPlannerRos() { nh.shutdown(); }

    void loadParamServer();

    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);

    void laserCallback(const sensor_msgs::LaserScan::ConstPtr &msg);

    void updateSensorMap(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);

    void updateRobotPosition(const ros::Time &time);

    void updateGoalPosition(const geometry_msgs::Pose &goal_pose);

private:
    ros::NodeHandle nh;

    ros::Publisher pub_path;
    ros::Publisher pub_eband_path;
    ros::Publisher pub_gridmap;
    ros::Publisher pub_laser;
    ros::Publisher pub_localmap;
    ros::Publisher pub_bubble;

    ros::Subscriber sub_laser;
    ros::Subscriber sub_goal;

    std::string topic_path_pub;
    std::string topic_eband_path_pub;
    std::string topic_gridmap_pub;
    std::string topic_laser_pub;
    std::string topic_localmap_pub;
    std::string topic_laser_sub;
    std::string topic_bubble_pub;

    ros::ServiceClient map_client;
    nav_msgs::GetMap get_map;

    static void publishMap(const grid_map::GridMap &gridmap, const ros::Publisher &publisher);
    void publishSubmap(const grid_map::GridMap &gridmap, const ros::Publisher &publisher, const grid_map::Length &length);

private:
    grid_map::GridMap map_;
    grid_map::Position position_goal_;
    grid_map::Position position_robot_;

private:
    RosTFManager tf_;
    mapConverterRos map_converter_;
    laser_geometry::LaserProjection laser2pc_;
    DijkstraPathRosConverter path_converter_;
    ElasticBandsRosConverter eband_converter_;

    bool goal_received_;
    int inflation_size_;
};

#endif