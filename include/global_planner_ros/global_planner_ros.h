#ifndef GRIDMAP_navigation_GLOBAL_NAV_PLANNER_HPP
#define GRIDMAP_navigation_GLOBAL_NAV_PLANNER_HPP

#include <global_planner_ros/tf_manager.h>
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

using namespace grid_map;

class GlobalNavPlannerRos
{
    const float OCCUPIED = 100;
    const float FREE = 0;
    const float UNKNWON = -10;

    // value in occupancy grid map
    const int OCC_MAP = 100;
    const int FREE_MAP = 0;

private:
    bool initialize();

public:
    GlobalNavPlannerRos();

    virtual ~GlobalNavPlannerRos() { nh.shutdown(); }

    void loadParamServer();

    void waitOccupancyMap();

    void allocateMapMemory();

    void resetGridMapMemory();
    void resetLocalMapMemory();

    void gridInflation(GridMap &gridmap, const std::string &layer_in, int inflate_state, const std::string &layer_out) const;

    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);

    void laserCallback(const sensor_msgs::LaserScan::ConstPtr &msg);

    void updateSensorMap(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);

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

    static void publishMap(const GridMap &gridmap, const ros::Publisher &publisher);

private:
    nav_msgs::OccupancyGrid occupancymap_;
    GridMap gridmap_;
    Position position_goal_;
    Position position_robot_;
    std::vector<Position> global_path_;

private:
    RosTFManager tf_;
    laser_geometry::LaserProjection laser2pc_;
    DijkstraPathRosConverter path_converter_;
    ElasticBandsRosConverter eband_converter_;

    bool goal_received_;
    float max_intrinsic_cost_;
    int inflation_size_;
};

#endif