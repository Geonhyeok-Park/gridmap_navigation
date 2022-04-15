#ifndef GRIDMAP_navigation_GLOBAL_NAV_PLANNER_HPP
#define GRIDMAP_navigation_GLOBAL_NAV_PLANNER_HPP

#include "global_planner_ros/TFManager.hpp"

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

    void scaleLayerValue(const std::string &layer);
    void gridInflation(GridMap &gridMap, const std::string &layerIn, int inflateState, const std::string &layerOut) const;

    void getTraversableSearchspace();

    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);

    void laserCallback(const sensor_msgs::LaserScan::ConstPtr &msg);

    void updateSensorMap(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);

private:
    ros::NodeHandle nh;

    ros::Publisher pubPath;
    ros::Publisher pubEbandPath;
    ros::Publisher pubGridmap;
    ros::Publisher pubLaser;
    ros::Publisher pubLocalmap;

    ros::Publisher pubBubble;
    ros::Subscriber subLaser;
    ros::Subscriber subGoal;

    std::string topicPathPub;
    std::string topicEbandPathPub;
    std::string topicGridMapPub;
    std::string topicLaserPub;
    std::string topicLocalMapPub;
    std::string topicLaserSub;
    std::string topicBubblePub;

    ros::ServiceClient mapClient;
    nav_msgs::GetMap getMap;

    static void publishMap(const GridMap &gridmap, const ros::Publisher &publisher);
    void toRosMsg(std::vector<Position> &poseList, nav_msgs::Path &msg) const;

private:
    nav_msgs::OccupancyGrid occupancyMap_;
    GridMap gridMap_;
    Position goalPosition_;
    Position robotPosition_;
    std::vector<Position> globalPath_;

private:
    RosTFManager tf_;
    laser_geometry::LaserProjection laser2pc_;

    bool goalReceived_;
    float maxIntrinsicCost_;
    int inflationSize_;
};

#endif