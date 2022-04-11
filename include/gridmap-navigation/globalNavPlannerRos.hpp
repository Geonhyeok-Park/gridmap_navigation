#ifndef GRIDMAP_navigation_GLOBAL_NAV_PLANNER_HPP
#define GRIDMAP_navigation_GLOBAL_NAV_PLANNER_HPP

// grid map ROS
#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/GridMapRosConverter.hpp>

// TF manager
#include "gridmap-navigation/TFManager.hpp"

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

float getDist(const Position &pos1, const Position &pos2)
{
    return (float)sqrt(pow(pos1.x() - pos2.x(), 2) + pow(pos1.y() - pos2.y(), 2));
}

class GlobalNavPlannerRos
{
    const int OCCUPIED = 100;
    const int FREE = 0;
    const int UNKNWON = -10;

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

    bool updateIntrinsicCost(double &maxCost);

    bool findGradientPath(const std::string &layer, std::vector<Position> &poseList);

    void laserCallback(const sensor_msgs::LaserScan::ConstPtr &msg);

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
    std::string topicGoalSub;

    std::string topicBubblePub;

    ros::ServiceClient mapClient;
    nav_msgs::GetMap getMap;

    void publishMap(const GridMap &gridmap, ros::Publisher publisher);

    void toRosMsg(std::vector<Position> &poseList, nav_msgs::Path &msg) const;

private:
    GridMap gridMap_;

private:
    RosTFManager tf_;
    laser_geometry::LaserProjection laser2pc_;

    nav_msgs::OccupancyGrid occupancyMap_;
    int inflationSize_;
    Position goalPosition_;
    Position robotPosition_;
    bool goalReceived_;

    float goalBoundary_;
    double maxIntrinsicCost_;
};

#endif