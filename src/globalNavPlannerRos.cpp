#include "globalNavPlannerRos.hpp"

#ifndef GRIDMAP_NAVIATION_GLOBAL_NAV_PLANNER_HPP
#define GRIDMAP_NAVIATION_GLOBAL_NAV_PLANNER_HPP

// grid map ROS
#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/GridMapRosConverter.hpp>

// TF manager
#include "gridmap-naviation/TFManager.hpp"

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

#define duration(a) std::chrono::duration_cast<std::chrono::milliseconds>(a).count()
typedef std::chrono::high_resolution_clock clk;

using namespace grid_map;

float getDist(const Position &pos1, const Position &pos2)
{
    return (float)sqrt(pow(pos1.x() - pos2.x(), 2) + pow(pos1.y() - pos2.y(), 2));
}

class globalNavPlannerRos
{
    const int OCCUPIED = 100;
    const int FREE = 0;

private:
    bool initialize();

public:
    globalNavPlannerRos();

    virtual ~globalNavPlannerRos() { nh.shutdown(); }

    void loadParamServer();

    void waitOccupancyMap();

    void allocateMapMemory();
    void allocateGlobalMap();
    void allocateLocalMap();

    void resetGridMapMemory();
    void resetLocalMapMemory();

    void gridInflation(GridMap &gridMap, const std::string &layerIn, float inflateVal, const std::string &layerOut) const;

    void getTraversableGrid();

    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);

    bool updateIntrinsicCost(double &maxCost);

    bool findGradientPath(std::vector<Position> &poseList);

    void laserCallback(const sensor_msgs::LaserScan::ConstPtr &msg);

private:
    ros::NodeHandle nh;

    ros::Publisher pubPath;
    ros::Publisher pubGridmap;
    ros::Publisher pubLaser;
    ros::Publisher pubLocalmap;
    ros::Subscriber subLaser;
    ros::Subscriber subGoal;

    std::string topicPathPub;
    std::string topicGridMapPub;
    std::string topicLaserPub;
    std::string topicLocalMapPub;
    std::string topicLaserSub;
    std::string topicGoalSub;

    ros::ServiceClient mapClient;
    nav_msgs::GetMap getMap;

    void publishMap(const GridMap &gridmap, ros::Publisher publisher);

    void toRosMsg(std::vector<Position> &poseList, nav_msgs::Path &msg) const;

private:
    GridMap gridMap_;
    GridMap localMap_;

private:
    RosTFManager tf_;
    laser_geometry::LaserProjection laser2pc_;

    nav_msgs::OccupancyGrid occupancyMap_;
    int inflationSize_;
    Position goalPosition_;
    Position robotPosition_;

    float goalBoundary_;
    double maxIntrinsicCost_;
};

#endif

globalNavPlannerRos::globalNavPlannerRos()
    : nh("global_nav_planner")
{
    // 1. load ROS params
    loadParamServer();

    // 2. check ROS static tf_ has been constructed
    while (!tf_.getStaticTF("tim581_front"))
        ros::Duration(1.0).sleep();

    // 3. allocate memories
    waitOccupancyMap();
    allocateMapMemory();

    // 4. ROS subscriber, publisher
    subGoal = nh.subscribe("/move_base_simple/goal", 10, &globalNavPlannerRos::goalCallback, this);
    subLaser = nh.subscribe(topicLaserSub, 10, &globalNavPlannerRos::laserCallback, this);
    pubGridmap = nh.advertise<grid_map_msgs::GridMap>(topicGridMapPub, 1);
    pubPath = nh.advertise<nav_msgs::Path>(topicPathPub, 10);
    pubLaser = nh.advertise<sensor_msgs::PointCloud2>(topicLaserPub, 1);
    pubLocalmap = nh.advertise<grid_map_msgs::GridMap>(topicLocalMapPub, 1);

    // 5. initialize map values
    initialize();
}

void globalNavPlannerRos::waitOccupancyMap()
{

    mapClient = nh.serviceClient<nav_msgs::GetMap>("/static_map");
    while (!mapClient.call(getMap))
    {
        ROS_ERROR("No response for calling static map. Wait until map is valid.");
        ros::Duration(1.0).sleep();
    }
    ROS_INFO("Static map recieved. Start allocating Grid map memory");
    occupancyMap_ = getMap.response.map;
}

void globalNavPlannerRos::allocateMapMemory()
{
    allocateGlobalMap();
    allocateLocalMap();
}

void globalNavPlannerRos::allocateGlobalMap()
{
    // static map
    gridMap_.add("map_raw");
    gridMap_.add("map_inflated");
    gridMap_.add("traversable_global");

    // cost map in traversable region
    gridMap_.add("intrinsicCost");
    gridMap_.add("totalCost");
}

void globalNavPlannerRos::allocateLocalMap()
{
    localMap_.add("laser_raw");
    localMap_.add("laser_inflated");
}

void globalNavPlannerRos::resetGridMapMemory()
{
    gridMap_.clear("intrinsicCost");
}

void globalNavPlannerRos::resetLocalMapMemory()
{
    localMap_.clearAll();
}

bool globalNavPlannerRos::initialize()
{

    // global map
    bool converted = GridMapRosConverter::fromOccupancyGrid(occupancyMap_, "map_raw", gridMap_);
    if (!converted)
    {
        ROS_ERROR("Conversion from occupancy map to gridmap failed.");
        return false;
    }

    gridMap_["map_inflated"] = gridMap_["map_raw"];
    gridInflation(gridMap_, "map_raw", OCCUPIED, "map_inflated");
    getTraversableGrid();

    // local map
    Length mapSize(20, 20);
    localMap_.setFrameId("map");
    localMap_.setGeometry(mapSize, gridMap_.getResolution());
    localMap_["laser_raw"].setConstant(FREE);
    localMap_["laser_inflated"].setConstant(FREE);

    return true;
}

// TODO needs to be fixed for general usage
void globalNavPlannerRos::gridInflation(GridMap &gridmap, const std::string &layerIn, float inflateState, const std::string &layerOut) const
{
    for (GridMapIterator iter(gridmap); !iter.isPastEnd(); ++iter)
    {
        const auto &cellIndex = *iter;
        const auto &cellState = gridmap.at(layerIn, cellIndex);

        if (!std::isfinite(cellState))
            continue;

        if (cellState != inflateState)
            continue;

        Position cellPosition;
        gridmap.getPosition(cellIndex, cellPosition);

        Index inflation(inflationSize_, inflationSize_);
        Index bufferSize(2 * inflationSize_ + 1, 2 * inflationSize_ + 1);
        SubmapIterator subIter(gridmap, cellIndex - inflation, bufferSize);
        for (subIter; !subIter.isPastEnd(); ++subIter)
        {
            gridmap.at(layerOut, *subIter) = cellState;
        }
    }
}

void globalNavPlannerRos::getTraversableGrid()
{
    for (GridMapIterator iter(gridMap_); !iter.isPastEnd(); ++iter)
    {
        const auto &cellIndex = *iter;
        const auto &cellState = gridMap_.at("map_inflated", cellIndex);

        if (!std::isfinite(cellState))
            continue;

        if (cellState != OCCUPIED)
        {
            gridMap_.at("traversable_global", cellIndex) = FREE;
        }
    }
}

void globalNavPlannerRos::goalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    publishMap(gridMap_, pubGridmap);

    resetGridMapMemory();

    goalPosition_(0) = msg->pose.position.x;
    goalPosition_(1) = msg->pose.position.y;

    tf_.getTF(msg->header.stamp);
    robotPosition_(0) = tf_.BaseToMap.translation.x;
    robotPosition_(1) = tf_.BaseToMap.translation.y;

    clk::time_point t1 = clk::now();
    if (!updateIntrinsicCost(maxIntrinsicCost_))
    {
        ROS_ERROR("update IntrinsicCost failed.");
        return;
    }
    clk::time_point t2 = clk::now();
    std::cout << "Duration update Intrinsic Cost map: " << duration(t2 - t1) << "ms" << std::endl;

    gridMap_["totalCost"] = gridMap_["intrinsicCost"];

    t1 = clk::now();
    std::vector<Position> poseList;
    if (!findGradientPath(poseList))
    {
        ROS_ERROR("Finding Path failed. clear path");
        return;
    }
    t2 = clk::now();
    std::cout << "Duration finding lowest cost path : " << duration(t2 - t1) << "ms" << std::endl;

    // publish msgs
    nav_msgs::Path pathMsg;
    toRosMsg(poseList, pathMsg);
    pubPath.publish(pathMsg);
    publishMap(gridMap_, pubGridmap);
}

bool globalNavPlannerRos::updateIntrinsicCost(double &maxCost)
{

    // lock goal position, robot position
    const auto &goalPosition = goalPosition_;
    const auto &robotPosition = robotPosition_;

    const auto &intrinsicCostMap = gridMap_["intrinsicCost"];

    if (!gridMap_.isInside(goalPosition))
    {
        ROS_WARN("Goal is out of map region");
        return false;
    }

    Index goalIndex;
    gridMap_.getIndex(goalPosition, goalIndex);

    Index robotIndex;
    gridMap_.getIndex(robotPosition, robotIndex);

    std::queue<Index> gridCandidateList;
    gridCandidateList.push(goalIndex);

    // Set goal grid as start point of waveFront algorithm
    gridMap_.atPosition("intrinsicCost", goalPosition) = 1.0;
    maxCost = 1.0;

    while (!gridCandidateList.empty())
    {

        const auto waveFrontIndex = gridCandidateList.front();
        Position waveFrontPosition;
        gridMap_.getPosition(waveFrontIndex, waveFrontPosition);

        gridCandidateList.pop();

        const auto searchSize = 1.0;
        Index startIndex(searchSize, searchSize);
        Index windowSize(2 * searchSize + 1, 2 * searchSize + 1);
        SubmapIterator subIter(gridMap_, waveFrontIndex - startIndex, windowSize);
        for (subIter; !subIter.isPastEnd(); ++subIter)
        {

            const auto &searchIndex = *subIter;
            Position searchPosition;
            gridMap_.getPosition(searchIndex, searchPosition);

            if (searchIndex.isApprox(waveFrontIndex))
                continue;

            // only search in traversable space
            if (!std::isfinite(gridMap_.atPosition("traversable_global", searchPosition)))
                continue;

            double margin = 3.0; // check margin is needed or not
            const auto midPoint = (goalPosition + robotPosition) / 2;
            if (getDist(midPoint, searchPosition) > getDist(goalPosition, midPoint) + margin)
                continue;

            auto &searchCost = gridMap_.at("intrinsicCost", searchIndex);
            // for performance, use direct access rather than map.at()
            const auto &waveFrontCost = intrinsicCostMap(waveFrontIndex(0), waveFrontIndex(1));
            const auto transitionCost = getDist(waveFrontPosition, searchPosition);

            if (!std::isfinite(searchCost))
            {
                searchCost = waveFrontCost + transitionCost;
                gridCandidateList.push(searchIndex);
                if (searchCost > maxCost)
                    maxCost = searchCost;
            }
            else if (searchCost > waveFrontCost + transitionCost)
            {
                searchCost = waveFrontCost + transitionCost;
                if (searchCost > maxCost)
                    maxCost = searchCost;
            }
        }
    }

    if (!std::isfinite(gridMap_.atPosition("intrinsicCost", robotPosition)))
    {
        ROS_WARN("Goal is out of traversable region");
        return false;
    }

    return true;
}

bool globalNavPlannerRos::findGradientPath(std::vector<Position> &poseList)
{

    // lock goal position, robot position
    const auto &robotPosition = robotPosition_;
    const auto &goalPosition = goalPosition_;

    Index robotIndex, goalIndex;
    gridMap_.getIndex(robotPosition, robotIndex);
    gridMap_.getIndex(goalPosition, goalIndex);

    std::queue<Index> pathCandidateList;
    pathCandidateList.push(robotIndex);

    while (!pathCandidateList.empty())
    {

        const auto waveFrontIndex = pathCandidateList.front();
        Position waveFrontPosition;
        gridMap_.getPosition(waveFrontIndex, waveFrontPosition);

        // reached the goal, then stop propagating
        if (waveFrontIndex.isApprox(goalIndex))
            return true;

        Index minIndex = waveFrontIndex;
        double minCost = gridMap_.at("totalCost", waveFrontIndex);

        Index inflation(inflationSize_, inflationSize_);
        Index bufferSize(2 * inflationSize_ + 1, 2 * inflationSize_ + 1);
        SubmapIterator subIter(gridMap_, waveFrontIndex - inflation, bufferSize);
        for (subIter; !subIter.isPastEnd(); ++subIter)
        {

            const auto &searchIndex = *subIter;
            Position searchPosition;

            if (!std::isfinite(gridMap_.at("totalCost", searchIndex)))
                continue;

            if (searchIndex.isApprox(waveFrontIndex))
                continue;

            const auto &searchCost = gridMap_.at("totalCost", searchIndex);
            const auto &waveFrontCost = gridMap_.at("totalCost", waveFrontIndex);
            if (searchCost < minCost)
            {
                minCost = searchCost;
                minIndex = searchIndex;
            }

        } // end of near grid search loop -- finding minimum cost cell

        Position nextPosition;
        gridMap_.getPosition(minIndex, nextPosition);
        poseList.push_back(nextPosition);
        pathCandidateList.push(minIndex);
        pathCandidateList.pop();
    }
}

void globalNavPlannerRos::laserCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    // laserScan type conversion
    sensor_msgs::PointCloud2 pcWithDist;
    laser2pc_.projectLaser(*msg, pcWithDist, -1, laser_geometry::channel_option::Intensity | laser_geometry::channel_option::Distance);

    // move point cloud
    tf_.getTF(msg->header.stamp);
    const auto sensorToMap = tf_.fuseTransform(tf_.SensorToBase, tf_.BaseToMap);
    pcl_ros::transformPointCloud("map", sensorToMap, pcWithDist, pcWithDist);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudM(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(pcWithDist, *cloudM);

    // move map
    Position robotPosition(tf_.BaseToMap.translation.x, tf_.BaseToMap.translation.y);
    localMap_.move(robotPosition);

    for (const auto &p : cloudM->points)
    {
        Position point(p.x, p.y);
        Index pointIndex;
        if (!localMap_.getIndex(point, pointIndex))
            continue;

        localMap_.at("laser_raw", pointIndex) = maxIntrinsicCost_;
    }
    gridInflation(localMap_, "laser_raw", maxIntrinsicCost_, "laser_inflated");

    pubLaser.publish(pcWithDist);
    publishMap(localMap_, pubLocalmap);

    gridMap_["totalCost"] = gridMap_["intrinsicCost"]

    resetLocalMapMemory();
}

void globalNavPlannerRos::publishMap(const GridMap &gridmap, ros::Publisher publisher)
{
    if (publisher.getNumSubscribers() < 1)
        return;

    grid_map_msgs::GridMap msg;
    GridMapRosConverter::toMessage(gridmap, msg);
    publisher.publish(msg);
}

void globalNavPlannerRos::loadParamServer()
{
    nh.param<std::string>("input_cloud_topic", topicLaserSub, "laserScan");
    nh.param<std::string>("input_cloud_topic", topicGridMapPub, "gridmap");
    nh.param<std::string>("input_cloud_topic", topicPathPub, "path");
    nh.param<int>("test", inflationSize_, 5);
    nh.param<std::string>("test2", topicLaserSub, "/tim581_front/scan");
    nh.param<std::string>("test3", topicLaserPub, "/scan");
    nh.param<std::string>("test4", topicLocalMapPub, "localmap");
}

void globalNavPlannerRos::toRosMsg(std::vector<Position> &poseList, nav_msgs::Path &msg) const
{

    msg.header.frame_id = occupancyMap_.header.frame_id;
    msg.header.stamp = ros::Time::now();

    for (const auto &pose : poseList)
    {
        geometry_msgs::PoseStamped poseMsg;
        poseMsg.pose.position.x = pose.x();
        poseMsg.pose.position.y = pose.y();
        msg.poses.push_back(poseMsg);
    }
}