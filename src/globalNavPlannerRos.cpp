#include "gridmap-navigation/globalNavPlannerRos.hpp"
#include "gridmap-navigation/elasticBands.hpp"
#include "gridmap-navigation/elasticBandsRos.hpp"

GlobalNavPlannerRos::GlobalNavPlannerRos()
    : nh("global_planner"),
      goalReceived_(false)
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
    subGoal = nh.subscribe("/move_base_simple/goal", 10, &GlobalNavPlannerRos::goalCallback, this);
    subLaser = nh.subscribe(topicLaserSub, 10, &GlobalNavPlannerRos::laserCallback, this);
    pubGridmap = nh.advertise<grid_map_msgs::GridMap>(topicGridMapPub, 1);
    pubPath = nh.advertise<nav_msgs::Path>(topicPathPub, 10);
    pubEbandPath = nh.advertise<nav_msgs::Path>(topicEbandPathPub, 10);
    pubLaser = nh.advertise<sensor_msgs::PointCloud2>(topicLaserPub, 1);
    pubLocalmap = nh.advertise<grid_map_msgs::GridMap>(topicLocalMapPub, 1);

    pubBubble = nh.advertise<visualization_msgs::Marker>(topicBubblePub, 10);


    // 5. initialize map values
    if (!initialize())
    {
        ROS_ERROR("GlobalNavPlannerRos initialization failed. Shut down process");
        nh.shutdown();
    }
}

void GlobalNavPlannerRos::waitOccupancyMap()
{
    mapClient = nh.serviceClient<nav_msgs::GetMap>("/static_map");
    while (!mapClient.call(getMap))
    {
        ROS_ERROR("No response for calling static map. Wait until map is valid.");
        ros::Duration(1.0).sleep();
    }
    ROS_INFO("Static map recieved. Start conversion to Grid map structure");
    occupancyMap_ = getMap.response.map;
}

void GlobalNavPlannerRos::allocateMapMemory()
{
    // static map
    gridMap_.add("raw");
    gridMap_.add("obstacleCost");
    gridMap_.add("traversable_global");

    // cost map in traversable region
    gridMap_.add("intrinsicCost");
    gridMap_.add("totalCost");

    // local map
    gridMap_.add("laser_raw");
    gridMap_.add("laser_inflated");
}

void GlobalNavPlannerRos::resetGridMapMemory()
{
    gridMap_.clear("intrinsicCost");
}

void GlobalNavPlannerRos::resetLocalMapMemory()
{
    gridMap_["laser_raw"].setZero();
    gridMap_["laser_inflated"].setZero();
}

bool GlobalNavPlannerRos::initialize()
{
    // global map
    bool converted = GridMapRosConverter::fromOccupancyGrid(occupancyMap_, "raw", gridMap_);
    if (!converted)
    {
        ROS_ERROR("Conversion from occupancy map to gridmap failed.");
        return false;
    }

    gridInflation(gridMap_, "raw", OCC_MAP, "obstacleCost");
    // scaleLayerValue("obstacleCost");
    getTraversableSearchspace();

    ROS_INFO("Conversion done. Global map initialized.");

    // local map
    gridMap_["laser_raw"].setConstant(FREE);
    gridMap_["laser_inflated"].setConstant(FREE);

    ROS_INFO("Local map initialized to zero.");

    return true;
}

void GlobalNavPlannerRos::scaleLayerValue(const std::string &layer)
{
    auto &mapData = gridMap_[layer];
    for (GridMapIterator iter(gridMap_); !iter.isPastEnd(); ++iter)
    {
        const size_t i = iter.getLinearIndex();
        float &cellVal = mapData(i);

        if (!std::isfinite(cellVal))
            cellVal = OCCUPIED;

        if (std::abs(cellVal - OCC_MAP) < FLT_EPSILON)
            cellVal = OCCUPIED;

        if (std::abs(cellVal - FREE_MAP) < FLT_EPSILON)
            cellVal = FREE;
    }
}

void GlobalNavPlannerRos::gridInflation(GridMap &gridmap, const std::string &layerIn, int inflateState, const std::string &layerOut) const
{
    const auto &mapData = gridmap[layerIn];
    auto &inflatedData = gridmap[layerOut];

    // copy first
    gridmap[layerOut] = gridmap[layerIn];

    // inflate
    for (GridMapIterator iter(gridmap); !iter.isPastEnd(); ++iter)
    {
        size_t i = iter.getLinearIndex();
        const auto &cellIndex = *iter;
        const float cellState = mapData(i);
        float &inflatedState = inflatedData(i);

        if (!std::isfinite(cellState))
            continue;

        if (std::abs(cellState - inflateState) > FLT_EPSILON)
            continue;

        Index startIndex(inflationSize_, inflationSize_);
        Index inflationBuffer(2 * inflationSize_ + 1, 2 * inflationSize_ + 1);
        SubmapIterator subIter(gridmap, cellIndex - startIndex, inflationBuffer);
        for (subIter; !subIter.isPastEnd(); ++subIter)
        {
            // check out of range while searching
            Position searchPos;
            if (!gridmap.getPosition(*subIter, searchPos))
                continue;

            gridmap.at(layerOut, *subIter) = cellState;
        }
    }
}

void GlobalNavPlannerRos::getTraversableSearchspace()
{
    for (GridMapIterator iter(gridMap_); !iter.isPastEnd(); ++iter)
    {
        const auto &cellIndex = *iter;
        const auto &cellState = gridMap_.at("obstacleCost", cellIndex);

        if (!std::isfinite(cellState))
            continue;

        if (cellState != OCCUPIED)
        {
            gridMap_.at("traversable_global", cellIndex) = FREE;
        }
    }
}

//////////////////////////////////////////
//////////// Goal Callback ///////////////
//////////////////////////////////////////

void GlobalNavPlannerRos::goalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    goalReceived_ = true;
    gridMap_.clear("intrinsicCost");

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
    std::cout << "Duration update Intrinsic Cost map: " << duration(t2 - t1) << "us" << std::endl;

    // t1 = clk::now();
    std::vector<Position> poseList;
    if (!findGradientPath("intrinsicCost", poseList))
    {
        ROS_ERROR("Finding Path failed. clear path");
        poseList.clear();
        return;
    }
    t2 = clk::now();
    std::cout << "Duration finding path : " << duration(t2 - t1) << "us" << std::endl;

    ElasticBands eband(poseList, gridMap_, "intrinsicCost");
    eband.updateElasticBand();
    
    // publish eband path msgs
    ElasticBandsRosConverter ebandRos;
    nav_msgs::Path bandedPathMsg;
    visualization_msgs::MarkerArray bubbleMsg;
    ebandRos.toROSMsg(eband, bubbleMsg, bandedPathMsg);
    pubEbandPath.publish(bandedPathMsg);

    // publish dijkstra path msgs
    nav_msgs::Path rawPathMsg;
    toRosMsg(poseList, rawPathMsg);
    pubPath.publish(rawPathMsg);

    

    // publish map
    bool getSubmap;
    auto submap = gridMap_.getSubmap(robotPosition_, Length(100, 100), getSubmap);
    if (!getSubmap)
        ROS_WARN("Could not get submap to publish.");
    else
    {
        publishMap(submap, pubGridmap);
        ROS_INFO("Costmap has been published.");
    }
}

bool GlobalNavPlannerRos::updateIntrinsicCost(double &maxCost)
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
        Position waveFrontPosition;
        const auto waveFrontIndex = gridCandidateList.front();
        gridMap_.getPosition(waveFrontIndex, waveFrontPosition);
        gridCandidateList.pop();

        Index nearestIndex(1, 1);
        Index searchSize(3, 3);
        SubmapIterator subIter(gridMap_, waveFrontIndex - nearestIndex, searchSize);
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

            // only search in region between goal and robot
            double margin = 5.0; // check margin is needed or not
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

bool GlobalNavPlannerRos::findGradientPath(const std::string &costLayer, std::vector<Position> &poseList)
{
    clk::time_point startTime = clk::now();
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
        const auto centerIndex = pathCandidateList.front();

        // reached the goal, then stop propagating
        if (centerIndex.isApprox(goalIndex))
        {
            ROS_INFO("findGradientPath: reached the goal");
            return true;
        }

        Index minIndex = centerIndex;
        double minCost = gridMap_.at(costLayer, centerIndex);

        Index searchSize(inflationSize_, inflationSize_);
        Index bufferSize(2 * inflationSize_ + 1, 2 * inflationSize_ + 1);
        SubmapIterator subIter(gridMap_, centerIndex - searchSize, bufferSize);
        bool validNearFound = false;

        for (subIter; !subIter.isPastEnd(); ++subIter)
        {
            const auto &searchIndex = *subIter;
            Position searchPosition;

            if (!std::isfinite(gridMap_.at(costLayer, searchIndex)))
                continue;

            if (searchIndex.isApprox(centerIndex))
                continue;

            const auto &searchCost = gridMap_.at(costLayer, searchIndex);
            const auto &waveFrontCost = gridMap_.at(costLayer, centerIndex);
            if (searchCost < minCost)
            {
                minCost = searchCost;
                minIndex = searchIndex;
                validNearFound = true;
            }

        } // end of nearest search loop -- finding minimum cost cell
        if (!validNearFound)
        {
            ROS_WARN("No valid path found. Maybe blocked");
            return false;
        }

        Position nextPosition;
        gridMap_.getPosition(minIndex, nextPosition);
        poseList.push_back(nextPosition);
        pathCandidateList.push(minIndex);
        pathCandidateList.pop();
    }

    ROS_WARN("No path available. Maybe blocked");
    return false;
}

//////////////////////////////////////////
//////////// laser Callback //////////////
//////////////////////////////////////////

void GlobalNavPlannerRos::laserCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    // laserScan type conversion
    sensor_msgs::PointCloud2 pcWithDist;
    laser2pc_.projectLaser(*msg, pcWithDist, -1, laser_geometry::channel_option::Intensity | laser_geometry::channel_option::Distance);

    // current robot position
    tf_.getTF(msg->header.stamp);
    robotPosition_(0) = tf_.BaseToMap.translation.x;
    robotPosition_(1) = tf_.BaseToMap.translation.y;
    const Position &robotPosition = robotPosition_;

    // move point cloud
    const auto sensorToMap = tf_.fuseTransform(tf_.SensorToBase, tf_.BaseToMap);
    pcl_ros::transformPointCloud("map", sensorToMap, pcWithDist, pcWithDist);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudM(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(pcWithDist, *cloudM);

    // visualize cloud
    pubLaser.publish(pcWithDist);

    if (!goalReceived_)
        return;

    clk::time_point t1 = clk::now();
    // laser cost
    for (const auto &p : cloudM->points)
    {
        Position point(p.x, p.y);
        Index pointIndex;
        if (!gridMap_.getIndex(point, pointIndex))
            continue;

        gridMap_.at("laser_raw", pointIndex) = maxIntrinsicCost_;

        Index startIndex(inflationSize_, inflationSize_);
        Index subMapSize(2 * inflationSize_ + 1, 2 * inflationSize_ + 1);
        SubmapIterator inflationIter(gridMap_, pointIndex - startIndex, subMapSize);
        for (inflationIter; !inflationIter.isPastEnd(); ++inflationIter)
        {
            gridMap_.at("laser_inflated", *inflationIter) = gridMap_.at("laser_raw", pointIndex);
        }
    }
    clk::time_point t2 = clk::now();
    std::cout << "Duration laser_inflated mapping: " << duration(t2 - t1) << "us" << std::endl;

    // visualize local map
    bool getSubmap;
    publishMap(gridMap_.getSubmap(robotPosition, Length(20, 20), getSubmap), pubLocalmap);
    std::cout << "Submap published " << std::endl;

    // total Costmap
    gridMap_["totalCost"] = gridMap_["intrinsicCost"] + gridMap_["laser_inflated"];

    // find path and publish
    std::vector<Position> poseList;
    findGradientPath("totalCost", poseList);

    //
    // add eband function
    //
    
    nav_msgs::Path pathMsg;
    toRosMsg(poseList, pathMsg);
    pubPath.publish(pathMsg);

    publishMap(gridMap_.getSubmap(robotPosition_, Length(100, 100), getSubmap), pubGridmap);

    // intrinsicCost recovery
    gridMap_["totalCost"] -= gridMap_["laser_inflated"];
    resetLocalMapMemory();
}

void GlobalNavPlannerRos::publishMap(const GridMap &gridmap, ros::Publisher publisher)
{
    if (publisher.getNumSubscribers() < 1)
        return;

    grid_map_msgs::GridMap msg;
    GridMapRosConverter::toMessage(gridmap, msg);
    publisher.publish(msg);
}

void GlobalNavPlannerRos::loadParamServer()
{
    nh.param<std::string>("input_cloud_topic", topicLaserSub, "/laserScan");
    nh.param<std::string>("input_cloud_topic", topicGridMapPub, "/gridmap");
    nh.param<std::string>("path_raw", topicPathPub, "/path_raw");
    nh.param<std::string>("eband_path", topicEbandPathPub, "/path_banded");
    nh.param<std::string>("bubble", topicBubblePub, "/elastic_bands");

    nh.param<int>("test", inflationSize_, 3);
    nh.param<std::string>("test2", topicLaserSub, "/tim581_front/scan");
    nh.param<std::string>("test3", topicLaserPub, "/scan");
    nh.param<std::string>("test4", topicLocalMapPub, "localmap");
}

void GlobalNavPlannerRos::toRosMsg(std::vector<Position> &poseList, nav_msgs::Path &msg) const
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