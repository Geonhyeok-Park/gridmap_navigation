#include <global_planner_ros/globalNavPlannerRos.hpp>

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

    pubBubble = nh.advertise<visualization_msgs::MarkerArray>(topicBubblePub, 10);

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
    gridMap_.add("staticObstacle");

    // cost map in traversable region
    gridMap_.add("intrinsicCost");

    // local map
    gridMap_.add("sensorObstacle");
}

void GlobalNavPlannerRos::resetGridMapMemory()
{
    gridMap_.clear("intrinsicCost");
}

void GlobalNavPlannerRos::resetLocalMapMemory()
{
    gridMap_["sensorObstacle"].setZero();
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

    gridInflation(gridMap_, "raw", OCC_MAP, "staticObstacle");

    ROS_INFO("Conversion done. Global map initialized.");

    // local map
    gridMap_["sensorObstacle"].setConstant(FREE);

    ROS_INFO("Local map initialized to zero.");

    return true;
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

//////////////////////////////////////////
/////////                     ////////////
/////////    Goal Callback    ////////////
/////////                     ////////////
//////////////////////////////////////////

void GlobalNavPlannerRos::goalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    if (!goalReceived_)
        goalReceived_ = true;

    goalPosition_(0) = msg->pose.position.x;
    goalPosition_(1) = msg->pose.position.y;

    tf_.getTF(msg->header.stamp);
    robotPosition_(0) = tf_.BaseToMap.translation.x;
    robotPosition_(1) = tf_.BaseToMap.translation.y;

    DijkstraSearch globalPlanner(gridMap_, "staticObstacle", "intrinsicCost",
                                 robotPosition_, goalPosition_);

    if (!globalPlanner.run())
        return;

    maxIntrinsicCost_ = globalPlanner.getMaxCost();

    // publish dijkstra path msgs
    globalPath_ = globalPlanner.getPath();
    nav_msgs::Path rawPathMsg;
    dijkstraRos_.toRosMsg(globalPath_, rawPathMsg);
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

//////////////////////////////////////////
/////////                      ///////////
/////////    laser Callback    ///////////
/////////                      ///////////
//////////////////////////////////////////

void GlobalNavPlannerRos::laserCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    clk::time_point t1, t2;

    if (!goalReceived_)
        return;

    // update robot position
    tf_.getTF(msg->header.stamp);
    robotPosition_(0) = tf_.BaseToMap.translation.x;
    robotPosition_(1) = tf_.BaseToMap.translation.y;
    const Position &robotPosition = robotPosition_;

    // publish global path
    DijkstraSearch globalPlanner(gridMap_, "staticObstacle", "intrinsicCost",
                                 robotPosition, goalPosition_);
    globalPlanner.findPath();
    globalPath_ = globalPlanner.getPath();
    nav_msgs::Path rawPathMsg;
    dijkstraRos_.toRosMsg(globalPath_, rawPathMsg);
    pubPath.publish(rawPathMsg);

    // laserScan type conversion
    sensor_msgs::PointCloud2 pcWithDist;
    laser2pc_.projectLaser(*msg, pcWithDist, -1, laser_geometry::channel_option::Intensity | laser_geometry::channel_option::Distance);

    // transform cloud && visualize
    const auto sensorToMap = tf_.fuseTransform(tf_.SensorToBase, tf_.BaseToMap);
    pcl_ros::transformPointCloud("map", sensorToMap, pcWithDist, pcWithDist);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudM(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(pcWithDist, *cloudM);
    pubLaser.publish(pcWithDist);

    // update Sensor map && visualize
    updateSensorMap(cloudM);
    bool getSubmap;
    publishMap(gridMap_.getSubmap(robotPosition, Length(20, 20), getSubmap), pubLocalmap);

    // modify global path locally && visualize
    ElasticBands eband(globalPath_, gridMap_, "sensorObstacle");
    eband.updateElasticBand();
    nav_msgs::Path bandedPathMsg;
    visualization_msgs::MarkerArray bubbleMsg;
    ebandRos_.toROSMsg(eband, bubbleMsg, bandedPathMsg);
    pubEbandPath.publish(bandedPathMsg);
    pubBubble.publish(bubbleMsg);

    resetLocalMapMemory();
}

void GlobalNavPlannerRos::updateSensorMap(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{
    for (const auto &p : cloud->points)
    {
        Position point(p.x, p.y);
        Index pointIndex;
        if (!gridMap_.getIndex(point, pointIndex))
            continue;

        gridMap_.at("sensorObstacle", pointIndex) = maxIntrinsicCost_;
    }
}

void GlobalNavPlannerRos::publishMap(const GridMap &gridmap, const ros::Publisher &publisher)
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

    nh.param<int>("test", inflationSize_, 4);
    nh.param<std::string>("test2", topicLaserSub, "/tim581_front/scan");
    nh.param<std::string>("test3", topicLaserPub, "/scan");
    nh.param<std::string>("test4", topicLocalMapPub, "/localmap");
}