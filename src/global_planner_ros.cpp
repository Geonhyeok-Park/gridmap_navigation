#include <global_planner_ros/global_planner_ros.h>

GlobalNavPlannerRos::GlobalNavPlannerRos()
    : nh("global_planner"),
      goal_received_(false)
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
    sub_goal = nh.subscribe("/move_base_simple/goal", 10, &GlobalNavPlannerRos::goalCallback, this);
    sub_laser = nh.subscribe(topic_laser_sub, 10, &GlobalNavPlannerRos::laserCallback, this);
    pub_gridmap = nh.advertise<grid_map_msgs::GridMap>(topic_gridmap_pub, 1);
    pub_path = nh.advertise<nav_msgs::Path>(topic_path_pub, 10);
    pub_eband_path = nh.advertise<nav_msgs::Path>(topic_eband_path_pub, 10);
    pub_laser = nh.advertise<sensor_msgs::PointCloud2>(topic_laser_pub, 1);
    pub_localmap = nh.advertise<grid_map_msgs::GridMap>(topic_localmap_pub, 1);

    pub_bubble = nh.advertise<visualization_msgs::MarkerArray>(topic_bubble_pub, 10);

    // 5. initialize map values
    if (!initialize())
    {
        ROS_ERROR("GlobalNavPlannerRos initialization failed. Shut down process");
        nh.shutdown();
    }
}

void GlobalNavPlannerRos::waitOccupancyMap()
{
    map_client = nh.serviceClient<nav_msgs::GetMap>("/static_map");
    while (!map_client.call(get_map))
    {
        ROS_ERROR("No response for calling static map. Wait until map is valid.");
        ros::Duration(1.0).sleep();
    }
    ROS_INFO("Static map recieved. Start conversion to Grid map structure");
    occupancymap_ = get_map.response.map;
}

void GlobalNavPlannerRos::allocateMapMemory()
{
    // static map
    gridmap_.add("raw");
    gridmap_.add("staticObstacle");

    // cost map in traversable region
    gridmap_.add("intrinsicCost");

    // local map
    gridmap_.add("sensorObstacle");
}

void GlobalNavPlannerRos::resetGridMapMemory()
{
    gridmap_.clear("intrinsicCost");
}

void GlobalNavPlannerRos::resetLocalMapMemory()
{
    gridmap_["sensorObstacle"].setZero();
}

bool GlobalNavPlannerRos::initialize()
{
    // global map
    bool converted = GridMapRosConverter::fromOccupancyGrid(occupancymap_, "raw", gridmap_);
    if (!converted)
    {
        ROS_ERROR("Conversion from occupancy map to gridmap failed.");
        return false;
    }

    gridInflation(gridmap_, "raw", OCC_MAP, "staticObstacle");

    ROS_INFO("Conversion done. Global map initialized.");

    // local map
    gridmap_["sensorObstacle"].setConstant(FREE);

    ROS_INFO("Local map initialized to zero.");

    return true;
}

void GlobalNavPlannerRos::gridInflation(GridMap &gridmap, const std::string &layer_in, int inflate_state, const std::string &layer_out) const
{
    const auto &map_data = gridmap[layer_in];
    auto &inflated_data = gridmap[layer_out];

    // copy first
    gridmap[layer_out] = gridmap[layer_in];

    // inflate
    for (GridMapIterator iter(gridmap); !iter.isPastEnd(); ++iter)
    {
        size_t i = iter.getLinearIndex();
        const auto &cell_index = *iter;
        const float cell_state = map_data(i);
        float &inflated_state = inflated_data(i);

        if (!std::isfinite(cell_state))
            continue;

        if (std::abs(cell_state - inflate_state) > FLT_EPSILON)
            continue;

        Index start_index(inflation_size_, inflation_size_);
        Index inflation_buffer(2 * inflation_size_ + 1, 2 * inflation_size_ + 1);
        SubmapIterator sub_iter(gridmap, cell_index - start_index, inflation_buffer);
        for (sub_iter; !sub_iter.isPastEnd(); ++sub_iter)
        {
            // check out of range while searching
            Position search_pos;
            if (!gridmap.getPosition(*sub_iter, search_pos))
                continue;

            gridmap.at(layer_out, *sub_iter) = cell_state;
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
    if (!goal_received_)
        goal_received_ = true;

    position_goal_(0) = msg->pose.position.x;
    position_goal_(1) = msg->pose.position.y;

    tf_.getTF(msg->header.stamp);
    position_robot_(0) = tf_.BaseToMap.translation.x;
    position_robot_(1) = tf_.BaseToMap.translation.y;

    DijkstraSearch global_planner(gridmap_, "staticObstacle", "intrinsicCost",
                                 position_robot_, position_goal_);

    if (!global_planner.run())
        return;

    max_intrinsic_cost_ = global_planner.getMaxCost();

    // publish dijkstra path msgs
    global_path_ = global_planner.getPath();
    nav_msgs::Path rawPathMsg;
    path_converter_.toRosMsg(global_path_, rawPathMsg);
    pub_path.publish(rawPathMsg);

    // publish map
    bool get_submap;
    auto submap = gridmap_.getSubmap(position_robot_, Length(100, 100), get_submap);
    if (!get_submap)
        ROS_WARN("Could not get submap to publish.");
    else
    {
        publishMap(submap, pub_gridmap);
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

    if (!goal_received_)
        return;

    // update robot position
    tf_.getTF(msg->header.stamp);
    position_robot_(0) = tf_.BaseToMap.translation.x;
    position_robot_(1) = tf_.BaseToMap.translation.y;
    const Position &robot_position = position_robot_;

    // publish global path
    DijkstraSearch global_planner(gridmap_, "staticObstacle", "intrinsicCost",
                                 robot_position, position_goal_);
    global_planner.findPath();
    global_path_ = global_planner.getPath();
    nav_msgs::Path rawPathMsg;
    path_converter_.toRosMsg(global_path_, rawPathMsg);
    pub_path.publish(rawPathMsg);

    // laserScan type conversion
    sensor_msgs::PointCloud2 pc_with_dist;
    laser2pc_.projectLaser(*msg, pc_with_dist, -1, laser_geometry::channel_option::Intensity | laser_geometry::channel_option::Distance);

    // transform cloud && visualize
    const auto sensor_to_map = tf_.fuseTransform(tf_.SensorToBase, tf_.BaseToMap);
    pcl_ros::transformPointCloud("map", sensor_to_map, pc_with_dist, pc_with_dist);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_m(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(pc_with_dist, *cloud_m);
    pub_laser.publish(pc_with_dist);

    // update Sensor map && visualize
    updateSensorMap(cloud_m);
    bool get_submap;
    publishMap(gridmap_.getSubmap(robot_position, Length(20, 20), get_submap), pub_localmap);

    // modify global path locally && visualize
    ElasticBands eband(global_path_, gridmap_, "sensorObstacle");
    eband.updateElasticBand();
    nav_msgs::Path msg_banded_path;
    visualization_msgs::MarkerArray msg_bubble;
    eband_converter_.toROSMsg(eband, msg_bubble, msg_banded_path);
    pub_eband_path.publish(msg_banded_path);
    pub_bubble.publish(msg_bubble);

    resetLocalMapMemory();
}

void GlobalNavPlannerRos::updateSensorMap(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{
    for (const auto &p : cloud->points)
    {
        Position point(p.x, p.y);
        Index point_index;
        if (!gridmap_.getIndex(point, point_index))
            continue;

        gridmap_.at("sensorObstacle", point_index) = max_intrinsic_cost_;
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
    nh.param<std::string>("input_cloud_topic", topic_laser_sub, "/laserScan");
    nh.param<std::string>("input_cloud_topic", topic_gridmap_pub, "/gridmap");
    nh.param<std::string>("path_raw", topic_path_pub, "/path_raw");
    nh.param<std::string>("eband_path", topic_eband_path_pub, "/path_banded");
    nh.param<std::string>("bubble", topic_bubble_pub, "/elastic_bands");

    nh.param<int>("test", inflation_size_, 4);
    nh.param<std::string>("test2", topic_laser_sub, "/tim581_front/scan");
    nh.param<std::string>("test3", topic_laser_pub, "/scan");
    nh.param<std::string>("test4", topic_localmap_pub, "/localmap");
}