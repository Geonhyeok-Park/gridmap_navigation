#include <global_planner_ros/global_planner_ros.h>

using namespace grid_map;

GlobalNavPlannerRos::GlobalNavPlannerRos()
    : nh("global_planner"),
      map_converter_(nh),
      map_({"raw", "staticObstacle", "intrinsicCost", "sensorObstacle"}),
      goal_received_(false)
{
    // 1. load ROS params
    loadParamServer();

    // 2. check ROS static tf_ has been constructed
    while (!tf_.getStaticTF("tim581_front"))
        ros::Duration(1.0).sleep();

    while (!map_converter_.queryOccupancyMap())
    {
        ROS_ERROR("No response for calling static map. Wait until map is valid.");
        ros::Duration(1.0).sleep();
    }

    if (!map_converter_.convertOccupancyToGridmapLayer(map_, "raw"))
    {
        ROS_ERROR("Conversion from occupancy map to gridmap failed. Node shutdown");
        nh.shutdown();
    }
    else
    {
        map_converter_.gridInflation(map_, "raw", "staticObstacle", OCCUPIED, inflation_size_);
        ROS_INFO("Conversion and inflation done. Global map initialized.");
    }

    sub_goal = nh.subscribe("/move_base_simple/goal", 10, &GlobalNavPlannerRos::goalCallback, this);
    sub_laser = nh.subscribe(topic_laser_sub, 10, &GlobalNavPlannerRos::laserCallback, this);
    pub_gridmap = nh.advertise<grid_map_msgs::GridMap>(topic_gridmap_pub, 1);
    pub_path = nh.advertise<nav_msgs::Path>(topic_path_pub, 10);
    pub_eband_path = nh.advertise<nav_msgs::Path>(topic_eband_path_pub, 10);
    pub_laser = nh.advertise<sensor_msgs::PointCloud2>(topic_laser_pub, 1);
    pub_localmap = nh.advertise<grid_map_msgs::GridMap>(topic_localmap_pub, 1);

    pub_bubble = nh.advertise<visualization_msgs::MarkerArray>(topic_bubble_pub, 10);
}

void GlobalNavPlannerRos::goalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    goal_received_ = true;
    updateGoalPosition(msg->pose);
    updateRobotPosition(msg->header.stamp);

    DijkstraSearch grid_search(map_, "staticObstacle", position_robot_, position_goal_);
    if (!grid_search.updateCostmap("intrinsicCost"))
        return;

    DijkstraSearch planner(map_, "intrinsicCost", position_robot_, position_goal_);
    if (!planner.findPath())
        return;

    // publish dijkstra path msgs
    nav_msgs::Path msg_path_raw;
    path_converter_.toRosMsg(planner.getPath(), msg_path_raw);
    pub_path.publish(msg_path_raw);

    // publish map
    publishSubmap(map_, pub_gridmap, Length(50, 50));
}

void GlobalNavPlannerRos::laserCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    clk::time_point t1, t2;

    if (!goal_received_)
        return;

    updateRobotPosition(msg->header.stamp);

    // publish global path
    DijkstraSearch planner(map_, "intrinsicCost", position_robot_, position_goal_);
    planner.findPath();
    nav_msgs::Path msg_path_raw;
    path_converter_.toRosMsg(planner.getPath(), msg_path_raw);
    pub_path.publish(msg_path_raw);

    // laserScan type conversion
    sensor_msgs::PointCloud2 laser_cloud;
    laser2pc_.projectLaser(*msg, laser_cloud, -1, laser_geometry::channel_option::Intensity | laser_geometry::channel_option::Distance);

    // transform cloud && visualize
    const auto sensor_to_map = tf_.fuseTransform(tf_.SensorToBase, tf_.BaseToMap);
    pcl_ros::transformPointCloud("map", sensor_to_map, laser_cloud, laser_cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_m(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(laser_cloud, *cloud_m);
    pub_laser.publish(laser_cloud);

    // update Sensor map && visualize
    updateSensorMap(cloud_m);
    publishSubmap(map_, pub_localmap, Length(20, 20));

    // modify global path locally && visualize
    ElasticBands eband(planner.getPath(), map_, "sensorObstacle");
    eband.updateElasticBand();
    nav_msgs::Path msg_banded_path;
    visualization_msgs::MarkerArray msg_bubble;
    eband_converter_.toROSMsg(eband, msg_bubble, msg_banded_path);
    pub_eband_path.publish(msg_banded_path);
    pub_bubble.publish(msg_bubble);
}

void GlobalNavPlannerRos::updateSensorMap(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{
    map_["sensorObstacle"].setZero();

    for (const auto &p : cloud->points)
    {
        Position point(p.x, p.y);
        Index point_index;
        if (!map_.getIndex(point, point_index))
            continue;

        map_.at("sensorObstacle", point_index) = OCCUPIED;
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

void GlobalNavPlannerRos::publishSubmap(const grid_map::GridMap &gridmap, const ros::Publisher &publisher, const grid_map::Length &length)
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

void GlobalNavPlannerRos::updateRobotPosition(const ros::Time &time)
{
    tf_.getTF(time);
    position_robot_(0) = tf_.BaseToMap.translation.x;
    position_robot_(1) = tf_.BaseToMap.translation.y;
}

void GlobalNavPlannerRos::updateGoalPosition(const geometry_msgs::Pose &goal_pose)
{
    position_goal_(0) = goal_pose.position.x;
    position_goal_(1) = goal_pose.position.y;
}