//
// Created by Ikhyeon Cho on 22. 3. 17..
//
#include <global_planner_ros/global_planner_ros.h>

using namespace grid_map;

GlobalPlannerRos::GlobalPlannerRos()
    : nh("global_planner"),
      map_converter_(nh),
      map_({"obstacle_static_raw", "obstacle_static", "intrinsic_cost", "obstacle_laser_raw", "obstacle_laser", "obstacle_all"}),
      goal_received_(false)
{
    loadParamServer();

    while (!tf_.getStaticTF("tim581_front"))
        ros::Duration(1.0).sleep();

    while (!map_converter_.queryOccupancyMap())
    {
        ROS_ERROR("No response for calling static map. Wait until map is valid.");
        ros::Duration(1.0).sleep();
    }

    if (!map_converter_.convertOccupancyToGridmapLayer(map_, "obstacle_static_raw"))
    {
        ROS_ERROR("Conversion from occupancy map to gridmap failed. Node shutdown");
        nh.shutdown();
    }
    else
    {
        map_converter_.gridInflation(map_, "obstacle_static_raw", "obstacle_static", OCCUPIED, inflation_size_);
        ROS_INFO("Conversion and inflation done. Global map initialized.");
    }

    sub_goal = nh.subscribe("/move_base_simple/goal", 10, &GlobalPlannerRos::goalCallback, this);
    sub_laser = nh.subscribe(topic_laser_sub, 10, &GlobalPlannerRos::laserCallback, this);
    pub_gridmap = nh.advertise<grid_map_msgs::GridMap>(topic_gridmap_pub, 1);
    pub_path = nh.advertise<nav_msgs::Path>(topic_path_pub, 10);
    pub_eband_path = nh.advertise<nav_msgs::Path>(topic_eband_path_pub, 10);
    pub_laser = nh.advertise<sensor_msgs::PointCloud2>(topic_laser_pub, 1);
    pub_localmap = nh.advertise<grid_map_msgs::GridMap>(topic_localmap_pub, 1);

    pub_bubble = nh.advertise<visualization_msgs::MarkerArray>(topic_bubble_pub, 10);
}

void GlobalPlannerRos::goalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    goal_received_ = true;
    updateGoalPosition(msg->pose);
    updateRobotPosition(msg->header.stamp);

    DijkstraSearch grid_search(map_, "obstacle_static", position_robot_, position_goal_);
    if (!grid_search.updateCostmap("intrinsic_cost"))
        return;
    max_intrinsic_cost_ = grid_search.getMaxCost();

    DijkstraSearch planner(map_, "intrinsic_cost", position_robot_, position_goal_);
    if (!planner.findPath())
        return;

    // publish dijkstra path msgs
    path_converter_.toRosMsg(planner.getPath(), path_msg_);
    pub_path.publish(path_msg_);

    // publish map
    publishSubmap(map_, pub_gridmap, Length(50, 50));
}

void GlobalPlannerRos::laserCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    clk::time_point t1, t2;

    if (!goal_received_)
        return;

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
    updateSensorMap(cloud_m, max_intrinsic_cost_);
    publishSubmap(map_, pub_localmap, Length(20, 20));

    map_["obstacle_all"] = map_["intrinsic_cost"] + map_["obstacle_laser_raw"];

    updateRobotPosition(msg->header.stamp);
    DijkstraSearch planner(map_, "obstacle_all", position_robot_, position_goal_);
    if (!planner.findPath())
        return;

    // publish global path
    nav_msgs::Path msg_path_raw;
    path_converter_.toRosMsg(planner.getPath(), msg_path_raw);
    pub_path.publish(msg_path_raw);

    map_["obstacle_all"] = map_["obstacle_static_raw"] + map_["obstacle_laser_raw"];

    // modify global path locally && visualize
    ElasticBands eband(map_, "obstacle_all", planner.getPath());
    eband.update();
    eband_converter_.toROSMsg(eband, bubble_msg_, path_msg_);
    pub_eband_path.publish(path_msg_);
    pub_bubble.publish(bubble_msg_);
}

void GlobalPlannerRos::loadParamServer()
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

int main(int argc, char **argv)
{
    ros::init(argc, argv, "global_planner");

    GlobalPlannerRos node;

    ros::Duration(1.0).sleep(); // Need this to get the TF caches fill up.

    ros::spin();

    return 0;
}
