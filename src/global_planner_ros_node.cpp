//
// Created by Ikhyeon Cho on 22. 3. 17..
//
#include <gridmap_navigation/global_planner_ros.h>

using namespace grid_map;

GlobalPlannerRos::GlobalPlannerRos()
    : nh("global_planner"),
      map_converter_(nh),
      map_({"obstacle_static_raw", "obstacle_static", "intrinsic_cost", "obstacle_laser_raw", "obstacle_laser", "obstacle_all", "label"}),
      goal_received_(false)
{
    loadParamServer();

    while (!tf_.getStaticTF(sensor_frame_))
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
    sub_localmap = nh.subscribe(topic_localmap_sub, 1, &GlobalPlannerRos::localmapCallback, this);

    sub_laser = nh.subscribe(topic_laser_sub, 10, &GlobalPlannerRos::laserCallback, this);
    pub_gridmap = nh.advertise<grid_map_msgs::GridMap>(topic_gridmap_pub, 1);
    pub_path = nh.advertise<nav_msgs::Path>(topic_path_pub, 10);
    pub_eband_path = nh.advertise<nav_msgs::Path>(topic_eband_path_pub, 10);
    pub_laser = nh.advertise<sensor_msgs::PointCloud2>("/laser_in_cloud", 1);
    pub_submap = nh.advertise<grid_map_msgs::GridMap>(topic_submap_pub, 1);
    // pub_bubble = nh.advertise<visualization_msgs::MarkerArray>(topic_bubble_pub, 10);
    pub_bubble = nh.advertise<visualization_msgs::Marker>(topic_bubble_pub, 10);

    pub_local_goal = nh.advertise<geometry_msgs::PoseStamped>(topic_local_goal_pub, 10);
    pub_local_map = nh.advertise<nav_msgs::OccupancyGrid>("/local_map", 1);
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
    scan2cloud_.projectLaser(*msg, laser_cloud, -1, laser_geometry::channel_option::Intensity | laser_geometry::channel_option::Distance);

    // transform cloud
    const auto sensor_to_map = tf_.fuseTransform(tf_.SensorToBase, tf_.BaseToMap);
    pcl_ros::transformPointCloud("map", sensor_to_map, laser_cloud, laser_cloud);
    pub_laser.publish(laser_cloud);

    // update Sensor map && visualize
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_m(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(laser_cloud, *cloud_m);
    updateSensorMap(cloud_m, max_intrinsic_cost_);
    publishSubmap(map_, pub_submap, Length(20, 20));

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

void GlobalPlannerRos::localmapCallback(const grid_map_msgs::GridMapConstPtr &msg)
{
    if (!goal_received_)
        return;

    updateRobotPosition(msg->info.header.stamp);

    GridMapRosConverter::fromMessage(*msg, localmap_);

    if (!use_global_map_)
    {
        localmap_["label"] *= OCCUPIED;
        localmap_.add("intrinsic_cost");
        DijkstraSearch grid_search(localmap_, "label", position_robot_, position_goal_);
        if (!grid_search.updateCostmap("intrinsic_cost"))
            return;
        max_intrinsic_cost_ = grid_search.getMaxCost();

        DijkstraSearch planner(localmap_, "intrinsic_cost", position_robot_, position_goal_);
        if (!planner.findPath())
            return;

        nav_msgs::Path msg_path_raw;
        path_converter_.toRosMsg(planner.getPath(), msg_path_raw);
        pub_path.publish(msg_path_raw);
        ElasticBands eband(localmap_, "label", planner.getPath());
        // eband.update();
        // eband_converter_.toROSMsg(eband, bubble_msg_, path_msg_);
        // pub_eband_path.publish(path_msg_);
        // pub_bubble.publish(bubble_msg_);

        // ///////////////////////////////////////////////////////////////////////
        visualization_msgs::Marker bubble;
        bubble.type = visualization_msgs::Marker::SPHERE;
        bubble.action = visualization_msgs::Marker::ADD;
        bubble.header.frame_id = "map";
        bubble.header.stamp = ros::Time(0);
        bubble.ns = "eband";
        bubble.id = 1;

        // position
        bubble.pose.position.x = eband.getBubbles().at(15).getPosition().x();
        bubble.pose.position.y = eband.getBubbles().at(15).getPosition().y();
        bubble.pose.position.z = 0;
        bubble.pose.orientation.x = 0;
        bubble.pose.orientation.y = 0;
        bubble.pose.orientation.z = 0;
        bubble.pose.orientation.w = 1;

        // color
        bubble.color.a = 0.3;
        bubble.color.r = 0.0;
        bubble.color.g = 0.5;
        bubble.color.b = 0.0;

        // size
        bubble.scale.x = eband.getBubbles().at(15).getRadius() * 2.0;
        bubble.scale.y = eband.getBubbles().at(15).getRadius() * 2.0;
        bubble.scale.z = 0.05;

        bubble.lifetime = ros::Duration();

        pub_bubble.publish(bubble);

        ////////////////////////////////////////////////////////////////////////////
        // // local goal pub
        // geometry_msgs::PoseStamped msg_local_goal;
        // msg_local_goal.header.frame_id = "map";
        // msg_local_goal.header.stamp = msg->info.header.stamp;
        // msg_local_goal.pose = bubble_msg_.markers.at(2).pose;
        // pub_local_goal.publish(msg_local_goal);

        // // local map pub
        // nav_msgs::OccupancyGrid occupancy_map;
        // GridMapRosConverter::toOccupancyGrid(localmap_, "label", 0, OCCUPIED, occupancy_map);
        // pub_local_map.publish(occupancy_map);
        // publishSubmap(map_, pub_submap, Length(4, 4));
    }
    else
    {
        GridMapRosConverter::fromMessage(*msg, localmap_);
        map_["label"].setZero();
        map_.addDataFrom(localmap_, false, true, false, {"label"});
        map_["obstacle_all"] = map_["intrinsic_cost"] + map_["label"] * max_intrinsic_cost_;

        updateRobotPosition(msg->info.header.stamp);
        DijkstraSearch planner(map_, "obstacle_all", position_robot_, position_goal_);
        if (!planner.findPath())
            return;

        // publish global path
        nav_msgs::Path msg_path_raw;
        path_converter_.toRosMsg(planner.getPath(), msg_path_raw);
        pub_path.publish(msg_path_raw);

        map_["obstacle_all"] = map_["obstacle_static_raw"] + map_["label"];
        // modify global path locally && visualize
        ElasticBands eband(map_, "obstacle_all", planner.getPath());
        // eband.update();
        eband_converter_.toROSMsg(eband, bubble_msg_, path_msg_);
        pub_eband_path.publish(path_msg_);
        pub_bubble.publish(bubble_msg_);
    }
}

void GlobalPlannerRos::loadParamServer()
{
    nh.param<std::string>("sensorFrameId", sensor_frame_, "tim581_front");

    nh.param<std::string>("laserScanTopic", topic_laser_sub, "/tim581_front/scan");
    nh.param<std::string>("localmapTopic", topic_localmap_sub, "/traversability/map");

    nh.param<std::string>("globalMapTopic", topic_gridmap_pub, "/gridmap");
    nh.param<std::string>("globalSubmapTopic", topic_submap_pub, "/gridmap_sub");
    nh.param<std::string>("dijkstraPathTopic", topic_path_pub, "/path_raw");
    nh.param<std::string>("ebandPathTopic", topic_eband_path_pub, "/path_banded");
    nh.param<std::string>("ebandBubbleTopic", topic_bubble_pub, "/elastic_bands");
    nh.param<std::string>("localGoalTopic", topic_local_goal_pub, "/local_goal");

    nh.param<int>("inflationGridSize", inflation_size_, 4);
    nh.param<bool>("useGlobalmap", use_global_map_, false);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "global_planner");

    GlobalPlannerRos node;

    ros::Duration(1.0).sleep(); // Need this to get the TF caches fill up.

    ros::spin();

    return 0;
}
