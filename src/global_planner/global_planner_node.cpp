#include <gridmap_navigation/global_planner_node.h>

#define duration_ms(a) std::chrono::duration_cast<std::chrono::milliseconds>(a).count()
#define duration_us(a) std::chrono::duration_cast<std::chrono::microseconds>(a).count()
typedef std::chrono::high_resolution_clock clk;

namespace grid_map
{
    GlobalPlannerNode::GlobalPlannerNode(ros::NodeHandle &_nh)
        : nh(_nh),
          tf2_listener(tf2_buffer),
          globalmap_received_(false),
          costmap_updated_(false),
          globalpath_updated_(false)
    {
        useParameterServer();

        if (param_use_globalmap && !param_getmap_from_topic)
        {
            ROS_INFO("Path Planning With Global Map Service!!");

            ros::ServiceClient client;
            nav_msgs::GetMap service;
            client = nh.serviceClient<nav_msgs::GetMap>("/static_map");
            clk::time_point start_time = clk::now();
            while (!globalmap_received_ && duration_ms(clk::now() - start_time) < 5000) // 5sec
            {
                ROS_INFO_THROTTLE(1, "use Global map:: Waiting until the service is valid....");
                globalmap_received_ = client.call(service);
            }
            if (!globalmap_received_)
            {
                ROS_ERROR("use Global map:: Global map not received... Node Shutdown");
                ros::shutdown();
                return;
            }

            ROS_INFO("use Global map:: Global map received.");
            costmapPtr_ = std::make_unique<Costmap>(service.response.map, param_robot_radius);
            costmapPtr_->setTimeLimit(param_timelimit_ms);
        }
        else if (param_use_globalmap && param_getmap_from_topic)
        {
            ROS_INFO("Path Planning With Global Map Topic!!");

            ros::Subscriber sub_globalmap = nh.subscribe("/map", 1, &GlobalPlannerNode::globalmapCallback, this);
            ros::Rate rate(param_Hz);
            clk::time_point start_time = clk::now();
            while (!globalmap_received_ && duration_ms(clk::now() - start_time) < 10000) // 10sec
            {
                ROS_INFO_THROTTLE(1, "use Global map:: Waiting until the topic is valid....");
                ros::spinOnce();
                rate.sleep();
            }
            if (!globalmap_received_)
            {
                ROS_ERROR("use Global map:: Global map not received... Node Shutdown");
                ros::shutdown();
                return;
            }
        }

        else if (!param_use_globalmap)
        {
            ROS_INFO("Path Planning Without Global Map!!");
            costmapPtr_ = std::make_unique<Costmap>();
            costmapPtr_->setTimeLimit(param_timelimit_ms);
        }

        // sub & pub
        sub_goal = nh.subscribe("/move_base_simple/goal", 1, &GlobalPlannerNode::goalCallback, this);
        sub_localmap = nh.subscribe(param_sub_localmap, 1, &GlobalPlannerNode::localmapCallback, this);
        pub_path = nh.advertise<nav_msgs::Path>(param_pub_path, 10);
        pub_localpath = nh.advertise<nav_msgs::Path>(param_pub_localpath, 10);
        pub_ebandmarker = nh.advertise<visualization_msgs::MarkerArray>(param_pub_eband, 10);
        pub_map = nh.advertise<nav_msgs::OccupancyGrid>(param_pub_map, 1);
        pub_localmap = nh.advertise<pcl::PointCloud<pcl::PointXYZI>>("test", 10);

        ROS_INFO("Global Planner Node Initialized.");
        ROS_INFO_STREAM("Waiting for the topic " << sub_goal.getTopic());
    }

    void GlobalPlannerNode::globalmapCallback(const nav_msgs::OccupancyGridConstPtr &msg)
    {
        ROS_INFO("use Global map:: Global map recieved.");
        globalmap_received_ = true;
        costmapPtr_ = std::make_unique<Costmap>(*msg, param_robot_radius);
        costmapPtr_->setTimeLimit(param_timelimit_ms);
    }

    void GlobalPlannerNode::goalCallback(const geometry_msgs::PoseStampedConstPtr &msg)
    {
        costmap_updated_ = false;
        updateGoalPosition(msg->pose);
        if (!updateRobotPosition(msg->header.stamp))
        {
            ROS_WARN("[Goal Callback] Update Robot Position Failed. Check TF tree from map to base_link");
            return;
        }

        clk::time_point start_point = clk::now();
        if (!costmapPtr_->update(robot_position_, goal_position_))
        {
            ROS_WARN("[Goal Callback] Stopped Glboal Path Topic");
            return;
        }
        costmap_updated_ = true;

        ROS_INFO_STREAM("Update Costmap takes " << duration_ms(clk::now() - start_point) << "ms");

        start_point = clk::now();
        nav_msgs::OccupancyGrid occupancy;
        costmapPtr_->toRosMsg("cost", occupancy);
        pub_map.publish(occupancy);
        ROS_INFO_STREAM("Publish Costmap takes " << duration_ms(clk::now() - start_point) << "ms");
    }

    void GlobalPlannerNode::run()
    {
        ros::Rate loop_rate(param_Hz);
        while (ros::ok())
        {
            if (!costmap_updated_) // Check new callbacks in given rates when costmap not initialized or failed to update
            {
                ros::spinOnce();
                loop_rate.sleep();
                continue;
            }

            clk::time_point start_point = clk::now();
            if (!updateRobotPosition(ros::Time::now())) // Check robot tf in given rates
            {
                ROS_WARN_THROTTLE(1, "[Process] Update Robot Position Failed. Check TF tree from map to base_link");
                loop_rate.sleep();
                continue;
            }
            ROS_INFO_STREAM("Load Robot Pose from TF tree takes " << duration_ms(clk::now() - start_point) << "ms");

            start_point = clk::now();
            globalpath_updated_ = false;
            if (!findGlobalPath())
            {
                ROS_WARN_THROTTLE(1, "[Process] No Valid Path Found from the Current Position. Pause path messages");
                ros::spinOnce();
                loop_rate.sleep();
                continue;
            }
            ROS_INFO_STREAM("Finding Path takes " << duration_us(clk::now() - start_point) << "us");

            smoothing(path_, 20);

            nav_msgs::Path msg;
            toRosMsg(path_, msg);
            pub_path.publish(msg);
            globalpath_updated_ = true;

            ros::spinOnce();
            loop_rate.sleep();
        }
    }

    bool GlobalPlannerNode::findGlobalPath()
    {
        return findPathInMatrix(costmapPtr_->get("cost"), robot_position_, goal_position_, path_);
    }

    void GlobalPlannerNode::localmapCallback(const sensor_msgs::PointCloud2Ptr &msg)
    {
        clk::time_point start_point = clk::now();

        if (!globalpath_updated_)
            return;

        auto &layers = costmapPtr_->getLayers();
        if (std::find(layers.begin(), layers.end(), "cost_local") == layers.end())
            costmapPtr_->add("cost_local");

        costmapPtr_->get("cost_local").setZero();
        // costmapPtr_->get("cost_local") += costmapPtr_->get("cost");

        ROS_INFO_STREAM("Submap takes " << duration_us(clk::now() - start_point) << "us");

        bool good;
        auto test = costmapPtr_->getSubmap(robot_position_, Length(20,20), good);
        std::unique_ptr<Costmap> ptr = costmapPtr_->getSubmap(robot_position_, Length(20,20), good);

        // cloud transform to map frame
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(*msg, *cloud);
        if (msg->header.frame_id != "map")
        {
            geometry_msgs::TransformStamped sensor_to_map;
            try
            {
                sensor_to_map = tf2_buffer.lookupTransform("map", msg->header.frame_id, msg->header.stamp, ros::Duration(1.0));
            }
            catch (tf2::TransformException &ex)
            {
                ROS_ERROR("%s", ex.what());
                return;
            }
            pc.transform(cloud, cloud, sensor_to_map.transform, "map");
            pub_localmap.publish(*cloud);
        }

        // cloud to local map

        auto &obstacles = costmapPtr_->get("cost_local");
        for (const auto &point : cloud->points)
        {
            Position obstacle(point.x, point.y);
            Index obstacle_index;
            if (!costmapPtr_->getIndex(obstacle, obstacle_index))
                continue;

            obstacles(obstacle_index(0), obstacle_index(1)) += 100;
            int inflation = param_robot_radius / costmapPtr_->getResolution();
            SubmapIterator obs_inflation(*costmapPtr_, obstacle_index - Index(inflation, inflation), Index(2 * inflation + 1, 2 * inflation + 1));
            for (obs_inflation; !obs_inflation.isPastEnd(); ++ obs_inflation)
            {
                auto &inflation_index = *obs_inflation;
                Position inflation_position;
                if (!costmapPtr_->getPosition(inflation_index, inflation_position))
                    continue;
                obstacles(inflation_index(0), inflation_index(1)) += 100;
            }
        }

        std::vector<Position> path;
        if (!findPathInMatrix(costmapPtr_->get("cost_local"), robot_position_, goal_position_, path))
            ROS_ERROR("Local path could not be found");
        ROS_INFO_STREAM("find local path takes " << duration_us(clk::now() - start_point) << "us");

        nav_msgs::Path msg_path_1;
        toRosMsg(path, msg_path_1);
        pub_localpath.publish(msg_path_1);

        // sensor_msgs::PointCloud2 msg_temp;
        // GridMapRosConverter::toPointCloud(localmap, "occupancy", msg_temp);
        // pcl::moveFromROSMsg(msg_temp, *cloud);
        // pub_localmap.publish(*cloud);

        // // Elastic band

        // ElasticBand eband(path_, 15);
        // if (!eband.update(localmap, costmapPtr_))
        // {
        //     ROS_WARN("[Localmap Callback] Update Local Path Failed. Return No Path!");
        //     return;
        // }

        // // smoothing is valid!! TODO: change publish things
        // // std::vector<Position> path;
        // // smoothing_eband(eband, 5, path);
        // nav_msgs::Path path_msg;
        // // toRosMsg(path, path_msg);
        // // ROS_ERROR_STREAM(path_msg.poses.size());

        // visualization_msgs::MarkerArray bubble_msg;
        // eband.toRosMsg(bubble_msg, path_msg);
        // pub_localpath.publish(path_msg);
        // pub_ebandmarker.publish(bubble_msg);
        // ROS_INFO_STREAM("Local + smoothing takes " << duration_ms(clk::now() - start_point) << "ms");
    }

    void GlobalPlannerNode::useParameterServer()
    {
        nh.param<bool>("useGlobalMap", param_use_globalmap, true);
        nh.param<bool>("getGlobalMapFromTopic", param_getmap_from_topic, false);

        nh.param<std::string>("globalPathTopic", param_pub_path, "path");
        nh.param<std::string>("localPathTopic", param_pub_localpath, "local_path");

        nh.param<std::string>("gridMapTopic", param_pub_map, "map");
        nh.param<std::string>("localMapTopic", param_sub_localmap, "localmap");

        nh.param<double>("robotRadius", param_robot_radius, 0.5);
        nh.param<int>("frequency", param_Hz, 10);
        nh.param<int>("maxProcessTime", param_timelimit_ms, 2000); // 2 sec
    }

    void GlobalPlannerNode::updateGoalPosition(const geometry_msgs::Pose &goal)
    {
        goal_position_(0) = goal.position.x;
        goal_position_(1) = goal.position.y;
        ROS_INFO_STREAM("Updated Goal Position: "
                        << "[" << goal_position_(0) << ", " << goal_position_(1) << "]");
    }

    bool GlobalPlannerNode::updateRobotPosition(const ros::Time &time)
    {
        geometry_msgs::TransformStamped robot_pose;
        try
        {
            robot_pose = tf2_buffer.lookupTransform("map", "base_link", time, ros::Duration(1.0));
        }
        catch (tf2::TransformException &ex)
        {
            ROS_ERROR("%s", ex.what());
            return false;
        }
        robot_position_(0) = robot_pose.transform.translation.x;
        robot_position_(1) = robot_pose.transform.translation.y;
        return true;
    }

    bool GlobalPlannerNode::findPathInMatrix(const GridMap::Matrix &map, const Position &robot, const Position &goal, std::vector<Position> &path)
    {
        path.clear();

        Index robot_idx, goal_idx, waypoint_idx;
        Position waypoint_position;

        if (!costmapPtr_->getIndex(robot, robot_idx))
        {
            ROS_ERROR("[find GlobalPath] Robot is out of Map Boundary. Check TF between Map and Robot.");
            return false;
        }

        if (!costmapPtr_->getIndex(goal, goal_idx))
        {
            ROS_ERROR("[find GlobalPath] Goal is out of Map Boundary.");
            return false;
        }

        if (!std::isfinite(map(robot_idx(0), robot_idx(1))))
        {
            ROS_WARN_STREAM("[find GlobalPath] Robot is Out of map layer. Check the Robot Position!");
            return false;
        }

        waypoint_idx = robot_idx;
        waypoint_position = robot;
        Index search_size(1, 1);
        Index search_area(3, 3);

        clk::time_point start_time = clk::now();
        while (!waypoint_idx.isApprox(goal_idx))
        {
            bool near_path_exists = false;
            double cost_min = map(waypoint_idx(0), waypoint_idx(1)); // get min cost in search area : initialize

            for (SubmapIterator search_iter(*costmapPtr_, waypoint_idx - search_size, search_area); !search_iter.isPastEnd(); ++search_iter)
            {
                const auto &search_idx = *search_iter;

                // pass invalid cost (unexplored region)
                if (!std::isfinite(map(search_idx.x(), search_idx.y())))
                    continue;

                if (search_idx.isApprox(waypoint_idx))
                    continue;

                const auto &searchcost = map(search_idx.x(), search_idx.y());
                if (searchcost <= cost_min)
                {
                    cost_min = searchcost;
                    waypoint_idx = search_idx;
                    near_path_exists = true;
                }
            }

            if (!near_path_exists)
            {
                ROS_ERROR("[Current Path] No valid path found in cost map");
                return false;
            }

            if (duration_ms(clk::now() - start_time) > param_timelimit_ms)
            {
                ROS_WARN("[Current Path] TIMEOUT!! Finding Path from Cost map Failed.");
                return false;
            }

            costmapPtr_->getPosition(waypoint_idx, waypoint_position);
            path.push_back(waypoint_position);
        }

        return true;
    }

    void GlobalPlannerNode::toRosMsg(const std::vector<Position> &path, nav_msgs::Path &msg)
    {
        msg.header.frame_id = "map";
        msg.header.stamp = ros::Time::now();

        for (const auto &pathpoint : path)
        {
            geometry_msgs::PoseStamped poseMsg;
            poseMsg.pose.position.x = pathpoint.x();
            poseMsg.pose.position.y = pathpoint.y();
            msg.poses.push_back(poseMsg);
        }
    }

}

int main(int argc, char **argv)
{
    std::string name("global_planner");

    ros::init(argc, argv, name);                   // set name of the node
    ros::NodeHandle node_handle(name);             // set namespace of the node through handle. ex: node_name/topic_name
    grid_map::GlobalPlannerNode node(node_handle); // pass handle
    node.run();

    return 0;
}