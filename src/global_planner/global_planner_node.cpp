#include <gridmap_navigation/global_planner_node.h>

#define duration_ms(a) std::chrono::duration_cast<std::chrono::milliseconds>(a).count()
#define duration_us(a) std::chrono::duration_cast<std::chrono::microseconds>(a).count()
typedef std::chrono::high_resolution_clock clk;

namespace grid_map
{
    GlobalPlannerNode::GlobalPlannerNode(ros::NodeHandle &_nh)
        : nh(_nh),
          tf2_listener(tf2_buffer),
          valid_cost_(false)
    {
        useParameterServer();

        if (use_global_map)
        {
            ROS_INFO("Path Planning With Global Map!!");

            ros::ServiceClient client;
            nav_msgs::GetMap service;
            client = nh.serviceClient<nav_msgs::GetMap>("/static_map");
            bool service_responsed = false;
            clk::time_point start_time = clk::now();
            while (!service_responsed && duration_ms(clk::now() - start_time) < 5000) // 5sec
            {
                ROS_INFO_THROTTLE(1, "use Global map:: Wait until Global map is valid");
                service_responsed = client.call(service);
            }

            if (!service_responsed)
            {
                ROS_ERROR("use Global map:: Global map not received... Node Shutdown");
                nh.shutdown();
                return;
            }

            ROS_INFO("use Global map:: Global map received.");
            costmapPtr_ = std::make_unique<Costmap>(service.response.map, param_inflation_size);
            costmapPtr_->setTimeLimit(param_timelimit_ms);
        }

        else if (!use_global_map)
        {
            ROS_INFO("Path Planning Without Global Map!!");
            costmapPtr_ = std::make_unique<Costmap>();
            costmapPtr_->setTimeLimit(param_timelimit_ms);
        }

        // sub & pub
        sub_goal = nh.subscribe("/move_base_simple/goal", 10, &GlobalPlannerNode::goalCallback, this);
        pub_path = nh.advertise<nav_msgs::Path>(pubtopic_path, 10);
        pub_map = nh.advertise<grid_map_msgs::GridMap>(pubtopic_map, 1);

        ROS_INFO_STREAM("Global Planner Node Initialized. Wait for the topic " << sub_goal.getTopic());
    }

    void GlobalPlannerNode::process()
    {
        ros::Rate loop_rate(param_Hz);
        while (ros::ok())
        {
            if (!valid_cost_)
            {
                ros::spinOnce();
                loop_rate.sleep();
                continue;
            }

            updateRobotPosition(ros::Time::now());
            std::vector<Position> path;
            if (!costmapPtr_->findPath(robot_position_, goal_position_, path))
            {
                ROS_WARN("[Find Path] No Valid Path Found from the Current Position. Return empty Path");
                // clear path in here?
            }

            smoothing(path, 10);

            // Do something to create bubble with costmap, local costmap, and 
            // 1. filter path points in close range (local area : suppose 5 meter)
            // 2. With local grid map, for all points, search with spiral iterator and find distance from closest obstacle
            // 3. move bubble position --> How? ex. move away with the direction vector (point - obstacle), then sort with distance from the robot
            //     1) movable: calculated new position is on the valid cost map >> ok
            //     2) not movable: calculated new position is out of the valid cost map >> do not publish local paths
            // 4. recalculate bubble size with occupancy map

            nav_msgs::Path msg;
            toRosMsg(path, msg);
            pub_path.publish(msg);

            ros::spinOnce();
            loop_rate.sleep();
        }
    }

    void GlobalPlannerNode::goalCallback(const geometry_msgs::PoseStampedConstPtr &msg)
    {
        valid_cost_ = false;
        updateGoalPosition(msg->pose);
        updateRobotPosition(msg->header.stamp);

        clk::time_point start_point = clk::now();
        if (!costmapPtr_->update(robot_position_, goal_position_))
        {
            ROS_WARN("[Goal Callback] Stopped Glboal Path Topic");
            return;
        }
        valid_cost_ = true;

        ROS_INFO_STREAM("Update Costmap takes " << duration_ms(clk::now() - start_point) << "ms");
        publishCostmap();
    }

    void GlobalPlannerNode::useParameterServer()
    {
        nh.param<std::string>("globalPathTopic", pubtopic_path, "path");
        nh.param<std::string>("gridMapTopic", pubtopic_map, "map");
        nh.param<bool>("useGlobalMap", use_global_map, true);

        nh.param<int>("inflationGridSize", param_inflation_size, 3);
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

    void GlobalPlannerNode::updateRobotPosition(const ros::Time &time)
    {
        geometry_msgs::TransformStamped localization_pose;
        try
        {
            localization_pose = tf2_buffer.lookupTransform("map", "base_link", time, ros::Duration(1.0));
        }
        catch (tf2::TransformException &ex)
        {
            ROS_ERROR("%s", ex.what());
            return;
        }

        robot_position_(0) = localization_pose.transform.translation.x;
        robot_position_(1) = localization_pose.transform.translation.y;
    }

    void GlobalPlannerNode::publishCostmap()
    {
        
        grid_map_msgs::GridMap msg;
        GridMapRosConverter::toMessage(*costmapPtr_, msg);
        msg.info.header.stamp = ros::Time::now();
        pub_map.publish(msg);
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

    ros::Duration(1.0).sleep(); // Need this to get the TF caches fill up.
    node.process();

    return 0;
}