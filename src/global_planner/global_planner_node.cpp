#include <gridmap_navigation/global_planner_node.h>

#include <memory>

#define duration_ms(a) std::chrono::duration_cast<std::chrono::milliseconds>(a).count()
typedef std::chrono::high_resolution_clock clk;

namespace grid_map
{
    GlobalPlannerNode::GlobalPlannerNode(ros::NodeHandle &_nh)
        : nh(_nh),
          tf2_listener(tf2_buffer),
          costmap_initialized_(false)
    {
        useParameterServer();

        if (use_global_map)
        {
            ros::ServiceClient client;
            nav_msgs::GetMap service;
            client = nh.serviceClient<nav_msgs::GetMap>("/static_map");
            bool service_responsed = false;
            clk::time_point start_time = clk::now();
            while (!service_responsed && duration_ms(clk::now() - start_time) < 5000) // 5sec
            {
                ROS_INFO("use Global map:: Wait until Global map is valid");
                service_responsed = client.call(service);
                ros::Duration(1.0).sleep();
            }

            if (!service_responsed)
            {
                ROS_ERROR("use Global map:: Global map not received... Node Shutdown");
                nh.shutdown();
                return;
            }

            ROS_INFO("use Global map:: Global map received.");
            costmapPtr_ = std::make_unique<Costmap>(service.response.map, 3);
        }

        else if (!use_global_map)
        {
            costmapPtr_ = std::make_unique<Costmap>();
            costmapPtr_->get("occupancy").setConstant(0);
        }

        // sub & pub
        sub_goal = nh.subscribe("/move_base_simple/goal", 10, &GlobalPlannerNode::goalCallback, this);
        pub_path = nh.advertise<nav_msgs::Path>(pubtopic_path, 10);
        pub_map = nh.advertise<grid_map_msgs::GridMap>(pubtopic_map, 1);

        ROS_INFO_STREAM("Global Planner Node Initialized. Wait for the topic " << sub_goal.getTopic());
    }

    void GlobalPlannerNode::useParameterServer()
    {
        nh.param<std::string>("globalPathTopic", pubtopic_path, "/global_path");
        nh.param<std::string>("gridMapTopic", pubtopic_map, "/gridmap");
        nh.param<bool>("useGlobalMap", use_global_map, false);

        nh.param<int>("inflationGridSize", param_inflation_size, 3);
        nh.param<int>("frequency", param_Hz, 10);
    }

    void GlobalPlannerNode::goalCallback(const geometry_msgs::PoseStampedConstPtr &msg)
    {
        const int TIME_LIMIT_MS = 2000;

        updateGoalPosition(msg->pose);
        updateRobotPosition(msg->header.stamp);
        publishCostmap();

        clk::time_point start_point = clk::now();
        while (!costmapPtr_->update(robot_position_, goal_position_))
        {
            if (duration_ms(clk::now() - start_point) > TIME_LIMIT_MS)
            {
                ROS_WARN("GRADIENT TIMEOUT! Update Costmap failed");
                return;
            }
        }
        if (!costmap_initialized_)
            costmap_initialized_ = true;

        ROS_INFO_STREAM(duration_ms(clk::now() - start_point));

        publishCostmap();
    }

    void GlobalPlannerNode::updateGoalPosition(const geometry_msgs::Pose &goal)
    {
        goal_position_(0) = goal.position.x;
        goal_position_(1) = goal.position.y;
        ROS_INFO_STREAM("Updated Goal Position: " << goal_position_(0) << " " << goal_position_(1));
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

    void GlobalPlannerNode::process()
    {
        ros::Rate loop_rate(param_Hz);
        while (ros::ok())
        {
            if (!costmap_initialized_)
            {
                ROS_INFO_THROTTLE(1, "Cost Map needs to be Initialized. Set proper Goal First.");
                ros::spinOnce();
                loop_rate.sleep();
                continue;
            }

            updateRobotPosition(ros::Time::now());

            std::vector<Position> path;
            if (!costmapPtr_->findPath(robot_position_, goal_position_, path))
            {
                ROS_WARN("No Valid Path Found from the Cost Map. Return empty Path");
                // do we need clear path in here?
            }
            ROS_INFO("test");

            ros::spinOnce();
            loop_rate.sleep();
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