#include <gridmap_navigation/global_planner_node.h>
#include <grid_map_ros/GridMapRosConverter.hpp>

#define duration_ms(a) std::chrono::duration_cast<std::chrono::milliseconds>(a).count()
typedef std::chrono::high_resolution_clock clk;

namespace grid_map
{
    GlobalPlannerNode::GlobalPlannerNode(ros::NodeHandle &_nh)
        : nh(_nh),
          tf2_listener(tf2_buffer),
          costmap_(),
          costmap_initialized_(false)
    {
        useParameterServer();

        if (use_global_map)
        {
            ros::ServiceClient client;
            nav_msgs::GetMap service;
            bool has_map = false;
            clk::time_point start_time = clk::now();
            while (!has_map && duration_ms(clk::now() - start_time) < 5000) // 5sec
            {
                ROS_INFO("use Global map:: Wait until Global map is valid");
                has_map = client.call(service);
            }

            if (!has_map)
            {
                ROS_ERROR("use Global map:: Global map not received... Node Shutdown");
                nh.shutdown();
                return;
            }
            
            ROS_INFO("use Global map:: Global map received.");
            costmapPtr_.reset(new Costmap(service.response.map, 3));
        }

        else if (!use_global_map)
        {
            costmap_.get("occupancy").setConstant(0);
        }

        // sub & pub
        sub_goal = nh.subscribe("/move_base_simple/goal", 10, &GlobalPlannerNode::goalCallback, this);
        pub_path = nh.advertise<nav_msgs::Path>(pubtopic_path, 10);

        ROS_INFO_STREAM("Global Planner Node Initialized. Wait for the topic " << sub_goal.getTopic());
    }

    void GlobalPlannerNode::useParameterServer()
    {
        nh.param<std::string>("test", pubtopic_path, "test");
        nh.param<bool>("usePrebuiltMap", use_global_map, false);
        nh.param<bool>("useInflation", use_inflation, true);

        nh.param<int>("inflationGridSize", param_inflation_size, 3);
        nh.param<float>("goalAcceptanceDistance", param_goal_acceptance, 0.2);
        nh.param<int>("frequency", param_Hz, 10);
    }

    void GlobalPlannerNode::goalCallback(const geometry_msgs::PoseStampedConstPtr &msg)
    {
        const int TIME_LIMIT_MS = 1000;

        updateGoalPosition(msg->pose);
        updateRobotPosition(msg->header.stamp);

        // global planner search
        clk::time_point start_point = clk::now();
        auto search_radius = Costmap::getDistance(robot_position_, goal_position_);
        while (!costmapPtr_->update(robot_position_, goal_position_))
        {
            search_radius += Costmap::getDistance(robot_position_, goal_position_);
            clk::time_point check_point = clk::now();
            if (duration_ms(check_point - start_point) > TIME_LIMIT_MS)
            {
                ROS_WARN("GRADIENT TIMEOUT! Update Costmap failed");
                return;
            }
        }
        if (!costmap_initialized_)
            costmap_initialized_ = true;

        ROS_INFO_STREAM(duration_ms(clk::now() - start_point));

        ROS_INFO_STREAM(duration_ms(clk::now() - start_point));

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

        ROS_INFO_STREAM("Updated Robot Position: " << robot_position_(0) << " " << robot_position_(1));
    }

    void GlobalPlannerNode::process()
    {
        ros::Rate loop_rate(param_Hz);

        while (ros::ok())
        {
            clk::time_point start_point = clk::now();
            if (costmap_initialized_)
            {
                updateRobotPosition(ros::Time::now());

                std::vector<Position> path;
                if (!costmapPtr_->findPath(robot_position_, goal_position_, path))
                {
                    ROS_WARN("No Valid Path Found from the Cost Map. Return empty Path");
                    // do we need clear path in here?
                }
                ROS_INFO("test");
                // end of process
            }
            else
            {
                ROS_INFO("Goal not initlized. Set goal before the process");
            }

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