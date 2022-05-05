#include <ros/ros.h>
#include "local_planner/local_planner_ros.h"

using namespace grid_map;



// LocalPlannerNode::LocalPlannerNode()
//     : nh("local_planner"),
//       map_({"obstacle_local"}),
//       map_converter_(nh),
//       recieved_goal_(false),
//       recieved_vel_(false),
//       recieved_obstacle_(false),
//       time_interval_(0.1)
// {
//     registerNodeParams();

//     while (!tf_.getStaticTF("tim581_front"))
//         ros::Duration(1.0).sleep();

//     setRobotParams();

//     map_converter_.setSubTopic(subtopic_localmap);

//     sub_goal = nh.subscribe(subtopic_goal, 10, &LocalPlannerNode::goalCallback, this);
//     sub_cmdvel = nh.subscribe(subtopic_cmdvel, 10, &LocalPlannerNode::velocityCallback, this);

//     pub_cmdvel = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
// }

void LocalPlannerNode::goalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    ROS_INFO("goal recieved");

    if (!recieved_vel_)
    {
        geometry_msgs::Twist msg_vel;
        Velocity zeroVel(0, 0);
        velocityToRosMsg(zeroVel, msg_vel);
        pub_cmdvel.publish(msg_vel);
    }
    goal_ = Position(msg->pose.position.x, msg->pose.position.y);
    recieved_goal_ = true;
}

// main thread
void LocalPlannerNode::velocityCallback(const geometry_msgs::TwistConstPtr &msg)
{
    ROS_INFO("command velocity recieved");

    if (!map_converter_.convertOccupancyToGridmapLayer(map_, "obstacle_local"))
        return;

    if (!recieved_goal_)
        return;

    setRobotState(*msg);
    DWA dwa(robot_, goal_, map_, time_interval_);
    dwa.planning(sim_time_);
    recieved_vel_ = true;
}



