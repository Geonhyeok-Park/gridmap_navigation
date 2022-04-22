#include <ros/ros.h>
#include "local_planner_ros.h"

using namespace grid_map;

LocalPlannerRos::LocalPlannerRos()
    : nh("local_planner"),
      map_({"obstacle_local"}),
      map_converter_(nh),
      recieved_goal_(false),
      recieved_vel_(false),
      recieved_obstacle_(false),
      time_interval_(0.1)
{
    loadParamServer();

    while (!tf_.getStaticTF("tim581_front"))
        ros::Duration(1.0).sleep();

    setRobotParams();

    map_converter_.setSubTopic(topic_localmap_sub);

    sub_goal = nh.subscribe(topic_goal_sub, 10, &LocalPlannerRos::goalCallback, this);
    sub_vel = nh.subscribe(topic_vel_sub, 10, &LocalPlannerRos::velocityCallback, this);

    pub_vel = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
}

void LocalPlannerRos::goalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    ROS_INFO("goal recieved");

    if (!recieved_vel_)
    {
        geometry_msgs::Twist msg_vel;
        Velocity zeroVel(0, 0);
        velocityToRosMsg(zeroVel, msg_vel);
        pub_vel.publish(msg_vel);
    }
    goal_ = Position(msg->pose.position.x, msg->pose.position.y);
    recieved_goal_ = true;
}

// main thread
void LocalPlannerRos::velocityCallback(const geometry_msgs::TwistConstPtr &msg)
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

void LocalPlannerRos::loadParamServer()
{
    nh.param<std::string>("local_obstacle", topic_localmap_sub, "/traversability/map");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "local_planner");

    LocalPlannerRos node;

    ros::Duration(1.0).sleep(); // Need this to get the TF caches fill up.

    ros::spin();

    return 0;
}
