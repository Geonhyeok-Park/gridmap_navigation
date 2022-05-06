//
// Created by Ikhyeon Cho on 22. 4. 21.
//

#include "local_planner_ros.h"

LocalPlannerNode::LocalPlannerNode(ros::NodeHandle &_nh)
    : nh(_nh),
      recieved_cmdvel_(false),
      recieved_localgoal_(false),
      recieved_localmap_(false)
//   map_converter_(nh)
{
    registerNodeParams();

    // robot setup
    robot_.setTread(param_tread);
    robot_.setSimulationTime(param_forward_simtime);
    robot_.setMaxCmdvel(param_max_linear_vel, param_max_angular_vel);
    robot_.setMaxAccel(param_max_wheel_accel);

    // robot initial state
    robot_.setPose(0, 0, 0);
    robot_.setVel(0, 0);

    // window setup
    window_.setRobot(robot_);
    window_.setWindowResolution(Velocity(param_linear_vel_res, param_angular_vel_res));
    window_.setControlTimeInSec(param_controltime);

    window_.initialize();

    sub_cmdvel = nh.subscribe("/cmd_vel", 1, &LocalPlannerNode::velocityCallback, this);
    sub_localgoal = nh.subscribe(subtopic_eband, 1, &LocalPlannerNode::localgoalCallback, this);
    sub_localmap = nh.subscribe(subtopic_localmap, 1, &LocalPlannerNode::localmapCallback, this);

    pub_cmdvel = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
}

void LocalPlannerNode::velocityCallback(const geometry_msgs::TwistConstPtr &msg)
{
    recieved_cmdvel_ = true;
    window_.setVelocity(Velocity(msg->linear.x, msg->angular.z));
}

void LocalPlannerNode::localmapCallback(const grid_map_msgs::GridMapConstPtr &msg)
{
    recieved_localmap_ = true;
}

void LocalPlannerNode::localgoalCallback(const geometry_msgs::PoseStampedConstPtr &msg)
{
    recieved_localgoal_ = true;
    bool can_control = (recieved_cmdvel_ && recieved_localmap_);
    if (!can_control)
        return;

    window_.setGoal(Pose(msg->pose.position.x, msg->pose.position.y, tf::getYaw(msg->pose.orientation)));
    window_.updateCost();

    // publish cmd vel
}

int main(int argc, char **argv)
{
    std::string name("local_planner");

    ros::init(argc, argv, name);        // set name of the node
    ros::NodeHandle node_handle(name);  // set namespace of the node through handle. ex: node_name/topic_name
    LocalPlannerNode node(node_handle); // pass handle

    ros::Duration(1.0).sleep(); // Need this to get the TF caches fill up.
    ros::spin();

    return 0;
}

void LocalPlannerNode::registerNodeParams()
{
    nh.param<std::string>("elasticBand", subtopic_eband, "/elastic_bands");
    nh.param<std::string>("localMap", subtopic_localmap, "/traversability/map");

    // robot
    nh.param<double>("robotTread", param_tread, 0.58);
    nh.param<double>("maxLinearVelocity", param_max_linear_vel, 0.5);
    nh.param<double>("maxAngularVelocity", param_max_angular_vel, 0.7854);
    nh.param<double>("maxWheelAcceleration", param_max_wheel_accel, 0.5);
    nh.param<double>("forwardSimulationTime", param_forward_simtime, 1.0);

    // window
    nh.param<double>("linearVelocityRes", param_linear_vel_res, 0.05);
    nh.param<double>("angularVelocityRes", param_angular_vel_res, 0.035);
    nh.param<double>("controlCycleTime", param_controltime, 0.2);
}
