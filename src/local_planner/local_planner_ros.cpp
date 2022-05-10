//
// Created by Ikhyeon Cho on 22. 4. 21.
//

#include "local_planner_ros.h"

using namespace grid_map;

LocalPlannerNode::LocalPlannerNode(ros::NodeHandle &_nh)
    : nh(_nh),
      recieved_cmdvel_(false),
      recieved_localgoal_(false),
      recieved_localmap_(false)
//   map_converter_(nh)
{
    registerNodeParams();

    // robot setup
    ROBOT robot;
    robot.setTread(param_tread);
    robot.setMaxVel(Velocity(param_max_linear_vel, param_max_angular_vel));
    robot.setMaxAccel(param_max_wheel_accel);
    robot.setSimulationTime(param_forward_simtime, param_controltime);

    // robot initial state
    robot.setPose(0, 0, 0);

    // window setup
    window_.setRobot(robot);
    window_.setWindowResolution(Velocity(param_linear_vel_res, param_angular_vel_res));
    window_.initialize();

    // robot alias
    *pRobot_ = window_.getRobot();

    sub_cmdvel = nh.subscribe("/cmd_vel", 1, &LocalPlannerNode::velocityCallback, this);
    sub_localgoal = nh.subscribe(subtopic_eband, 1, &LocalPlannerNode::localgoalCallback, this);
    sub_localmap = nh.subscribe(subtopic_localmap, 1, &LocalPlannerNode::localmapCallback, this);

    pub_cmdvel = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
}

void LocalPlannerNode::velocityCallback(const geometry_msgs::TwistConstPtr &msg)
{
    window_.updateVelocity(Velocity(msg->linear.x, msg->angular.z));
    recieved_cmdvel_ = true;
}

void LocalPlannerNode::localmapCallback(const grid_map_msgs::GridMapConstPtr &msg)
{
    if (!recieved_cmdvel_)
        return;

    // update local map
    GridMapRosConverter::fromMessage(*msg, localmap_);
    recieved_localmap_ = true;

    // update robot pose
    tf_.getTF(msg->info.header.stamp);
    pRobot_->setPose(tf_.BaseToMap.translation.x, tf_.BaseToMap.translation.y, tf::getYaw(tf_.BaseToMap.rotation));

    // forward sim: update trajectory
    for (int i = 0; i < window_.getSize(); ++i)
    {
        for (int j = 0; j < window_.getSize(); ++j)
        {
            const auto &vl = window_.access(i, j).vl;
            const auto &vr = window_.access(i, j).vr;
            sim_trajectory_ = pRobot_->forwardSimulation(Velocity(vl, vr));
            for (const auto &pose : sim_trajectory_)
            {
                checkCollision(localmap_, "label", pose, pRobot_->getRadius());
            }
        }
    }
}

void LocalPlannerNode::localgoalCallback(const geometry_msgs::PoseStampedConstPtr &msg)
{
    bool can_control = (recieved_cmdvel_ && recieved_localmap_);
    if (!can_control)
        return;

    pRobot_->setGoal(Pose(msg->pose.position.x, msg->pose.position.y, tf::getYaw(msg->pose.orientation)));
    recieved_localgoal_ = true;

    window_.updateCost();

    // publish cmd vel
}

double LocalPlannerNode::checkCollision(const GridMap &map, const std::string &layer,
                                        Pose pose, double search_radius)
{
    auto position = pose.head(2);
    Index grid_index;
    if(!map.getIndex(position, grid_index))
        return search_radius;
    
    float occupied = 100;
    // if ((map.at(layer, grid_index) - occupied) < DBL_EPSILON)
    //     return 0;

    SpiralIterator iter(map, pose.head(2), search_radius);
    for(iter; !iter.isPastEnd(); ++iter)
    {
        // check occupied or not
    }
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
