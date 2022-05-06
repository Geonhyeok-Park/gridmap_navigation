//
// Created by isr on 22. 4. 21.
//

#include "local_planner_ros.h"

LocalPlannerNode::LocalPlannerNode(ros::NodeHandle &_nh)
    : nh(_nh),
      map_converter_(nh)
{
    registerNodeParams();

    window_.setControlCycleTime(0.2);
    window_.setMaxVelocity(linvel_max_, angvel_max_);
    window_.setMaxWheelAccel(0.5);
    window_.setWindowResolution();

    sub_goal = nh.subscribe("/move_base_simple/goal", 10, &LocalPlannerNode::goalCallback, this);
    sub_cmdvel = nh.subscribe("/cmd_vel", 10, &LocalPlannerNode::velocityCallback, this);
    sub_eband = nh.subscribe(subtopic_eband, 10, &LocalPlannerNode::ebandCallback, this);
    sub_localmap = nh.subscribe(subtopic_localmap, 10, &LocalPlannerNode::localmapCallback, this);

    pub_cmdvel = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
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
}
