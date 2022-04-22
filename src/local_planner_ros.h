//
// Created by isr on 22. 4. 21.
//

#ifndef GRIDMAP_NAVIGATION_LOCAL_PLANNER_ROS_H
#define GRIDMAP_NAVIGATION_LOCAL_PLANNER_ROS_H

#include <gridmap_navigation/tf_manager.h>
#include <laser_geometry/laser_geometry.h>
#include <gridmap_navigation/map_converter_ros.h>
#include "dynamic_window_approach.h"

// ROS msgs
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>

// pcl
#include <pcl/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

class LocalPlannerRos
{
    Robot robot_;
    grid_map::GridMap map_;
    grid_map::Position goal_;

public:
    LocalPlannerRos();
    ~LocalPlannerRos() { nh.shutdown(); }

    void loadParamServer();

    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void velocityCallback(const geometry_msgs::TwistConstPtr &msg);

    void velocityToRosMsg(const Velocity &vel, geometry_msgs::Twist &msg);
    void setRobotParams();
    void setRobotState(const geometry_msgs::Twist &msg);

private:
    ros::NodeHandle nh;

    ros::Publisher pub_vel;

    ros::Subscriber sub_goal;
    ros::Subscriber sub_vel;
    ros::Subscriber sub_localmap;

    std::string topic_vel_sub;
    std::string topic_vel_pub;
    std::string topic_localmap_sub;
    std::string topic_goal_sub;

private:
    TFManagerRos tf_;
    laser_geometry::LaserProjection scan2cloud_;
    MapConverterRos map_converter_;

private:
    // control flags
    bool recieved_goal_;
    bool recieved_vel_;
    bool recieved_obstacle_;

    // robot parameters
    double robot_radius_;
    double robot_tread_length_;
    double robot_max_acc_;
    double robot_max_yawrate_;

    // time
    double sim_time_;
    double time_interval_;
};


void LocalPlannerRos::velocityToRosMsg(const Velocity &vel, geometry_msgs::Twist &msg)
{
    double linear_vel = vel.x();
    double angular_vel = vel.y();

    msg.linear.x = linear_vel;
    msg.linear.y = 0;
    msg.linear.z = 0;
    msg.angular.x = 0;
    msg.angular.y = 0;
    msg.angular.z = angular_vel;
}

void LocalPlannerRos::setRobotParams()
{
    robot_.setGeometry("base_link", robot_radius_, robot_tread_length_);
    robot_.setConstraints(robot_max_acc_, robot_max_yawrate_);
}

void LocalPlannerRos::setRobotState(const geometry_msgs::Twist &msg)
{
    tf_.getTF(ros::Time::now());
    
    Pose current_pose;
    current_pose.setPosition(grid_map::Position(tf_.BaseToMap.translation.x, tf_.BaseToMap.translation.y));
    current_pose.setYaw(tf::getYaw(tf_.BaseToMap.rotation));
    robot_.setPose(current_pose);

    Velocity current_vel(msg.linear.x, msg.angular.z);
    robot_.setVelocity(current_vel);
}

#endif // GRIDMAP_NAVIGATION_LOCAL_PLANNER_ROS_H
