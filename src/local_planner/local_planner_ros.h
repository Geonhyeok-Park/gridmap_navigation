//
// Created by isr on 22. 4. 21.
//

#ifndef GRIDMAP_NAVIGATION_LOCAL_PLANNER_ROS_H
#define GRIDMAP_NAVIGATION_LOCAL_PLANNER_ROS_H

#include <gridmap_navigation/tf_manager.h>
#include <laser_geometry/laser_geometry.h>
#include <gridmap_navigation/map_converter_ros.h>
#include "dwa.h"

// ROS msgs
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/MarkerArray.h>

// pcl
#include <pcl/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

class LocalPlannerNode
{
    using Position = Eigen::Vector2d;
    using Velocity = Eigen::Vector2d;
    using Pose = Eigen::Vector3d;

    // needs initializer
private:
    ros::NodeHandle &nh;
    // MapConverterRos map_converter_;

    // ros interface
private:
    ros::Publisher pub_cmdvel;
    std::string pubtopic_cmdvel;

    ros::Subscriber sub_cmdvel;
    std::string subtopic_cmdvel;

    ros::Subscriber sub_localmap;
    std::string subtopic_localmap;

    ros::Subscriber sub_localgoal;
    std::string subtopic_eband;

    // parameters
private:
    ROBOT robot_;
    double param_tread;
    double param_max_linear_vel;
    double param_max_angular_vel;
    double param_max_wheel_accel;
    double param_forward_simtime;

    DWA window_;
    double param_linear_vel_res;
    double param_angular_vel_res;
    double param_controltime;

    double param_goal_distance;

    // grid_map::GridMap map_;
    // grid_map::Position goal_;

private:
    TFManagerRos tf_;
    laser_geometry::LaserProjection scan2cloud_;

private:
    void velocityToRosMsg(const Velocity &, geometry_msgs::Twist &);
    double getDistance(Position a, Position b) { return (a - b).norm(); }

public:
    LocalPlannerNode(ros::NodeHandle &);
    ~LocalPlannerNode() { nh.shutdown(); }
    void registerNodeParams();

    bool recieved_cmdvel_;
    void velocityCallback(const geometry_msgs::TwistConstPtr &);

    bool recieved_localgoal_;
    void localgoalCallback(const geometry_msgs::PoseStamped::ConstPtr &);

    bool recieved_localmap_;
    void localmapCallback(const grid_map_msgs::GridMapConstPtr &);
};

void LocalPlannerNode::velocityToRosMsg(const Velocity &vel, geometry_msgs::Twist &msg)
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

// void LocalPlannerNode::setRobotState(const geometry_msgs::Twist &msg)
// {
//     tf_.getTF(ros::Time::now());

//     Pose current_pose;
//     current_pose.setPosition(grid_map::Position(tf_.BaseToMap.translation.x, tf_.BaseToMap.translation.y));
//     current_pose.setYaw(tf::getYaw(tf_.BaseToMap.rotation));
//     robot_.setPose(current_pose);

//     Velocity current_vel(msg.linear.x, msg.angular.z);
//     robot_.setVelocity(current_vel);
// }

#endif // GRIDMAP_NAVIGATION_LOCAL_PLANNER_ROS_H
