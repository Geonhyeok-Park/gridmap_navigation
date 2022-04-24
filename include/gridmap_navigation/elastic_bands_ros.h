//
// Created by Ikhyeon Cho on 22. 4. 8..
//
#ifndef GRIDMAP_NAVIGATION_ELASTICBANDS_ROS_H
#define GRIDMAP_NAVIGATION_ELASTICBANDS_ROS_H

#include <gridmap_navigation/elastic_bands.h>

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Path.h>

class ElasticBandsRosConverter
{
private:
public:
    ElasticBandsRosConverter() = default;
    ~ElasticBandsRosConverter() = default;

    void toROSMsg(const ElasticBands &data, visualization_msgs::MarkerArray &bubble_msg, nav_msgs::Path &path_msg)
    {
        bubble_msg.markers.clear();
        path_msg.poses.clear();

        const auto &eband = data.getBubbles();

        bubble_msg.markers.resize(eband.size());
        path_msg.poses.resize(eband.size());

        path_msg.header.frame_id = data.getFrameId();
        path_msg.header.seq++;
        path_msg.header.stamp = ros::Time::now();

        for (int i = 0; i < eband.size(); ++i)
        {
            visualization_msgs::Marker bubble;

            // shape and header
            bubble.type = visualization_msgs::Marker::SPHERE;
            bubble.action = visualization_msgs::Marker::ADD;
            bubble.header.frame_id = data.getFrameId();
            bubble.header.stamp = ros::Time::now();
            bubble.ns = "eband";
            bubble.id = i;

            path_msg.poses.at(i).header.frame_id = data.getFrameId();
            path_msg.poses.at(i).header.stamp = ros::Time::now();

            // position
            bubble.pose.position.x = eband.at(i).getPosition().x();
            bubble.pose.position.y = eband.at(i).getPosition().y();
            bubble.pose.position.z = 0;
            bubble.pose.orientation.x = 0;
            bubble.pose.orientation.y = 0;
            bubble.pose.orientation.z = 0;
            bubble.pose.orientation.w = 1;

            path_msg.poses.at(i).pose = bubble.pose;

            // color
            bubble.color.a = 0.3;
            bubble.color.r = 0.0;
            bubble.color.g = 0.5;
            bubble.color.b = 0.0;

            // size
            bubble.scale.x = eband.at(i).getRadius() * 2.0;
            bubble.scale.y = eband.at(i).getRadius() * 2.0;
            bubble.scale.z = 0.05;

            bubble.lifetime = ros::Duration(0.5);

            bubble_msg.markers.at(i) = bubble;
        }
    }
};

#endif // GRIDMAP_NAVIGATION_ELASTICBANDS_ROS_H