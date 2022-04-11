#include <gridmap-navigation/elasticBands.hpp>

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Path.h>

class ElasticBandsRosConverter
{
private:
    visualization_msgs::MarkerArray bubbleMsg_;
    nav_msgs::Path pathMsg_;

public:
    ElasticBandsRosConverter() = default;
    ~ElasticBandsRosConverter() = default;

    void toROSMsg(const ElasticBands &data, visualization_msgs::MarkerArray &bubbleMsg, nav_msgs::Path &pathMsg)
    {
        const auto& eband = data.getEbands();

        bubbleMsg_.markers.resize(eband.size());
        pathMsg_.poses.resize(eband.size());

        pathMsg_.header.frame_id = data.getMapFrame();
        pathMsg_.header.seq++;
        pathMsg_.header.stamp = ros::Time::now();

        for (int i = 0; i < eband.size(); ++i)
        {
            visualization_msgs::Marker bubble;

            // shape and header
            bubble.type = visualization_msgs::Marker::SPHERE;
            bubble.action = visualization_msgs::Marker::ADD;
            bubble.header.frame_id = data.getMapFrame();
            bubble.header.stamp = ros::Time::now();
            bubble.ns = "eband";
            bubble.id = i;

            pathMsg_.poses.at(i).header.frame_id = data.getMapFrame();
            pathMsg_.poses.at(i).header.stamp = ros::Time::now();

            // position
            bubble.pose.position.x = eband.at(i).getPosition().x();
            bubble.pose.position.y = eband.at(i).getPosition().y();
            bubble.pose.position.z = 0;
            bubble.pose.orientation.x = 0;
            bubble.pose.orientation.y = 0;
            bubble.pose.orientation.z = 0;
            bubble.pose.orientation.w = 1;

            pathMsg_.poses.at(i).pose = bubble.pose;

            // color
            bubble.color.a = 0.3;
            bubble.color.r = 0.0;
            bubble.color.g = 0.5;
            bubble.color.b = 0.0;

            // size
            bubble.scale.x = eband.at(i).getRadius() * 2.0;
            bubble.scale.y = eband.at(i).getRadius() * 2.0;
            bubble.scale.z = 0.05;

            bubble.lifetime = ros::Duration(0.1);

            bubbleMsg_.markers.at(i) = bubble;
        }
    }
};