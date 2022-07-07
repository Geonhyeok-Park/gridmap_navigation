#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <laser_geometry/laser_geometry.h>

sensor_msgs::PointCloud2 laser_cloud;


void callback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    laser_geometry::LaserProjection scan_to_cloud;
    scan_to_cloud.projectLaser(*msg, laser_cloud, -1, laser_geometry::channel_option::Intensity | laser_geometry::channel_option::Distance);
}

int main(int argc, char **argv)
{
    std::string name("laser_converter");

    ros::init(argc, argv, name);                   // set name of the node
    static ros::NodeHandle node_handle(name);             // set namespace of the node through handle. ex: node_name/topic_name
    
    ros::Subscriber sub_laser = node_handle.subscribe("/tim581_front/scan", 10, callback);
    ros::Publisher pub_cloud = node_handle.advertise<sensor_msgs::PointCloud2>("/laser_cloud", 1);

    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        ros::spinOnce();
        pub_cloud.publish(laser_cloud);
        loop_rate.sleep();
    }
    
    return 0;
}
