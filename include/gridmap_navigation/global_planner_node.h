// ROS tools
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>

// Grid Map Library
#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>

// ROS msgs
#include <nav_msgs/GetMap.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseStamped.h>

#include <gridmap_navigation/costmap_ros.h>
#include <gridmap_navigation/eband_ros.h>

namespace grid_map
{
    class GlobalPlannerNode
    {
    public: // Node Parameters
        std::string param_pub_path;
        std::string param_pub_localpath;
        std::string param_pub_map;
        std::string param_pub_eband;
        std::string param_sub_localmap;
        bool param_use_globalmap;
        int param_inflation_size;
        int param_Hz;
        int param_timelimit_ms;

    public: // ROS Things
        ros::NodeHandle &nh;
        ros::Subscriber sub_goal;
        ros::Subscriber sub_localmap;
        ros::Publisher pub_path;
        ros::Publisher pub_localpath;
        ros::Publisher pub_ebandmarker;
        ros::Publisher pub_map;

        tf2_ros::Buffer tf2_buffer;
        tf2_ros::TransformListener tf2_listener;

    private:
        std::unique_ptr<Costmap> costmapPtr_;
        std::vector<Position> path_;
        Position goal_position_;
        Position robot_position_;

        // Node Status
        bool costmap_updated_;
        bool path_updated_;

    public:
        GlobalPlannerNode(ros::NodeHandle &);
        ~GlobalPlannerNode() { nh.shutdown(); }
        void run();

    private:
        void useParameterServer();
        void goalCallback(const geometry_msgs::PoseStamped::ConstPtr &);
        void localmapCallback(const grid_map_msgs::GridMapConstPtr &);

        void updateGoalPosition(const geometry_msgs::Pose &goal);
        bool updateRobotPosition(const ros::Time &time);

        void publishCostmap();
        void toRosMsg(const std::vector<Position> &path, nav_msgs::Path &msg);
    };

}

void smoothing(std::vector<grid_map::Position> &path, int neighbor)
{
    std::vector<grid_map::Position> path_copy = path;
    for (int i = neighbor; i < path.size() - neighbor; i++)
    {
        auto &waypoint_center = path.at(i);
        auto num_neighbors = 2 * neighbor + 1;
        for (int j = -neighbor; j <= neighbor; j++)
        {
            if (j == 0)
                continue;
            waypoint_center += path_copy.at(i + j);
        }
        waypoint_center(0) /= num_neighbors;
        waypoint_center(1) /= num_neighbors;
    }
}

void smoothing_eband(const grid_map::ElasticBand &eband, int neighbor, std::vector<grid_map::Position> &path)
{
    for (int i = neighbor; i < eband.getEband().size() - neighbor; ++i)
    {
        auto waypoint_center = eband.getEband().at(i).getPosition();
        auto num_neighbors = 2 * neighbor + 1;
        for (int j = -neighbor; j <= neighbor; j++)
        {
            if (j ==0)
                continue;
            waypoint_center += eband.getEband().at(i + j).getPosition();
        }
        waypoint_center(0) /= num_neighbors;
        waypoint_center(1) /= num_neighbors;
        path.push_back(waypoint_center);
    }
}