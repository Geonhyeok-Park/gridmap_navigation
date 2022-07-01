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
#include <gridmap_navigation/CloudManager.hpp>

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
        bool param_getmap_from_topic;
        double param_robot_radius;
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
        ros::Publisher pub_localmap;

        tf2_ros::Buffer tf2_buffer;
        tf2_ros::TransformListener tf2_listener;

        CloudManager<pcl::PointXYZI> pc;

    private:
        std::unique_ptr<Costmap> costmapPtr_;
        std::vector<Position> path_;
        Position goal_position_;
        Position robot_position_;

        // Node Status
        bool costmap_updated_;
        bool globalpath_updated_;
        bool globalmap_received_;

    public:
        GlobalPlannerNode(ros::NodeHandle &);
        ~GlobalPlannerNode() = default;
        void run();

        void useParameterServer();
        void goalCallback(const geometry_msgs::PoseStamped::ConstPtr &);
        void localmapCallback(const sensor_msgs::PointCloud2Ptr &);
        void globalmapCallback(const nav_msgs::OccupancyGridConstPtr &);

        void updateGoalPosition(const geometry_msgs::Pose &goal);
        bool updateRobotPosition(const ros::Time &time);

        bool findGlobalPath(const Position &robot, const Position &goal, std::vector<Position> &path);
        void toRosMsg(const std::vector<Position> &path, nav_msgs::Path &msg);
    };

}

// moving average filter
void smoothing(std::vector<grid_map::Position> &path, int neighbor)
{
    if (path.size() < neighbor)
        return;

    std::vector<grid_map::Position> path_copy;
    std::queue<grid_map::Position> queue;
    grid_map::Position sum(0.0, 0.0);
    grid_map::Position average;
    for (int i = 0; i < neighbor; ++i)
    {
        queue.push(path.at(i));
        sum += path.at(i);
        average = sum / (i + 1);
        path_copy.emplace_back(average);
    }
    for (int i = neighbor; i < path.size(); ++i)
    {
        sum -= queue.front();
        queue.pop();
        queue.push(path.at(i));
        sum += queue.back();
        average = sum / neighbor;
        path_copy.emplace_back(average);
    }
    path_copy.emplace_back(path.back());

    path = path_copy;
}

void smoothing_eband(const grid_map::ElasticBand &eband, int neighbor, std::vector<grid_map::Position> &path)
{
    for (int i = neighbor; i < eband.getEband().size() - neighbor; ++i)
    {
        auto waypoint_center = eband.getEband().at(i).getPosition();
        auto num_neighbors = 2 * neighbor + 1;
        for (int j = -neighbor; j <= neighbor; j++)
        {
            if (j == 0)
                continue;
            waypoint_center += eband.getEband().at(i + j).getPosition();
        }
        waypoint_center(0) /= num_neighbors;
        waypoint_center(1) /= num_neighbors;
        path.push_back(waypoint_center);
    }
}