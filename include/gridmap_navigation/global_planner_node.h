#include <ros/ros.h>
#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <nav_msgs/GetMap.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/transform_listener.h>
#include <gridmap_navigation/costmap_ros.h>

namespace grid_map
{
    class GlobalPlannerNode
    {
    private:
        ros::NodeHandle &nh;
        ros::Subscriber sub_goal;
        ros::Publisher pub_path;
        ros::Publisher pub_map;
        std::string pubtopic_path;
        std::string pubtopic_map;

        tf2_ros::Buffer tf2_buffer;
        tf2_ros::TransformListener tf2_listener;

        std::unique_ptr<Costmap> costmapPtr_;
        bool valid_cost_;
        Position goal_position_;
        Position robot_position_;

        // options
        bool use_global_map;
        int param_inflation_size;
        int param_Hz;
        int param_timelimit_ms;

    public:
        GlobalPlannerNode(ros::NodeHandle &);
        ~GlobalPlannerNode() { nh.shutdown(); }
        void process();

    private:
        void useParameterServer();
        void goalCallback(const geometry_msgs::PoseStamped::ConstPtr &);

        void updateGoalPosition(const geometry_msgs::Pose &goal);
        void updateRobotPosition(const ros::Time &time);

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
