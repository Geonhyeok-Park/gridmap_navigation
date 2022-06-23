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

        std::unique_ptr<Costmap> costmapPtr_;
        bool costmap_initialized_;
        Position goal_position_;
        Position robot_position_;

        tf2_ros::Buffer tf2_buffer;
        tf2_ros::TransformListener tf2_listener;

        // options
        bool use_global_map;
        int param_inflation_size;
        int param_Hz;

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
    };

}
