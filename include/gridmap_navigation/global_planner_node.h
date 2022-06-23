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
        std::string pubtopic_path;

        Costmap costmap_;  // need this to build...
        std::unique_ptr<Costmap> costmapPtr_;
        bool costmap_initialized_;
        Position goal_position_;
        Position robot_position_;

        tf2_ros::Buffer tf2_buffer;
        tf2_ros::TransformListener tf2_listener;

        // options
        bool use_global_map;
        bool use_inflation;
        int param_inflation_size;
        float param_goal_acceptance;
        int param_Hz;

    public:
        GlobalPlannerNode(ros::NodeHandle &);
        ~GlobalPlannerNode() { nh.shutdown(); }
        void useParameterServer();
        void goalCallback(const geometry_msgs::PoseStamped::ConstPtr &);

        void updateGoalPosition(const geometry_msgs::Pose &goal);
        void updateRobotPosition(const ros::Time &time);

        void process();
    };

    class OccupancyGridHandler : public GridMapRosConverter
    {
        const int OCCUPIED = 100;
        const int FREE = 0;

    public:
        OccupancyGridHandler() = default;
        virtual ~OccupancyGridHandler() = default;

        static void inflateOccupancyGrid(int inflation_size, const std::string &layer, GridMap &gridmap)
        {
            auto layer_out = layer + "_inflated";
            gridmap.add(layer_out);

            // save obstacle index
            std::vector<Index> obstacles;
            for (GridMapIterator it(gridmap); !it.isPastEnd(); ++it)
            {
                const auto state = gridmap.at(layer, *it);

                // skip for unknown cell
                if (!std::isfinite(state))
                    continue;
                // skip for free cell
                if (std::abs(state) > FLT_EPSILON)
                    continue;

                obstacles.push_back(*it);
            }

            // copy original
            gridmap[layer_out] = gridmap[layer];

            for (const auto index : obstacles)
            {
                Index start_index(inflation_size, inflation_size);
                Index inflation_window(2 * inflation_size + 1, 2 * inflation_size + 1);
                SubmapIterator search_it(gridmap, index - start_index, inflation_window);
                for (search_it; !search_it.isPastEnd(); ++search_it)
                {
                    // skip for position out of map
                    Position search_position;
                    if (!gridmap.getPosition(*search_it, search_position))
                        continue;
                    gridmap.at(layer_out, *search_it) = gridmap.at(layer, index);
                }
            }
        }
    };
}
