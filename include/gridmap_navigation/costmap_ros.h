#ifndef GRIDMAP_NAVIGATION_COSTMAP_H
#define GRIDMAP_NAVIGATION_COSTMAP_H

#include <grid_map_core/grid_map_core.hpp>
#include <nav_msgs/OccupancyGrid.h>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <queue>
#include <chrono>

namespace grid_map
{
    class Costmap : public GridMap
    {
    private:
        int time_limit_ms_ = 2000;
        double robot_radius_ = 0.3;
        float min_cost_ = 1.0;
        float max_cost_;

    private:
        bool isUnknown(const float &cell) { return !std::isfinite(cell); };
        bool isKnown(const float &cell) { return std::isfinite(cell); };
        bool isOccupied(const float &cell) { return cell > 0; };
        bool isFree(const float &cell) { return cell < FLT_EPSILON; };
        bool isSameGrid(const Index &c1, const Index &c2) { return c1.isApprox(c2); };

        bool fromOccupancyGrid(const nav_msgs::OccupancyGrid &occupancyGrid);

        void inflateOccupancyGrid(int inflation_size);

        float getDistance(const Position &pos1, const Position &pos2) { return std::sqrt(std::pow(pos1.x() - pos2.x(), 2) + std::pow(pos1.y() - pos2.y(), 2)); }

    public:
        Costmap();
        explicit Costmap(const nav_msgs::OccupancyGrid &occupancy_map, double robot_radius);
        virtual ~Costmap() = default;

        void setTimeLimit(int time_ms) { time_limit_ms_ = time_ms; };

        bool update(const Position &robot, const Position &goal);

        bool returnPath(const Position &robot, const Position &goal, std::vector<Position> &path);

        void toRosMsg(nav_msgs::OccupancyGrid &occupancyGrid);
    };

    struct CostCell
    {
        Index index;
        float cost;
        CostCell(Index _index, float _cost) : index(_index), cost(_cost) {}

        bool operator<(const CostCell &cell) const
        {
            return this->cost > cell.cost;
        }
    };

}

#endif