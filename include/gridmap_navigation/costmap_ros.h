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

    private:
        bool fromOccupancyGrid(const nav_msgs::OccupancyGrid &occupancyGrid,
                               const std::string &layer, grid_map::GridMap &gridMap);

        void inflateOccupancyGrid(int inflation_size);

        float getDistance(const Position &pos1, const Position &pos2) { return std::sqrt(std::pow(pos1.x() - pos2.x(), 2) + std::pow(pos1.y() - pos2.y(), 2)); }

    public:
        Costmap();
        explicit Costmap(const nav_msgs::OccupancyGrid &occupancy_map, int inflation_size = 0);
        virtual ~Costmap() = default;

        void setTimeLimit(int time_ms) { time_limit_ms_ = time_ms; };

        bool update(const Position &robot, const Position &goal);

        bool findGlobalPath(const Position &robot, const Position &goal, std::vector<Position> &path);

        void toOccupancyGrid(const grid_map::GridMap &gridMap,
                             const std::string &layer, float dataMin, float dataMax,
                             nav_msgs::OccupancyGrid &occupancyGrid);
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