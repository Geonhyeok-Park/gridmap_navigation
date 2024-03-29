#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <nav_msgs/OccupancyGrid.h>
#include <queue>
#include <chrono>
#include <iostream>

namespace grid_map
{
    class Costmap : public GridMap
    {
    private:
        void inflateOccupancyGrid(int inflation_size);

    public:
        Costmap();
        Costmap(nav_msgs::OccupancyGrid &occupancy_map, int inflation_size = 0);
        virtual ~Costmap() = default;

        bool update(const Position &robot, const Position &goal);

        bool findPath(const Position &robot, const Position &goal, std::vector<Position> &path);

        static float getDistance(const Position &pos1, const Position &pos2) { return std::sqrt(std::pow(pos1.x() - pos2.x(), 2) + std::pow(pos1.y() - pos2.y(), 2)); }
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
