#include <grid_map_core/grid_map_core.hpp>

namespace grid_map
{
    class WaveFrontCostmap : public GridMap
    {
    public:
        WaveFrontCostmap();
        virtual ~WaveFrontCostmap() = default;

        bool update(const std::string &layer, const Position &robot, const Position &goal, double search_radius);
        
        bool findPath(const std::string &layer, const Position &robot, const Position &goal, std::vector<Position> &path);
        
        static float getDistance(const Position &pos1, const Position &pos2) { return std::sqrt(std::pow(pos1.x() - pos2.x(), 2) + std::pow(pos1.y() - pos2.y(), 2)); }
    };
}
