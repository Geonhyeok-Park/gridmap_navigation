#include <gridmap_navigation/wavefront_costmap.h>
#include <queue>
#include <chrono>
#include <iostream>

#define duration_ms(a) std::chrono::duration_cast<std::chrono::milliseconds>(a).count()
typedef std::chrono::high_resolution_clock clk;

namespace grid_map
{
    WaveFrontCostmap::WaveFrontCostmap() : GridMap({"cost"})
    {
        setBasicLayers({"cost"});
    }

    bool WaveFrontCostmap::update(const std::string &layer, const Position &robot, const Position &goal, double search_radius)
    {
        clk::time_point start_point = clk::now();

        // check layer does not exist
        auto &layers = getLayers();
        auto it = std::find(layers.begin(), layers.end(), layer);
        if (it == layers.end())
            add(layer);

        clear(layer);
        add("log_x");
        add("log_y");

        const auto &occupancy_map = get("occupancy");
        auto &cost_map = get(layer);

        Index robot_index, goal_index;
        if (!getIndex(robot, robot_index))
            return false;
        if (!getIndex(goal, goal_index))
            return false;
        if (!isInside(goal))
            return false;

        std::queue<Index> search_areas;

        at(layer, goal_index) = 1.0;
        search_areas.push(goal_index);

        Index search_size(1, 1);
        Index search_window(3, 3);
        const int TIME_LIMIT_MS = 5000;
        while (!search_areas.empty())
        {
            if (duration_ms(clk::now() - start_point) > TIME_LIMIT_MS )
            {
                std::cout << "timeout"<< std::endl;
                return false;
            }
            const auto center_grid_index = search_areas.front();
            search_areas.pop();
            Position center_grid_position;
            getPosition(center_grid_index, center_grid_position);

            SubmapIterator near_iterator(*this, center_grid_index - search_size, search_window);
            for (near_iterator; !near_iterator.isPastEnd(); ++near_iterator)
            {
                const auto &near_index = *near_iterator;

                // pass out of map region
                Position near_position;
                if (!getPosition(near_index, near_position))
                    continue;

                // pass center grid
                if (near_index.isApprox(center_grid_index))
                    continue;

                // pass unknown grid
                if (!isValid(near_index))
                    continue;

                // only search in free grid
                if (at("occupancy", near_index) > 0)
                    continue;

                // only search in search radius
                const auto center_position = (robot + goal) / 2;
                double margin = 3.0;
                if (getDistance(center_position, near_position) > search_radius + margin)
                    continue;

                auto &near_cost = cost_map(near_index(0), near_index(1));
                const auto &center_grid_cost = cost_map(center_grid_index(0), center_grid_index(1));
                const auto &traveling_cost = getDistance(center_grid_position, near_position);
                if (!std::isfinite(near_cost))
                {
                    near_cost = center_grid_cost + traveling_cost;
                    search_areas.push(near_index);
                    at("log_x", near_index) = center_grid_position(0);
                    at("log_y", near_index) = center_grid_position(1);
                }
                else if (near_cost > center_grid_cost + traveling_cost)
                {
                    near_cost = center_grid_cost + traveling_cost;
                    at("log_x", near_index) = center_grid_position(0);
                    at("log_y", near_index) = center_grid_position(1);
                }
            }
        }

        if (!std::isfinite(at(layer, robot_index)))
            return false;

        return true;
    }

    bool WaveFrontCostmap::findPath(const std::string &layer, const Position &robot, const Position &goal, std::vector<Position> &path)
    {
        auto cost_map = get(layer);
        Index robot_index, goal_index;
        if (!getIndex(robot, robot_index))
            return false;
        if (!getIndex(goal, goal_index))
            return false;

        auto log_index = robot_index;
        while (std::isfinite(at("log_x", log_index)))
        {
            auto waypoint_x = at("log_x", log_index);
            auto waypoint_y = at("log_y", log_index);
            Position waypoint(waypoint_x, waypoint_y);
            path.push_back(waypoint);

            if (!getIndex(waypoint, log_index))
                return false;
        }

        return true;
    }
}
