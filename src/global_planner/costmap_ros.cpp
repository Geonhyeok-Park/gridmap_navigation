#include <gridmap_navigation/costmap_ros.h>

#define duration_ms(a) std::chrono::duration_cast<std::chrono::milliseconds>(a).count()
using clk = std::chrono::high_resolution_clock;

namespace grid_map
{
    Costmap::Costmap() : GridMap({"cost"})
    {
        setBasicLayers({"cost"});
        setFrameId("map");
        setGeometry(Length(300, 300), 0.1);
    }

    Costmap::Costmap(nav_msgs::OccupancyGrid &occupancy_grid, int inflation_size) : GridMap({"occupancy", "cost"})
    {
        setBasicLayers({"cost"});
        try
        {
            if (!GridMapRosConverter::fromOccupancyGrid(occupancy_grid, "occupancy", *this))
                throw std::runtime_error("Occupancy map rotated or has strange size");
        }
        catch (const std::exception &e)
        {
            std::cerr << e.what() << '\n';
        }

        if (inflation_size != 0)
        {
            inflateOccupancyGrid(inflation_size);
            ROS_INFO("Grid Inflation Done.");
        }

        ROS_INFO("Costmap with Occupancy map Initialized.");
    }

    void Costmap::inflateOccupancyGrid(int inflation_size)
    {
        add("inflation");

        // save obstacle index
        std::vector<Index> obstacles;
        for (GridMapIterator it(*this); !it.isPastEnd(); ++it)
        {
            const auto state = at("inflation", *it);

            // skip for Nan
            if (!std::isfinite(state))
                continue;
            // skip for free (0)
            if (state < FLT_EPSILON)
                continue;

            obstacles.push_back(*it);
        }

        for (const auto &obstacle_index : obstacles)
        {
            Index start_index(inflation_size, inflation_size);
            Index inflation_window(2 * inflation_size + 1, 2 * inflation_size + 1);
            SubmapIterator inflation_iterator(*this, obstacle_index - start_index, inflation_window);
            for (inflation_iterator; !inflation_iterator.isPastEnd(); ++inflation_iterator)
            {
                // skip for position out of map
                Position search_position;
                if (!getPosition(*inflation_iterator, search_position))
                    continue;
                at("inflation", *inflation_iterator) = at("occupancy", obstacle_index);
            }
        }
    }

    bool Costmap::update(const Position &robot, const Position &goal)
    {
        clear("cost");
        add("history_x");
        add("history_y");

        auto &costmap = get("cost");
        const auto &occupancymap = get("occupancy");
        auto &historymap_x = get("history_x");
        auto &historymap_y = get("history_y");

        Index robot_index, goal_index;
        if (!getIndex(robot, robot_index))
        {
            ROS_WARN("[update] Failed to get Index from robot position.");
            return false;
        }
        if (!getIndex(goal, goal_index))
        {
            ROS_WARN("[update] Failed to get Index from Goal position.");
            return false;
        }

        std::priority_queue<CostCell> visited_list;
        int size_neighbor = 1;
        Index index_offset(size_neighbor, size_neighbor);
        Index search_buffer(2 * size_neighbor + 1, 2 * size_neighbor + 1);

        visited_list.push(CostCell(goal_index, 1.0));

        while (!visited_list.empty())
        {
            const auto prioritycell = visited_list.top();
            Position prioritycell_position;
            getPosition(prioritycell.index, prioritycell_position);
            visited_list.pop();

            if (std::isfinite(atPosition("cost", robot)))
                return true;

            SubmapIterator neighbor_iterator(*this, prioritycell.index - index_offset, search_buffer);
            for (neighbor_iterator; !neighbor_iterator.isPastEnd(); ++neighbor_iterator)
            {
                const auto &neighbor_index = *neighbor_iterator;

                // pass out of map region
                Position neighbor_position;
                if (!getPosition(neighbor_index, neighbor_position))
                    continue;

                // pass center grid
                if (neighbor_index.isApprox(prioritycell.index))
                    continue;

                // pass unknown grid and occupied grid
                const auto &neighbor_occupancy = occupancymap(neighbor_index(0), neighbor_index(1));
                if (!std::isfinite(neighbor_occupancy) || neighbor_occupancy > 0)
                    continue;

                // // only search in search radius
                // const auto center_position = (robot + goal) / 2;
                // double margin = 3.0;
                // if (getDistance(center_position, neighbor_position) > search_radius + margin)
                //     continue;

                auto &neighbor_cost = costmap(neighbor_index(0), neighbor_index(1));
                auto &neighbor_history_x = historymap_x(neighbor_index(0), neighbor_index(1));
                auto &neighbor_history_y = historymap_y(neighbor_index(0), neighbor_index(1));
                const auto &moving_cost = getDistance(prioritycell_position, neighbor_position);
                if (!std::isfinite(neighbor_cost)) // never visited before
                {
                    neighbor_cost = prioritycell.cost + moving_cost;
                    visited_list.push(CostCell(neighbor_index, neighbor_cost));
                    neighbor_history_x = prioritycell_position(0);
                    neighbor_history_y = prioritycell_position(1);
                }
                else if (neighbor_cost > prioritycell.cost + moving_cost) // if found shorter path
                {
                    neighbor_cost = prioritycell.cost + moving_cost;
                    neighbor_history_x = prioritycell_position(0);
                    neighbor_history_y = prioritycell_position(1);
                }
            }
        }

        return false;
    }

    bool Costmap::findPath(const Position &robot, const Position &goal, std::vector<Position> &path)
    {
        Index robot_index, goal_index, pathpoint_index;
        if (!getIndex(robot, robot_index))
        {
            ROS_WARN("[FindPath] Failed to get Index from robot position.");
            return false;
        }
        if (!getIndex(goal, goal_index))
        {
            ROS_WARN("[FindPath] Failed to get Index from Goal position.");
            return false;
        }

        pathpoint_index = robot_index;
        const auto &history_x = get("history_x");
        const auto &history_y = get("history_y");
        while (pathpoint_index.isApprox(goal_index))
        {
            auto pathpoint_x = history_x(pathpoint_index(0), pathpoint_index(1));
            auto pathpoint_y = history_y(pathpoint_index(0), pathpoint_index(1));
            if (std::isfinite(pathpoint_x))
            {
                Position pathpoint(pathpoint_x, pathpoint_y);
                path.push_back(pathpoint);

                if (!getIndex(pathpoint, pathpoint_index))
                    return false;
            }
            else
            {
                ROS_WARN("[FindPath] Current Robot position has never been visited when Planning.");
                // TODO: find nearest valid visited cell
                return false;
            }
        }

        return true;
    }
}
