#include <gridmap_navigation/costmap_ros.h>

#define duration_ms(a) std::chrono::duration_cast<std::chrono::milliseconds>(a).count()
typedef std::chrono::high_resolution_clock clk;

namespace grid_map
{
    Costmap::Costmap() : GridMap({"occupancy", "cost", "history_x", "history_y"})
    {
        // setBasicLayers({"cost"});
        setFrameId("map");
        setGeometry(Length(300, 300), 0.1);
        get("occupancy").setConstant(0);
    }

    Costmap::Costmap(const nav_msgs::OccupancyGrid &occupancy_grid, int inflation_size) : GridMap({"occupancy", "cost", "history_x", "history_y"})
    {
        // setBasicLayers({"cost"});
        try
        {
            if (!fromOccupancyGrid(occupancy_grid, "occupancy", *this))
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

        ROS_INFO("Costmap is ready to be Updated.");
    }

    void Costmap::inflateOccupancyGrid(int inflation_size)
    {
        add("inflation");

        // save obstacle index
        std::vector<Index> obstacles;
        for (GridMapIterator it(*this); !it.isPastEnd(); ++it)
        {
            const auto &state = at("occupancy", *it);

            // skip for Nan
            if (!std::isfinite(state))
                continue;
            // skip for free (0)
            if (state < FLT_EPSILON)
                continue;

            obstacles.push_back(*it);
        }

        get("inflation") = get("occupancy");
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

    bool Costmap::fromOccupancyGrid(const nav_msgs::OccupancyGrid &occupancyGrid,
                                    const std::string &layer, grid_map::GridMap &gridMap)
    {
        const Size size(occupancyGrid.info.width, occupancyGrid.info.height);
        const double resolution = occupancyGrid.info.resolution;
        const Length length = resolution * size.cast<double>();
        const std::string &frameId = occupancyGrid.header.frame_id;
        Position position(occupancyGrid.info.origin.position.x, occupancyGrid.info.origin.position.y);
        // Different conventions of center of map.
        position += 0.5 * length.matrix();

        const auto &orientation = occupancyGrid.info.origin.orientation;
        if (orientation.w != 1.0 && !(orientation.x == 0 && orientation.y == 0 && orientation.z == 0 && orientation.w == 0))
        {
            ROS_WARN_STREAM("Conversion of occupancy grid: Grid maps do not support orientation.");
            ROS_INFO_STREAM("Orientation of occupancy grid: " << std::endl
                                                              << occupancyGrid.info.origin.orientation);
            return false;
        }

        if (static_cast<size_t>(size.prod()) != occupancyGrid.data.size())
        {
            ROS_WARN_STREAM("Conversion of occupancy grid: Size of data does not correspond to width * height.");
            return false;
        }

        // TODO: Split to `initializeFrom` and `from` as for Costmap2d.
        if ((gridMap.getSize() != size).any() || gridMap.getResolution() != resolution || (gridMap.getLength() != length).any() || gridMap.getPosition() != position || gridMap.getFrameId() != frameId || !gridMap.getStartIndex().isZero())
        {
            gridMap.setTimestamp(occupancyGrid.header.stamp.toNSec());
            gridMap.setFrameId(frameId);
            gridMap.setGeometry(length, resolution, position);
        }

        // Reverse iteration is required because of different conventions
        // between occupancy grid and grid map.
        grid_map::Matrix data(size(0), size(1));
        for (auto iterator = occupancyGrid.data.rbegin();
             iterator != occupancyGrid.data.rend(); ++iterator)
        {
            size_t i = std::distance(occupancyGrid.data.rbegin(), iterator);
            data(i) = *iterator != -1 ? *iterator : NAN;
        }

        gridMap.add(layer, data);
        return true;
    }

    void Costmap::toOccupancyGrid(const grid_map::GridMap &gridMap,
                                  const std::string &layer, float dataMin, float dataMax,
                                  nav_msgs::OccupancyGrid &occupancyGrid)
    {
        occupancyGrid.header.frame_id = gridMap.getFrameId();
        occupancyGrid.header.stamp.fromNSec(gridMap.getTimestamp());
        occupancyGrid.info.map_load_time = occupancyGrid.header.stamp; // Same as header stamp as we do not load the map.
        occupancyGrid.info.resolution = gridMap.getResolution();
        occupancyGrid.info.width = gridMap.getSize()(0);
        occupancyGrid.info.height = gridMap.getSize()(1);
        Position position = gridMap.getPosition() - 0.5 * gridMap.getLength().matrix();
        occupancyGrid.info.origin.position.x = position.x();
        occupancyGrid.info.origin.position.y = position.y();
        occupancyGrid.info.origin.position.z = 0.0;
        occupancyGrid.info.origin.orientation.x = 0.0;
        occupancyGrid.info.origin.orientation.y = 0.0;
        occupancyGrid.info.origin.orientation.z = 0.0;
        occupancyGrid.info.origin.orientation.w = 1.0;
        size_t nCells = gridMap.getSize().prod();
        occupancyGrid.data.resize(nCells);

        // Occupancy probabilities are in the range [0,100]. Unknown is -1.
        const float cellMin = 0;
        const float cellMax = 100;
        const float cellRange = cellMax - cellMin;

        for (GridMapIterator iterator(gridMap); !iterator.isPastEnd(); ++iterator)
        {
            float value = (gridMap.at(layer, *iterator) - dataMin) / (dataMax - dataMin);
            if (std::isnan(value))
                continue;
            else
                value = cellMin + std::min(std::max(0.0f, value), 1.0f) * cellRange;

            size_t index = getLinearIndexFromIndex(iterator.getUnwrappedIndex(), gridMap.getSize(), false);
            // Reverse cell order because of different conventions between occupancy grid and grid map.
            occupancyGrid.data[nCells - index - 1] = value;
        }
    }

    bool Costmap::update(const Position &robot, const Position &goal)
    {
        clear("cost");
        clear("history_x");
        clear("history_y");

        const auto &occupancymap = (std::find(getLayers().begin(), getLayers().end(), "inflation") != getLayers().end() ? get("inflation") : get("occupancy"));
        auto &costmap = get("cost");
        auto &historymap_x = get("history_x");
        auto &historymap_y = get("history_y");

        Index robot_index, goal_index;
        if (!getIndex(robot, robot_index))
        {
            ROS_WARN_THROTTLE(1, "[update Costmap] Robot is Out of Map Boundary. Failed to get Index from robot position.");
            return false;
        }
        if (!getIndex(goal, goal_index))
        {
            ROS_WARN_THROTTLE(1, "[update Costmap] Goal is Out of Map Boundary. Failed to get Index from Goal position.");
            return false;
        }
        const auto &goal_state = occupancymap(goal_index(0), goal_index(1));
        if (!std::isfinite(goal_state) || goal_state > 0) // goal in unknown or occupied grid
        {
            ROS_WARN_THROTTLE(1, "[update Costmap] Please put Goal into Valid Region. Failed to Update Costmap.");
            return false;
        }

        std::priority_queue<CostCell> visited_list;
        visited_list.push(CostCell(goal_index, 1.0));

        int neighbor_size = 1;
        Index index_offset(neighbor_size, neighbor_size);
        Index search_buffer(2 * neighbor_size + 1, 2 * neighbor_size + 1);

        bool updated = false;
        clk::time_point start_time = clk::now();
        while (!visited_list.empty())
        {
            const auto prioritycell = visited_list.top();
            Position prioritycell_position;
            getPosition(prioritycell.index, prioritycell_position);
            visited_list.pop();

            if (duration_ms(clk::now() - start_time) > time_limit_ms_)
            {
                ROS_WARN("[Update Costmap] TIMEOUT! Suspend Searching...Set Closer Goal");
                return false;
            }

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

                auto &neighbor_cost = costmap(neighbor_index(0), neighbor_index(1));
                auto &neighbor_history_x = historymap_x(neighbor_index(0), neighbor_index(1));
                auto &neighbor_history_y = historymap_y(neighbor_index(0), neighbor_index(1));
                const auto &moving_cost = getDistance(prioritycell_position, neighbor_position);
                if (!std::isfinite(neighbor_cost)) // never visited before
                {
                    neighbor_cost = prioritycell.cost + moving_cost;
                    neighbor_history_x = prioritycell_position(0);
                    neighbor_history_y = prioritycell_position(1);
                    if (!updated)
                        visited_list.push(CostCell(neighbor_index, neighbor_cost));
                }
                else if (neighbor_cost > prioritycell.cost + moving_cost) // if found shorter path
                {
                    neighbor_cost = prioritycell.cost + moving_cost;
                    neighbor_history_x = prioritycell_position(0);
                    neighbor_history_y = prioritycell_position(1);
                }
            }

            if (std::isfinite(atPosition("cost", robot)))
                updated = true;
        }

        if (!updated)
        {
            ROS_WARN("[Update Costmap] Searched All Connected Grid Cells. Goal is blocked from the Robot.");
            return false;
        }
        return true;
    }

    bool Costmap::findGlobalPath(const Position &robot, const Position &goal, std::vector<Position> &path)
    {
        Position waypoint_position(robot);
        Index waypoint_index;

        const auto &history_x = get("history_x");
        const auto &history_y = get("history_y");
        clk::time_point start_time = clk::now();
        while ((waypoint_position - goal).norm() > 0.3)
        {
            if (duration_ms(clk::now() - start_time) > time_limit_ms_)
            {
                ROS_WARN("[Current Path] TIMEOUT!! Finding Path from Cost map Failed.");
                return false;
            }
            getIndex(waypoint_position, waypoint_index);
            auto pathpoint_x = history_x(waypoint_index(0), waypoint_index(1));
            auto pathpoint_y = history_y(waypoint_index(0), waypoint_index(1));
            if (std::isfinite(pathpoint_x))
            {
                Position pathpoint(pathpoint_x, pathpoint_y);
                path.push_back(pathpoint);
                waypoint_position = pathpoint;
            }
            else
            {
                ROS_WARN("[FindPath] Current Robot position has never been visited when Planning.");
                return false;
            }
        }

        std::reverse(path.begin(), path.end());
        return true;
    }
}
