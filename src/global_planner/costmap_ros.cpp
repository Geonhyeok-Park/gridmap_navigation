#include <gridmap_navigation/costmap_ros.h>

#define duration_ms(a) std::chrono::duration_cast<std::chrono::milliseconds>(a).count()
typedef std::chrono::high_resolution_clock clk;

namespace grid_map
{
    Costmap::Costmap() : GridMap({"occupancy", "cost"})
    {
        setFrameId("map");
        setGeometry(Length(300, 300), 0.1);
        get("occupancy").setConstant(0);
    }

    Costmap::Costmap(const nav_msgs::OccupancyGrid &occupancy_grid, double robot_radius) : GridMap({"occupancy", "cost"})
    {
        try
        {
            if (!fromOccupancyGrid(occupancy_grid))
                throw std::runtime_error("Occupancy map rotated or has strange size");
        }
        catch (const std::exception &e)
        {
            std::cerr << e.what() << '\n';
        }

        int inflation_size;
        if (std::fmod(robot_radius, getResolution()) < DBL_EPSILON)
            inflation_size = robot_radius / getResolution();
        else
            inflation_size = robot_radius / getResolution() + 1;

        if (inflation_size != 0)
        {
            inflateOccupancyGrid(inflation_size);
            ROS_INFO_STREAM("Grid Inflation with size " << inflation_size << " Done.");
        }

        ROS_INFO("Costmap is ready to be Updated.");
    }

    void Costmap::inflateOccupancyGrid(int inflation_size)
    {
        add("inflation");

        std::vector<Index> obstacles;
        for (GridMapIterator it(*this); !it.isPastEnd(); ++it)
        {
            const auto &state = at("occupancy", *it);

            if (isUnknown(state))
                continue;

            if (isFree(state))
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

    bool Costmap::fromOccupancyGrid(const nav_msgs::OccupancyGrid &occupancyGrid)
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
        if ((getSize() != size).any() || getResolution() != resolution || (getLength() != length).any() || getPosition() != position || getFrameId() != frameId || !getStartIndex().isZero())
        {
            setTimestamp(occupancyGrid.header.stamp.toNSec());
            setFrameId(frameId);
            setGeometry(length, resolution, position);
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

        add("occupancy", data);
        return true;
    }

    void Costmap::toRosMsg(nav_msgs::OccupancyGrid &occupancyGrid)
    {
        occupancyGrid.header.frame_id = getFrameId();
        occupancyGrid.header.stamp.fromNSec(getTimestamp());
        occupancyGrid.info.map_load_time = occupancyGrid.header.stamp; // Same as header stamp as we do not load the map.
        occupancyGrid.info.resolution = getResolution();
        occupancyGrid.info.width = getSize()(0);
        occupancyGrid.info.height = getSize()(1);
        Position position = getPosition() - 0.5 * getLength().matrix();
        occupancyGrid.info.origin.position.x = position.x();
        occupancyGrid.info.origin.position.y = position.y();
        occupancyGrid.info.origin.position.z = 0.0;
        occupancyGrid.info.origin.orientation.x = 0.0;
        occupancyGrid.info.origin.orientation.y = 0.0;
        occupancyGrid.info.origin.orientation.z = 0.0;
        occupancyGrid.info.origin.orientation.w = 1.0;
        size_t nCells = getSize().prod();
        occupancyGrid.data.resize(nCells);

        // Occupancy probabilities are in the range [0,100]. Unknown is -1.
        const float cellMin = 0;
        const float cellMax = 100;
        const float cellRange = cellMax - cellMin;

        const auto &costmap = get("cost");
        for (GridMapIterator iterator(*this); !iterator.isPastEnd(); ++iterator)
        {
            float value = (costmap((*iterator).x(), (*iterator).y()) - (min_cost_ - 10)) / (max_cost_ - (min_cost_ - 10));
            if (std::isnan(value))
                continue;
            else
                value = cellMin + std::min(std::max(0.0f, value), 1.0f) * cellRange;

            size_t index = getLinearIndexFromIndex(iterator.getUnwrappedIndex(), getSize(), false);
            // Reverse cell order because of different conventions between occupancy grid and grid map.
            occupancyGrid.data[nCells - index - 1] = value;
        }
    }

    bool Costmap::update(const Position &robot, const Position &goal)
    {
        clear("cost");

        const auto &occupancymap = (std::find(getLayers().begin(), getLayers().end(), "inflation") != getLayers().end() ? get("inflation") : get("occupancy"));
        auto &costmap = get("cost");

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
        if (isUnknown(goal_state) || isOccupied(goal_state))
        {
            ROS_WARN_THROTTLE(1, "[update Costmap] Please put Goal into Valid Region. Failed to Update Costmap.");
            return false;
        }

        // put searched grid cells into visited list : start from goal position
        std::priority_queue<CostCell> visited_list;
        CostCell start_grid(goal_index, min_cost_);
        visited_list.push(start_grid);

        int neighbor_size = 1;
        Index index_offset(neighbor_size, neighbor_size);
        Index search_buffer(2 * neighbor_size + 1, 2 * neighbor_size + 1);

        bool updated = false;
        max_cost_ = min_cost_;
        clk::time_point start_time = clk::now();
        while (!visited_list.empty())
        {
            CostCell prioritycell = visited_list.top();
            visited_list.pop();

            Position prioritycell_position;
            getPosition(prioritycell.index, prioritycell_position);

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
                if (isSameGrid(neighbor_index, prioritycell.index))
                    continue;

                const auto &neighbor_state = occupancymap(neighbor_index(0), neighbor_index(1));
                if (isUnknown(neighbor_state) || isOccupied(neighbor_state))
                    continue;

                auto &neighbor_cost = costmap(neighbor_index(0), neighbor_index(1));
                const auto &moving_cost = getDistance(prioritycell_position, neighbor_position);
                if (isUnknown(neighbor_cost))
                {
                    neighbor_cost = prioritycell.cost + moving_cost;
                    max_cost_ = std::max(max_cost_, neighbor_cost);
                    if (!updated)
                        visited_list.push(CostCell(neighbor_index, neighbor_cost));
                }
                else if (neighbor_cost > prioritycell.cost + moving_cost) // if found shorter path
                {
                    // neighbor_cost = prioritycell.cost + moving_cost;
                }
            }

            if (isKnown(atPosition("cost", robot)))
                updated = true;
        }

        if (!updated)
        {
            ROS_WARN("[Update Costmap] Searched All Connected Grid Cells. Goal is blocked from the Robot.");
            return false;
        }
        return true;
    }
}
