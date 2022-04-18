//
// Created by Ikhyeon Cho on 22. 4. 15.
//
#include <global_planner_ros/dijkstra_search.h>

DijkstraSearch::DijkstraSearch(GridMap &_map, const std::string &_layer_obstacle, const std::string &_layer_cost,
                               const Position &_position_robot, const Position &_position_goal)
{
    map_ptr_ = &_map;
    layer_obstacle_ = _layer_obstacle;
    layer_cost_ = _layer_cost;

    position_robot_ = _position_robot;
    position_goal_ = _position_goal;

}

bool DijkstraSearch::run()
{
    Position position_mid = (position_goal_ + position_robot_) / 2;
    double search_radius = getDistance(position_mid, position_robot_);

    clk::time_point start_time = clk::now();
    clk::time_point check_point = clk::now();

    time_limit_ms = 1000;
    while (!updateCostmap(search_radius))
    {
        if (duration_ms(check_point - start_time) > time_limit_ms)
        {
            std::cout << "GRADIENT TIMEOUT!! Update Costmap Failed" << std::endl;
            return false;
        }

        check_point = clk::now();
        map_ptr_->clear(layer_cost_);
        search_radius += getDistance(position_mid, position_robot_);
    }

    if (!findPath())
    {
        std::cout << "clear Path" << std::endl;
        path_.clear();
        return false;
    }

    return true;
}

std::vector<Position> &DijkstraSearch::getPath()
{
    return path_;
}

double DijkstraSearch::getMaxCost()
{
    return max_cost_;
}

float DijkstraSearch::getDistance(const Position &_pos1, const Position &_pos2)
{
    return (float)sqrt(pow(_pos1.x() - _pos2.x(), 2) + pow(_pos1.y() - _pos2.y(), 2));
}

bool DijkstraSearch::updateCostmap(double _search_radius)
{
    map_ptr_->clear(layer_cost_);

    const auto &map = map_ptr_->get(layer_obstacle_);
    auto &cost_map = map_ptr_->get(layer_cost_);

    Index index_goal, index_robot;
    map_ptr_->getIndex(position_goal_, index_goal);
    map_ptr_->getIndex(position_robot_, index_robot);

    if (!map_ptr_->isInside(position_goal_))
    {
        std::cout << "Goal out of map region!" << std::endl;
        return false;
    }

    std::queue<Index> grid_search_areas;
    grid_search_areas.push(index_goal);

    map_ptr_->atPosition(layer_cost_, position_goal_) = 1.0;
    max_cost_ = 1.0;

    while (!grid_search_areas.empty())
    {
        Position position_current_cell;
        const auto index_current_cell = grid_search_areas.front();
        grid_search_areas.pop();
        map_ptr_->getPosition(index_current_cell, position_current_cell);

        Index search_size(1, 1);
        Index search_area(3, 3);
        SubmapIterator search_iter(*map_ptr_, index_current_cell - search_size, search_area);
        for (search_iter; !search_iter.isPastEnd(); ++search_iter)
        {
            const auto &index_search = *search_iter;
            Position position_search;
            map_ptr_->getPosition(index_search, position_search);

            // only search nearby cell
            if (index_search.isApprox(index_current_cell))
                continue;

            // pass unknown region
            if (!std::isfinite(map_ptr_->atPosition(layer_obstacle_, position_search)))
                continue;

            // only search in free space
            if ((map_ptr_->atPosition(layer_obstacle_, position_search) - FREE) > __FLT_EPSILON__)
                continue;

            // only search in region between goal and robot
            const auto position_mid = (position_goal_ + position_robot_) / 2;
            double margin = 3.0;
            if (getDistance(position_mid, position_search) > _search_radius + margin)
                continue;

            auto &cost_search = cost_map(index_search(0), index_search(1));
            const auto &cost_current_cell = cost_map(index_current_cell(0), index_current_cell(1));
            const auto &cost_transition = getDistance(position_current_cell, position_search);
            if (!std::isfinite(cost_search))
            {
                cost_search = cost_current_cell + cost_transition;
                grid_search_areas.push(index_search);
                if (cost_search > max_cost_)
                    max_cost_ = cost_search;
            }
            else if (cost_search > cost_current_cell + cost_transition)
            {
                cost_search = cost_current_cell + cost_transition;
            }
        }
    }

    if (!std::isfinite(cost_map(index_robot(0), index_robot(1))))
    {
        std::cout << "Could not find valid path from goal to robot" << std::endl;
        return false;
    }

    return true;
}

// TODO: fix to improve speed
bool DijkstraSearch::findPath()
{
    path_.clear();

    auto cost_map = map_ptr_->get(layer_cost_);

    Index index_robot, index_goal;
    map_ptr_->getIndex(position_robot_, index_robot);
    map_ptr_->getIndex(position_goal_, index_goal);

    std::queue<Index> path_candidates;
    path_candidates.push(index_robot);

    while (!path_candidates.empty())
    {
        const auto index_current_cell = path_candidates.front();

        if (index_current_cell.isApprox(index_goal))
        {
            return true;
        }

        Index path_index;
        bool near_path_exists = false;
        double min_cost = cost_map(index_current_cell(0), index_current_cell(1));

        Index search_size(1, 1);
        Index search_area(3, 3);
        SubmapIterator search_iter(*map_ptr_, index_current_cell - search_size, search_area);
        for (search_iter; !search_iter.isPastEnd(); ++search_iter)
        {
            const auto &index_search = *search_iter;

            // pass invalid cost (unexplored)
            if (!std::isfinite(cost_map(index_search(0), index_search(1))))
                continue;

            if (index_search.isApprox(index_current_cell))
                continue;

            const auto &cost_search = cost_map(index_search(0), index_search(1));
            if (cost_search < min_cost)
            {
                min_cost = cost_search;
                path_index = index_search;
                near_path_exists = true;
            }
        }
        if (!near_path_exists)
        {
            std::cout << "No valid path found with constructing cost map" << std::endl;
            return false;
        }

        path_candidates.push(path_index);
        path_candidates.pop();
        Position path_position;
        map_ptr_->getPosition(path_index, path_position);
        path_.push_back(path_position);
    }

    std::cout << "searched all path Candidates. No path available" << std::endl;
    return false;
}
