//
// Created by Ikhyeon Cho on 22. 4. 15.
//

#ifndef GRIDMAP_NAVIGATION_DIJKSTRASEARCH_HPP
#define GRIDMAP_NAVIGATION_DIJKSTRASEARCH_HPP

// grid map library
#include <grid_map_core/GridMap.hpp>
#include <grid_map_core/grid_map_core.hpp>

#include <chrono>
#include <iostream>
#include <queue>

#define duration_ms(a) std::chrono::duration_cast<std::chrono::milliseconds>(a).count()
typedef std::chrono::high_resolution_clock clk;

using namespace grid_map;

class DijkstraSearch
{
    const float FREE = 0.f;

private:
    GridMap *map_ptr_;
    std::string layer_obstacle_;
    std::string layer_cost_;

    Position position_goal_;
    Position position_robot_;

    std::vector<Position> path_;

    double max_cost_;
    int time_limit_ms;

public:

    DijkstraSearch(GridMap &_map, const std::string &_layer_obstacle, const std::string &_layer_cost,
                   const Position &_position_robot, const Position &_position_goal);
    ~DijkstraSearch() = default;

    bool run();
    double getMaxCost();
    std::vector<Position> &getPath();
    bool findPath();

private:
    bool updateCostmap(double _searchRadius);
    float getDistance(const Position &_pos1, const Position &_pos2);
};

#endif // GRIDMAP_NAVIGATION_DIJKSTRASEARCH_HPP