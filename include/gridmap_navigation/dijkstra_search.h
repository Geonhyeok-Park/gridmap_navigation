//
// Created by Ikhyeon Cho on 22. 4. 15.
//
#ifndef GRIDMAP_NAVIGATION_DIJKSTRASEARCH_H
#define GRIDMAP_NAVIGATION_DIJKSTRASEARCH_H

// grid map library
#include <grid_map_core/GridMap.hpp>
#include <grid_map_core/grid_map_core.hpp>

#include <chrono>
#include <iostream>
#include <queue>

#define duration_ms(a) std::chrono::duration_cast<std::chrono::milliseconds>(a).count()
typedef std::chrono::high_resolution_clock clk;

class DijkstraSearch
{
    const float FREE = 0.f;
    const int TIME_LIMIT_MS = 1000;

private:
    grid_map::GridMap *map_ptr_;
    std::string layer_;

    grid_map::Position position_goal_;
    grid_map::Position position_robot_;

    std::vector<grid_map::Position> path_;

    double max_cost_;

public:
    DijkstraSearch(grid_map::GridMap &map, const std::string &layer, const grid_map::Position &robot_pos, const grid_map::Position &goal_pos);
    ~DijkstraSearch() = default;

    bool run();
    bool updateCostmap(const std::string &layer_cost);
    bool findPath();

    inline double getMaxCost() { return max_cost_; }
    inline std::vector<grid_map::Position> &getPath() { return path_; }

private:
    bool updateCostmap(const std::string &layer_cost, double searchRadius);
    inline float getDistance(const grid_map::Position &_pos1,
                             const grid_map::Position &_pos2)
    {
        return (float)sqrt(pow(_pos1.x() - _pos2.x(), 2) + pow(_pos1.y() - _pos2.y(), 2));
    }
};

#endif // GRIDMAP_NAVIGATION_DIJKSTRASEARCH_H