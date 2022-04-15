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

private:
    GridMap *mapPtr_;
    std::string mapLayer_;
    std::string costLayer_;

    Position goalPosition_;
    Position robotPosition_;

    std::vector<Position> path_;

    double maxCost_;
    int timeLimit_ms;

public:

    DijkstraSearch(GridMap &map, const std::string &inputLayer, const std::string &costLayer,
                   const Position &robotPosition, const Position &goalPosition);
    ~DijkstraSearch() = default;

    bool run();
    double getMaxCost();
    std::vector<Position> &getPath();
    bool findPath();

private:
    bool updateCostmap(double searchRadius);
    float getDistance(const Position &pos1, const Position &pos2);
};

#endif // GRIDMAP_NAVIGATION_DIJKSTRASEARCH_HPP