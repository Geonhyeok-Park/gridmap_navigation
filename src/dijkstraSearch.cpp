//
// Created by Ikhyeon Cho on 22. 4. 15.
//
#include "global_planner_ros/dijkstraSearch.hpp"

DijkstraSearch::DijkstraSearch(GridMap &map, const std::string &inputLayer, const std::string &costLayer,
                               const Position &robotPosition, const Position &goalPosition)
{
    mapPtr_ = &map;
    mapLayer_ = inputLayer;
    costLayer_ = costLayer;

    robotPosition_ = robotPosition;
    goalPosition_ = goalPosition;

    // construct appropriate map format
}

bool DijkstraSearch::run()
{
    Position midPoint = (goalPosition_ + robotPosition_) / 2;
    double searchRadius = getDistance(midPoint, robotPosition_);

    clk::time_point startTime = clk::now();
    clk::time_point checkPoint = clk::now();

    timeLimit_ms = 1000;
    while (!updateCostmap(searchRadius))
    {
        if (duration_ms(checkPoint - startTime) > timeLimit_ms)
        {
            std::cout << "GRADIENT TIMEOUT!! Update Costmap Failed" << std::endl;
            return false;
        }

        checkPoint = clk::now();
        mapPtr_->clear(costLayer_);
        searchRadius += getDistance(midPoint, robotPosition_);
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
    return maxCost_;
}

float DijkstraSearch::getDistance(const Position &pos1, const Position &pos2)
{
    return (float)sqrt(pow(pos1.x() - pos2.x(), 2) + pow(pos1.y() - pos2.y(), 2));
}

bool DijkstraSearch::updateCostmap(double searchRadius)
{
    mapPtr_->clear(costLayer_);

    const auto &map = mapPtr_->get(mapLayer_);
    auto &costMap = mapPtr_->get(costLayer_);

    Index goalIndex, robotIndex;
    mapPtr_->getIndex(goalPosition_, goalIndex);
    mapPtr_->getIndex(robotPosition_, robotIndex);

    if (!mapPtr_->isInside(goalPosition_))
    {
        std::cout << "Goal out of map region!" << std::endl;
        return false;
    }

    std::queue<Index> gridSearchAreas;
    gridSearchAreas.push(goalIndex);

    mapPtr_->atPosition(costLayer_, goalPosition_) = 1.0;
    maxCost_ = 1.0;

    while (!gridSearchAreas.empty())
    {
        Position currentCellPosition;
        const auto currentCellIndex = gridSearchAreas.front();
        gridSearchAreas.pop();
        mapPtr_->getPosition(currentCellIndex, currentCellPosition);

        Index searchSize(1, 1);
        Index searchArea(3, 3);
        SubmapIterator searchIter(*mapPtr_, currentCellIndex - searchSize, searchArea);
        for (searchIter; !searchIter.isPastEnd(); ++searchIter)
        {
            const auto &searchIndex = *searchIter;
            Position searchPosition;
            mapPtr_->getPosition(searchIndex, searchPosition);

            // only search nearby cell
            if (searchIndex.isApprox(currentCellIndex))
                continue;

            // only search in given region
            if (!std::isfinite(mapPtr_->atPosition(mapLayer_, searchPosition)))
                continue;

            // only search in region between goal and robot
            const auto midPoint = (goalPosition_ + robotPosition_) / 2;
            double margin = 3.0;
            if (getDistance(midPoint, searchPosition) > searchRadius + margin)
                continue;

            auto &searchCost = costMap(searchIndex(0), searchIndex(1));
            const auto &currentCellCost = costMap(currentCellIndex(0), currentCellIndex(1));
            const auto &transitionCost = getDistance(currentCellPosition, searchPosition);
            if (!std::isfinite(searchCost))
            {
                searchCost = currentCellCost + transitionCost;
                gridSearchAreas.push(searchIndex);
                if (searchCost > maxCost_)
                    maxCost_ = searchCost;
            }
            else if (searchCost > currentCellCost + transitionCost)
            {
                searchCost = currentCellCost + transitionCost;
            }
        }
    }

    if (!std::isfinite(costMap(robotIndex(0), robotIndex(1))))
    {
        std::cout << "Could not find valid path from goal to robot" << std::endl;
        return false;
    }

    return true;
}

bool DijkstraSearch::findPath()
{
    auto costMap = mapPtr_->get(costLayer_);

    Index robotIndex, goalIndex;
    mapPtr_->getIndex(robotPosition_, robotIndex);
    mapPtr_->getIndex(goalPosition_, goalIndex);

    std::queue<Index> pathCandidates;
    pathCandidates.push(robotIndex);

    while (!pathCandidates.empty())
    {
        const auto currentCellIndex = pathCandidates.front();

        if (currentCellIndex.isApprox(goalIndex))
        {
            std::cout << "found path!" << std::endl;
            return true;
        }

        Index pathIndex;
        bool validNearPathFound = false;
        double minCost = costMap(currentCellIndex(0), currentCellIndex(1));

        Index searchSize(1, 1);
        Index searchArea(3, 3);
        SubmapIterator searchIter(*mapPtr_, currentCellIndex - searchSize, searchArea);
        for (searchIter; !searchIter.isPastEnd(); ++searchIter)
        {
            const auto &searchIndex = *searchIter;

            // pass invalid cost (unexplored)
            if (!std::isfinite(costMap(searchIndex(0), searchIndex(1))))
                continue;

            if (searchIndex.isApprox(currentCellIndex))
                continue;

            const auto &searchCost = costMap(searchIndex(0), searchIndex(1));
            if (searchCost < minCost)
            {
                minCost = searchCost;
                pathIndex = searchIndex;
                validNearPathFound = true;
            }
        }
        if (!validNearPathFound)
        {
            std::cout << "No valid path found with constructing cost map" << std::endl;
            return false;
        }

        pathCandidates.push(pathIndex);
        pathCandidates.pop();
        Position pathPosition;
        mapPtr_->getPosition(pathIndex, pathPosition);
        path_.push_back(pathPosition);
    }

    std::cout << "searched all path Candidates. No path available" << std::endl;
    return false;
}
