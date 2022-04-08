//
// Created by isr on 22. 4. 8..
//

#include "elasticBands.hpp"

#ifndef GRIDMAP_NAVIGATION_ELASTICBANDS_HPP
#define GRIDMAP_NAVIGATION_ELASTICBANDS_HPP

// grid map library
#include <grid_map_core/GridMap.hpp>
#include <grid_map_core/grid_map_core.hpp>

#include <algorithm>
#include <iostream>

using namespace grid_map;

typedef struct _2DBUBBLE_
{
    Position position;
    int index;
    double radius;
} BubbleXYIR;

bool compareIndex(BubbleXYIR bubble1, BubbleXYIR bubble2)
{
    return bubble1.index < bubble2.index;
}

float getDist(const Position &pos1, const Position &pos2)
{
    return (float)sqrt(pow(pos1.x() - pos2.x(), 2) + pow(pos1.y() - pos2.y(), 2));
}

bool bubbleOverlaps(const BubbleXYIR &bubble1, const BubbleXYIR &bubble2)
{
    if (bubble1.radius + bubble2.radius < getDist(bubble1.position, bubble2.position))
        return false;
    else
        return true;
}
class elasticBands
{

    const double EPSILON = 0.0001;
    const int FREE = 0;
    const int OCCUPIED = 100;
    const double MAX_DISTANCE = 10;
    const double MIN_BUBBLE_SIZE = 0.5;
    const double FORCE_SCALING_FACTOR = 1.2;
    const double GLOBAL_CONTRACTION_GAIN = 0.5;
    const double GLOBAL_REPULSION_GAIN = -11.0;

public:
    elasticBands(const GridMap &occupancyMap, const std::string &layer, const std::vector<Position> &poseList);
    ~elasticBands();

    bool updateElasticBand();

private:
    bool createElasticBand();
    double minDistFromObstacle(const Position &position);
    bool isValidBubbleSize(const BubbleXYIR &bubble);
    void recursiveFilter(const BubbleXYIR &startBubble, const BubbleXYIR &endBubble);

    // update
    void viaPointsUpdate();

    void getTotalForce(const BubbleXYIR &prev, const BubbleXYIR &curr, const BubbleXYIR &next, Position &totalForce);
    Position getRepulsiveForce(const BubbleXYIR &currentBubble);
    Position getContractionForce(const BubbleXYIR &prev, const BubbleXYIR &curr, const BubbleXYIR &next);

    void deleteBubbleWhenDense();
    void addBubbleWhenSparse();

private:
    GridMap map_;
    std::string layer_;
    std::vector<Position> pathList_;
    std::vector<BubbleXYIR> EBraw_;
    std::vector<BubbleXYIR> EBfiltered_;
    int startBubbleIndex_;
    int endBubbleIndex_;
};

#endif // GRIDMAP_NAVIGATION_ELASTICBANDS_HPP

elasticBands::elasticBands(const GridMap &occupancyMap, const std::string &layer, const std::vector<Position> &poseList)
{
    if (poseList.empty())
        throw std::length_error("pose Length is zero. Empty path");

    map_ = occupancyMap;
    layer_ = layer;
    pathList_ = poseList;

    if (!createElasticBand())
        throw std::runtime_error("creating bubble failed");

    startBubbleIndex_ = EBraw_.front().index;
    endBubbleIndex_ = EBraw_.back().index;
}

double elasticBands::minDistFromObstacle(const Position &position)
{
    double minDist = MAX_DISTANCE;

    Index currentGridIndex;
    map_.getIndex(position, currentGridIndex);

    Index startIndex(MAX_DISTANCE);
    Index searchSize(2 * MAX_DISTANCE + 1, 2 * MAX_DISTANCE + 1);
    SubmapIterator searchIter(map_, currentGridIndex - startIndex, searchSize);
    for (searchIter; !searchIter.isPastEnd(); ++searchIter)
    {
        const Index &searchIndex = *searchIter;
        if (!map_.isInside(*searchIter))
            continue;

        if (searchIndex.isApprox(currentGridIndex))
            continue;

        const auto &cellState = map_.at(layer_, *searchIter);
        // pass unknown cell
        if (!std::isfinite(cellState))
            continue;

        // pass free cell
        if (std::abs(cellState - FREE) < EPSILON)
            continue;

        Position searchPosition;
        map_.getPosition(*searchIter, searchPosition);
        double distance = getDist(position, searchPosition);
        if (distance < minDist)
            minDist = distance;
    }

    return minDist;
}

bool elasticBands::createElasticBand()
{
    // fill in EBContainer
    for (int i = 0; i < pathList_.size(); ++i)
    {
        const auto &waypoint = pathList_[i];
        BubbleXYIR bubble;
        bubble.position = waypoint;
        bubble.index = i;
        bubble.radius = minDistFromObstacle(waypoint);

        EBraw_.push_back(bubble);
    }
    // fill in EBfiltered
    const auto startBubble = EBraw_.front();
    const auto endBubble = EBraw_.back();
    recursiveFilter(startBubble, endBubble);

    if (EBfiltered_.empty())
    {
        std::cout << "Filtered elabtic bands has no elements after recursive construction." << std::endl;
        return false;
    }

    for (auto bubble : EBfiltered_)
    {
        if (!isValidBubbleSize(bubble))
            return false;
    }

    std::sort(EBfiltered_.begin(), EBfiltered_.end(), compareIndex);
    
    // after sort, index reallocation
    for (int i = 0; i < EBfiltered_.size(); ++i)
    {
        auto &bubble = EBfiltered_[i];
        bubble.index = i;    
    }
    
    return true;
}

void elasticBands::recursiveFilter(const BubbleXYIR &startBubble, const BubbleXYIR &endBubble)
{
    int midIndex = (startBubble.index + endBubble.index) / 2;
    const auto midBubble = EBraw_.at(midIndex);
    if (midIndex == startBubble.index || midIndex == endBubble.index)
        return;

    if (bubbleOverlaps(startBubble, endBubble))
        return;

    EBfiltered_.push_back(midBubble);

    recursiveFilter(startBubble, midBubble);
    recursiveFilter(midBubble, endBubble);
}

bool elasticBands::isValidBubbleSize(const BubbleXYIR &bubble)
{
    if (bubble.radius < MIN_BUBBLE_SIZE)
        return false;

    // passed all condition, then true
    return true;
}

bool elasticBands::updateElasticBand()
{
    viaPointsUpdate();

    deleteBubbleWhenDense();

    addBubbleWhenSparse();

    // insert bubbles

    return true;
}

void elasticBands::viaPointsUpdate()
{
    // viaPoint update
    for (int bubbleIndex = 0; bubbleIndex < EBfiltered_.size(); ++bubbleIndex)
    {
        // pass first and last bubble for viapoint update
        if (bubbleIndex == EBfiltered_.front().index || bubbleIndex == EBfiltered_.back().index)
            continue;

        Position viaPoint, totalForce;

        auto &bubbleCurr = EBfiltered_[bubbleIndex]; // non const
        const auto &bubblePrev = EBfiltered_[bubbleIndex - 1];
        const auto &bubbleNext = EBfiltered_[bubbleIndex + 1];

        getTotalForce(bubblePrev, bubbleCurr, bubbleNext, totalForce);
        bubbleCurr.position += totalForce * FORCE_SCALING_FACTOR;
    }
}

void elasticBands::getTotalForce(const BubbleXYIR &prev, const BubbleXYIR &curr, const BubbleXYIR &next, Position &totalForce)
{
    totalForce = getRepulsiveForce(curr) + getContractionForce(prev, curr, next);
}

Position elasticBands::getRepulsiveForce(const BubbleXYIR &currentBubble)
{
    Position repulsiveForce;

    const auto &radius = currentBubble.radius;
    const auto &position = currentBubble.position;

    if (radius > MAX_DISTANCE)
        repulsiveForce.setZero();

    else
    {
        Position dx(radius, 0);
        Position dy(0, radius);

        const auto constant = (MAX_DISTANCE - radius) / (2 * radius);
        repulsiveForce.x() = GLOBAL_REPULSION_GAIN * (constant * minDistFromObstacle(position - dx) - minDistFromObstacle(position + dx));
        repulsiveForce.y() = GLOBAL_REPULSION_GAIN * (constant * minDistFromObstacle(position - dy) - minDistFromObstacle(position + dy));
    }

    return repulsiveForce;
}

Position elasticBands::getContractionForce(const BubbleXYIR &prev, const BubbleXYIR &curr, const BubbleXYIR &next)
{
    Position contractionForce;

    const auto &previousVector = prev.position;
    const auto &currentVector = curr.position;
    const auto &nextVector = next.position;

    const auto VectorLength1 = getDist(previousVector, currentVector);
    const auto VectorLength2 = getDist(currentVector, nextVector);

    contractionForce = GLOBAL_CONTRACTION_GAIN * ((previousVector - currentVector) / VectorLength1 +
                                                  (nextVector - currentVector) / VectorLength2);

    return contractionForce;
}

void elasticBands::deleteBubbleWhenDense()
{
    for (int bubbleIndex = 0; bubbleIndex < EBfiltered_.size(); ++bubbleIndex)
    {
        // pass first and last bubble for viapoint update
        if (bubbleIndex == startBubbleIndex_ || bubbleIndex == endBubbleIndex_)
            continue;
    }
}