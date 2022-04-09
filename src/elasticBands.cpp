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

float getDist(const Position &pos1, const Position &pos2)
{
    return (float)sqrt(pow(pos1.x() - pos2.x(), 2) + pow(pos1.y() - pos2.y(), 2));
}

typedef struct _2DBUBBLE_
{
    const double MAX_BUBBLE_RADIUS_M = 1.5;
    const double MIN_BUBBLE_RADIUS_M = 0.2;

    Position position;
    int index;
    double radius;

    bool isInside(const _2DBUBBLE_ &bubble)
    {
        if (getDist(position, bubble.position) < bubble.radius)
            return true;
        else
            return false;
    }

    bool hasValidSize()
    {
        if (radius < MIN_BUBBLE_RADIUS_M)
            return false;

        if (radius > MAX_BUBBLE_RADIUS_M)
            return false;

        // passed all condition, then true
        return true;
    }
} BubbleXYIR;

class elasticBands
{
    const double EPSILON = 0.0001;
    const int FREE = 0;
    const int OCCUPIED = 100;

    const double BETWEEN_THE_BANDS = 0.4;

    const double FORCE_SCALING_FACTOR = 1.2;
    const double GLOBAL_CONTRACTION_GAIN = 0.5;
    const double GLOBAL_REPULSION_GAIN = -11.0;

public:
    elasticBands(const GridMap &occupancyMap, const std::string &layer, const std::vector<Position> &poseList);
    ~elasticBands();

    bool updateElasticBand();

private:
    // bubble radius: min distance to obstacle
    bool createElasticBand();
    double minDistToObstacle(const Position &position);
    void recursiveFilter(const BubbleXYIR &startBubble, const BubbleXYIR &endBubble);

    // Update waypoints via force
    void viaPointsUpdate();
    void getTotalForce(const BubbleXYIR &prev, const BubbleXYIR &curr, const BubbleXYIR &next, Position &totalForce);
    Position getRepulsiveForce(const BubbleXYIR &currentBubble);
    Position getContractionForce(const BubbleXYIR &prev, const BubbleXYIR &curr, const BubbleXYIR &next);

    // rearrange bubbles
    void deleteBubbleWhenDense();
    void addBubbleWhenSparse();

private:
    GridMap map_;
    std::string layer_;
    std::vector<Position> pathList_;
    std::vector<BubbleXYIR> ebandRaw_;
    std::vector<BubbleXYIR> ebandFiltered_;

private:
    bool bubbleOverlaps(const BubbleXYIR &bubble1, const BubbleXYIR &bubble2)
    {
        if (bubble1.radius + bubble2.radius < getDist(bubble1.position, bubble2.position) / BETWEEN_THE_BANDS)
            return false;
        else
            return true;
    }

    bool compareIndex(BubbleXYIR bubble1, BubbleXYIR bubble2)
    {
        return bubble1.index < bubble2.index;
    }
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
        throw std::runtime_error("creating elastic bands failed");
}

double elasticBands::minDistToObstacle(const Position &position)
{
    BubbleXYIR bubbleNoUse;
    double minDist = bubbleNoUse.MAX_BUBBLE_RADIUS_M;

    Index currentGridIndex;
    map_.getIndex(position, currentGridIndex);

    CircleIterator searchIter(map_, position, bubbleNoUse.MAX_BUBBLE_RADIUS_M);
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
        bubble.radius = minDistToObstacle(waypoint);

        ebandRaw_.push_back(bubble);
    }
    // fill in EBfiltered
    const auto startBubble = ebandRaw_.front();
    const auto endBubble = ebandRaw_.back();
    recursiveFilter(startBubble, endBubble);

    if (ebandFiltered_.empty())
    {
        std::cout << "Filtered elabtic bands has no elements after recursive construction." << std::endl;
        return false;
    }

    for (auto bubble : ebandFiltered_)
    {
        if (!bubble.hasValidSize())
            return false;
    }

    std::sort(ebandFiltered_.begin(), ebandFiltered_.end(), compareIndex);

    // after sort, index reallocation
    for (int i = 0; i < ebandFiltered_.size(); ++i)
    {
        auto &bubble = ebandFiltered_[i];
        bubble.index = i;
    }

    return true;
}

void elasticBands::recursiveFilter(const BubbleXYIR &startBubble, const BubbleXYIR &endBubble)
{
    int midIndex = (startBubble.index + endBubble.index) / 2;
    const auto midBubble = ebandRaw_.at(midIndex);
    if (midIndex == startBubble.index || midIndex == endBubble.index)
        return;

    if (bubbleOverlaps(startBubble, endBubble))
        return;

    ebandFiltered_.push_back(midBubble);

    recursiveFilter(startBubble, midBubble);
    recursiveFilter(midBubble, endBubble);
}

// TODO: make function real boolean
bool elasticBands::updateElasticBand()
{
    viaPointsUpdate();

    deleteBubbleWhenDense();

    addBubbleWhenSparse();

    return true;
}

void elasticBands::viaPointsUpdate()
{
    // viaPoint update
    for (int bubbleIndex = 0; bubbleIndex < ebandFiltered_.size(); ++bubbleIndex)
    {
        // pass first and last bubble for viapoint update
        if (bubbleIndex == ebandFiltered_.front().index || bubbleIndex == ebandFiltered_.back().index)
            continue;

        Position viaPoint, totalForce;

        auto &bubbleCurr = ebandFiltered_[bubbleIndex]; // non const
        const auto &bubblePrev = ebandFiltered_[bubbleIndex - 1];
        const auto &bubbleNext = ebandFiltered_[bubbleIndex + 1];

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

    if (radius > MAX_BUBBLE_RADIUS_M)
        repulsiveForce.setZero();

    else
    {
        Position dx(radius, 0);
        Position dy(0, radius);

        const auto constant = (MAX_BUBBLE_RADIUS_M - radius) / (2 * radius);
        repulsiveForce.x() = GLOBAL_REPULSION_GAIN * (constant * minDistToObstacle(position - dx) - minDistToObstacle(position + dx));
        repulsiveForce.y() = GLOBAL_REPULSION_GAIN * (constant * minDistToObstacle(position - dy) - minDistToObstacle(position + dy));
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
    for (int bubbleIndex = 0; bubbleIndex < ebandFiltered_.size())
    {
        // pass first and last bubble
        if (bubbleIndex == ebandFiltered_.front().index || bubbleIndex == ebandFiltered_.back().index)
        {
            ++bubbleIndex;
            continue;
        }

        const auto &bubblePrev = ebandFiltered_[bubbleIndex - 1];
        auto &bubbleCurrent = ebandFiltered_[bubbleIndex];
        const auto &bubbleNext = ebandFiltered_[bubbleIndex + 1];

        if (bubblePrev.isInside(bubbleCurrent) && bubbleCurrent.isInside(bubbleNext))
            ebandFiltered_.erase(ebandFiltered_.begin() + bubbleIndex);
        else
            ++bubbleIndex;
    }
}

void elasticBands::addBubbleWhenSparse()
{
    for (int bubbleIndex = 0; bubbleIndex < ebandFiltered_.size())
    {
        const auto &front = bubbleIndex;
        const auto &back = bubbleIndex + 1;
        const auto &bubbleFront = ebandFiltered_[front];
        const auto &bubbleBack = ebandFiltered_[back];
        if (bubbleOverlaps(bubbleFront, bubbleBack))
        {
            ++bubbleIndex;
            continue;
        }
        else
        {
            BubbleXYIR bubbleNew;
            bubbleNew.position = (bubbleFront.position + bubbleBack.position) / 2;
            bubbleNew.radius = minDistToObstacle(bubble.position);

            if (bubbleNew.hasValidSize())
                ebandFiltered_.insert(ebandFiltered_.begin() + back, bubbleNew);
        }
    }
}