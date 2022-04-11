//
// Created by ikhyeon Cho on 22. 4. 8..
//
#ifndef GRIDMAP_NAVIGATION_ELASTICBANDS_HPP
#define GRIDMAP_NAVIGATION_ELASTICBANDS_HPP

// grid map library
#include <grid_map_core/GridMap.hpp>
#include <grid_map_core/grid_map_core.hpp>

#include <algorithm>
#include <iostream>

using namespace grid_map;



class ElasticBands
{
    const double EPSILON = 0.0001;
    const int FREE = 0;
    const int OCCUPIED = 100;

    const double BETWEEN_THE_BANDS = 0.4;

    const double FORCE_SCALING_FACTOR = 1.2;
    const double GLOBAL_CONTRACTION_GAIN = 0.5;
    const double GLOBAL_REPULSION_GAIN = -11.0;


class BubbleXYIR
{
private:
    Position position_;
    int index_;
    double radius_;

public:
    const double MAX_BUBBLE_RADIUS_M = 1.5;
    const double MIN_BUBBLE_RADIUS_M = 0.2;
    BubbleXYIR(const Position &bubblePos, double bubbleRadius, int bubbleIndex)
    {
        position_ = bubblePos;
        radius_ = bubbleRadius;
        index_ = bubbleIndex;
    }
    ~BubbleXYIR() = default;

    const Position &getPosition() const
    {
        return position_;
    }
    void setPosition(const Position &position)
    {
        position_ = position;
    }
    const double &getRadius() const
    {
        return radius_;
    }
    void setRadius(const double radius)
    {
        radius_ = radius;
    }
    const int &getIndex() const
    {
        return index_;
    }
    void setIndex(int index)
    {
        index_ = index;
    }
    bool isInside(const BubbleXYIR &bubble)
    {
        if (getDist(position_, bubble.getPosition()) < bubble.getRadius())
            return true;
        else
            return false;
    }

    bool hasValidSize() const
    {
        if (radius_ < MIN_BUBBLE_RADIUS_M)
            return false;

        if (radius_ > MAX_BUBBLE_RADIUS_M)
            return false;

        // passed all condition, then true
        return true;
    }
};

public:
    ElasticBands(const std::vector<Position> &poseList, const GridMap &occupancyMap, const std::string &layer);
    ~ElasticBands();

    bool updateElasticBand();
    const std::vector<BubbleXYIR> &getEbands() const
    {
        return eband_;
    }
    std::string getMapFrame() const
    {
        return map_.getFrameId();
    }

private:
    // bubble radius: min distance to obstacle
    bool
    createElasticBand();
    double minDistToObstacle(const Position &position);
    static float getDist(const Position &pos1, const Position &pos2)
    {
        return (float)sqrt(pow(pos1.x() - pos2.x(), 2) + pow(pos1.y() - pos2.y(), 2));
    }
    void recursiveFilter(const std::vector<BubbleXYIR> &ebandRaw, const BubbleXYIR &startBubble, const BubbleXYIR &endBubble);

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
    std::vector<BubbleXYIR> eband_;

private:
    bool bubbleOverlaps(const BubbleXYIR &bubble1, const BubbleXYIR &bubble2) const
    {
        if (bubble1.getRadius() + bubble2.getRadius() < getDist(bubble1.getPosition(), bubble2.getPosition()) / BETWEEN_THE_BANDS)
            return false;
        else
            return true;
    }

public:
    static bool compareIndex(const BubbleXYIR& bubble1, const BubbleXYIR& bubble2)
    {
        return bubble1.getIndex() < bubble2.getIndex();
    }
};

#endif // GRIDMAP_NAVIGATION_ELASTICBANDS_HPP
