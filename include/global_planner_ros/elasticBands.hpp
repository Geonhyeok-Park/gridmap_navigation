//
// Created by Ikhyeon Cho on 22. 4. 8..
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
public:
    class BubbleXYIR
    {
    private:
        Position position_;
        int index_;
        double radius_;

    public:
        double MAX_BUBBLE_RADIUS_M = 1.5;
        double MIN_BUBBLE_RADIUS_M = 0.3;

    public:
        BubbleXYIR(const Position &bubblePos, double bubbleRadius, int bubbleIndex);

        ~BubbleXYIR() = default;

        const Position &getPosition() const;

        void setPosition(const Position &position);

        const double &getRadius() const;

        void setRadius(double radius);

        const int &getIndex() const;

        void setIndex(int index);

        bool isInside(const BubbleXYIR &bubble);

        bool hasValidSize() const;
    };

private:
    const double EPSILON = 0.0001;
    const int FREE = 0;
    const int OCCUPIED = 100;

private:
    const GridMap *mapPtr_;
    std::string layer_;
    std::vector<Position> pathList_;
    std::vector<BubbleXYIR> eband_;

public:
    double BETWEEN_THE_BANDS = 0.4;
    double FORCE_SCALING_FACTOR = 1.2;
    double GLOBAL_CONTRACTION_GAIN = 0.2; // usually half of between the bands
    double GLOBAL_REPULSION_GAIN = -11.0;

public:
    ElasticBands(const std::vector<Position> &poseList, GridMap &occupancyMap, const std::string &layer);

    ~ElasticBands();

    const std::vector<BubbleXYIR> &getBubbles() const;

    bool updateElasticBand();

    std::string getMapFrame() const;

private:
    // bubble radius: min distance to obstacle
    bool createElasticBand();

    double minDistToObstacle(const Position &position);

    void recursiveFilter(const std::vector<BubbleXYIR> &ebandRaw, const BubbleXYIR &startBubble,
                         const BubbleXYIR &endBubble);

    // Update waypoints via force
    void updateBubbles();

    void getTotalForce(const BubbleXYIR &prev, const BubbleXYIR &curr, const BubbleXYIR &next, Position &totalForce);

    Position getRepulsiveForce(const BubbleXYIR &currentBubble);

    Position getContractionForce(const BubbleXYIR &prev, const BubbleXYIR &curr, const BubbleXYIR &next) const;

    // rearrange bubbles
    void deleteBubbleWhenDense();

    void addBubbleWhenSparse();

private:
    static float getDist(const Position &pos1, const Position &pos2)
    {
        return (float)sqrt(pow(pos1.x() - pos2.x(), 2) + pow(pos1.y() - pos2.y(), 2));
    }

    bool bubbleOverlaps(const BubbleXYIR &bubble1, const BubbleXYIR &bubble2) const
    {
        if (bubble1.getRadius() + bubble2.getRadius() <
            getDist(bubble1.getPosition(), bubble2.getPosition()) / BETWEEN_THE_BANDS)
            return false;
        else
            return true;
    }

    static bool compareIndex(const BubbleXYIR &bubble1, const BubbleXYIR &bubble2)
    {
        return bubble1.getIndex() < bubble2.getIndex();
    }
};

#endif // GRIDMAP_NAVIGATION_ELASTICBANDS_HPP
