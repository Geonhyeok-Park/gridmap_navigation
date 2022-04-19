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

class ElasticBands
{
public:
    class BubbleXYIR
    {
    private:
        grid_map::Position position_;
        int index_;
        double radius_;

    public:
        double MAX_BUBBLE_RADIUS_M = 1.5;
        double MIN_BUBBLE_RADIUS_M = 0.3;

    public:
        BubbleXYIR(const grid_map::Position &bubblePos, double bubbleRadius, int bubbleIndex);

        ~BubbleXYIR() = default;

        const grid_map::Position &getPosition() const;

        void setPosition(const grid_map::Position &position);

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
    const grid_map::GridMap *map_ptr_;
    std::string layer_;
    std::vector<grid_map::Position> path_;
    std::vector<BubbleXYIR> eband_;

public:
    double BETWEEN_THE_BANDS = 0.4;
    double FORCE_SCALING_FACTOR = 1.2;
    double GLOBAL_CONTRACTION_GAIN = 0.2; // usually half of between the bands
    double GLOBAL_REPULSION_GAIN = -11.0;

public:
    ElasticBands(const std::vector<grid_map::Position> &_path, grid_map::GridMap &_occupancymap, const std::string &_layer);

    ~ElasticBands();

    const std::vector<BubbleXYIR> &getBubbles() const;

    bool updateElasticBand();

    std::string getMapFrame() const;

private:
    // bubble radius: min distance to obstacle
    bool createElasticBand();

    double minDistToObstacle(const grid_map::Position &_position);

    void recursiveFilter(const std::vector<BubbleXYIR> &_eband_raw, const BubbleXYIR &_start_bubble,
                         const BubbleXYIR &_end_bubble);

    // Update waypoints via force
    void updateBubbles();

    void getTotalForce(const BubbleXYIR &_prev, const BubbleXYIR &_curr, const BubbleXYIR &_next, grid_map::Position &_force_total);

    grid_map::Position getRepulsiveForce(const BubbleXYIR &bubble_current);

    grid_map::Position getContractionForce(const BubbleXYIR &_prev, const BubbleXYIR &_curr, const BubbleXYIR &_next) const;

    // rearrange bubbles
    void deleteBubbleWhenDense();

    void addBubbleWhenSparse();

private:
    static float getDist(const grid_map::Position &_pos1, const grid_map::Position &_pos2)
    {
        return (float)sqrt(pow(_pos1.x() - _pos2.x(), 2) + pow(_pos1.y() - _pos2.y(), 2));
    }

    bool bubbleOverlaps(const BubbleXYIR &bubble_1, const BubbleXYIR &bubble_2) const
    {
        if (bubble_1.getRadius() + bubble_2.getRadius() <
            getDist(bubble_1.getPosition(), bubble_2.getPosition()) / BETWEEN_THE_BANDS)
            return false;
        else
            return true;
    }

    static bool compareIndex(const BubbleXYIR &bubble_1, const BubbleXYIR &bubble_2)
    {
        return bubble_1.getIndex() < bubble_2.getIndex();
    }
};

#endif // GRIDMAP_NAVIGATION_ELASTICBANDS_HPP
