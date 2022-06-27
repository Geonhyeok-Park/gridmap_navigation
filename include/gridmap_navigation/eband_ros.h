//
// Created by Ikhyeon Cho on 22. 4. 8..
//
#ifndef GRIDMAP_NAVIGATION_ELASTICBANDS_H
#define GRIDMAP_NAVIGATION_ELASTICBANDS_H

#include <gridmap_navigation/costmap_ros.h>
#include <vector>

#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Path.h>

namespace grid_map
{
    class BubbleXYRD
    {
    private:
        Position position_;
        double radius_;
        double dist_from_robot_;

    public:
        static double radius_max;
        static double radius_min;

    public:
        BubbleXYRD(const Position &_position, double _dist_from_robot);
        ~BubbleXYRD() = default;

        const double &getDistanceFromRobot() const { return dist_from_robot_; };
        const double &getRadius() const { return radius_; };
        const Position &getPosition() const { return position_; };
        void setPosition(const Position &pos) { position_ = pos; };
        void setDistanceFromRobot(double _distance) { dist_from_robot_ = _distance; };

        float getDistance(const Position &pos1, const Position &pos2) { return std::sqrt(std::pow(pos1.x() - pos2.x(), 2) + std::pow(pos1.y() - pos2.y(), 2)); }

        bool overlaps(const Position &_position) { return getDistance(position_, _position) < radius_ && (position_ - _position).norm() > 0.001 ? true : false; };

        Position moveFrom(const Position &obstacle);

        Position searchNearestObstacleFrom(const GridMap &localmap);
    };

    double BubbleXYRD::radius_min = 0.4;
    double BubbleXYRD::radius_max = 0.8;

    class ElasticBand
    {
    private:
        std::vector<BubbleXYRD> eband_;
        Position robot_position_;

    public:
        ElasticBand(const std::vector<Position> &_path, double _range);
        ~ElasticBand() = default;

        const std::vector<BubbleXYRD> &getEband() const { return eband_; };

        static bool compareRange(const BubbleXYRD &_b1, const BubbleXYRD &_b2) { return _b1.getDistanceFromRobot() < _b2.getDistanceFromRobot(); };
        bool update(const GridMap &_localmap, const std::unique_ptr<Costmap> &_underlying_map);
        void toRosMsg(visualization_msgs::MarkerArray &bubble_msg, nav_msgs::Path &path_msg);

    private:
        float getDistance(const Position &pos1, const Position &pos2) { return std::sqrt(std::pow(pos1.x() - pos2.x(), 2) + std::pow(pos1.y() - pos2.y(), 2)); }
    };

}

#endif