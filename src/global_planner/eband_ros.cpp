#include <gridmap_navigation/eband_ros.h>

namespace grid_map
{
    BubbleXYRD::BubbleXYRD(const Position &_position, double _dist_from_robot)
    {
        position_ = _position;
        radius_ = radius_max;
        dist_from_robot_ = _dist_from_robot;
    }
    Position BubbleXYRD::moveFrom(const Position &obstacle)
    {
        Position direction_vector = (position_ - obstacle);
        auto norm = direction_vector.norm();
        direction_vector.stableNormalize();

        return position_ + direction_vector * (radius_ - norm);
    }

    Position BubbleXYRD::searchNearestObstacleFrom(const GridMap &localmap)
    {
        Index bubble_grid_index;
        localmap.getIndex(position_, bubble_grid_index);
        const auto &obstacle_map = localmap.get("inflation");
        // const auto &obstacle_map = localmap.get("occupancy"); // TODO: layer has to be changed

        SpiralIterator search_iterator(localmap, position_, radius_);
        for (search_iterator; !search_iterator.isPastEnd(); ++search_iterator)
        {
            const Index &search_index = *search_iterator;
            Position search_position;

            // skip ouf of map size
            if (!localmap.getPosition(search_index, search_position))
                continue;

            // skip unknown and free grid cells
            const auto &search_state = obstacle_map(search_index(0), search_index(1));
            if (!std::isfinite(search_state) || search_state < FLT_EPSILON)
                continue;

            // when find obstacles
            return search_position;
        }

        // when no obstacles found, return bubble position
        return position_;
    }

    ElasticBand::ElasticBand(const std::vector<Position> &_path, double _range)
    {
        robot_position_ = _path.front();
        for (const auto &waypoint : _path)
        {
            auto distance_from_robot = getDistance(waypoint, robot_position_);
            if (distance_from_robot > _range)
                continue;
            eband_.emplace_back(waypoint, distance_from_robot);
        }
        // std::sort(eband_.begin(), eband_.end(), compareRange);
    }

    bool ElasticBand::update(const GridMap &_localmap, const std::unique_ptr<Costmap> &_underlying_map)
    {
        const auto &layers = _underlying_map->getLayers();
        const auto &occupancymap = (std::find(layers.begin(), layers.end(), "inflation") != layers.end()
                                        ? _underlying_map->get("inflation")
                                        : _underlying_map->get("occupancy"));
        for (auto &bubble : eband_)
        {
            Position obstacle = bubble.searchNearestObstacleFrom(_localmap);
            if (!bubble.overlaps(obstacle))
                continue;

            Position updated_position = bubble.moveFrom(obstacle);
            Index updated_position_index;
            if (!_underlying_map->getIndex(updated_position, updated_position_index))
            {
                ROS_WARN("[EB Update] Updated Bubble is Out of Map.");
                return false;
            }
            // cannot move to unknown grd and occupied grid
            const auto &updated_state = occupancymap(updated_position_index(0), updated_position_index(1));
            if (!std::isfinite(updated_state) || updated_state > 0)
            {
                ROS_WARN("[EB Update] Updated Bubble is on Invalid Region.");
                ROS_WARN_STREAM(updated_position(0) << " " << updated_position(1));
                return false;
            }

            // set position & set distance from robot
            bubble.setPosition(updated_position);
            bubble.setDistanceFromRobot(getDistance(updated_position, robot_position_));
        }

        // std::sort(eband_.begin(), eband_.end(), compareRange);
        return true;
    }

    void ElasticBand::toRosMsg(visualization_msgs::MarkerArray &bubble_msg, nav_msgs::Path &path_msg)
    {
        bubble_msg.markers.clear();
        bubble_msg.markers.resize(eband_.size());

        path_msg.poses.clear();
        path_msg.poses.resize(eband_.size());

        path_msg.header.frame_id = "map";
        path_msg.header.seq++;
        path_msg.header.stamp = ros::Time::now();

        for (int i = 0; i < eband_.size(); ++i)
        {
            visualization_msgs::Marker bubble;

            // shape and header
            bubble.type = visualization_msgs::Marker::SPHERE;
            bubble.action = visualization_msgs::Marker::ADD;
            bubble.header.frame_id = "map";
            bubble.header.stamp = ros::Time::now();
            bubble.ns = "local";
            bubble.id = i;

            path_msg.poses.at(i).header.frame_id = "map";
            path_msg.poses.at(i).header.stamp = ros::Time::now();

            bubble.pose.position.x = eband_.at(i).getPosition().x();
            bubble.pose.position.y = eband_.at(i).getPosition().y();
            bubble.pose.position.z = 0;
            bubble.pose.orientation.x = 0;
            bubble.pose.orientation.y = 0;
            bubble.pose.orientation.z = 0;
            bubble.pose.orientation.w = 0;

            path_msg.poses.at(i).pose = bubble.pose;

            // color
            bubble.color.a = 0.3;
            bubble.color.r = 0.0;
            bubble.color.g = 0.5;
            bubble.color.b = 0.0;

            // size
            bubble.scale.x = eband_.at(i).getRadius() * 2.0;
            bubble.scale.y = eband_.at(i).getRadius() * 2.0;
            bubble.scale.z = 0.05;

            bubble.lifetime = ros::Duration();

            bubble_msg.markers.at(i) = bubble;
        }
    }
}