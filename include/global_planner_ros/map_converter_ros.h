//
// Created by Ikhyeon Cho on 22. 4. 19.
//

#ifndef GRIDMAP_NAVIGATION_MAP_CONVERTER_H
#define GRIDMAP_NAVIGATION_MAP_CONVERTER_H

#include <ros/ros.h>
#include <nav_msgs/GetMap.h>
#include <grid_map_ros/GridMapRosConverter.hpp>

class mapConverterRos
{
public:
    mapConverterRos(ros::NodeHandle &_nh);
    ~mapConverterRos() = default;

    bool queryOccupancyMap();
    bool convertOccupancyToGridmapLayer(grid_map::GridMap &gridmap, const std::string &layer);
    void gridInflation(grid_map::GridMap &gridmap, const std::string &layer_in, const std::string &layer_out,
                       int inflate_state, int inflation_size);

private:
    ros::ServiceClient map_client_;
    nav_msgs::GetMap query_map_;
    nav_msgs::OccupancyGrid occupancymap_;

private:
};

#endif // GRIDMAP_NAVIGATION_MAP_CONVERTER_H

mapConverterRos::mapConverterRos(ros::NodeHandle &nh)
{
    map_client_ = nh.serviceClient<nav_msgs::GetMap>("/static_map");
}

bool mapConverterRos::queryOccupancyMap()
{
    if (!map_client_.call(query_map_))
        return false;

    ROS_INFO("Static map recieved. Start conversion to Grid map structure");
    occupancymap_ = query_map_.response.map;
    return true;
}

bool mapConverterRos::convertOccupancyToGridmapLayer(grid_map::GridMap &gridmap, const std::string &layer)
{
    bool converted = grid_map::GridMapRosConverter::fromOccupancyGrid(occupancymap_, layer, gridmap);
    return converted;
}

void mapConverterRos::gridInflation(grid_map::GridMap &gridmap, const std::string &layer_in, const std::string &layer_out,
                                    int inflate_state, int inflation_size)
{
    const auto &map_data = gridmap[layer_in];
    auto &inflated_data = gridmap[layer_out];

    // copy first
    gridmap[layer_out] = gridmap[layer_in];

    // inflate
    for (grid_map::GridMapIterator iter(gridmap); !iter.isPastEnd(); ++iter)
    {
        size_t i = iter.getLinearIndex();
        const auto &cell_index = *iter;
        const float cell_state = map_data(i);
        float &inflated_state = inflated_data(i);

        if (!std::isfinite(cell_state))
            continue;

        if (std::abs(cell_state - inflate_state) > FLT_EPSILON)
            continue;

        grid_map::Index start_index(inflation_size, inflation_size);
        grid_map::Index inflation_buffer(2 * inflation_size + 1, 2 * inflation_size + 1);
        grid_map::SubmapIterator sub_iter(gridmap, cell_index - start_index, inflation_buffer);
        for (sub_iter; !sub_iter.isPastEnd(); ++sub_iter)
        {
            // check out of range while searching
            grid_map::Position search_pos;
            if (!gridmap.getPosition(*sub_iter, search_pos))
                continue;

            gridmap.at(layer_out, *sub_iter) = cell_state;
        }
    }
}
