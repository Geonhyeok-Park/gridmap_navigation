//
// Created by Ikhyeon Cho on 22. 4. 8..
//
#include <gridmap_navigation/elastic_bands.h>

using namespace grid_map;

/////////////////////////////////////////////////
/////////                            ////////////
/////////        Eband Class         ////////////
/////////                            ////////////
/////////////////////////////////////////////////

ElasticBands::ElasticBands(GridMap &map,
                           const std::string &layer,
                           const std::vector<Position> &path)
{
    // if (path.empty())
    //     throw std::length_error("pose Length is zero. Empty path");

    map_ptr_ = &map;
    layer_ = layer;

    if (!createElasticBand(path))
    {
        
    }
        // throw std::runtime_error("creating elastic bands failed");
}

ElasticBands::~ElasticBands()
{
    layer_.clear();
    eband_.clear();
}

bool ElasticBands::createElasticBand(const std::vector<Position> &path)
{
    if (path.empty())
        return false;
        
    std::vector<BubbleXYIR> eband_raw;
    for (int i = 0; i < path.size(); ++i)
    {
        const auto &waypoint = path[i];
        BubbleXYIR bubble(waypoint, nearestDistance(waypoint), i);

        eband_raw.push_back(bubble);
    }

    // sample waypoints
    const auto bubble_start = eband_raw.front();
    eband_.push_back(bubble_start);

    const auto bubble_end = eband_raw.back();
    eband_.push_back(bubble_end);

    recursiveFilter(eband_raw, bubble_start, bubble_end);

    if (eband_.empty())
    {
        std::cout << "Filtered elabtic bands has no elements after recursive construction." << std::endl;
        return false;
    }

    std::sort(eband_.begin(), eband_.end(), compareIndex);
    // after sort, index reallocation
    for (int i = 0; i < eband_.size(); ++i)
        eband_[i].setIndex(i);

    return true;
}

void ElasticBands::recursiveFilter(const std::vector<BubbleXYIR> &_eband_raw, const BubbleXYIR &_bubble_start,
                                   const BubbleXYIR &_bubble_end)
{
    int index_mid = (_bubble_start.getIndex() + _bubble_end.getIndex()) / 2;
    const auto bubble_mid = _eband_raw.at(index_mid);

    if (index_mid == _bubble_start.getIndex() || index_mid == _bubble_end.getIndex())
        return;

    if (bubbleOverlaps(_bubble_start, _bubble_end))
        return;

    eband_.push_back(bubble_mid);

    recursiveFilter(_eband_raw, _bubble_start, bubble_mid);
    recursiveFilter(_eband_raw, bubble_mid, _bubble_end);
}

double ElasticBands::nearestDistance(const Position &current_position)
{
    // current position
    Index current_grid_index;
    map_ptr_->getIndex(current_position, current_grid_index);

    // obstacle map to search
    const auto &obstacle_map = map_ptr_->get(layer_);

    double distance;
    SpiralIterator search_iter(*map_ptr_, current_position, MAX_BUBBLE_RADIUS_M);
    for (search_iter; !search_iter.isPastEnd(); ++search_iter)
    {
        const Index &search_index = *search_iter;
        Position search_position;

        // pass out of map boundary
        if (!map_ptr_->getPosition(search_index, search_position))
            continue;

        const auto &cell_state = obstacle_map(search_index(0), search_index(1));
        // pass unknown cell
        if (!std::isfinite(cell_state))
            continue;

        // pass free cell
        if (std::abs(cell_state - FREE) < EPSILON)
            continue;

        // when find obstacles
        distance = getDist(current_position, search_position);
        return distance;
    }

    return MAX_BUBBLE_RADIUS_M;
}

// TODO: make function real boolean
bool ElasticBands::update()
{
    updateBubbles();

    // deleteBubbleWhenDense();

    // addBubbleWhenSparse();

    return true;
}

void ElasticBands::updateBubbles()
{
    for (int bubble_index = 0; bubble_index < eband_.size(); ++bubble_index)
    {
        // pass first and last bubble for viapoint update
        if (bubble_index == eband_.front().getIndex() || bubble_index == eband_.back().getIndex())
            continue;

        auto &bubble_curr = eband_[bubble_index];
        const auto &bubble_prev = eband_[bubble_index - 1];
        const auto &bubble_next = eband_[bubble_index + 1];

        Position total_force;
        getTotalForceVector(bubble_prev, bubble_curr, bubble_next, total_force);

        Position unit_normal;
        unit_normal = getUnitNormal(bubble_prev.getPosition(),
                                    bubble_curr.getPosition(),
                                    bubble_next.getPosition());

        total_force = total_force.dot(unit_normal) * unit_normal;
        Position new_position = bubble_curr.getPosition() + FORCE_SCALING_FACTOR * total_force;
        doValidUpdateOrStay(bubble_curr, new_position);

        if (!hasValidSize(bubble_curr))
        {
            std::cout << "Non-valid size bubble exists. return empty eband" << std::endl;
            eband_.clear();
            return;
        }
    }
}

void ElasticBands::getTotalForceVector(const BubbleXYIR &prev, const BubbleXYIR &curr, const BubbleXYIR &next,
                                       Position &total_force)
{
    total_force = getRepulsiveForce(curr) + getContractionForce(prev, curr, next);
}

Position ElasticBands::getRepulsiveForce(const BubbleXYIR &bubble_current)
{
    Position repulsive_force;

    const auto &radius = bubble_current.getRadius();
    const auto &position = bubble_current.getPosition();

    if (radius > MAX_BUBBLE_RADIUS_M)
        repulsive_force.setZero();

    else
    {
        Position dx(radius, 0);
        Position dy(0, radius);

        const auto radius_diff_coeff = (MAX_BUBBLE_RADIUS_M - radius);

        double radius_derivative_x = (nearestDistance(position - dx) - nearestDistance(position + dx)) / (2 * radius);
        double radius_derivative_y = (nearestDistance(position - dy) - nearestDistance(position + dy)) / (2 * radius);

        repulsive_force.x() = GLOBAL_REPULSION_GAIN * radius_diff_coeff * radius_derivative_x;
        repulsive_force.y() = GLOBAL_REPULSION_GAIN * radius_diff_coeff * radius_derivative_y;
    }

    return repulsive_force;
}

Position ElasticBands::getContractionForce(const BubbleXYIR &_prev, const BubbleXYIR &_curr, const BubbleXYIR &_next) const
{
    Position force_contraction;

    const auto &vector_prev = _prev.getPosition();
    const auto &vector_curr = _curr.getPosition();
    const auto &vector_next = _next.getPosition();

    const auto vector_length_1 = getDist(vector_prev, vector_curr);
    const auto vector_length_2 = getDist(vector_curr, vector_next);

    force_contraction = GLOBAL_CONTRACTION_GAIN * ((vector_prev - vector_curr) / vector_length_1 + (vector_next - vector_curr) / vector_length_2) / 2;

    return force_contraction;
}

void ElasticBands::deleteBubbleWhenDense()
{
    for (int bubble_index = 0; bubble_index < eband_.size();)
    {
        // pass first and last bubble
        if (bubble_index == eband_.front().getIndex() || bubble_index == eband_.back().getIndex())
        {
            ++bubble_index;
            continue;
        }

        auto &bubble_prev = eband_[bubble_index - 1];
        auto &bubble_current = eband_[bubble_index];
        auto &bubble_next = eband_[bubble_index + 1];

        if (bubble_prev.isInside(bubble_current) && bubble_current.isInside(bubble_next))
            eband_.erase(eband_.begin() + bubble_index);
        else
            ++bubble_index;
    }
}

// void ElasticBands::addBubbleWhenSparse()
// {
//     const auto index_end = eband_.size();
//     for (int bubble_index = 0; bubble_index < index_end;)
//     {
//         auto front = bubble_index;
//         auto back = bubble_index + 1;
//         auto &bubble_front = eband_[front];
//         auto &bubble_back = eband_[back];
//         if (bubbleOverlaps(bubble_front, bubble_back))
//         {
//             ++bubble_index;
//             continue;
//         }
//         else
//         {
//             Position mid = (bubble_front.getPosition() + bubble_back.getPosition()) / 2;
//             BubbleXYIR bubble_new(mid, vectorFromNearestObstacle(mid), -1);

//             if (bubble_new.hasValidSize())
//                 eband_.insert(eband_.begin() + back, bubble_new);
//         }
//     }
// }

Position ElasticBands::getUnitNormal(const Position &prev, const Position &curr, const Position &next)
{
    Position unit_normal;

    auto tangential_vector = next - prev;
    unit_normal.x() = tangential_vector.y();
    unit_normal.y() = tangential_vector.x() * (-1);

    return unit_normal.normalized();
}

void ElasticBands::doValidUpdateOrStay(BubbleXYIR &bubble, const Position &new_position)
{
    // save current info
    BubbleXYIR current_bubble = bubble;

    bubble.setPosition(new_position);
    bubble.setRadius(nearestDistance(new_position));

    if (!hasValidSize(bubble))
    {
        bubble = current_bubble;
    }
}

/////////////////////////////////////////////////
/////////                            ////////////
/////////        Bubble Class        ////////////
/////////                            ////////////
/////////////////////////////////////////////////

ElasticBands::BubbleXYIR::BubbleXYIR(const Position &_bubble_position, double _bubble_radius, int _bubble_index)
{
    position_ = _bubble_position;
    radius_ = _bubble_radius;
    index_ = _bubble_index;
}

bool ElasticBands::BubbleXYIR::isInside(const BubbleXYIR &_bubble)
{
    if (getDist(position_, _bubble.getPosition()) < _bubble.getRadius())
        return true;
    else
        return false;
}

bool ElasticBands::hasValidSize(const BubbleXYIR &bubble) const
{
    if (!std::isfinite(bubble.getRadius()))
        return false;

    if (bubble.getRadius() < MIN_BUBBLE_RADIUS_M)
        return false;

    if (bubble.getRadius() > MAX_BUBBLE_RADIUS_M)
        return false;

    return true;
}
