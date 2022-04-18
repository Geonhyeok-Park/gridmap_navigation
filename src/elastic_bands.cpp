//
// Created by Ikhyeon Cho on 22. 4. 8..
//
#include <global_planner_ros/elastic_bands.h>
#include <iostream>

ElasticBands::BubbleXYIR::BubbleXYIR(const Position &_bubble_position, double _bubble_radius, int _bubble_index)
{
    position_ = _bubble_position;
    radius_ = _bubble_radius;
    index_ = _bubble_index;
}

const Position &ElasticBands::BubbleXYIR::getPosition() const
{
    return position_;
}

void ElasticBands::BubbleXYIR::setPosition(const Position &_position)
{
    position_ = _position;
}

const double &ElasticBands::BubbleXYIR::getRadius() const
{
    return radius_;
}

void ElasticBands::BubbleXYIR::setRadius(double _radius)
{
    radius_ = _radius;
}

const int &ElasticBands::BubbleXYIR::getIndex() const
{
    return index_;
}

void ElasticBands::BubbleXYIR::setIndex(int _index)
{
    index_ = _index;
}

bool ElasticBands::BubbleXYIR::isInside(const BubbleXYIR &_bubble)
{
    if (getDist(position_, _bubble.getPosition()) < _bubble.getRadius())
        return true;
    else
        return false;
}

bool ElasticBands::BubbleXYIR::hasValidSize() const
{
    if (radius_ < MIN_BUBBLE_RADIUS_M)
        return false;

    if (radius_ > MAX_BUBBLE_RADIUS_M)
        return false;

    return true;
}

const std::vector<ElasticBands::BubbleXYIR> &ElasticBands::getBubbles() const
{
    return eband_;
}

std::string ElasticBands::getMapFrame() const
{
    return map_ptr_->getFrameId();
}

ElasticBands::ElasticBands(const std::vector<Position> &_path, GridMap &_occupancymap,
                           const std::string &_layer)
{
    if (_path.empty())
        throw std::length_error("pose Length is zero. Empty path");

    map_ptr_ = &_occupancymap;
    layer_ = _layer;
    path_ = _path;

    if (!createElasticBand())
        throw std::runtime_error("creating elastic bands failed");
}

ElasticBands::~ElasticBands()
{
    layer_.clear();
    path_.clear();
    eband_.clear();
}

double ElasticBands::minDistToObstacle(const Position &_position)
{
    BubbleXYIR bubble_no_use(_position, 0, -1);
    double min_dist = bubble_no_use.MAX_BUBBLE_RADIUS_M;
    const auto &max_search_radius = bubble_no_use.MAX_BUBBLE_RADIUS_M;

    bool get_submap;
    auto submap = map_ptr_->getSubmap(_position, Length(max_search_radius, max_search_radius), get_submap);

    Index index_current_grid;
    submap.getIndex(_position, index_current_grid);

    CircleIterator search_iter(submap, _position, max_search_radius);
    for (search_iter; !search_iter.isPastEnd(); ++search_iter)
    {
        const Index &index_search = *search_iter;
        Position position_search;
        if (!map_ptr_->getPosition(index_search, position_search))
            continue;

        if (index_search.isApprox(index_current_grid))
            continue;

        const auto &cell_state = map_ptr_->at(layer_, *search_iter);
        // pass unknown cell
        if (!std::isfinite(cell_state))
            continue;

        // pass free cell
        if (std::abs(cell_state - FREE) < EPSILON)
            continue;

        // treat non-free cell as obstacle
        double distance = getDist(_position, position_search);
        if (distance < min_dist)
            min_dist = distance;
    }

    return min_dist;
}

bool ElasticBands::createElasticBand()
{
    std::vector<BubbleXYIR> eband_raw;
    // fill in EBContainer: no radius calculation yet (set -1);
    for (int i = 0; i < path_.size(); ++i)
    {
        const auto &waypoint = path_[i];
        BubbleXYIR bubble(waypoint, -1, i);

        eband_raw.push_back(bubble);
    }

    // fill in EBfiltered
    const auto bubble_start = eband_raw.front();
    const auto bubble_end = eband_raw.back();
    eband_.push_back(bubble_start);
    eband_.push_back(bubble_end);
    recursiveFilter(eband_raw, bubble_start, bubble_end);

    if (!eband_.empty())
    {
        std::sort(eband_.begin(), eband_.end(), compareIndex);
        // after sort, index reallocation
        for (int i = 0; i < eband_.size(); ++i)
        {
            auto &bubble = eband_[i];
            bubble.setIndex(i);
        }

        return true;
    }
    else
    {
        std::cout << "Filtered elabtic bands has no elements after recursive construction." << std::endl;
        return false;
    }
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

// TODO: make function real boolean
bool ElasticBands::updateElasticBand()
{
    updateBubbles();

    deleteBubbleWhenDense();

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

        Position totalForce;

        auto &bubble_curr = eband_[bubble_index];
        const auto &bubble_prev = eband_[bubble_index - 1];
        const auto &bubble_next = eband_[bubble_index + 1];

        getTotalForce(bubble_prev, bubble_curr, bubble_next, totalForce);
        bubble_curr.setPosition(bubble_curr.getPosition() + totalForce * FORCE_SCALING_FACTOR);
        bubble_curr.setRadius(minDistToObstacle(bubble_curr.getPosition()));
        if (!bubble_curr.hasValidSize())
        {
            std::cout << "Non-valid size bubble exists. return empty eband" << std::endl;
            eband_.clear();
            return;
        }
    }
}

void ElasticBands::getTotalForce(const BubbleXYIR &_prev, const BubbleXYIR &_curr, const BubbleXYIR &_next,
                                 Position &_force_total)
{
    _force_total = getRepulsiveForce(_curr) + getContractionForce(_prev, _curr, _next);
}

// TODO: apply repulsive force first, then apply contraction force.
Position ElasticBands::getRepulsiveForce(const BubbleXYIR &bubble_current)
{
    Position force_repulsive;

    const auto &radius = bubble_current.getRadius();
    const auto &position = bubble_current.getPosition();

    if (radius > bubble_current.MAX_BUBBLE_RADIUS_M)
        force_repulsive.setZero();

    else
    {
        Position dx(radius, 0);
        Position dy(0, radius);

        const auto radius_diff_coeff = (bubble_current.MAX_BUBBLE_RADIUS_M - radius);
        double radius_derivative_x = (minDistToObstacle(position - dx) - minDistToObstacle(position + dx)) / (2 * radius);
        double radius_derivative_y = (minDistToObstacle(position - dy) - minDistToObstacle(position + dy)) / (2 * radius);

        force_repulsive.x() = GLOBAL_REPULSION_GAIN * radius_diff_coeff * radius_derivative_x;
        force_repulsive.y() = GLOBAL_REPULSION_GAIN * radius_diff_coeff * radius_derivative_y;
    }

    return force_repulsive;
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

void ElasticBands::addBubbleWhenSparse()
{
    const auto index_end = eband_.size();
    for (int bubble_index = 0; bubble_index < index_end;)
    {
        auto front = bubble_index;
        auto back = bubble_index + 1;
        auto &bubble_front = eband_[front];
        auto &bubble_back = eband_[back];
        if (bubbleOverlaps(bubble_front, bubble_back))
        {
            ++bubble_index;
            continue;
        }
        else
        {
            Position mid = (bubble_front.getPosition() + bubble_back.getPosition()) / 2;
            BubbleXYIR bubble_new(mid, minDistToObstacle(mid), -1);

            if (bubble_new.hasValidSize())
                eband_.insert(eband_.begin() + back, bubble_new);
        }
    }
}