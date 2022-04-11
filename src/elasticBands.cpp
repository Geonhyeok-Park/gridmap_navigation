//
// Created by Ikhyeon Cho on 22. 4. 8..
//
#include "gridmap-navigation/elasticBands.hpp"

ElasticBands::BubbleXYIR::BubbleXYIR(const Position &bubblePos, double bubbleRadius, int bubbleIndex) {
    position_ = bubblePos;
    radius_ = bubbleRadius;
    index_ = bubbleIndex;
}

const Position &ElasticBands::BubbleXYIR::getPosition() const {
    return position_;
}

void ElasticBands::BubbleXYIR::setPosition(const Position &position) {
    position_ = position;
}

const double &ElasticBands::BubbleXYIR::getRadius() const {
    return radius_;
}

void ElasticBands::BubbleXYIR::setRadius(double radius) {
    radius_ = radius;
}

const int &ElasticBands::BubbleXYIR::getIndex() const {
    return index_;
}

void ElasticBands::BubbleXYIR::setIndex(int index) {
    index_ = index;
}

bool ElasticBands::BubbleXYIR::isInside(const BubbleXYIR &bubble) {
    if (getDist(position_, bubble.getPosition()) < bubble.getRadius())
        return true;
    else
        return false;
}

bool ElasticBands::BubbleXYIR::hasValidSize() const {
    if (radius_ < MIN_BUBBLE_RADIUS_M)
        return false;

    if (radius_ > MAX_BUBBLE_RADIUS_M)
        return false;

    return true;
}

const std::vector<ElasticBands::BubbleXYIR> &ElasticBands::getEbands() const {
    return eband_;
}

std::string ElasticBands::getMapFrame() const {
    return map_.getFrameId();
}

ElasticBands::ElasticBands(const std::vector<Position> &poseList, const GridMap &occupancyMap,
                           const std::string &layer) {
    if (poseList.empty())
        throw std::length_error("pose Length is zero. Empty path");

    map_ = occupancyMap;
    layer_ = layer;
    pathList_ = poseList;

    if (!createElasticBand())
        throw std::runtime_error("creating elastic bands failed");
}

ElasticBands::~ElasticBands() {
    map_.clearAll();
    layer_.clear();
    pathList_.clear();
    eband_.clear();
}

double ElasticBands::minDistToObstacle(const Position &position) {
    BubbleXYIR bubbleNoUse(position, 0, 0);
    double minDist = bubbleNoUse.MAX_BUBBLE_RADIUS_M;

    Index currentGridIndex;
    map_.getIndex(position, currentGridIndex);

    CircleIterator searchIter(map_, position, bubbleNoUse.MAX_BUBBLE_RADIUS_M);
    for (searchIter; !searchIter.isPastEnd(); ++searchIter) {
        const Index &searchIndex = *searchIter;
        Position searchPosition;
        if (!map_.getPosition(searchIndex, searchPosition))
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

        double distance = getDist(position, searchPosition);
        if (distance < minDist)
            minDist = distance;
    }

    return minDist;
}

bool ElasticBands::createElasticBand() {
    std::vector<BubbleXYIR> ebandRaw;
    // fill in EBContainer
    for (int i = 0; i < pathList_.size(); ++i) {
        const auto &waypoint = pathList_[i];
        BubbleXYIR bubble(waypoint, minDistToObstacle(waypoint), i);

        ebandRaw.push_back(bubble);
    }
    // fill in EBfiltered
    const auto startBubble = ebandRaw.front();
    const auto endBubble = ebandRaw.back();
    recursiveFilter(ebandRaw, startBubble, endBubble);

    if (eband_.empty()) {
        std::cout << "Filtered elabtic bands has no elements after recursive construction." << std::endl;
        return false;
    }

    for (auto &bubble: eband_) {
        if (!bubble.hasValidSize()) {
            std::cout << "bubble size not valid." << std::endl;
            return false;
        }
    }

    std::sort(eband_.begin(), eband_.end(), compareIndex);

    // after sort, index reallocation
    for (int i = 0; i < eband_.size(); ++i) {
        auto &bubble = eband_[i];
        bubble.setIndex(i);
    }

    return true;
}

void ElasticBands::recursiveFilter(const std::vector<BubbleXYIR> &ebandRaw, const BubbleXYIR &startBubble,
                                   const BubbleXYIR &endBubble) {
    int midIndex = (startBubble.getIndex() + endBubble.getIndex()) / 2;
    const auto midBubble = ebandRaw.at(midIndex);
    if (midIndex == startBubble.getIndex() || midIndex == endBubble.getIndex())
        return;

    if (bubbleOverlaps(startBubble, endBubble))
        return;

    eband_.push_back(midBubble);

    recursiveFilter(ebandRaw, startBubble, midBubble);
    recursiveFilter(ebandRaw, midBubble, endBubble);
}

// TODO: make function real boolean
bool ElasticBands::updateElasticBand() {
    viaPointsUpdate();

    deleteBubbleWhenDense();

    addBubbleWhenSparse();

    return true;
}

void ElasticBands::viaPointsUpdate() {
    // viaPoint update
    for (int bubbleIndex = 0; bubbleIndex < eband_.size(); ++bubbleIndex) {
        // pass first and last bubble for viapoint update
        if (bubbleIndex == eband_.front().getIndex() || bubbleIndex == eband_.back().getIndex())
            continue;

        Position viaPoint, totalForce;

        auto &bubbleCurr = eband_[bubbleIndex]; // non const
        const auto &bubblePrev = eband_[bubbleIndex - 1];
        const auto &bubbleNext = eband_[bubbleIndex + 1];

        getTotalForce(bubblePrev, bubbleCurr, bubbleNext, totalForce);
        bubbleCurr.setPosition(bubbleCurr.getPosition() + totalForce * FORCE_SCALING_FACTOR);
    }
}

void ElasticBands::getTotalForce(const BubbleXYIR &prev, const BubbleXYIR &curr, const BubbleXYIR &next,
                                 Position &totalForce) {
    totalForce = getRepulsiveForce(curr) + getContractionForce(prev, curr, next);
}

Position ElasticBands::getRepulsiveForce(const BubbleXYIR &currentBubble) {
    Position repulsiveForce;

    const auto &radius = currentBubble.getRadius();
    const auto &position = currentBubble.getPosition();

    if (radius > currentBubble.MAX_BUBBLE_RADIUS_M)
        repulsiveForce.setZero();

    else {
        Position dx(radius, 0);
        Position dy(0, radius);

        const auto constant = (currentBubble.MAX_BUBBLE_RADIUS_M - radius) / (2 * radius);
        repulsiveForce.x() = GLOBAL_REPULSION_GAIN *
                             (constant * minDistToObstacle(position - dx) - minDistToObstacle(position + dx));
        repulsiveForce.y() = GLOBAL_REPULSION_GAIN *
                             (constant * minDistToObstacle(position - dy) - minDistToObstacle(position + dy));
    }

    return repulsiveForce;
}

Position
ElasticBands::getContractionForce(const BubbleXYIR &prev, const BubbleXYIR &curr, const BubbleXYIR &next) const {
    Position contractionForce;

    const auto &previousVector = prev.getPosition();
    const auto &currentVector = curr.getPosition();
    const auto &nextVector = next.getPosition();

    const auto VectorLength1 = getDist(previousVector, currentVector);
    const auto VectorLength2 = getDist(currentVector, nextVector);

    contractionForce = GLOBAL_CONTRACTION_GAIN * ((previousVector - currentVector) / VectorLength1 +
                                                  (nextVector - currentVector) / VectorLength2);

    return contractionForce;
}

void ElasticBands::deleteBubbleWhenDense() {
    for (int bubbleIndex = 0; bubbleIndex < eband_.size();) {
        // pass first and last bubble
        if (bubbleIndex == eband_.front().getIndex() || bubbleIndex == eband_.back().getIndex()) {
            ++bubbleIndex;
            continue;
        }

        auto &bubblePrev = eband_[bubbleIndex - 1];
        auto &bubbleCurrent = eband_[bubbleIndex];
        auto &bubbleNext = eband_[bubbleIndex + 1];

        if (bubblePrev.isInside(bubbleCurrent) && bubbleCurrent.isInside(bubbleNext))
            eband_.erase(eband_.begin() + bubbleIndex);
        else
            ++bubbleIndex;
    }
}

void ElasticBands::addBubbleWhenSparse() {
    for (int bubbleIndex = 0; bubbleIndex < eband_.size();) {
        auto front = bubbleIndex;
        auto back = bubbleIndex + 1;
        auto &bubbleFront = eband_[front];
        auto &bubbleBack = eband_[back];
        if (bubbleOverlaps(bubbleFront, bubbleBack)) {
            ++bubbleIndex;
            continue;
        } else {
            Position mid = (bubbleFront.getPosition() + bubbleBack.getPosition()) / 2;
            BubbleXYIR bubbleNew(mid, minDistToObstacle(mid), -1);

            if (bubbleNew.hasValidSize())
                eband_.insert(eband_.begin() + back, bubbleNew);
        }
    }
}