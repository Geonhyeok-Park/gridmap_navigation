//
// Created by isr on 22. 5. 5.
//

#include "dwa.h"

ROBOT::ROBOT()
{
    pose_.setZero();
    velocity_.setZero();
    max_velocity_.setConstant(1.0);
    max_acceleration_.setConstant(0.5);
    tread_ = 0.0;
    forward_simtime_ = 1.0;
}

ROBOT::ROBOT(double tread, Eigen::Vector2d &max_velocity, Eigen::Vector2d &max_acceleration)
    : tread_(tread),
      max_velocity_(max_velocity),
      max_acceleration_(max_acceleration)

{
    pose_.setZero();
    velocity_.setZero();
    forward_simtime_ = 1.0;
}

void ROBOT::setPose(double position_x, double position_y, double yaw)
{
    pose_(0) = position_x;
    pose_(1) = position_y;
    pose_(2) = yaw;
}

Eigen::Vector3d ROBOT::simulMotion(double dt)
{
    
}

void ROBOT::setMaxVel(double linear, double angular)
{
    max_velocity_(0) = linear;
    max_velocity_(1) = angular;
}

void ROBOT::setMaxAccel(double linear, double angular)
{
    max_acceleration_(0) = linear;
    max_acceleration_(1) = angular;
}

void ROBOT::setSimulationTime(double sim_time)
{
    forward_simtime_ = sim_time;
}

void ROBOT::cmdvelToWheelvel(const Velocity &cmdvel, double &vl, double &vr)
{
    vr = cmdvel(0) + cmdvel(1) * tread_ / 2.0;
    vl = cmdvel(0) - cmdvel(1) * tread_ / 2.0;
}

void ROBOT::wheelvelTocmdvel(const Velocity &wheelvel, double &v, double &w)
{
    const auto &vl = wheelvel(0);
    const auto &vr = wheelvel(1);
    v = (vl + vr) / 2.0;
    w = (vr - vl) / tread_;
}

DWA::DWA()
    : max_acceleration_(0),
      acceleration_res_(0),
      control_cycletime_(0.2),  // 5hz
      size_(0)
{
    resetWindow();
}

void DWA::resetCost()
{
    for (auto &row_vector : window_)
    {
        for (auto &vel_cost : row_vector)
        {
            auto &cost = vel_cost.cost;
            cost.global_path = 0;
            cost.target_heading = 0;
            cost.velocity = 0;
            cost.clearance = 0;
            cost.total = 0;
        }
    }
}

void DWA::resetWindow()
{
    for (auto &row_vector : window_)
    {
        for (auto &vel_cost : row_vector)
        {
            vel_cost.vl = 0;
            vel_cost.vr = 0;

            auto &cost = vel_cost.cost;
            cost.global_path = 0;
            cost.target_heading = 0;
            cost.velocity = 0;
            cost.clearance = 0;
            cost.total = 0;
        }
    }
}

void DWA::setMaxWheelAccel(double linear_max)
{
}

void DWA::setMaxVelocity(double linear_max, double angular_max)
{
}



void DWA::setControlCycleTime(double cycle_time)
{
    control_cycletime_ = cycle_time;
}

void DWA::setWindowResolution(double resolution)
{
    acceleration_res_ = resolution;
}