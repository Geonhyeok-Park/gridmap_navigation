//
// Created by Ikhyeon on 22. 5. 5.
//

#include "dwa.h"

using Pose = Eigen::Vector3d;
using Velocity = Eigen::Vector2d;
using Acceleration = Eigen::Vector2d;

ROBOT::ROBOT()
{
    pose_.setZero();
    velocity_.setZero();

    max_velocity_.setConstant(1.0);
    max_acceleration_.setConstant(0.3);
    tread_ = 0.0;

    forward_simtime_ = 1.0;
}

ROBOT::ROBOT(double tread, Velocity &max_velocity, Acceleration &max_acceleration)
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

Pose ROBOT::motion(double dt)
{
    const auto &v = velocity_(0);
    const auto &w = velocity_(1);
    double dx = v * dt * std::cos(w * dt);
    double dy = v * dt * std::sin(w * dt);
    double dtheta = w * dt;

    return Pose local(dx, dy, dtheta);
}

Pose ROBOT::forwardSimulation(double dt)
{
    const auto &n_iteration = forward_simtime_ / dt;
    Pose delta(0,0,0);
    for (int i = 0; i < n_iteration; ++i)
    {
        delta += motion(dt);
    }
    
}

Pose ROBOT::transformToMap(Pose &local, const Pose &local_origin)
{
    const auto &local_origin_theta = local_origin(2);
    double position_x = local_origin.x() + local.x() * std::cos(local_origin_theta) - local.y() * std::sin(local_origin_theta);
    double position_y = local_origin.y() + local.x() * std::sin(local_origin_theta) + local.y() * std::cos(local_origin_theta);
    double theta = local_origin_theta + local(2);
    
    return Pose(position_x, position_y, theta);
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
      control_cycletime_(0.2), // 5hz
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