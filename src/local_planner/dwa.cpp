//
// Created by Ikhyeon on 22. 5. 5.
//

#include "dwa.h"
#include <iostream>

using Pose = Eigen::Vector3d;
using Velocity = Eigen::Vector2d;
using Acceleration = Eigen::Vector2d;

//////////////////////////////////////
//////////////////////////////////////
///////////  ROBOT class  ////////////
//////////////////////////////////////
//////////////////////////////////////

ROBOT::ROBOT()
{
    // geometry
    tread_ = 0.58;
    radius_ = 0.4;

    // state for motion model
    pose_.setZero();
    forward_simtime_ = 1.0;
    control_time_ = 0.1;

    max_velocity_ = Velocity(0.7, 0.7854);
    max_wheel_acceleration_ = 0.5;
}

ROBOT::ROBOT(double tread, const Velocity &max_velocity, double max_acceleration)
    : tread_(tread),
      radius_(0.4),
      max_velocity_(max_velocity),
      max_wheel_acceleration_(max_acceleration)

{
    pose_.setZero();

    forward_simtime_ = 1.0;
    control_time_ = 0.1;
}

void ROBOT::setPose(double position_x, double position_y, double yaw)
{
    pose_(0) = position_x;
    pose_(1) = position_y;
    pose_(2) = yaw;
}
const Pose &ROBOT::getPose() const { return pose_; }

void ROBOT::setMaxVel(Velocity max_velocity)
{
    max_velocity_ = max_velocity;
}
const Velocity &ROBOT::getMaxVel() const { return max_velocity_; }

void ROBOT::setMaxAccel(double max_wheel_accel)
{
    max_wheel_acceleration_ = max_wheel_accel;
}
double ROBOT::getMaxAccel() const { return max_wheel_acceleration_; }

void ROBOT::setSimulationTime(double sim_time, double delta)
{
    forward_simtime_ = sim_time;
    control_time_ = delta;
}

Pose ROBOT::motion(const Pose &init_pose, const Velocity &cmd_vel)
{
    const auto &v = cmd_vel(0);
    const auto &w = cmd_vel(1);
    const auto &dt = control_time_;

    double dx = v * dt * std::cos(w * dt);
    double dy = v * dt * std::sin(w * dt);
    double dtheta = w * dt;

    Pose delta(dx, dy, dtheta);
    auto pose_after_motion = transformToMap(delta, init_pose);

    return pose_after_motion;
}

std::vector<Pose> ROBOT::forwardSimulation(const Velocity &cmd_vel)
{
    const auto &n_iteration = forward_simtime_ / control_time_;
    std::vector<Pose> poselist;
    Pose moving = pose_;
    for (int i = 0; i < n_iteration; ++i)
    {
        moving = motion(moving, cmd_vel);
        poselist.push_back(moving);
    }

    return poselist;
}

Pose ROBOT::transformToMap(Pose &local, const Pose &baseToMap)
{
    double position_x = baseToMap.x() + local.x() * std::cos(baseToMap(2)) - local.y() * std::sin(baseToMap(2));
    double position_y = baseToMap.y() + local.x() * std::sin(baseToMap(2)) + local.y() * std::cos(baseToMap(2));
    double theta = baseToMap(2) + local(2);
    theta = angles::normalize_angle(theta);

    return Pose(position_x, position_y, theta);
}

void ROBOT::cmdvelToWheelvel(const Velocity &cmdvel, VlVrVelocity &wheelvel)
{
    const auto &v = cmdvel(0);
    const auto &w = cmdvel(1);
    wheelvel(0) = v - w * tread_ / 2.0;
    wheelvel(1) = v + w * tread_ / 2.0;
}

void ROBOT::wheelvelTocmdvel(const VlVrVelocity &wheelvel, Velocity &cmdvel)
{
    const auto &vl = wheelvel(0);
    const auto &vr = wheelvel(1);
    cmdvel(0) = (vl + vr) / 2.0;
    cmdvel(1) = (vr - vl) / tread_;
}

double ROBOT::angResToWheelRes(double angvel_res)
{
    return tread_ * angvel_res;
}

double ROBOT::linResToWheelRes(double linvel_res)
{
    return linvel_res / 2;
}

//////////////////////////////////////
//////////////////////////////////////
////////////  DWA class  /////////////
//////////////////////////////////////
//////////////////////////////////////

DWA::DWA()
    : vlvr_resolution_(0),
      size_(0),
      initialized_(false)
{
}

DWA::DWA(ROBOT robot, double linvel_res, double angvel_res)
    : vlvr_resolution_(0),
      size_(0),
      initialized_(false)
{
    setRobot(robot);
    setWindowResolution(Velocity(linvel_res, angvel_res));

    initialize();
}

void DWA::setWindowResolution(Velocity resolution)
{
    const auto &linear_resolution = resolution(0);
    const auto &angular_resolution = resolution(1);
    double wheel_res1 = robot_.linResToWheelRes(linear_resolution);
    double wheel_res2 = robot_.angResToWheelRes(angular_resolution);
    vlvr_resolution_ = std::min(wheel_res1, wheel_res2);
}

void DWA::initialize()
{
    double maxdelta_vlvr = robot_.getMaxAccel() * robot_.getControlTime();
    size_ = 2 * (maxdelta_vlvr / vlvr_resolution_) + 1;

    VlVrCostPair zero_init = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    window_.resize(size_, std::vector<VlVrCostPair>(size_, zero_init));

    initialized_ = true;
}

void DWA::updateVelocity(Velocity cmd_vel)
{
    VlVrVelocity window_center, half_width, window_min;
    robot_.cmdvelToWheelvel(cmd_vel, window_center);
    half_width.setConstant(robot_.getMaxAccel() * robot_.getControlTime());
    window_min = window_center - half_width;
    const auto &min_vl = window_min(0);
    const auto &min_vr = window_min(1);

    min_v_ = 1e3;
    max_v_ = -1e3;
    for (int i = 0; i < size_; ++i)
    {
        for (int j = 0; j < size_; ++j)
        {
            auto &vl = window_[i][j].vl;
            auto &vr = window_[i][j].vr;
            vl = min_vl + i * vlvr_resolution_;
            vr = min_vr + j * vlvr_resolution_;
            if (!isValidWheelVelocity(vl, vr))
            {
                vl = 0;
                vr = 0;
            }
            // save min max linear vel
            Velocity cmd_vel;
            robot_.wheelvelTocmdvel(VlVrVelocity(vl, vr), cmd_vel);
            window_[i][j].cost.velocity = cmd_vel(0);
            const auto &v = cmd_vel(0);
            if (v > max_v_)
                max_v_ = v;
            if (v < min_v_)
                min_v_ = v;
        }
    }
}

// pass all condition, then return true;
bool DWA::isValidWheelVelocity(double vl, double vr)
{
    Velocity cmd_vel;
    robot_.wheelvelTocmdvel(VlVrVelocity(vl, vr), cmd_vel);
    const auto &v = cmd_vel(0);
    const auto &w = cmd_vel(1);
    // 1. go backward
    if (v < 0)
        return false;

    // 2. over max linear velocity
    const auto &max_linear_vel = robot_.getMaxVel()(0);
    if (v > max_linear_vel)
        return false;

    // 3. over max angular velocity
    const auto &max_angular_vel = robot_.getMaxVel()(1);
    if (std::abs(w) > max_angular_vel)
        return false;

    return true;
}

void DWA::updateCost()
{
}

bool DWA::resetCost()
{
    if (!initialized_)
    {
        std::cout << "Dynamic Window not initialized." << std::endl;
        return false;
    }

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

    return true;
}

bool DWA::resetAll()
{
    if (!initialized_)
    {
        std::cout << "Dynamic Window not initialized." << std::endl;
        return false;
    }

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

    return true;
}