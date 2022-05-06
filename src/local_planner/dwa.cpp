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
    pose_.setZero();
    velocity_.setZero();

    max_velocity_ = Velocity(0.7, 0.7854);
    max_wheel_acceleration_ = 0.5;
    tread_ = 0.58;

    forward_simtime_ = 1.0;
}

ROBOT::ROBOT(double tread, const Velocity &max_velocity, double max_acceleration)
    : tread_(tread),
      max_velocity_(max_velocity),
      max_wheel_acceleration_(max_acceleration)

{
    pose_.setZero();
    velocity_.setZero();

    forward_simtime_ = 1.0;
}

void ROBOT::setTread(double tread)
{
    tread_ = tread;
}
double ROBOT::getTread() const { return tread_; }

void ROBOT::setPose(double position_x, double position_y, double yaw)
{
    pose_(0) = position_x;
    pose_(1) = position_y;
    pose_(2) = yaw;
}
const Pose &ROBOT::getPose() const { return pose_; }

void ROBOT::setVel(double linear, double angular)
{
    velocity_(0) = linear;
    velocity_(1) = angular;
}
const Velocity &ROBOT::getVel() const { return velocity_; }

void ROBOT::setMaxCmdvel(double linear, double angular)
{
    max_velocity_(0) = linear;
    max_velocity_(1) = angular;
}
const Velocity &ROBOT::getMaxCmdvel() const { return max_velocity_; }

void ROBOT::setMaxAccel(double max_wheel_accel)
{
    max_wheel_acceleration_ = max_wheel_accel;
}
double ROBOT::getMaxAccel() const { return max_wheel_acceleration_; }

void ROBOT::setSimulationTime(double sim_time)
{
    forward_simtime_ = sim_time;
}

Pose ROBOT::motion(Pose init_pose, double dt)
{
    const auto &v = velocity_(0);
    const auto &w = velocity_(1);
    double dx = v * dt * std::cos(w * dt);
    double dy = v * dt * std::sin(w * dt);
    double dtheta = w * dt;

    Pose delta(dx, dy, dtheta);
    auto pose_after_motion = transformToMap(delta, init_pose);

    return pose_after_motion;
}

std::vector<Pose> ROBOT::forwardSimulation(double dt)
{
    const auto &n_iteration = forward_simtime_ / dt;
    std::vector<Pose> poselist;
    Pose moving = pose_;
    for (int i = 0; i < n_iteration; ++i)
    {
        moving = motion(moving, dt);
        poselist.push_back(moving);
    }

    return poselist;
}

Pose ROBOT::transformToMap(Pose &local, Pose &baseToMap)
{
    double position_x = baseToMap.x() + local.x() * std::cos(baseToMap(2)) - local.y() * std::sin(baseToMap(2));
    double position_y = baseToMap.y() + local.x() * std::sin(baseToMap(2)) + local.y() * std::cos(baseToMap(2));
    double theta = baseToMap(2) + local(2);
    theta = angles::normalize_angle(theta);

    return Pose(position_x, position_y, theta);
}

void ROBOT::cmdvelToWheelvel(const Velocity &cmdvel, double &vl, double &vr)
{
    const auto &v = cmdvel(0);
    const auto &w = cmdvel(1);
    vr = v + w * tread_ / 2.0;
    vl = v - w * tread_ / 2.0;
}

void ROBOT::wheelvelTocmdvel(const Velocity &wheelvel, double &v, double &w)
{
    const auto &vl = wheelvel(0);
    const auto &vr = wheelvel(1);
    v = (vl + vr) / 2.0;
    w = (vr - vl) / tread_;
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
    : max_acceleration_(0),
      vl_lower_bound_(0),
      vr_lower_bound_(0),
      vl_upper_bound_(0),
      vr_upper_bound_(0),
      vlvr_resolution_(0),
      control_time_(0),
      size_(0),
      initialized_(false)
{
}

DWA::DWA(ROBOT robot, double linvel_res, double angvel_res, double cycle_time)
{
    setRobot(robot);
    setWindowResolution(Velocity(linvel_res, angvel_res));
    setControlTimeInSec(cycle_time);

    initialize();
}

void DWA::setRobot(const ROBOT &robot)
{
    robot_ = robot;
    setMaxWheelAccel(robot_.getMaxAccel());
    setMaxVelocity(robot_.getMaxCmdvel());
}

void DWA::setMaxVelocity(Velocity max_cmdvel)
{
    auto max_linear = max_cmdvel(0);
    auto max_angular = max_cmdvel(1);
    vl_lower_bound_ = -robot_.getTread() * max_angular / 2;
    vr_lower_bound_ = -robot_.getTread() * max_angular / 2;
    vl_upper_bound_ = max_linear + robot_.getTread() + max_angular / 2;
    vr_upper_bound_ = max_linear + robot_.getTread() + max_angular / 2;
}

void DWA::setWindowResolution(Velocity resolution)
{
    const auto &linear_vel_resolution = resolution(0);
    const auto &angular_vel_resolution = resolution(1);
    double wheel_res1 = robot_.linResToWheelRes(linear_vel_resolution);
    double wheel_res2 = robot_.angResToWheelRes(angular_vel_resolution);
    vlvr_resolution_ = std::min(wheel_res1, wheel_res2);
}

void DWA::setControlTimeInSec(double control_time)
{
    control_time_ = control_time;
}

void DWA::initialize()
{
    double max_vel_deviation = max_acceleration_ * control_time_;
    size_ = 2 * (max_vel_deviation / vlvr_resolution_) + 1;

    VelCostPair zeroInit = {0,  // vl
                            0,  // vr
                            0,  // cost - target heading
                            0,  // cost - clearance
                            0,  // cost - velocity
                            0,  // cost - global_path
                            0}; // cost - total
    window_.resize(size_, std::vector<VelCostPair>(size_, zeroInit));

    initialized_ = true;
}

void DWA::setVelocity(Velocity cmd_vel)
{
    double vl_center, vr_center, vl_min, vr_min;
    robot_.cmdvelToWheelvel(cmd_vel, vl_center, vr_center);
    vl_min = vl_center - max_acceleration_ * control_time_;
    vr_min = vr_center - max_acceleration_ * control_time_;

    min_v_ = 1e3;
    max_v_ = -1e3;
    for (int i = 0; i < size_; ++i)
    {
        for (int j = 0; j < size_; ++j)
        {
            auto &left_velocity = window_[i][j].vl;
            auto &right_velocity = window_[i][j].vr;
            left_velocity = vl_min + i * vlvr_resolution_;
            right_velocity = vr_min + j * vlvr_resolution_;

            if (!isValidWheelVelocity(left_velocity, right_velocity))
            {
                left_velocity = 0;
                right_velocity = 0;
            }

            double v, w;
            robot_.wheelvelTocmdvel(Velocity(left_velocity, right_velocity), v, w);
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
    double v, w;
    robot_.wheelvelTocmdvel(Velocity(vl, vr), v, w);

    // 1. go backward
    if (v < 0)
        return false;

    // 2. over max linear velocity
    const auto &max_linear_vel = robot_.getMaxCmdvel().x();
    if (v > max_linear_vel)
        return false;

    // 3. over max angular velocity
    const auto &max_angular_vel = robot_.getMaxCmdvel().y();
    if (std::abs(w) > max_angular_vel)
        return false;

    return true;
}

void DWA::setGoal(Pose goal)
{
    goal_ = goal;
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