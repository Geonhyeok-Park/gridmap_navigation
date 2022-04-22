//
// Created by isr on 22. 4. 21.
//

#ifndef GRIDMAP_NAVIGATION_DYNAMIC_WINDOW_APPROACH_H
#define GRIDMAP_NAVIGATION_DYNAMIC_WINDOW_APPROACH_H

// Grid map
#include <grid_map_core/grid_map_core.hpp>

using Velocity = grid_map::Position;
using Acceleration = grid_map::Position;

struct Pose
{
    grid_map::Position position;
    double yaw;
    void setPosition(const grid_map::Position &_position) { position = _position; }
    void setYaw(double _yaw) { yaw = _yaw; }
};

class State
{
public:
    Pose pose;
    Velocity vel;

public:
    State() = default;
    State(Pose _pose, Velocity _vel)
    {
        pose = _pose;
        vel = _vel;
    }
    ~State() = default;
};

////////////////////////////////
//////// Robot Class ///////////
////////////////////////////////

class Robot
{
public:
private:
    std::string framd_id_;
    double tread_length_;
    double robot_radius_;

    double max_acceleration_;
    double max_yaw_d_rate_;

    State state_;

public:
    Robot() = default;
    ~Robot() = default;

    const State getState() const { return state_; }
    const Pose getPose() const { return state_.pose; }
    const Velocity getVelocity() const { return state_.vel; }
    void setPose(Pose _pose) { state_.pose = _pose; }
    void setVelocity(Velocity _vel) { state_.vel = _vel; }
    void setMaxAcceleration(double max) { max_acceleration_ = max; }
    void setMaxYawRate(double max) { max_yaw_d_rate_ = max; }
    double getMaxAcceleration() const { return max_acceleration_; }
    double getMaxYawDRate() const { return max_yaw_d_rate_; }
    void setGeometry(std::string frame_id, double robot_radius, double tread_length)
    {
        framd_id_ = frame_id;
        robot_radius_ = robot_radius;
        tread_length_ = tread_length;
    }
    void setConstraints(double max_acceleration, double max_yawrate)
    {
        max_acceleration_ = max_acceleration;
        max_yaw_d_rate_ = max_yawrate;
    }
};

////////////////////////////////
///////// DWA Class ////////////
////////////////////////////////

class DWA
{
public:
    class Window
    {
    public:
        Velocity min_;
        Velocity max_;
        double linear_res_;
        double angular_res_;

    public:
        Window();
        ~Window() = default;
        void setSize(const Robot &robot, double dt)
        {
            min_.x() = robot.getVelocity().x() - robot.getMaxAcceleration() * dt;
            max_.x() = robot.getVelocity().x() + robot.getMaxAcceleration() * dt;
            min_.y() = robot.getVelocity().y() - robot.getMaxYawDRate() * dt;
            max_.y() = robot.getVelocity().y() + robot.getMaxYawDRate() * dt;
        }
        void getBoundaries(double &min_linear, double &max_linear, double &min_angular, double &max_angular)
        {
            min_linear = min_.x();
            min_angular = min_.y();
            max_linear = max_.x();
            max_angular = max_.y();
        }
        void setResolution(double linear, double angular)
        {
            linear_res_ = linear;
            angular_res_ = angular;
        }
        double getLinearRes() { return linear_res_; }
        double getAngularRes() { return angular_res_; }
    };

public:
    DWA(Robot robot, grid_map::Position goal, const grid_map::GridMap &obstacle_map, double dt);
    ~DWA() = default;
    std::vector<State> planning(double sim_time);
    void motion(State &state, double dt);

private:
    Robot robot_;
    grid_map::Position goal_;
    const grid_map::GridMap *pmap_;
    double time_interval_;

    Velocity target_vel_;

    Window window_;
};

#endif // GRIDMAP_NAVIGATION_DYNAMIC_WINDOW_APPROACH_H
