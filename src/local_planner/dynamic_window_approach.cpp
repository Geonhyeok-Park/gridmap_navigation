//
// Created by isr on 22. 4. 21.
//
#include "dynamic_window_approach.h"

using namespace grid_map;

DWA::Window::Window()
{
    min_.setZero();
    max_.setZero();
}

DWA::DWA(Robot robot, grid_map::Position goal, const grid_map::GridMap &obstacle_map, double dt)
{
    robot_ = robot;
    goal_ = goal;
    pmap_ = &obstacle_map;
    time_interval_ = dt;
    window_.setSize(robot_, time_interval_);
    window_.setResolution(0.05, 3.0 * (M_PI / 180.0));
}

std::vector<State> DWA::planning(double sim_time)
{
    float min_cost = 1e6;
    float min_goal_cost = min_cost;
    float min_velocity_cost = min_cost;
    float min_obstacle_cost = min_cost;

    std::vector<std::vector<State>> trajectory_candidates;
    std::vector<State> best_trajectory;

    double min_linear, max_linear, min_angular, max_angular;
    window_.getBoundaries(min_linear, max_linear, min_angular, max_angular);

    Pose current_pose = robot_.getPose();
    for (double vel = min_linear; vel <= max_linear; vel += window_.getLinearRes())
    {
        for (double yrate = min_angular; yrate <= max_angular; yrate += window_.getAngularRes())
        {
            Robot robot_sim = robot_;
            Velocity cmd_vel(vel, yrate);
            robot_sim.setVelocity(cmd_vel);

            State robot_sim_state(robot_sim.getPose(), cmd_vel);
            std::vector<State> trajectory;
            for (double t = 0; t <= sim_time; t += time_interval_)
            {
                motion(robot_sim_state, time_interval_);
                trajectory.push_back(robot_sim_state);
            }
            trajectory_candidates.push_back(trajectory);
            //
            // find min cost trajectory
            //
        }
    }

    return best_trajectory;
}

void DWA::motion(State &state, double dt)
{
    const auto &linear_vel = state.vel.x();
    const auto &angular_vel = state.vel.y();

    state.pose.yaw += angular_vel * dt;
    state.pose.position.x() += (linear_vel * dt) * std::cos(state.pose.yaw);
    state.pose.position.y() += (linear_vel * dt) * std::sin(state.pose.yaw);
}