//
// Created by Ikhyeon on 22. 5. 5.
//

#ifndef GRIDMAP_NAVIGATION_DWA_H
#define GRIDMAP_NAVIGATION_DWA_H

#include <vector>
#include <eigen/eigen.hpp>
#include <angles.h>

class ROBOT
{
    using Pose = Eigen::Vector3d;
    using Velocity = Eigen::Vector2d;

private:
    double robot_radius_;
    double tread_;

    // for motion model
    Pose pose_;
    Velocity velocity_;
    double forward_simtime_;

    // user defined
    Velocity max_velocity_;

    // platform defined
    double max_wheel_acceleration_;

    Pose transformToMap(Pose &local, Pose &baseToMap);
    Pose motion(Pose, double);

public:
    ROBOT();
    ROBOT(double tread, const Velocity &max_vel, double max_accel);
    ~ROBOT() = default;

    // robot geometry
    void setTread(double);
    double getTread() const;

    // 2D Pose
    void setPose(double position_x, double position_y, double yaw);
    const Pose &getPose() const;

    void setVel(double, double);
    const Velocity &getVel() const;

    // max cmd velocity
    void setMaxCmdvel(double linear, double angular);
    const Velocity &getMaxCmdvel() const;

    // max wheel acceleration
    void setMaxAccel(double);
    double getMaxAccel() const;

    // forward simulation
    void setSimulationTime(double);
    std::vector<Pose> forwardSimulation(double);

    void cmdvelToWheelvel(const Velocity &cmdvel, double &vl, double &vr);
    void wheelvelTocmdvel(const Velocity &wheelvel, double &v, double &w);
    double angResToWheelRes(double);
    double linResToWheelRes(double);
};

class DWA
{
    using Pose = Eigen::Vector3d;
    using Velocity = Eigen::Vector2d;
    struct Cost
    {
        double target_heading;
        double clearance;
        double velocity;
        double global_path;
        double total;
    };
    struct VelCostPair
    {
        double vl;
        double vr;
        Cost cost;
    };

private:
    // robot param
    ROBOT robot_;
    double max_acceleration_;

    Pose goal_;

    // TODO: need this?
    double vl_lower_bound_;
    double vl_upper_bound_;
    double vr_lower_bound_;
    double vr_upper_bound_;

    // TODO: need other info to debug?
    double min_v_;
    double max_v_;
    // window size
    double vlvr_resolution_;
    int size_;

    // dt
    double control_time_;

    // error debug
    bool initialized_;

    std::vector<std::vector<VelCostPair>> window_;

private:
    void setMaxWheelAccel(double max_wheel_accel) { max_acceleration_ = max_wheel_accel; }
    void setMaxVelocity(Velocity);
    Velocity getMinMaxWheelVelocity() const { return Velocity(vl_lower_bound_, vl_upper_bound_); }
    bool isValidWheelVelocity(double, double);

public:
    DWA();
    DWA(ROBOT, double, double, double);
    ~DWA() = default;

    void setRobot(const ROBOT &);
    void setWindowResolution(Velocity);
    void setControlTimeInSec(double);
    void initialize();

    void setVelocity(Velocity);
    void setGoal(Pose);

    void updateCost();

    bool resetCost();
    bool resetAll();
    void resetWindow(Velocity cmdvel);

};

#endif // GRIDMAP_NAVIGATION_DWA_H
