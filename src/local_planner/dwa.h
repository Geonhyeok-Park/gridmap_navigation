//
// Created by Ikhyeon on 22. 5. 5.
//

#ifndef GRIDMAP_NAVIGATION_DWA_H
#define GRIDMAP_NAVIGATION_DWA_H

#include <vector>
#include <eigen3/Eigen/Core>
#include <angles/angles.h>

class ROBOT
{
    using Pose = Eigen::Vector3d;
    using Velocity = Eigen::Vector2d;
    using VlVrVelocity = Eigen::Vector2d;

private:
    // geometry
    double radius_; // 40cm default -ISR M2
    double tread_;        // 58cm default -ISR M2

    // state for motion model
    Pose pose_;
    double forward_simtime_; // 1sec default
    double control_time_;    // 10hz default

    Pose goal_;

    // constraints
    Velocity max_velocity_;         // linear: 0.7m/s angular: 45deg/s default
    double max_wheel_acceleration_; // 0.5m/s2 default

private:
    Pose transformToMap(Pose &local, const Pose &baseToMap);
    Pose motion(const Pose &, const Velocity &);

public:
    ROBOT();
    ROBOT(double tread, const Velocity &max_vel, double max_accel);
    ~ROBOT() = default;

    // robot geometry
    void setTread(double tread) { tread_ = tread; }
    double getTread() const { return tread_; }

    void setRadius(double radius) { radius_ = radius; }
    double getRadius() const { return radius_; }

    // 2D Pose
    void setPose(double position_x, double position_y, double yaw);
    const Pose &getPose() const;
    void setGoal(Pose goal) { goal_ = goal; }

    // max cmd velocity
    void setMaxVel(Velocity);
    const Velocity &getMaxVel() const;

    // max wheel acceleration
    void setMaxAccel(double);
    double getMaxAccel() const;

    // forward simulation
    void setSimulationTime(double, double);
    std::vector<Pose> forwardSimulation(const Velocity &);
    double getControlTime() const { return control_time_; }

    // conversion between cmdvel and wheelvel
    void cmdvelToWheelvel(const Velocity &, VlVrVelocity &);
    void wheelvelTocmdvel(const VlVrVelocity &, Velocity &);
    double angResToWheelRes(double);
    double linResToWheelRes(double);
};

class DWA
{
    using Pose = Eigen::Vector3d;
    using Velocity = Eigen::Vector2d;
    using VlVrVelocity = Eigen::Vector2d;
    struct Cost
    {
        double target_heading;
        double clearance;
        double velocity;
        double global_path;
        double total;
    };
    struct VlVrCostPair
    {
        double vl;
        double vr;
        Cost cost;
    };

private:
    ROBOT robot_;
    std::vector<std::vector<VlVrCostPair>> window_;

    // window param
    double vlvr_resolution_;
    int size_;
    bool initialized_;

    // TODO: need other info to debug?
    double min_v_;
    double max_v_;

private:
    bool isValidWheelVelocity(double, double);

public:
    DWA();
    DWA(ROBOT, double, double);
    ~DWA() = default;

    // set window geometry
    void setRobot(const ROBOT &robot) { robot_ = robot; }
    void setWindowResolution(Velocity);
    void initialize();

    ROBOT &getRobot() { return robot_; }
    double getSize() { return size_; }
    const VlVrCostPair &access(double i, double j) const { return window_[i][j]; }
    void updateVelocity(Velocity);
    void updateCost();

    bool resetCost();
    bool resetAll();
    void resetWindow(Velocity cmdvel);
};

#endif // GRIDMAP_NAVIGATION_DWA_H
