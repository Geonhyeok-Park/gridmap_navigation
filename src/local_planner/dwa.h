//
// Created by Ikhyeon on 22. 5. 5.
//

#ifndef GRIDMAP_NAVIGATION_DWA_H
#define GRIDMAP_NAVIGATION_DWA_H

#include <vector>
#include <grid_map_core/grid_map_core.hpp>

class ROBOT
{
    using Pose = Eigen::Vector3d;
    using Velocity = Eigen::Vector2d;
    using Acceleration = Eigen::Vector2d;

private:
    Pose pose_;
    Velocity velocity_;
    Velocity max_velocity_;
    Acceleration max_acceleration_;
    double tread_;
    double forward_simtime_;


public:
    ROBOT();
    ROBOT(double tread, Eigen::Vector2d &max_vel, Eigen::Vector2d &max_accel);
    ~ROBOT() = default;

    void setMaxVel(double linear, double angular);
    void setMaxAccel(double linear, double angular);
    const Velocity &getMaxVel() const;
    const Acceleration &getMaxAccel() const;

    void setPose(double position_x, double position_y, double yaw);
    void setSimulationTime(double);
    Pose simulMotion(double);

    void cmdvelToWheelvel(const Velocity &cmdvel, double &vl, double &vr);
    void wheelvelTocmdvel(const Velocity &wheelvel, double &v, double &w);
};

class DWA
{
    using Pose = Eigen::Vector3d;
    using Velocity = Eigen::Vector2d;
    using Acceleration = Eigen::Vector2d;

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
    double max_acceleration_;
    double acceleration_res_;
    double control_cycletime_;

    int size_;

    std::vector<std::vector<VelCostPair>> window_;

public:
    DWA();
    ~DWA() = default;
    void resetCost();
    void resetWindow();
    void resetWindow(Velocity cmdvel);
    
    void setMaxWheelAccel(double);
    void setWindowResolution(double);
    
    // temp
    void setMaxVelocity(double, double);

    void setSimulationTime(double);
    void setControlCycleTime(double);
};



#endif // GRIDMAP_NAVIGATION_DWA_H
