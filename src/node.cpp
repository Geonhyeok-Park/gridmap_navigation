//
// Created by Ikhyeon Cho on 22. 3. 17..
//
#include "global_planner_ros.cpp"

int main(int argc, char **argv) {
    ros::init(argc, argv, "global_planner");

    GlobalNavPlannerRos node;

    ros::Duration(1.0).sleep(); // Need this to get the TF caches fill up.

    ros::spin();

    return 0;
}
