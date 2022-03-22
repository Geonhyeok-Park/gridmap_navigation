//
// Created by isr on 22. 3. 17..
//

#include "globalNavPlannerRos.cpp"

int main(int argc, char **argv) {
    ros::init(argc, argv, "global_nav_planner");

    globalNavPlannerRos node;

    ros::Duration(1.0).sleep(); // Need this to get the TF caches fill up.

    ros::spin();

    return 0;
}
