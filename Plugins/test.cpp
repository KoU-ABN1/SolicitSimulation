#include "./oat_planner.h"
#include "math.h"
#include <iostream>
#include <chrono>

int main()
{
    std::chrono::time_point<std::chrono::system_clock> ts = std::chrono::system_clock::now();

    Eigen::Vector3f robot_pose(0, 0, 180 / 180.0 * M_PI);
    Eigen::Vector3f human_pose(-6, 2, 0 / 180.0 * M_PI);
    float robot_vel = 0.7;
    float human_vel = 0.8;

    float vel_set = 0.7;

    // do plan
    std::pair<float, float> vel = std::make_pair(0, 0);
    OatPlanner oat_planner = OatPlanner(robot_pose, human_pose, robot_vel, human_vel, vel_set);

    if (oat_planner.isApproachable())
    {
        //std::cout << "target is appraochable" << std::endl;
        vel = oat_planner.doPlan();
    }

    std::chrono::time_point<std::chrono::system_clock> te = std::chrono::system_clock::now();
    float dt = std::chrono::duration<float>(te - ts).count();
    std::cout << "dt " << dt << std::endl;

    std::cout << vel.first << " " << vel.second << std::endl;
}