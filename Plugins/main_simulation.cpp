#include "main_simulation.h"
#include "simLib.h"
#include "oat_planner.h"
#include "math.h"
#include <iostream>

// object handles
static int ibot;
static int bill;
static int left_wheel;
static int right_wheel;
static int red_drawer;
static int green_drawer;

void doSimulation()
{
    // prepare data
    float ibot_pos[3];
    float ibot_ornt[3];
    float ibot_linear_vel[3];
    float ibot_angular_vel[3];
    simGetObjectPosition(ibot, -1, ibot_pos);
    simGetObjectOrientation(ibot, -1, ibot_ornt);
    simGetObjectVelocity(ibot, ibot_linear_vel, ibot_angular_vel);

    float bill_pos[3];
    float bill_ornt[3];
    float bill_linear_vel[3];
    float bill_angular_vel[3];
    simGetObjectPosition(bill, -1, bill_pos);
    simGetObjectOrientation(bill, -1, bill_ornt);
    simGetObjectVelocity(bill, bill_linear_vel, bill_angular_vel);

    Eigen::Vector3f robot_pose(ibot_pos[0], ibot_pos[1], ibot_ornt[2]);
    Eigen::Vector3f human_pose(bill_pos[0], bill_pos[1], bill_ornt[2]);
    float robot_vel = sqrt(ibot_linear_vel[0] * ibot_linear_vel[0] + ibot_linear_vel[1] * ibot_linear_vel[1]);
    float human_vel = sqrt(bill_linear_vel[0] * bill_linear_vel[0] + bill_linear_vel[1] * bill_linear_vel[1]);

    // do plan
    float vel_set = 0.7;
    std::pair<float, float> vel = std::make_pair(0, 0);
    std::vector<Eigen::Vector2f> path;

    OatPlanner oat_planner(robot_pose, human_pose, robot_vel, human_vel, vel_set);

    if (oat_planner.isApproachable())
    {
        vel = oat_planner.doPlan();
        path = oat_planner.outputPath();
    }

    std::cout << "vel: " << vel.first << " " << vel.second << std::endl;

    // drive motor
    float left_vel = (vel.first + vel.second * 0.54 / 2) / 0.15;
    float right_vel = (vel.first - vel.second * 0.54 / 2) / 0.15;
    simSetJointTargetVelocity(left_wheel, left_vel);
    simSetJointTargetVelocity(right_wheel, right_vel);

    // draw planned path
    simAddDrawingObjectItem(green_drawer, nullptr);
    for (int i = 0; i < path.size(); i++)
    {
        float point[3] = {path[i][0], path[i][1], 0.1};
        simAddDrawingObjectItem(green_drawer, point);
    }

    // draw robot path
    float point[3] = {ibot_pos[0], ibot_pos[1], 0.1};
    simAddDrawingObjectItem(red_drawer, point);
}

void initSimulation()
{
    // get object handle
    ibot = simGetObjectHandle("iBot");
    bill = simGetObjectHandle("Bill");

    left_wheel = simGetObjectHandle("Right_Wheel_Joint");
    right_wheel = simGetObjectHandle("Left_Wheel_Joint");

    // add drawer
    float red[3] = {1, 0, 0};
    float green[3] = {0, 1, 0};
    red_drawer = simAddDrawingObject(sim_drawing_points, 2, 0, -1, 999999, red, nullptr, nullptr, nullptr);
    green_drawer = simAddDrawingObject(sim_drawing_points, 2, 0, -1, 999999, green, nullptr, nullptr, nullptr);
}

void cleanSimulation()
{
    // remove drawer
    simRemoveDrawingObject(red_drawer);
    simRemoveDrawingObject(green_drawer);
}