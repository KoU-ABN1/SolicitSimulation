#pragma once

#include <iostream>
#include <math.h>
#include <vector>
#include <Eigen/Dense>
#include <memory>
#include "simLib.h"

#define PI 3.14159265

struct SimHandles
{
    int left_wheel;
    int right_wheel;
    int waist_joint;
    int head_joint_1;
    int head_joint_2;
    int left_arm_joint_1;
    int left_arm_joint_2;
    int right_arm_joint_1;
    int right_arm_joint_2;

    int robot;
    int customer;
    int drawer;
};

struct SimInfo
{
    float robot_x; // robot coordinates
    float robot_y;
    float robot_yaw;

    float customer_x; // customer coordinates
    float customer_y;
    float customer_yaw;

    float head_x; // position of the customer's head in the camera
    float head_y;

    float waist_joint_position; // joint positions
    float head_joint_1_position;
    float head_joint_2_position;
    float left_arm_joint_1_position;
    float left_arm_joint_2_position;
    float right_arm_joint_1_position;
    float right_arm_joint_2_position;

    float time_cur; // current time
};

extern SimHandles handles;
extern SimInfo data;

void updateAllInfo();
void getObjectHandles(std::vector<int>);

inline float rectifyAngle(float angle)
{
    if (angle > PI)
        angle -= 2 * PI;
    else if (angle <= -PI)
        angle += 2 * PI;
    return angle;
}