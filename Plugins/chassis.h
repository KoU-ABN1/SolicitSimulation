#pragma once

#include "common.h"
#include "motor.h"

class Chassis
{
public:
    void moveToCustomer();
    void stop();

private:
    const float TRACK_WIDTH = 0.27;
    const float WHEEL_RADIUS = 0.15;

    float linear_vel_set_ = 0.8;
    float acc_set_ = 0.8 / WHEEL_RADIUS;

    std::unique_ptr<Motor> left_wheel = std::make_unique<Motor>(Motor(handles.left_wheel));
    std::unique_ptr<Motor> right_wheel = std::make_unique<Motor>(Motor(handles.right_wheel));
};
