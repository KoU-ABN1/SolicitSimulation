#include "chassis.h"

void Chassis::moveToCustomer()
{
    float bias = 1;
    Eigen::Vector2f p1(data.robot_x, data.robot_y);
    // Eigen::Vector2f p2(data.robot_x + bias * cos(data.robot_yaw), data.robot_y + bias * sin(data.robot_yaw));
    // Eigen::Vector2f p3(data.customer_x + bias * cos(data.customer_yaw), data.customer_y + bias * sin(data.customer_yaw));
    Eigen::Vector2f p4(data.customer_x, data.customer_y);
    

    float t = 0.1;
    Eigen::Vector2f target = p1 * (1 - t) * (1 - t) * (1 - t) + p2 * 3 * (1 - t) * (1 - t) * t + p3 * 3 * (1 - t) * t * t + p4 * t * t * t;

    static int k = 0;

    if (k % 30 == 0)
    {
        float t = 0;
        for (int i = 0; i < 200; i++)
        {
            Eigen::Vector2f target = p1 * (1 - t) * (1 - t) * (1 - t) + p2 * 3 * (1 - t) * (1 - t) * t + p3 * 3 * (1 - t) * t * t + p4 * t * t * t;
            float point[3] = {target[0], target[1], 0};
            simAddDrawingObjectItem(handles.drawer, point);
            t += 1.0 / 200;
        }
    }
    k++;

    float m = target[0] - data.robot_x;
    float n = target[1] - data.robot_y;
    float r = (m * m + n * n) / (2 * m * sin(data.robot_yaw) - 2 * n * cos(data.robot_yaw));
    float v1 = (r + TRACK_WIDTH) / r * linear_vel_set_;
    float v2 = (r - TRACK_WIDTH) / r * linear_vel_set_;

    float w1 = v1 / WHEEL_RADIUS;
    float w2 = v2 / WHEEL_RADIUS;
    left_wheel->setTargetVelocity(w1, acc_set_);
    right_wheel->setTargetVelocity(w2, acc_set_);
}

void Chassis::stop()
{
    left_wheel->setTargetVelocity(0, acc_set_);
    right_wheel->setTargetVelocity(0, acc_set_);
}