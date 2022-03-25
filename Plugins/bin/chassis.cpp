#include "chassis.h"

void Chassis::moveToCustomer()
{
    float a1 = tan(data.robot_yaw);
    float b1 = -1;
    float c1 = data.robot_y - data.robot_x * tan(data.robot_yaw);
    float a2 = tan(data.customer_yaw);
    float b2 = -1;
    float c2 = data.customer_y - data.customer_x * tan(data.customer_yaw);

    float cx = (c2 * b1 - c1 * b2) / (a1 * b2 - a2 * b1);
    float cy = (c1 * a2 - c2 * a1) / (a1 * b2 - a2 * b1);

    std::cout << "(s, t) = " << cx << " " << cy << std::endl;

    float k = 1;
    Eigen::Vector2d p1(data.robot_x, data.robot_y);
    Eigen::Vector2d p2(data.robot_x + k * cos(data.robot_yaw), data.robot_y + k * sin(data.robot_yaw));
    Eigen::Vector2d p3(data.customer_x + k * cos(data.customer_yaw), data.customer_y + k * sin(data.customer_yaw));
    Eigen::Vector2d p4(data.customer_x, data.customer_y);

    float t = 0.1;
    Eigen::Vector2d target = p1 * (1 - t) * (1 - t) * (1 - t) + p2 * 3 * (1 - t) * (1 - t) * t + p3 * 3 * (1 - t) * t * t + p4 * t * t *t;

    simAddDrawingObjectItem(handles.drawer, nullptr);
    for (int i = 0; i < 200; i++)
    {
        float t = (float)i / 200;
        Eigen::Vector2d p = p1 * (1 - t) * (1 - t) * (1 - t) + p2 * 3 * (1 - t) * (1 - t) * t + p3 * 3 * (1 - t) * t * t + p4 * t * t * t;
        float point[3] = {p[0], p[1], 0};
        simAddDrawingObjectItem(handles.drawer, point);
    }

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

// void Chassis::moveToCustomer()
// {
//     // float theta = 3.141549 / 2 + data.robot_yaw;
//     // float a = (data.customer_x - data.robot_x) * cos(theta) + (data.customer_y - data.robot_y) * sin(theta);
//     // float b = -(data.customer_x - data.robot_x) * sin(theta) + (data.customer_y - data.robot_y) * cos(theta);
//     // float cx = -b * sin(theta) + data.robot_x;
//     // float cy = b * cos(theta) + data.robot_y;

//     float a1 = tan(data.robot_yaw);
//     float b1 = -1;
//     float c1 = data.robot_y - data.robot_x * tan(data.robot_yaw);
//     float a2 = tan(data.customer_yaw);
//     float b2 = -1;
//     float c2 = data.customer_y - data.customer_x * tan(data.customer_yaw);

//     float cx = (c2 * b1 - c1 * b2) / (a1 * b2 - a2 * b1);
//     float cy = (c1 * a2 - c2 * a1) / (a1 * b2 - a2 * b1);

//     std::cout << "(s, t) = " << cx << " " << cy << std::endl;

//     Eigen::Vector2d p1(data.robot_x, data.robot_y);
//     Eigen::Vector2d p2(cx, cy);
//     Eigen::Vector2d p3(data.customer_x, data.customer_y);

//     float t = 0.1;
//     Eigen::Vector2d target = p1 * (1 - t) * (1 - t) + p2 * 2 * (1 - t) * t + p3 * t * t;

//     simAddDrawingObjectItem(handles.drawer, nullptr);
//     for (int i = 0; i < 200; i++)
//     {
//         float t = (float)i / 200;
//         Eigen::Vector2d p = p1 * (1 - t) * (1 - t) + p2 * 2 * (1 - t) * t + p3 * t * t;
//         float point[3] = {p[0], p[1], 0};
//         simAddDrawingObjectItem(handles.drawer, point);
//     }

//     float m = target[0] - data.robot_x;
//     float n = target[1] - data.robot_y;
//     float r = (m * m + n * n) / (2 * m * sin(data.robot_yaw) - 2 * n * cos(data.robot_yaw));
//     float v1 = (r + TRACK_WIDTH) / r * linear_vel_set_;
//     float v2 = (r - TRACK_WIDTH) / r * linear_vel_set_;

//     float w1 = v1 / WHEEL_RADIUS;
//     float w2 = v2 / WHEEL_RADIUS;
//     left_wheel->setTargetVelocity(w1, acc_set_);
//     right_wheel->setTargetVelocity(w2, acc_set_);
// }


void Chassis::stop()
{
    left_wheel->setTargetVelocity(0, 10);
    right_wheel->setTargetVelocity(0, 10);
}