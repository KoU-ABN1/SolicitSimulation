#include "chassis.h"

void Chassis::moveToCustomer()
{
    float v1 = linear_vel_set_;
    float v2 = 1.0;
    float k = 1.2;
    float r = 2;
    float theta = data.customer_yaw;
    float s = (data.customer_x - data.robot_x) * cos(theta) + (data.customer_y - data.robot_y) * sin(theta);
    float t = -(data.customer_x - data.robot_x) * sin(theta) + (data.customer_y - data.robot_y) * cos(theta);

    std::cout << "(s, t) = " << s << " " << t << std::endl;

    float u = v1 * v1 / (k * k * v2 * v2);

    float alpha = -0 / 180.0 * 3.14159;
    bool target_reachable = false;
    float m;

    for (; alpha >= -60 / 180.0 * 3.14159; alpha -= 1 / 180.0 * 3.14159)
    {
        float a = 1 - u * u;
        float b = 2 * r * cos(alpha) + 2 * s * u * u;
        float c = r * r + t * t - u * u * s * s + 2 * r * t * sin(alpha);
        float delta = b * b - 4 * a * c;

        if (delta >= 0)
        {
            float m1 = (-b - sqrt(delta)) / (2 * a);
            float m2 = (-b + sqrt(delta)) / (2 * a);

            bool m1_valid = m1 > s && m1 + r * cos(alpha) < -0.5;
            bool m2_valid = m2 > s && m2 + r * cos(alpha) < -0.5;

            if (m1_valid && m2_valid)
            {
                m = m1 > m2 ? m1 : m2;
                target_reachable = true;
                break;
            }
            else if (m1_valid || m2_valid)
            {
                m = m1_valid ? m1 : m2;
                target_reachable = true;
                break;
            }
        }
    }

    if (target_reachable)
    {
        Eigen::Vector2d start_pos(data.robot_x, data.robot_y);
        float start_yaw = data.robot_yaw;

        float qx = m + r * cos(alpha);
        float qy = t + r * sin(alpha);
        Eigen::Vector2d end_pos(qx * cos(theta) - qy * sin(theta) + data.robot_x,
                                qx * sin(theta) + qy * cos(theta) + data.robot_y);
        float end_yaw = theta + alpha + 3.14159;

        float k1 = 0.5, k2 = 0.3;
        float dist = (start_pos - end_pos).norm();
        Eigen::Vector2d p1 = start_pos;
        Eigen::Vector2d p2 = start_pos + k1 * dist * Eigen::Vector2d(cos(start_yaw), sin(start_yaw));
        Eigen::Vector2d p3 = end_pos - k2 * dist * Eigen::Vector2d(cos(end_yaw), sin(end_yaw));
        Eigen::Vector2d p4 = end_pos;

        float t = 0.1;
        Eigen::Vector2d target = p1 * (1 - t) * (1 - t) * (1 - t) + p2 * 3 * (1 - t) * (1 - t) * t + p3 * 3 * (1 - t) * t * t + p4 * t * t * t;

        simAddDrawingObjectItem(handles.drawer, nullptr);
        for (int i = 0; i < 200; i++)
        {
            float t = (float)i / 200;
            Eigen::Vector2d target = p1 * (1 - t) * (1 - t) * (1 - t) + p2 * 3 * (1 - t) * (1 - t) * t + p3 * 3 * (1 - t) * t * t + p4 * t * t * t;
            float point[3] = {target[0], target[1], 0};
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

        std::cout << alpha / 3.14159 * 180 << " " << end_pos.x() << " " << end_pos.y() << std::endl;
        // float p1[3] = {end_pos.x(), end_pos.y(), 0};
        // float p2[3] = {m, t, 0};
        // simAddDrawingObjectItem(handles.drawer, p1);
        // simAddDrawingObjectItem(handles.drawer, p2);
    }
    else
    {
        stop();
    }
}

void Chassis::estimateTargetVel()
{
    static std::vector<Eigen::Vector2d> past_vels(2, Eigen::Vector2d(0, 0));
    static Eigen::Vector2d last_pos(0, 0);
    static float last_time = 0;
    static int count = 0;

    float cur_time = data.time_cur;
    float dt = cur_time - last_time;

    if (dt > 0.01)
    {
        Eigen::Vector2d cur_pos(data.customer_x, data.customer_y);
        Eigen::Vector2d vel = (cur_pos - last_pos) / dt;
        target_vel_ = (past_vels[0] + past_vels[1] + vel) / 3;

        past_vels[0] = past_vels[1];
        past_vels[1] = vel;
        last_time = cur_time;
        last_pos = cur_pos;
    }
}

void Chassis::moveToCustomer()
{
    float k1 = tan(data.robot_yaw);
    float k2 = tan(-30 / 180.0 * 3.14159);
    float x1 = data.robot_x;
    float y1 = data.robot_y;
    float x2 = -0.5;
    float y2 = 2;
    float p = (y2 - y1 + k1 * x1 - k2 * x2) / (k1 - k2);
    float q = (k1 * y2 - k2 * y1 + k1 * k2 * x1 - k1 * k2 * x2) / (k1 - k2);

    Eigen::Vector2f p1(data.robot_x, data.robot_y);
    Eigen::Vector2f p2(p, q);
    Eigen::Vector2f p3(x2, y2);

    float t = 0.1;
    // Eigen::Vector2f target = p1 * (1 - t) * (1 - t) * (1 - t) + p2 * 3 * (1 - t) * (1 - t) * t + p3 * 3 * (1 - t) * t * t + p4 * t * t * t;
    Eigen::Vector2f target = p1 * (1 - t) * (1 - t) + p2 * 2 * (1 - t) * t + p3 * t * t;

    static int k = 0;

    if (k % 30 == 0)
    {
        float t = 0;
        for (int i = 0; i < 200; i++)
        {
            // Eigen::Vector2f target = p1 * (1 - t) * (1 - t) * (1 - t) + p2 * 3 * (1 - t) * (1 - t) * t + p3 * 3 * (1 - t) * t * t + p4 * t * t * t;
            Eigen::Vector2f target = p1 * (1 - t) * (1 - t) + p2 * 2 * (1 - t) * t + p3 * t * t;
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
    left_wheel->setTargetVelocity(0, 10);
    right_wheel->setTargetVelocity(0, 10);
}