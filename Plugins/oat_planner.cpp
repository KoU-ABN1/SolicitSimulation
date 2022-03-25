#include "oat_planner.h"
#include "math.h"
#include <algorithm>
#include <iostream>

bool OatPlanner::isApproachable()
{
    float dist_interval = 0.1;

    float min_cost = 9999999;
    bool target_found = false;
    Eigen::Vector3f optimal_pose;
    Eigen::Vector2f h_pos;

    int s1 = 60;
    int s2 = 10;

    for (int i = 0; i < s1; i++) 
    {
        Eigen::Vector2f person_pos(human_pose_[0] + i * dist_interval * cos(human_pose_[2]),
                                   human_pose_[1] + i * dist_interval * sin(human_pose_[2]));

        float t2 = (float)i * dist_interval / human_vel_;

        for (int j = 0; j < s2; j++)
        {
            float theta = (human_pose_[2] - human_fov_theta_ / 2) + j * (human_fov_theta_ / (s2 - 1));
            Eigen::Vector3f candidate(
                person_pos[0] + human_fov_length_ * cos(theta),
                person_pos[1] + human_fov_length_ * sin(theta),
                rectifyAngle(theta + M_PI));

            float t1 = 1.3 * sqrt(pow(robot_pose_[0] - candidate[0], 2) + pow(robot_pose_[1] - candidate[1], 2)) / vel_set_;

            if (t1 < t2)
            {
                float cost = calcTargetCost(robot_pose_, candidate, t2, theta - human_pose_[2]);

                if (cost < min_cost)
                {
                    target_found = true;
                    min_cost = cost;
                    optimal_pose = candidate;
                    h_pos = person_pos;
                }
            }
        }
    }

    if (!target_found)
        return false;

    std::vector<Eigen::Vector2f> path = calcBezierPath(robot_pose_, optimal_pose);
    float l1 = calcArcLength(path);
    float t1 = l1 / vel_set_;

    float l2 = sqrt(pow(h_pos[0] - human_pose_[0], 2) + pow(h_pos[1] - human_pose_[1], 2));
    float t2 = l2 / human_vel_;

    if (t1 < t2)
    {
        human_approachable_ = true;
        target_pose_ = optimal_pose;
        return true;
    }

    return false;
}

float OatPlanner::calcTargetCost(const Eigen::Vector3f& start, const Eigen::Vector3f& end, float T, float alpha)
{
    float te = atan2(end[1] - start[1], end[0] - start[0]);

    float angular_diff = abs(rectifyAngle(start[2] - te)) + abs(rectifyAngle(end[2] - te));
    float linear_diff = sqrt(pow(end[0] - start[0], 2) + pow(end[1] - start[1], 2));

    float w[5] = {1, 1, 1, 0.5, 2.5};
    float c[5] = {0, 0, 0, 0, 0};
    c[0] = linear_diff;
    c[1] = angular_diff;
    c[2] = angular_diff / linear_diff;
    c[3] = T;
    c[4] = abs(alpha);

    float cost = 0;
    for (int i = 0; i < 5; i++)
    {
        cost += w[i] * c[i];
    }

    return cost;
}

std::vector<Eigen::Vector2f> OatPlanner::outputPath()
{
    std::vector<Eigen::Vector2f> path;
    if (human_approachable_)
    {
       path = calcBezierPath(robot_pose_, target_pose_);
    }
    return path;
}

std::vector<Eigen::Vector2f> OatPlanner::calcBezierPath(const Eigen::Vector3f &start, const Eigen::Vector3f &end)
{
    float a1 = sin(start[2]);
    float b1 = -cos(start[2]);
    float c1 = start[1] * cos(start[2]) - start[0] * sin(start[2]);
    float a2 = sin(end[2]);
    float b2 = -cos(end[2]);
    float c2 = end[1] * cos(end[2]) - end[0] * sin(end[2]);

    float l = sqrt(pow((end[0] - start[0]), 2) + pow((end[1] - start[1]), 2));
    float d1 = abs(a1 * end[0] + b1 * end[1] + c1);
    float d2 = abs(a2 * start[0] + b2 * start[1] + c2);

    float k1 = 0.5 * sqrt(l * l - d1 * d1);
    float k2 = 0.5 * sqrt(l * l - d2 * d2);

    Eigen::Vector2f p1(start[0], start[1]);
    Eigen::Vector2f p2(start[0] + k1 * cos(start[2]), start[1] + k1 * sin(start[2]));
    Eigen::Vector2f p3(end[0] - k2 * cos(end[2]), end[1] - k2 * sin(end[2]));
    Eigen::Vector2f p4(end[0], end[1]);

    float num_points = 100;
    float interval = 1 / (num_points - 1);
    std::vector<Eigen::Vector2f> points;
    for (float t = 0; t <= 1; t += interval)
    {
        Eigen::Vector2f point = (1 - t) * (1 - t) * (1 - t) * p1 + 3 * (1 - t) * (1 - t) * t * p2 + 3 * (1 - t) * t * t * p3 + t * t * t * p4;
        points.push_back(point);
    }

    return points;
}

float OatPlanner::calcArcLength(const std::vector<Eigen::Vector2f> &points)
{
    float length = 0;
    for (int i = 0; i < points.size() - 1; i++)
    {
        float dist = sqrt(pow(points[i + 1][0] - points[i][0], 2) + pow(points[i + 1][1] - points[i][1], 2));
        length += dist;
    }
    return length;
}

std::pair<float, float> OatPlanner::doPlan()
{
    if (!human_approachable_)
        return std::make_pair(0, 0);

    float a1 = sin(robot_pose_[2]);
    float b1 = -cos(robot_pose_[2]);
    float c1 = robot_pose_[1] * cos(robot_pose_[2]) - robot_pose_[0] * sin(robot_pose_[2]);
    float a2 = sin(target_pose_[2]);
    float b2 = -cos(target_pose_[2]);
    float c2 = target_pose_[1] * cos(target_pose_[2]) - target_pose_[0] * sin(target_pose_[2]);

    float l = sqrt(pow((target_pose_[0] - robot_pose_[0]), 2) + pow((target_pose_[1] - robot_pose_[1]), 2));
    float d1 = abs(a1 * target_pose_[0] + b1 * target_pose_[1] + c1);
    float d2 = abs(a2 * robot_pose_[0] + b2 * robot_pose_[1] + c2);

    float k1 = 0.5 * sqrt(l * l - d1 * d1);
    float k2 = 0.5 * sqrt(l * l - d2 * d2);

    Eigen::Vector2f p1(robot_pose_[0], robot_pose_[1]);
    Eigen::Vector2f p2(robot_pose_[0] + k1 * cos(robot_pose_[2]), robot_pose_[1] + k1 * sin(robot_pose_[2]));
    Eigen::Vector2f p3(target_pose_[0] - k2 * cos(target_pose_[2]), target_pose_[1] - k2 * sin(target_pose_[2]));
    Eigen::Vector2f p4(target_pose_[0], target_pose_[1]);

    float t = 0.1;
    Eigen::Vector2f track_point = (1 - t) * (1 - t) * (1 - t) * p1 + 3 * (1 - t) * (1 - t) * t * p2 + 3 * (1 - t) * t * t * p3 + t * t * t * p4;

    float linear_vel = vel_set_;

    float m = track_point[0] - robot_pose_[0];
    float n = track_point[1] - robot_pose_[1];
    float r = (m * m + n * n) / (2 * m * sin(robot_pose_[2]) - 2 * n * cos(robot_pose_[2]));
    float angular_vel = linear_vel / r;

    angular_vel *= -1; // the direction of angular velocity should be changed
    angular_vel = std::min(angular_vel, 0.5f);
    angular_vel = std::max(angular_vel, -0.5f);

    return std::make_pair(linear_vel, angular_vel);
}
