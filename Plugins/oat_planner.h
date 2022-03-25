#pragma once

#include "/usr/include/eigen3/Eigen/Dense"
#include <vector>

class OatPlanner  // Optimal approaching trajectory planner
{
public:
    OatPlanner(Eigen::Vector3f robot_pose, Eigen::Vector3f human_pose, float robot_vel, float human_vel, float vel_set) : 
        robot_pose_(robot_pose), human_pose_(human_pose), robot_vel_(robot_vel), human_vel_(human_vel), vel_set_(vel_set) {}

    ~OatPlanner() {}

    bool isApproachable();
    std::pair<float, float> doPlan();
    std::vector<Eigen::Vector2f> outputPath();

private:
    Eigen::Vector3f robot_pose_;
    Eigen::Vector3f human_pose_;
    float robot_vel_;
    float human_vel_;
    float vel_set_;
    
    float human_fov_theta_ = 120 / 180.0 * M_PI;
    float human_fov_length_ = 1.5;

    bool human_approachable_ = false;
    Eigen::Vector3f target_pose_;

private:
    inline float rectifyAngle(float angle)
    {
        while (angle < -M_PI)
            angle += 2 * M_PI;
        while (angle >= M_PI)
            angle -= 2 * M_PI;
        return angle;
    }

    float calcTargetCost(const Eigen::Vector3f& start, const Eigen::Vector3f& end, float T, float alpha);
    std::vector<Eigen::Vector2f> calcBezierPath(const Eigen::Vector3f &start, const Eigen::Vector3f &end);
    float calcArcLength(const std::vector<Eigen::Vector2f> &points);
};
