#pragma once

#include "eigen3/Eigen/Eigen"
#include "perception_typedef.h"
#include "data_definition.h"
#include "robot_model/footprint.h"
#include "scenario.h"
#include "planner/opt/robot_type_param.h"

// stores all the input for the local plan
struct PlannerInput
{
    // robot state
    Eigen::Vector3d pos; // x: ,y: ,z: 0 (global coordinates)
    Eigen::Vector3d dir; // x: ,y: ,z: 0 (global direction)
    double vx;           // linear velocity
    double vtheta;       // angular velocity

    // solicit task
    bool solicit_task;
    bool target_found;

    // return base
    bool return_base;
    bool base_initialized = false;
    Eigen::Vector3d base_pos;
    Eigen::Vector3d base_dir;

    // robot env
    semantic::RoadMapOut frame;
};
