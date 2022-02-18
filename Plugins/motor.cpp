#include "motor.h"

void Motor::setTargetVelocity(const float velocity, const float acceleration)
{
    simSetObjectInt32Parameter(handle, sim_jointintparam_ctrl_enabled, 0); // enable velocity control mode;

    if (acceleration < 0)
    {
        simSetJointTargetVelocity(handle, velocity);
    }
    else
    {
        float v;
        float dv = velocity - vel_last;
        float dt = data.time_cur - time_last;

        if (dv / dt > acceleration)
            v = vel_last + acceleration * dt;
        else if (dv / dt < -acceleration)
            v = vel_last - acceleration * dt;
        else
            v = velocity;

        time_last = data.time_cur;
        vel_last = v;

        simSetJointTargetVelocity(handle, v);
    }
}

bool Motor::setTargetPosition(const float position, const float upper_velocity, const float kp, const float ki, const float kd)
{
    float position_cur;
    simGetJointPosition(handle, &position_cur);

    float tolerance = 5 / 180.0 * PI;
    if (abs(position_cur - position) < tolerance)
        return true;

    simSetObjectInt32Parameter(handle, sim_jointintparam_ctrl_enabled, 1); // enable position control mode

    simSetObjectFloatParameter(handle, sim_jointfloatparam_upper_limit, upper_velocity);
    simSetObjectFloatParameter(handle, sim_jointfloatparam_pid_p, kp);
    simSetObjectFloatParameter(handle, sim_jointfloatparam_pid_i, ki);
    simSetObjectFloatParameter(handle, sim_jointfloatparam_pid_d, kd);

    simSetJointTargetPosition(handle, position);

    return false;
}