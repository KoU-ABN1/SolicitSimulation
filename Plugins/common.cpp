#include "common.h"

SimHandles handles;
SimInfo data;

void updateAllInfo()
{
    float position[3];
    float orientation[3];

    simGetObjectPosition(handles.customer, -1, position); // update customer info
    simGetObjectOrientation(handles.customer, -1, orientation);
    data.customer_x = position[0];
    data.customer_y = position[1];
    data.customer_yaw = orientation[2];

    simGetObjectPosition(handles.robot, -1, position); // update robot info
    simGetObjectOrientation(handles.robot, -1, orientation);
    data.robot_x = position[0];
    data.robot_y = position[1];
    data.robot_yaw = orientation[2];

    simGetJointPosition(handles.waist_joint, &data.waist_joint_position); // update joint positions
    simGetJointPosition(handles.head_joint_1, &data.head_joint_1_position);
    simGetJointPosition(handles.head_joint_2, &data.head_joint_2_position);
    simGetJointPosition(handles.left_arm_joint_1, &data.left_arm_joint_1_position);
    simGetJointPosition(handles.left_arm_joint_2, &data.left_arm_joint_2_position);
    simGetJointPosition(handles.right_arm_joint_1, &data.right_arm_joint_1_position);
    simGetJointPosition(handles.right_arm_joint_2, &data.right_arm_joint_2_position);

    data.time_cur = simGetSimulationTime(); // update time
}

void getObjectHandles(std::vector<int> data)
{
    handles.left_wheel = data[0];
    handles.right_wheel = data[1];
    handles.waist_joint = data[2];
    handles.head_joint_1 = data[3];
    handles.head_joint_2 = data[4];
    handles.left_arm_joint_1 = data[5];
    handles.left_arm_joint_2 = data[6];
    handles.right_arm_joint_1 = data[7];
    handles.right_arm_joint_2 = data[8];
    handles.robot = data[9];
    handles.customer = data[10];
    handles.drawer = data[11];
}