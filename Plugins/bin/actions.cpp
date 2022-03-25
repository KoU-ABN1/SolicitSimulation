#include "actions.h"

int Robot::waitAtDoor()
{
    const float WELCOME_DIS = 10;
    float dist = sqrt(pow(data.robot_x - data.customer_x, 2) + pow(data.robot_y - data.customer_y, 2));

    if (dist < WELCOME_DIS)
        return MOVE_TO_CUSTOMER;

    return WAIT_AT_DOOR;
}

int Robot::moveToCustomer()
{
    float dist = sqrt(pow(data.robot_x - data.customer_x, 2) + pow(data.robot_y - data.customer_y, 2));

    if (dist > 2)
        chassis->moveToCustomer();
    else
        chassis->stop();

    return MOVE_TO_CUSTOMER;
}