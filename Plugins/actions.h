#pragma once

#include "common.h"
#include "chassis.h"

enum Actions
{
    WAIT_AT_DOOR,
    MOVE_TO_CUSTOMER,
};

class Robot
{
public:
    int waitAtDoor();
    int moveToCustomer();

private:
    const float TRACK_WIDTH = 0.54;
    const float WHEEL_DIAMETER = 0.3;

    std::unique_ptr<Chassis> chassis = std::make_unique<Chassis>(Chassis());
};
