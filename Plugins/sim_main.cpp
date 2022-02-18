#include "sim_main.h"
#include "common.h"
#include "actions.h"

void mainSimulation()
{
    updateAllInfo();

    static Robot robot;
    static int state = WAIT_AT_DOOR;

    switch (state)
    {
    case WAIT_AT_DOOR:
    {
        state = robot.waitAtDoor();
        break;
    }

    case MOVE_TO_CUSTOMER:
    {
        state = robot.moveToCustomer();
        break;
    }
    }
}