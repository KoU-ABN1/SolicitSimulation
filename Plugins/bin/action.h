#pragma once

#include "data_definition.h"
#include "planner_input.h"

using namespace local_planner;

class ActionBase
{
public:
    ActionBase(std::shared_ptr<PlannerInput> pi);

    virtual ~ActionBase();

    virtual void finish() = 0;

    virtual TrackResult action() = 0;

protected:
	std::shared_ptr<PlannerInput> pi_;

};