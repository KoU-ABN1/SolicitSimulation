#include "action/action.h"

using namespace local_planner;


ActionBase::ActionBase(std::shared_ptr<PlannerInput> pi)
{
	pi_ = pi;
}

ActionBase::~ActionBase()
{
	
}