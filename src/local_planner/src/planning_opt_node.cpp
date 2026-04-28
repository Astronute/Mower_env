#include "planning_opt.h"

#ifndef VERSION
#define VERSION " 2025-08-18"
#endif
#ifndef ROBOT_NAME
#define ROBOT_NAME "yat"
#endif

int main(){
    planningopt::PlanningOpt planning_opt;
    planning_opt.loadParams();
    planning_opt.execute();

    return 0;
}