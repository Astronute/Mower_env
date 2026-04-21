#include <iostream>
#include "zmq_glob_planner.h"


int main(){

    std::cout << "GlobalPlanner node start" << std::endl;

    std::shared_ptr<global_planner::GlobalPlanner> planner = std::make_shared<global_planner::GlobalPlanner>();
    if(!planner->initialize()){
        std::cout << "Global planner initialize fail" << std::endl;
        return 0;
    }
    planner->getInfoFromJSON("/home/tom/Mower_env/src/global_planner/params/mission_test.json");

    planner->spin();
    
    return 0;
}