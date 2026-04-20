#include <iostream>
#include "zmq_glob_planner.h"


int main(){

    std::cout << "GlobalPlanner node start" << std::endl;

    std::shared_ptr<global_planner::GlobalPlanner> planner = std::make_shared<global_planner::GlobalPlanner>();
    if(!planner->initialize()){
        std::cout << "Global planner initialize fail" << std::endl;
        return 0;
    }
    planner->spin();
    
    return 0;
}