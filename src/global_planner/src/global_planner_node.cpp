#include <iostream>
#include "zmq_glob_planner.h"


int main(){

    std::cout << "GlobalPlanner node start" << std::endl;

    std::shared_ptr<global_planner::GlobalPlanner> planner = std::make_shared<global_planner::GlobalPlanner>();
    if(!planner->initialize()){
        std::cout << "Global planner initialize fail" << std::endl;
        return 0;
    }
    // planner->getInfoFromJSON("/home/rpdzkj/Mower_env/src/global_planner/params/mission_test.json");

    // // test zmq client
    // ZmqSubscriber zmq_subscriber;
    // zmq_subscriber.initializeRequestClient("tcp://127.0.0.1:5544");
    // std::string req = "/home/rpdzkj/Mower_env/src/global_planner/params/mission_test.json";
    // zmq_subscriber.sendRequest(req);

    planner->spin();
    
    return 0;
}