#include <iostream>

#include "zmq_controller.h"

int main(){

    std::cout << "MotionControl node start" << std::endl;

    std::shared_ptr<CB::RosController> controller = std::make_shared<CB::RosController>();
    if(!controller->initialize()){
        std::cout << "controller initialize fail" << std::endl;
        return 0;
    }
    controller->spin();
    
    return 0;
}