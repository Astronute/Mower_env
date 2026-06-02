#include "turn_on_robot.h"
#include <iostream>




int main(){

    std::cout << "turn_on_robot node start" << std::endl;

    std::shared_ptr<turn_on_robot::TurnOnRobot> robot = std::make_shared<turn_on_robot::TurnOnRobot>();
    if(!robot->initialize()){
        std::cout << "turn_on_robot initialize fail" << std::endl;
        return 0;
    }

    robot->spin();
    
    return 0;
}