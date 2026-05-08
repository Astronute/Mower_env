#include <iostream>

#include "motion_control_node.h"

MotionControlNode::MotionControlNode():
    running_(true)
{

}

MotionControlNode::~MotionControlNode(){
    running_ = false;
    cv_.notify_one();
    subscribe_thread_.join();
    controller_thread_.join();
}

void MotionControlNode::SubscribeThread(){
    all_subscriber_ = std::make_shared<allsubscriber::AllSubscriber>();
    {
        std::unique_lock<std::mutex> lock(config_mutex_);
        if(!all_subscriber_->initialize(filter_config_yaml_)){
            std::cout << "SubscribeThread exit" << std::endl;
            return;
        }
    }
    all_subscriber_->spin();
}

void MotionControlNode::ControllerThread(){
    controller_ = std::make_shared<CB::RosController>();

    while(!all_subscriber_){
        std::cout << "waiting for all_subscriber initialization" << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    controller_->setPtr(all_subscriber_);

    {
        std::unique_lock<std::mutex> lock(config_mutex_);
        if(!controller_->initialize(filter_config_yaml_)){
            std::cout << "controller initialize fail" << std::endl;
            return ;
        }
    }
    controller_->spin();
}

bool MotionControlNode::initialize(){
    try{
        filter_config_yaml_ = YAML::LoadFile("/home/rpdzkj/Mower_env/src/motion_control/params/filter_params.yaml");
    } catch(const YAML::Exception& e){
        std::cout << "yaml parsing error: " << e.what() << std::endl;
    }

    subscribe_thread_ = std::thread(&MotionControlNode::SubscribeThread, this);
    controller_thread_ = std::thread(&MotionControlNode::ControllerThread, this);
    return true;
}

void MotionControlNode::spin(){
    std::unique_lock<std::mutex> lock(mtx_);
    cv_.wait(lock, [this]() { return !running_; });
}


int main(){
    std::cout << "MotionControl node start" << std::endl;
    
    MotionControlNode motion_control_node;
    if(!motion_control_node.initialize()){
        std::cout << "MotionControlNode initialization failed" << std::endl;
        return -1;
    }
    motion_control_node.spin();
    
    return 0;
}