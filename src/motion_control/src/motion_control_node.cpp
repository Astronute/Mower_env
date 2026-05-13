#include <iostream>

#include "motion_control_node.h"
#include "glog/logging.h"

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
            LOG(ERROR) << "yaml parsing error SubscribeThread exit";
            return;
        }
    }
    LOG(INFO) << "SubscribeThread start......";
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
            LOG(ERROR) << "yaml parsing error ControllerThread exit";
            return ;
        }
    }
    LOG(INFO) << "ControllerThread start......";
    controller_->spin();
}

bool MotionControlNode::initialize(){
    try{
        std::string yaml_path = "/home/rpdzkj/Mower_env/src/motion_control/params/filter_params.yaml";
        LOG(INFO) << "param file dir: " << yaml_path;
        filter_config_yaml_ = YAML::LoadFile(yaml_path);
    } catch(const YAML::Exception& e){
        LOG(ERROR) << "yaml parsing error: " << e.what();
    }

    subscribe_thread_ = std::thread(&MotionControlNode::SubscribeThread, this);
    controller_thread_ = std::thread(&MotionControlNode::ControllerThread, this);
    return true;
}

void MotionControlNode::spin(){
    std::unique_lock<std::mutex> lock(mtx_);
    cv_.wait(lock, [this]() { return !running_; });
}


int main(int argc, char* argv[]){

    google::InitGoogleLogging(argv[0]);
    FLAGS_logbufsecs = 0;
    std::string filename = "/home/rpdzkj/Mower_env/log/motion_control/";
    std::cout << "log dir: " << filename << std::endl;
    google::SetLogDestination(google::GLOG_INFO, filename.c_str());
    google::SetLogDestination(google::GLOG_WARNING, filename.c_str());
    google::SetLogDestination(google::GLOG_ERROR, filename.c_str());
    google::SetLogDestination(google::GLOG_FATAL, filename.c_str());
    LOG(INFO) << "MotionControl node start";

    MotionControlNode motion_control_node;
    if(!motion_control_node.initialize()){
        LOG(ERROR) << "MotionControlNode initialization failed";
        return -1;
    }
    motion_control_node.spin();

    google::ShutdownGoogleLogging();
    
    return 0;
}