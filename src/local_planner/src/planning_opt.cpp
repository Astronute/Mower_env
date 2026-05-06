#include "planning_opt.h"

namespace planningopt{
    PlanningOpt::PlanningOpt(){

    }

    PlanningOpt::~PlanningOpt(){
        subscribe_thread_.join();
        global_thread_.join();
        trajectory_thread_.join();
    }

    void PlanningOpt::loadParams(){
        std::unique_lock<std::mutex> lock(config_mutex_);
        try{
            filter_config_yaml_ = YAML::LoadFile("/home/rpdzkj/Mower_env/src/local_planner/params/subscribe_params.yaml");
        } catch(const YAML::Exception& e){
            std::cout << "yaml parsing error: " << e.what() << std::endl;
            return;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    void PlanningOpt::SubscribeThread(){
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

    void PlanningOpt::TrajectoryThread(){
        trajectory_planner_ = std::make_shared<trajectoryplanner::TrajectoryPlanner>();
        while(!all_subscriber_||!global_planner_){
            std::cout << "waiting for all_subscriber or global_planner initialization" << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        trajectory_planner_->setPtr(all_subscriber_, global_planner_);
        {
            std::unique_lock<std::mutex> lock(config_mutex_);
            if(!trajectory_planner_->init(filter_config_yaml_)){
                std::cout << "TrajectoryThread exit" << std::endl;
                return;
            }
        }
        trajectory_planner_->execute();
    }

    void PlanningOpt::GlobalThread(){
        global_planner_ = std::make_shared<globalplanner::GlobalPlanner>();
        while(!all_subscriber_){
            std::cout << "waiting for all_subscriber initialization" << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        global_planner_->setPtr(all_subscriber_);
        {
            std::unique_lock<std::mutex> lock(config_mutex_);
            if(!global_planner_->init(filter_config_yaml_)){
                std::cout << "GlobalThread exit" << std::endl;
                return;
            }
        }
        global_planner_->execute();
    }

    void PlanningOpt::execute(){
        subscribe_thread_ = std::thread(&PlanningOpt::SubscribeThread, this);
        trajectory_thread_ = std::thread(&PlanningOpt::TrajectoryThread, this);
        global_thread_ = std::thread(&PlanningOpt::GlobalThread, this);
    }

}