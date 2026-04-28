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

    }

    void PlanningOpt::SubscribeThread(){
        all_subscriber_ = std::make_shared<allsubscriber::AllSubscriber>();
        all_subscriber_->initialize();
        all_subscriber_->spin();
    }

    void PlanningOpt::TrajectoryThread(){
        trajectory_planner_ = std::make_shared<trajectoryplanner::TrajectoryPlanner>();
        while(!all_subscriber_){
            std::cout << "waiting for all_subscriber initialization" << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        trajectory_planner_->setPtr(all_subscriber_);
        trajectory_planner_->init();
        trajectory_planner_->execute();
    }

    void PlanningOpt::GlobalThread(){
        global_planner_ = std::make_shared<globalplanner::GlobalPlanner>();
        if(global_planner_->init()){
            global_planner_->execute();
        }
        else{
            std::cout << "GlobalThread exit" << std::endl;
        }
    }

    void PlanningOpt::execute(){
        subscribe_thread_ = std::thread(&PlanningOpt::SubscribeThread, this);
        trajectory_thread_ = std::thread(&PlanningOpt::TrajectoryThread, this);
    }

}