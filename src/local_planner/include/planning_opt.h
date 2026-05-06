#pragma once

#include <iostream>
#include <thread>
#include <memory>
#include <mutex>

#include "all_subscriber.h"
#include "trajectory_planner.h"
#include "global_plan.h"

namespace planningopt
{

    class PlanningOpt{
    public:
        PlanningOpt();

        ~PlanningOpt();

        void loadParams();

        void SubscribeThread();

        void TrajectoryThread();

        void GlobalThread();

        void execute();

    private:
        YAML::Node filter_config_yaml_;

        std::mutex config_mutex_;

        std::shared_ptr<allsubscriber::AllSubscriber> all_subscriber_;

        std::shared_ptr<trajectoryplanner::TrajectoryPlanner> trajectory_planner_;

        std::shared_ptr<globalplanner::GlobalPlanner> global_planner_;

        // thread
        std::thread subscribe_thread_;

        std::thread global_thread_;

        std::thread trajectory_thread_;

    };

}
