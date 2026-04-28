#pragma once

#include <iostream>
#include <thread>
#include <memory>

#include "all_subscriber.h"
#include "trajectory_planner.h"

namespace planningopt
{

    class PlanningOpt{
    public:
        PlanningOpt();

        ~PlanningOpt();

        void loadParams();

        void SubscribeThread();

        void TrajectoryThread();

        void execute();

    private:
        std::shared_ptr<allsubscriber::AllSubscriber> all_subscriber_;

        std::shared_ptr<trajectoryplanner::TrajectoryPlanner> trajectory_planner_;

        // thread
        std::thread subscribe_thread_;

        std::thread global_thread_;

        std::thread trajectory_thread_;

    };

}
