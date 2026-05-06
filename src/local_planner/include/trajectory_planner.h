#pragma once

#include <vector>
#include <string>
#include <queue>
#include <memory>
#include <chrono>
#include <map>
#include <thread>
#include <atomic>
#include <numeric>
#include <condition_variable>

#include "all_subscriber.h"
#include "global_plan.h"
#include "bezierpathpoint.h"
#include "wall_rate.h"
#include "zmq_publisher.h"

namespace trajectoryplanner
{
	enum StateMembers {
		// pose terms
		StateMemberX = 0,
		StateMemberY,
		StateMemberZ,
		StateMemberRoll,
		StateMemberPitch,
		StateMemberYaw,
		// twist terms
		StateMemberVx,
		StateMemberVy,
		StateMemberVz,
		StateMemberGx,
		StateMemberGy,
		StateMemberGz,
		// 
		StateMemberAx,
		StateMemberAy,
		StateMemberAz
	};

	// control vector
	const int STATE_SIZE = 15;
	const int TWIST_SIZE = 6;
	const int POSE_SIZE = 6;
	const int POSITION_SIZE = 3;
	const int ORIENTATION_SIZE = 3;
	const int LINEAR_VELOCITY_SIZE = 3;
	const int ACCELERATION_SIZE = 3;
	const int POSITION_OFFSET = StateMembers::StateMemberX;
	const int ORIENTATION_OFFSET = StateMembers::StateMemberRoll;
	const int POSITION_V_OFFSET = StateMembers::StateMemberVx;
	const int ORIENTATION_V_OFFSET = StateMembers::StateMemberGx;
	const int POSITION_A_OFFSET = StateMembers::StateMemberAx;

    class TrajectoryPlanner{
    public:
        TrajectoryPlanner();

        ~TrajectoryPlanner();

        void execute();

        bool init(const YAML::Node & yaml_cfg);

		bool loadParams(const YAML::Node & yaml_cfg);

        void setPtr(
			std::shared_ptr<allsubscriber::AllSubscriber> _all_subsrciber, 
			std::shared_ptr<globalplanner::GlobalPlanner> _global_planner
		){
            this->all_subsrciber_ = _all_subsrciber;
			this->global_planner_ = _global_planner;
        }

		auto now() const {
			return std::chrono::system_clock::now();
		}

    private:
		std::atomic<bool> running_;

		YAML::Node filter_config_yaml_;

		ZmqPublisher zmq_publisher_;

        std::shared_ptr<allsubscriber::AllSubscriber> all_subsrciber_;

		std::shared_ptr<globalplanner::GlobalPlanner> global_planner_;

		nav_msgs::Path global_path_pub_;

		std::vector<TrajPoint> local_optimal_trajectory_;
    };

}