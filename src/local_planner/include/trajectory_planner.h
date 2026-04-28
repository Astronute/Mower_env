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
#include "wall_rate.h"

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

        void init();

        void setPtr(std::shared_ptr<allsubscriber::AllSubscriber> _all_subsrciber){
            this->all_subsrciber_ = _all_subsrciber;
        }

    private:
        std::shared_ptr<allsubscriber::AllSubscriber> all_subsrciber_;
    };

}