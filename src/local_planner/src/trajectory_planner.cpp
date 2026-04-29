#include "trajectory_planner.h"


namespace trajectoryplanner
{
    TrajectoryPlanner::TrajectoryPlanner(){

    }

    TrajectoryPlanner::~TrajectoryPlanner(){

    }

    void TrajectoryPlanner::execute(){
        std::array<double, STATE_SIZE> robot_state;
        WallRate rate(1);
        while(true){
            robot_state = all_subsrciber_->getState();
            std::cout << "robot_x: " << robot_state[StateMemberX] 
                << " , robot_y: " << robot_state[StateMemberY] 
                << " , robot_yaw: " << robot_state[StateMemberYaw] 
                << " , robot_v: " << robot_state[StateMemberVx] 
                << " , robot_w: " << robot_state[StateMemberGz] 
                << std::endl;
            
            rate.sleep();
        }
    }

    void TrajectoryPlanner::init(){
        
    }


}