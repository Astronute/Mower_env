#include "trajectory_planner.h"


namespace trajectoryplanner
{
    TrajectoryPlanner::TrajectoryPlanner():
        running_(true)
    {

    }

    TrajectoryPlanner::~TrajectoryPlanner(){
        running_ = false;
    }

    bool TrajectoryPlanner::loadParams(const YAML::Node & yaml_cfg){
        filter_config_yaml_ = yaml_cfg;
        
        /*-------------------------------- zmq publisher-----------------------------------------*/

        if(filter_config_yaml_["zmq_pub_port"]){
            std::string port = filter_config_yaml_["zmq_pub_port"].as<std::string>();
            zmq_publisher_.initialize(port);
            std::cout << "localplanner publisher bind to: " << port << std::endl;
        }
        else{
            std::cout << "missing param 'zmq_pub_port' " << std::endl;
            return false;
        }

        return true;
    }

    bool TrajectoryPlanner::init(const YAML::Node & yaml_cfg){
        if(!loadParams(yaml_cfg)){
            std::cout << "Failed to load parameters!" << std::endl;
            return false;
        }

        return true;
    }

    void TrajectoryPlanner::execute(){
        std::array<double, STATE_SIZE> robot_state;
        std::vector<PathPoint> global_path;
        PathPoint point;
        WallRate rate(25);

        while(running_){
            robot_state = all_subsrciber_->getState();
            std::cout << "robot_x: " << robot_state[StateMemberX] 
                << " , robot_y: " << robot_state[StateMemberY] 
                << " , robot_yaw: " << robot_state[StateMemberYaw] 
                << " , robot_v: " << robot_state[StateMemberVx] 
                << " , robot_w: " << robot_state[StateMemberGz] 
                << std::endl;

            global_path_pub_ = global_planner_->GetGlobalPath();
            if(!global_path_pub_.poses().empty()){
                std::cout << "global path size: " << global_path_pub_.poses().size() << std::endl;
                global_path.clear();
                for(int i=0; i<global_path_pub_.poses().size(); ++i){
                    point.x = global_path_pub_.poses(i).pose().position().x();
                    point.y = global_path_pub_.poses(i).pose().position().y();
                    if(i > 0){
                        if(hypot(point.x - global_path.back().x, point.y - global_path.back().y) >= 0.05){
                            global_path.push_back(point);
                        }
                    }
                    else{
                        global_path.push_back(point);
                    }
                }
            }

            if(!global_path.empty()){
                pnc_msgs::PlanningTrajectory local_path;
                local_path.mutable_header()->mutable_stamp()->CopyFrom(navicommon::systimeToProto(this->now()));
                local_path.mutable_header()->set_frame_id("map");
                for(int i=0; i<global_path.size(); ++i){
                    pnc_msgs::PointVec8f p;
                    p.set_x(global_path[i].x);
                    p.set_y(global_path[i].y);
                    if(i < global_path.size() - 1){
                        p.set_yaw(atan2(global_path[i+1].y - global_path[i].y, global_path[i+1].x - global_path[i].x));
                    }
                    else if(i > 0){
                        p.set_yaw(atan2(global_path[i].y - global_path[i-1].y, global_path[i].x - global_path[i-1].x));
                    }
                    else{
                        p.set_yaw(0.0);
                    }
                    p.set_kappa(global_path[i].kappa);
                    p.set_s(global_path[i].s);
                    p.set_t(navicommon::toSec(this->now()) + i);
                    p.set_v(0.3);
                    p.set_w(0.0);
                    *local_path.add_trajectory() = p;
                }
                std::string serialized_data;
                local_path.SerializeToString(&serialized_data);
                zmq_publisher_.publishMessage("/optimalTrajectory/localOptimalPath", serialized_data);
                std::cout << "publish local optimal path, size: " << local_path.trajectory_size() << std::endl;
            }
            else{
                local_optimal_trajectory_.clear();
            }

            rate.sleep();
        }
    }


}