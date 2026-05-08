#include "zmq_controller.h"
#include "wall_rate.h"

#include <iostream>
#include <chrono>

namespace CB {
    using namespace std::chrono_literals;

    RosController::RosController(): 
        controller_run_status_(CONTROL_FREE),
        controller_run_mode_(UNDEFINED_CMD),
        running_(true)
    {

    }

    RosController::~RosController(){
        // topic_subs_.clear();
        running_ = false;
        cv_.notify_one();
        control_thread_.join();
        std::cout << "control thread exit." << std::endl;
    }

    void RosController::reset(){
        controller_run_status_ = ControllerRunStatus::CONTROL_FREE;
        controller_run_mode_ = ControllerMode::UNDEFINED_CMD;

    }

    void RosController::spin(){
        std::unique_lock<std::mutex> lock(mtx_);
        cv_.wait(lock, [this]() { return !running_; });
    }

    bool RosController::initialize(const YAML::Node & yaml_cfg){

        reset();

        if(!mpc_controller_.initialize()){
            return false;
        }

        if(mpc_controller_.is_initialized()){
            control_frequency_ = mpc_controller_.get_control_rate();
            std::cout << "control_frequency set to " << control_frequency_ << std::endl;
        }

        if(!loadParams(yaml_cfg)){
            return false;
        }
        
        // 控制线程
        control_thread_ = std::thread(&RosController::periodicControl, this);

        return true;

    }

    bool RosController::loadParams(const YAML::Node & yaml_cfg){
        filter_config_yaml_ = yaml_cfg;

        if(filter_config_yaml_["map_frame_id"]){
            map_frame_id_ = filter_config_yaml_["map_frame_id"].as<std::string>();
        }
        else{
            std::cout << "missing param 'map_frame_id'" << std::endl;
            return false;
        }
        if(filter_config_yaml_["odom_frame_id"]){
            odom_frame_id_ = filter_config_yaml_["odom_frame_id"].as<std::string>();
        }
        else{
            std::cout << "missing param 'odom_frame_id_' " << std::endl;
            return false;
        }
        if(filter_config_yaml_["base_link_frame_id"]){
            base_link_frame_id_ = filter_config_yaml_["base_link_frame_id"].as<std::string>();
        }
        else{
            std::cout << "missing param 'base_link_frame_id' " << std::endl;
            return false;
        }
        if(filter_config_yaml_["world_frame_id"]){
            world_frame_id_ = filter_config_yaml_["world_frame_id"].as<std::string>();
        }
        else{
            std::cout << "missing param 'world_frame_id' " << std::endl;
            return false;
        }

        if(!mpc_controller_.is_initialized()){
            control_frequency_ = filter_config_yaml_["control_frequency"].as<double>();
            std::cout << "control_frequency set to " << control_frequency_ << std::endl;
        }

        if(filter_config_yaml_["t_delta_max"]){
            t_delta_max_ = filter_config_yaml_["t_delta_max"].as<double>();
        }
        else{
            std::cout << "missing param 't_delta_max'" << std::endl;
            return false;
        }
        if(filter_config_yaml_["t_delta_min"]){
            t_delta_min_ = filter_config_yaml_["t_delta_min"].as<double>();
        }
        else{
            std::cout << "missing param 't_delta_min'" << std::endl;
            return false;
        }
        if(filter_config_yaml_["s_delta_max"]){
            s_delta_max_ = filter_config_yaml_["s_delta_max"].as<double>();
        }
        else{
            std::cout << "missing param 's_delta_max'" << std::endl;
            return false;
        }
        if(filter_config_yaml_["s_delta_min"]){
            s_delta_min_ = filter_config_yaml_["s_delta_min"].as<double>();
        }
        else{
            std::cout << "missing param 's_delta_min'" << std::endl;
            return false;
        }

        // mpc parameters load
        int topic_ind = 0;
        bool more_params = false;
        do{
            std::stringstream ss;
            ss << "mpc" << topic_ind++;
            std::string controller_name = ss.str();
            std::string param_name;
            if(filter_config_yaml_[controller_name]){
                more_params = true;
                param_name = filter_config_yaml_[controller_name].as<std::string>();
            }
            else{
                more_params = false;
            }

            if(more_params){
                std::vector<double> q, r;
                loadParamMatrix(controller_name, q, r);
                matrix_params_q_[param_name] = q;
                matrix_params_r_[param_name] = r;
                std::cout << param_name << std::endl;
                std::cout << "Q: ";
                common::printVector(q);
                std::cout << "R: ";
                common::printVector(r);
            }

        }while(more_params);

        /*-------------------------------- zmq publisher-----------------------------------------*/

        if(filter_config_yaml_["zmq_pub_port"]){
            std::string port = filter_config_yaml_["zmq_pub_port"].as<std::string>();
            zmq_publisher_.initialize(port);
            std::cout << "controller node publisher bind to: " << port << std::endl;
        }
        else{
            std::cout << "missing param 'zmq_pub_port' " << std::endl;
            return false;
        }

        std::cout << "now: " << common::toSec(this->now()) << std::endl;

        std::cout << "param load success." << std::endl;
        return true;
    }

    void RosController::periodicControl(){
        controller_run_status_ = CONTROL_FREE;
        stratagy_status_ = 0x00;
        double control_duration = 0;
        bool update_trajectory_flag = true;
        int motor_timeout = 0;
        double cmd_vel_v = 0.0;
        double cmd_vel_w = 0.0;
        WallRate rate_control_(control_frequency_);

        while(running_){

            clock_t time_begin_cpu = clock();

            last_stratagy_status_ = stratagy_status_;

            /*--------------------------------get robot state-----------------------------------------*/

            // check sensor data
            std::array<double, allsubscriber::STATE_SIZE> robot_state;
            std::vector<allsubscriber::TrajPoint>().swap(local_optimal_trajectory_);
            bool sensor_valid = all_subscriber_->getState(robot_state);
            bool local_path_valid = all_subscriber_->getLocalTrajectory(local_optimal_trajectory_);
            
            last_cmd_twist_ = cmd_twist_;
            SysTimePoint ctrl_sys_now = this->now();
            cmd_twist_.mutable_header()->mutable_stamp()->CopyFrom(common::systimeToProto(ctrl_sys_now));
            // std::cout << "sensor_delayed: " << sensor_delayed << std::endl;
            if(!sensor_valid){
                std::cout << "localization dead, stop" << std::endl;
                stratagy_status_ |= 0x80;
            }
            else{ // normal situation
                // robot state
                std::cout << "robot_x: " << robot_state[allsubscriber::StateMemberX] 
                    << " , robot_y: " << robot_state[allsubscriber::StateMemberY] 
                    << " , robot_yaw: " << robot_state[allsubscriber::StateMemberYaw] 
                    << " , robot_v: " << robot_state[allsubscriber::StateMemberVx] 
                    << " , robot_w: " << robot_state[allsubscriber::StateMemberGz] 
                    << std::endl;
                robot_pose_.set_x(robot_state[allsubscriber::StateMemberX]);
                robot_pose_.set_y(robot_state[allsubscriber::StateMemberY]);
                robot_pose_.set_theta(robot_state[allsubscriber::StateMemberYaw]);
                robot_twist_.mutable_linear()->set_x(robot_state[allsubscriber::StateMemberVx]);
                robot_twist_.mutable_angular()->set_z(robot_state[allsubscriber::StateMemberGz]);

                // local trajectory
                if(local_optimal_trajectory_.size() <= 1){
                    std::cout<<"get localpath empty"<<std::endl;
                    cmd_twist_.mutable_twist()->mutable_linear()->set_x(0.0);
                    cmd_twist_.mutable_twist()->mutable_linear()->set_y(0.0);
                    cmd_twist_.mutable_twist()->mutable_angular()->set_z(0.0);
                    std::string serialized_data;
                    cmd_twist_.SerializeToString(&serialized_data);
                    zmq_publisher_.publishMessage("/cmd_vel", serialized_data);
                }

                /*--------------------------------update stratagy priority-----------------------------------------*/
                // 根据车辆当前状态/控制器状态/上一策略/设定的控制器运行模式(controller_run_mode_)来叠加带优先级的策略变量(stratagy_status_)
                // 急停置位
                if(!(last_stratagy_status_ >> 7)){
                    if(controller_run_status_ != CONTROL_FREE){
                        // safety_check
                        // if not safe stratagy_status_ |= 0x80;
                        stratagy_status_ &= ~0x80;
                    }
                    else{
                        stratagy_status_ &= ~0x80;
                    }
                }
                else{
                    if(std::fabs(robot_state[allsubscriber::StateMemberVx]) < 0.02 && fabs(robot_state[allsubscriber::StateMemberGz]) < 0.02){
                        stratagy_status_ &= ~0x80;
                    }
                    else{
                        stratagy_status_ |= 0x80; // 未完全停下 -->  继续急停策略
                    }
                }

                // 缓停置位
                if(controller_run_status_ == CONTROL_TIMEOUT || controller_run_mode_ == SLOWLY_STOP_CMD){
                    stratagy_status_ |= 0x40; // 上一控制周期控制器超时
                }
                else{
                    if(std::fabs(robot_state[allsubscriber::StateMemberVx]) < 0.02 && fabs(robot_state[allsubscriber::StateMemberGz]) < 0.02){
                        stratagy_status_ &= ~0x40;
                    }
                }

                // 测试模式置位
                if(controller_run_mode_ == LINE_MOVE_CMD || controller_run_mode_ == SELF_ROTATE_CMD || controller_run_mode_ == CIRCLE_MOVE_CMD){
                    stratagy_status_ |= 0x20;
                }
                else{
                    stratagy_status_ &= ~0x20;
                }
                if(controller_run_mode_ == TO_POINT_CMD){
                    stratagy_status_ |= 0x10;
                }
                else{
                    stratagy_status_ &= ~0x10;
                }
                if(controller_run_mode_ == TO_POSE_CMD){
                    stratagy_status_ |= 0x08;
                }
                else{
                    stratagy_status_ &= ~0x08;
                }
                if(controller_run_mode_ == BACKWARD_TO_POSE_CMD || controller_run_mode_ == FORWARD_TO_POSE_CMD){
                    stratagy_status_ |= 0x04;
                }
                else{
                    stratagy_status_ &= ~0x04;
                }
                if(!local_optimal_trajectory_.empty()){
                    if(local_optimal_trajectory_.size() <= 1){
                        stratagy_status_ &= ~0x01;
                    }
                    else{
                        stratagy_status_ |= 0x01;
                    }
                }
                else{
                    stratagy_status_ &= ~0x01;
                }
            }

            if(last_stratagy_status_ != stratagy_status_){
                status_change_flag_ = true;
                update_trajectory_flag = true;
                std::vector<TrajPoint>().swap(target_traj_);
            }
            else{
                status_change_flag_ = false;
            }

            /*--------------------------------get stratagy-----------------------------------------*/
            if(stratagy_status_ != 0){
                if(stratagy_status_ >> 7){
                    controller_stratagy_ = EMERGENCY_BRAKING_STR;
                    update_trajectory_flag = true;
                }
                else if(stratagy_status_ >> 6){
                    controller_stratagy_ = SLOWLY_STOP_STR;
                }
                else if(stratagy_status_ >> 5){
                    controller_stratagy_ = TEST_STR;
                }
                else if(stratagy_status_ >> 4){
                    controller_stratagy_ = TO_POINT_STR;
                }
                else if(stratagy_status_ >> 3){
                    controller_stratagy_ = TO_POSE_STR;
                }
                else if(stratagy_status_ >> 2){
                    if(controller_run_mode_ == BACKWARD_TO_POSE_CMD){
                        controller_stratagy_ = GO_BACKWORD_STR;
                    }
                    if(controller_run_mode_ == FORWARD_TO_POSE_CMD){
                        controller_stratagy_ = GO_FORWARD_STR;
                    }
                }
                else if(stratagy_status_ >> 1){
                    std::cout << "stratagy_status_: " << stratagy_status_ << "undefined" << std::endl;
                }
                else{
                    controller_stratagy_ = NORMAL_MODE_STR;
                }
            }
            else{
                controller_stratagy_ = NO_MOTION_STR;
            }
            std::cout << "stratagy_status_: " << array_controller_stratagy_[controller_stratagy_] << std::endl;
            /*--------------------------------pick up target trajectory-----------------------------------------*/

            switch(controller_stratagy_){
                case NORMAL_MODE_STR:
                    pick_followed_trajectory_ = LOCAL_TRAJ;
                    break;
                case EMERGENCY_BRAKING_STR:
                    pick_followed_trajectory_ = EMERG_BRAKE_TRAJ;
                    break;
                case SLOWLY_STOP_STR:
                    pick_followed_trajectory_ = LOCAL_TRAJ;
                    break;
                case TEST_STR:
                    break;
                case TO_POINT_STR:
                    break;
                case TO_POSE_STR:
                    break;
                case GO_BACKWORD_STR:
                    break;
                case GO_FORWARD_STR:
                    break;
                case NO_MOTION_STR:
                    pick_followed_trajectory_ = LOCAL_TRAJ;
                    break;
                default:
                    // pick_followed_trajectory_ = 
                    std::cout << "No Stratagy Find." << std::endl;
                    break;
            }

            if(update_trajectory_flag){
                target_traj_.clear();
                switch(pick_followed_trajectory_){
                    case LOCAL_TRAJ:
                        target_traj_ = local_optimal_trajectory_;
                        update_trajectory_flag = true;
                        break;
                    case EMERG_BRAKE_TRAJ:
                        // target_traj_ = 
                        update_trajectory_flag = false;
                        break;
                    default:
                        update_trajectory_flag = true;
                        std::cout << "No stratagy generate,Stop plan trajectory picked." << std::endl;
                        break;
                }
            }

            /*--------------------------------controller state update-----------------------------------------*/

            double t_now = common::toSec(this->now());
            
            if(target_traj_.empty()){
                controller_run_status_ = CONTROL_FREE;
            }
            else{
                if(target_traj_.size() <= 1){
                    if(controller_run_status_ == CONTROL_FREE){
                        std::cout << "There is no target trajectory with controller_stratagy_: " << controller_stratagy_ << std::endl;
                    }
                    else{
                        controller_run_status_ = CONTROL_END;
                    }
                }
                else{
                    if(controller_stratagy_ == GO_BACKWORD_STR || controller_stratagy_ == GO_FORWARD_STR){

                    }
                    else if(controller_stratagy_ == SLOWLY_STOP_STR || controller_stratagy_ == EMERGENCY_BRAKING_STR){

                    }
                    else{
                        if(t_now <= target_traj_.back().t){
                            controller_run_status_ = CONTROL_BUSY;
                        }
                        else{
                            if(controller_run_status_ == CONTROL_FREE){
                                std::cout << "There is no effictive target traj. " << "now: " << t_now << " the last point of traj:" << target_traj_.back().t << std::endl;
                            }
                            else{
                                controller_run_status_ = CONTROL_END;
                            }
                        }
                    }

                    if(std::fabs(robot_state[allsubscriber::StateMemberVx]) < 0.005 && std::fabs(robot_state[allsubscriber::StateMemberGz]) < 0.005 && controller_run_status_ == CONTROL_BUSY){
                        motor_timeout++;
                    }
                    else{
                        motor_timeout = 0;
                    }
                }
            }

            /*--------------------------------controller execute-----------------------------------------*/
            std::cout << "controller_run_status_: " << controller_run_status_ << std::endl;
            cmd_vel_v = 0.0;
            cmd_vel_w = 0.0;
            if(controller_run_status_ == CONTROL_FREE){
                std::cout << "need new target trajectory" << std::endl;
                cmd_vel_v = 0.0;
                cmd_vel_w = 0.0;
                update_trajectory_flag = true;
            }
            else if(controller_run_status_ == CONTROL_TIMEOUT){
                target_traj_.clear();
                controller_run_status_ = CONTROL_END;
            }
            else if(controller_run_status_ == CONTROL_END){
                cmd_vel_v = 0.0;
                cmd_vel_w = 0.0;
                motor_timeout = 0;
                target_traj_.clear();
                
                if(controller_stratagy_ == EMERGENCY_BRAKING_STR || controller_stratagy_ == SLOWLY_STOP_STR){
                    controller_run_status_ = CONTROL_END;
                }
                else{
                    stratagy_status_ = 0x00;
                    controller_run_status_ = CONTROL_FREE;
                    controller_stratagy_ = NO_MOTION_STR;
                    update_trajectory_flag = true;
                }
            }
            else if(controller_run_status_ == CONTROL_BUSY){
                /*--------------------------------Get the Target Point-----------------------------------------*/
                double t_delta_min = (robot_state[allsubscriber::StateMemberVx] <= 1.01) ? t_delta_min_ : t_delta_max_; //  seconds;
                double t_delta = t_delta_min;
                target_t_ = common::toSec(this->now()) + t_delta;
                mpc_controller_.findTargetTrajTimepoint(target_traj_point_, target_traj_, target_t_);
                target_t_ = common::toSec(this->now()) + t_delta_min_;
                mpc_controller_.findTargetTrajTimepoint(target_traj_point2_, target_traj_, target_t_);
                /*--------------------------------Control-----------------------------------------*/
                if(!mpc_controller_.run_controller(
                    matrix_params_q_["slow_mode"], 
                    matrix_params_r_["slow_mode"], 
                    robot_pose_, robot_twist_, 
                    target_traj_point_, target_traj_, 
                    cmd_vel_v, cmd_vel_w))
                {
                    cmd_vel_v = 0.0;
                    cmd_vel_w = 0.0;
                }
            }
            
            /*--------------------------------execute Control-----------------------------------------*/

            if(controller_run_status_ != CONTROL_FREE){
                // clip and smooth
                if(target_traj_point_.v == 0){
                    cmd_twist_.mutable_twist()->mutable_linear()->set_x(0.0);
                    cmd_twist_.mutable_twist()->mutable_linear()->set_y(0.0);
                    cmd_twist_.mutable_twist()->mutable_angular()->set_z(target_traj_point2_.w);
                    std::cout << "run target vel: " << 0.0 << " w: " << target_traj_point2_.w << std::endl;
                }
                else{
                    // cmd_twist_.twist.linear.x = cmd_vel_v;
                    cmd_twist_.mutable_twist()->mutable_linear()->set_x(target_traj_point2_.v);
                    cmd_twist_.mutable_twist()->mutable_linear()->set_y(0.0);
                    cmd_twist_.mutable_twist()->mutable_angular()->set_z(cmd_vel_w);
                    std::cout << "run target vel: " << target_traj_point2_.v << " mpc w: " << cmd_vel_w << std::endl;
                }
                std::string serialized_data;
                cmd_twist_.SerializeToString(&serialized_data);
                zmq_publisher_.publishMessage("/cmd_vel", serialized_data);
            }


            clock_t time_end_cpu = clock();
            control_duration = (double)(time_end_cpu - time_begin_cpu) / CLOCKS_PER_SEC;
            std::cout<<"####time: "<<control_duration<<std::endl;
            if(control_duration > 1.0 / control_frequency_){
                std::cout << "motion control loop missed its desired rate of "
                                  << control_frequency_ << "Hz. "
                                  << "it actually takes " << control_duration
                                  << " seconds" << std::endl;
                controller_run_status_ = CONTROL_TIMEOUT;
            }

            rate_control_.sleep();
        }
    }

    bool RosController::loadParamMatrix(
        const std::string & param_name, 
        std::vector<double> & q, 
        std::vector<double> & r
    ){
        q.clear();
        r.clear();
        std::string config_name = param_name + "_state_error_weight";
        YAML::Node cfg_array = filter_config_yaml_[config_name];
        if(cfg_array.IsSequence()){
            for(int i=0; i<cfg_array.size(); ++i){
                q.push_back(cfg_array[i].as<double>());
            }
        }

        config_name = param_name + "_input_weight";
        cfg_array = filter_config_yaml_[config_name];
        if(cfg_array.IsSequence()){
            for(int i=0; i<cfg_array.size(); ++i){
                r.push_back(cfg_array[i].as<double>());
            }
        }

        return true;
    }
}
