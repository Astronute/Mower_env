#include "zmq_controller.h"
#include "wall_rate.h"

#include <iostream>
#include <chrono>

namespace CB {
    using namespace std::chrono_literals;

    std::string array_controller_status_[5] = {
		"CONTROL_FREE",
		"CONTROL_BUSY",
		"CONTROL_ERROR",
		"CONTROL_END",
		"CONTROL_TIMEOUT"
    };

	std::string array_controller_mode_[9] = {
		"SLOWLY_STOP_CMD",
		"RE_START_CMD",
		"LINE_MOVE_CMD",
		"ROTATE_MOVE_CMD",
		"CIRCLE_MOVE_CMD",
		"FORWARD_TO_POSE_CMD",
		"BACKWARD_TO_POSE_CMD",
        "ROTATE_TO_POSE_CMD",
		"UNDEFINED_CMD"
	};

	std::string array_controller_stratagy_[8] = {
		"EMERGENCY_BRAKING_STR",
		"SLOWLY_STOP_STR",
		"TEST_STR",
		"FORWARD_TO_POSE_STR",
		"BACKWARD_TO_POSE_STR",
        "ROTATE_TO_POSE_STR",
		"NORMAL_MODE_STR",
		"NO_MOTION_STR"
	};

    RosController::RosController(): 
        running_(true)
    {
        reset();
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
        set_outstratagy(ControllerMode::UNDEFINED_CMD);

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

        /*-------------------------------- zmq server-----------------------------------------*/
        if(filter_config_yaml_["zmq_server_port"]){
            std::string port = filter_config_yaml_["zmq_server_port"].as<std::string>();
            zmq_server_.initializeRequestHandler(port);
            zmq_server_.setRequestHandler([this](const std::string& request) -> std::string {
                return this->zmq_server_callback(request);
            });
            zmq_server_.startRequestHandler();
            std::cout << "mission execute server bind to: " << port << std::endl;
        }
        else{
            std::cout << "missing param 'zmq_server_port' " << std::endl;
            return false;
        }

        std::cout << "now: " << common::toSec(this->now()) << std::endl;

        std::cout << "param load success." << std::endl;
        return true;
    }

    void RosController::set_outstratagy(const ControllerMode& mode){
        {
            std::lock_guard<std::mutex> lock(out_stratagy_mtx_);
            controller_out_stratagy_.out_stratagy = mode;
        }
        std::cout << "set_outstratagy set to: " << array_controller_mode_[mode] << std::endl;
    }

    OutStratagyInfo RosController::get_outStratagyInfo(){
        std::lock_guard<std::mutex> lock(out_stratagy_mtx_);
        return controller_out_stratagy_;
    }

    void RosController::periodicControl(){
        controller_run_status_ = CONTROL_FREE;
        stratagy_param_ = 0x00;
        double control_duration = 0;
        bool update_trajectory_flag = true;
        double time_follow_error = 0.0;
        double path_follow_error = 0.0;
        double yaw_follow_error = 0.0;
        double last_yaw_follow_error = 0.0;
        int motor_timeout = 0;
        double cmd_vel_v = 0.0;
        double cmd_vel_w = 0.0;
        WallRate rate_control_(control_frequency_);


        while(running_){

            clock_t time_begin_cpu = clock();

            ControllerMode out_stratagy = get_outStratagyInfo().out_stratagy;
            pnc_msgs::MotionOutSignal out_strdata = get_outStratagyInfo().data;
            last_stratagy_param_ = stratagy_param_;

            /*--------------------------------get robot state-----------------------------------------*/
            /*--------------------------------update stratagy param-----------------------------------------*/
            std::array<double, allsubscriber::STATE_SIZE> robot_state;
            std::vector<allsubscriber::TrajPoint>().swap(local_optimal_trajectory_);
            bool sensor_valid = all_subscriber_->getState(robot_state);
            
            last_cmd_twist_ = cmd_twist_;
            SysTimePoint ctrl_sys_now = this->now();
            cmd_twist_.mutable_header()->mutable_stamp()->CopyFrom(common::systimeToProto(ctrl_sys_now));
            if(!sensor_valid){
                std::cout << "localization dead, stop" << std::endl;
                stratagy_param_ |= 0x80;
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
                std::vector<allsubscriber::TrajPoint>().swap(local_optimal_trajectory_);
                bool local_path_valid = all_subscriber_->getLocalTrajectory(local_optimal_trajectory_);
                /*
                    8: Emergency Braking Flag
                    7: Stop Flag due to new goal router got
                    6: Closed Loop Test Mode / Open Loop Test Mode
                    5: CarSelf walk to Specific Position Goal
                    4: Car walk  to Specific Pose Goal
                    3: Charge Flag from router or Command Client
                    2: Only Rotate due to near goal router got
                    1: Number of Trajectory size<=1 局部轨迹是否有效
                */
                // 急停置位
                if(!(last_stratagy_param_ >> 7)){ // If last str is not emergency stop
                    stratagy_param_ &= ~0x80;
                }
                else{
                    if(std::fabs(robot_state[allsubscriber::StateMemberVx]) < 0.02 && fabs(robot_state[allsubscriber::StateMemberGz]) < 0.02){
                        stratagy_param_ &= ~0x80;
                    }
                    else{
                        stratagy_param_ |= 0x80; // 未完全停下 -->  继续急停策略
                    }
                }
                // 缓停置位
                if(controller_run_status_ == CONTROL_TIMEOUT || out_stratagy == SLOWLY_STOP_CMD){
                    stratagy_param_ |= 0x40; // 上一控制周期控制器超时
                }
                else{
                    if(std::fabs(robot_state[allsubscriber::StateMemberVx]) < 0.02 && fabs(robot_state[allsubscriber::StateMemberGz]) < 0.02){
                        stratagy_param_ &= ~0x40;
                    }
                }
                // 测试模式置位
                if(out_stratagy == LINE_MOVE_CMD || out_stratagy == ROTATE_MOVE_CMD || out_stratagy == CIRCLE_MOVE_CMD){
                    stratagy_param_ |= 0x20;
                }
                else{
                    stratagy_param_ &= ~0x20;
                }
                if(out_stratagy == BACKWARD_TO_POSE_CMD || out_stratagy == FORWARD_TO_POSE_CMD){
                    stratagy_param_ |= 0x10;
                }
                else{
                    stratagy_param_ &= ~0x10;
                }
                if(out_stratagy == ROTATE_TO_POSE_CMD){
                    stratagy_param_ |= 0x08;
                }
                else{
                    stratagy_param_ &= ~0x08;
                }
                if(!local_optimal_trajectory_.empty() && local_path_valid){
                    if(local_optimal_trajectory_.size() <= 1){
                        stratagy_param_ &= ~0x01;
                    }
                    else{
                        stratagy_param_ |= 0x01;
                    }
                }
                else{
                    stratagy_param_ &= ~0x01;
                }
            }

            if(last_stratagy_param_ != stratagy_param_){
                status_change_flag_ = true;
                update_trajectory_flag = true;
                std::vector<TrajPoint>().swap(target_traj_);
            }
            else{
                status_change_flag_ = false;
            }

            /*--------------------------------get stratagy based on priority----------------------------------*/
            if(stratagy_param_ != 0){
                if(stratagy_param_ >> 7){
                    controller_stratagy_ = EMERGENCY_BRAKING_STR;
                    update_trajectory_flag = true;
                }
                else if(stratagy_param_ >> 6){
                    controller_stratagy_ = SLOWLY_STOP_STR;
                }
                else if(stratagy_param_ >> 5){
                    controller_stratagy_ = TEST_STR;
                }
                else if(stratagy_param_ >> 4){
                    if(out_stratagy == BACKWARD_TO_POSE_CMD){
                        controller_stratagy_ = BACKWARD_TO_POSE_STR;
                    }
                    if(out_stratagy == FORWARD_TO_POSE_CMD){
                        controller_stratagy_ = FORWARD_TO_POSE_STR;
                    }
                }
                else if(stratagy_param_ >> 3){
                    controller_stratagy_ = ROTATE_TO_POSE_STR;
                }
                else if(stratagy_param_ >> 2){
                    controller_stratagy_ = NORMAL_MODE_STR;
                }
                else if(stratagy_param_ >> 1){
                    controller_stratagy_ = NORMAL_MODE_STR;
                }
                else{
                    controller_stratagy_ = NORMAL_MODE_STR;
                }
            }
            else{
                controller_stratagy_ = NO_MOTION_STR;
            }
            std::cout << "Stratagy: " << array_controller_stratagy_[controller_stratagy_] << std::endl;
            /*--------------------------------pick up target trajectory-----------------------------------------*/

            switch(controller_stratagy_){
                case NORMAL_MODE_STR:
                    pick_followed_trajectory_ = LOCAL_TRAJ;
                    pick_control_algorithm_ = MPC_ALGO;
                    break;
                case EMERGENCY_BRAKING_STR:
                    pick_followed_trajectory_ = EMERG_BRAKE_TRAJ;
                    pick_control_algorithm_ = ABS_ALGO;
                    break;
                case SLOWLY_STOP_STR:
                    pick_followed_trajectory_ = LOCAL_TRAJ;
                    pick_control_algorithm_ = MPC_ALGO;
                    break;
                case TEST_STR:
                    pick_followed_trajectory_ = TEST_TRAJ;
                    pick_control_algorithm_ = MPC_ALGO;
                    break;
                case FORWARD_TO_POSE_STR:
                    pick_followed_trajectory_ = FORWARD_TO_GOAL_TRAJ;
                    pick_control_algorithm_ = MPC_ALGO;
                    break;
                case BACKWARD_TO_POSE_STR:
                    pick_followed_trajectory_ = BACKWARD_TO_GOAL_TRAJ;
                    pick_control_algorithm_ = MPC_ALGO;
                    break;
                case ROTATE_TO_POSE_STR:
                    pick_followed_trajectory_ = ROTATE_TO_GOAL_TRAJ;
                    pick_control_algorithm_ = LQR_ALGO;
                    break;
                case NO_MOTION_STR:
                    pick_followed_trajectory_ = NO_TRAJ;
                    pick_control_algorithm_ = NO_ALGO;
                    break;
                default:
                    std::cout << "No Stratagy Find." << std::endl;
                    pick_followed_trajectory_ = NO_TRAJ;
                    pick_control_algorithm_ = NO_ALGO;
                    break;
            }

            if(update_trajectory_flag){
                target_traj_.clear();
                switch(pick_followed_trajectory_){
                    case LOCAL_TRAJ:
                    {
                        target_traj_ = local_optimal_trajectory_;
                        update_trajectory_flag = true;
                        break;
                    }
                    case EMERG_BRAKE_TRAJ:
                    {
                        mpc_controller_.planBrakeTrajectory(target_traj_, robot_state[allsubscriber::StateMemberVx]);
                        update_trajectory_flag = false;
                        break;
                    }
                    case TEST_TRAJ:
                    {
                        if(out_stratagy == LINE_MOVE_CMD){
                            mpc_controller_.planLinearTrajectory(target_traj_, robot_pose_, robot_twist_, out_strdata.rsv_3());
                        }
                        else if(out_stratagy == ROTATE_MOVE_CMD){
                            mpc_controller_.planRotateTrajectory(target_traj_, robot_pose_, robot_twist_, out_strdata.rsv_2());
                        }
                        else if(out_stratagy == CIRCLE_MOVE_CMD){
                            
                        }
                        update_trajectory_flag = false;
                        break;
                    }
                    case FORWARD_TO_GOAL_TRAJ:
                    {
                        geometry_msgs::Pose2D test_goal_pose;
                        test_goal_pose.set_x(get_outStratagyInfo().data.rsv_0());
                        test_goal_pose.set_y(get_outStratagyInfo().data.rsv_1());
                        test_goal_pose.set_theta(0.0);
                        mpc_controller_.planLinearToPoseTrajectory(target_traj_, robot_pose_, robot_twist_, test_goal_pose);
                        update_trajectory_flag = false;
                        break;
                    }
                    case BACKWARD_TO_GOAL_TRAJ:
                    {

                        break;
                    }
                    case ROTATE_TO_GOAL_TRAJ:
                    {
                        mpc_controller_.planRotateToPoseTrajectory(target_traj_, robot_pose_, robot_twist_, out_strdata.rsv_2());
                        update_trajectory_flag = false;
                        break;
                    }
                    default:
                    {
                        update_trajectory_flag = true;
                        std::cout << "No stratagy generate, no trajectory picked." << std::endl;
                        break;
                    }
                }
            }

            /*--------------------------------controller state transform-----------------------------------------*/
            double t_now = common::toSec(this->now());
            
            if(target_traj_.empty()){
                controller_run_status_ = CONTROL_FREE;
            }
            else{
                if(target_traj_.size() <= 1){
                    if(controller_run_status_ == CONTROL_FREE){
                        std::cout << "There is no target trajectory with controller_stratagy_: " << array_controller_stratagy_[controller_stratagy_] << std::endl;
                    }
                    else{
                        controller_run_status_ = CONTROL_END;
                    }
                }
                else{
                    last_yaw_follow_error = yaw_follow_error;
                    time_follow_error = target_traj_.back().t - t_now;
                    path_follow_error = hypot(target_traj_.back().path_point.x - robot_pose_.x(), target_traj_.back().path_point.y - robot_pose_.y());
                    yaw_follow_error = fabs(common::shortest_angular_distance(robot_state[allsubscriber::StateMemberYaw], target_traj_.back().path_point.yaw));

                    if(controller_stratagy_ == FORWARD_TO_POSE_STR || controller_stratagy_ == BACKWARD_TO_POSE_STR){
                        if(time_follow_error > 1.0){ // 轨迹末端时间距离当前时间较远，继续跟踪
                            controller_run_status_ = CONTROL_BUSY;
                        }
                        else if(path_follow_error < 0.08){ // 接近末端时间且距离接近
                            if(controller_run_status_ == CONTROL_FREE){
                                std::cout << "There is no effective target traj, which need the longer distance to plan! With: " << array_controller_stratagy_[controller_stratagy_] << " Stratagy" << std::endl;
                            }
                            else{
                                controller_run_status_ = CONTROL_END;
                                std::cout << "Finish Straight Trajectory Move:\n The last distance error is:" << path_follow_error << std::endl;
                            }
                        }
                        else{ // 接近轨迹终端时间，但是距离较远，主动延长终端轨迹点时间
                            controller_run_status_ = CONTROL_BUSY;
                            target_traj_.back().setT(t_now + 5 * t_delta_max_);
                        }
                    }
                    else if(controller_stratagy_ == SLOWLY_STOP_STR || controller_stratagy_ == EMERGENCY_BRAKING_STR){
                        if(fabs(robot_state[allsubscriber::StateMemberVx]) < 0.01 && fabs(robot_state[allsubscriber::StateMemberGz]) < 0.01){
                            controller_run_status_ = CONTROL_END;
                        }
                        else{
                            controller_run_status_  = CONTROL_BUSY;
                            target_traj_.back().setT(t_now + 5 * t_delta_max_);
                        }
                    }
                    else if(controller_stratagy_ == ROTATE_TO_POSE_STR){
                        if(t_now <= target_traj_.back().t){
                            controller_run_status_ = CONTROL_BUSY;
                        }
                        else{
                            if(yaw_follow_error - last_yaw_follow_error > -0.001){
                                if(controller_run_status_ == CONTROL_FREE){
                                    std::cout << "There is no effective target traj, which need the longer distance to plan! With: " << array_controller_stratagy_[controller_stratagy_] << " Stratagy" << std::endl;
                                }
                                else{
                                    controller_run_status_ = CONTROL_END;
                                    std::cout << "Finish Rotate Trajectory:\n The last yaw error is:" << yaw_follow_error << std::endl;
                                }
                            }
                            else{
                                controller_run_status_ = CONTROL_BUSY;
                                target_traj_.back().setT(t_now + 5 * t_delta_max_);
                            }
                        }
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
            std::cout << "controller_run_status_: " << array_controller_status_[controller_run_status_] << std::endl;
            cmd_vel_v = 0.0;
            cmd_vel_w = 0.0;
            if(controller_run_status_ == CONTROL_FREE){
                std::cout << "CONTROL FREE: No Trajectory Generated" << std::endl;
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
                    // 停车不清除外部策略
                }
                else{
                    controller_run_status_ = CONTROL_FREE;
                    set_outstratagy(ControllerMode::UNDEFINED_CMD);
                    controller_stratagy_ = NO_MOTION_STR;
                    stratagy_param_ = 0x00;
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
                if(pick_control_algorithm_ == MPC_ALGO){
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
                else if(pick_control_algorithm_ == LQR_ALGO){
                    cmd_vel_v = 0.0;
                    cmd_vel_w = target_traj_point_.w;
                }
                else if(pick_control_algorithm_ == ABS_ALGO){
                    cmd_vel_v = (cmd_vel_v / target_traj_point_.v < 0.7) ? target_traj_point_.v : 0;
                    cmd_vel_w = 0; // Do Not change yaw when braking;
                }
                else if(pick_control_algorithm_ == NO_ALGO){
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

    std::string RosController::zmq_server_callback(const std::string& request){
        std::cout << "Received request: " << request << std::endl;

        std::string response;
        std::shared_ptr<pnc_msgs::MotionOutSignal> mode_ptr = std::make_shared<pnc_msgs::MotionOutSignal>();
        if(mode_ptr->ParseFromArray(request.data(), request.size())){
            switch(mode_ptr->id()){
                case 0:
                    set_outstratagy(ControllerMode::SLOWLY_STOP_CMD);
                    break;
                case 1:
                    set_outstratagy(ControllerMode::RE_START_CMD);
                    break;
                case 2:
                    set_outstratagy(ControllerMode::LINE_MOVE_CMD);
                    break;
                case 3:
                    set_outstratagy(ControllerMode::ROTATE_MOVE_CMD);
                    break;
                case 4:
                    set_outstratagy(ControllerMode::CIRCLE_MOVE_CMD);
                    break;
                case 5:
                    set_outstratagy(ControllerMode::FORWARD_TO_POSE_CMD);
                    break;
                case 6:
                    set_outstratagy(ControllerMode::BACKWARD_TO_POSE_CMD);
                    break;
                case 7:
                    set_outstratagy(ControllerMode::ROTATE_TO_POSE_CMD);
                    break;
                case 8:
                    set_outstratagy(ControllerMode::UNDEFINED_CMD);
                    break;
                default:
                    std::cout << "Unknown command id: " << mode_ptr->id() << std::endl;
                    break;
            }
        }
        response = mode_ptr->name();
        std::cout << "Sending response: " << response << std::endl;
        return response;
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
