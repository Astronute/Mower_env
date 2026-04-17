#include "zmq_controller.h"
#include "wall_rate.h"

#include <iostream>
#include <chrono>

namespace CB {
    using namespace std::chrono_literals;

    RosController::RosController(): 
        sensor_timeout_(0.1),
        local_path_timeout_(1.0),
        sensor_dead_(3.0),
        last_measurement_time_(std::chrono::seconds(0)),
        controller_run_status_(CONTROL_FREE),
        controller_run_mode_(UNDEFINED_CMD),
        running_(true)
    {

    }

    RosController::~RosController(){
        // topic_subs_.clear();
        running_ = false;
        timer_.stop();
        cv_.notify_one();
        while(!measurement_queue_.empty()){
            measurement_queue_.pop();
        }
        trajectory_queue_.clear();
    }

    void RosController::reset(){
        controller_run_status_ = ControllerRunStatus::CONTROL_FREE;
        controller_run_mode_ = ControllerMode::UNDEFINED_CMD;

        std::array<double, STATE_SIZE> state_vec;
        state_vec.fill(0.0);
        setState(state_vec);

        sensor_delayed_.store(2);
        local_path_delayed_.store(false);

        trajectory_queue_.clear();

        last_message_times_.clear();
    }

    void RosController::spin(){
        std::unique_lock<std::mutex> lock(mtx_);
        cv_.wait(lock, [this]() { return !running_; });
    }

    bool RosController::initialize(){

        reset();

        if(!mpc_controller_.initialize()){
            return false;
        }

        if(mpc_controller_.is_initialized()){
            control_frequency_ = mpc_controller_.get_control_rate();
            std::cout << "control_frequency set to " << control_frequency_ << std::endl;
        }

        if(!loadParams()){
            return false;
        }

        
        // 定时器线程，处理传感器
        const std::chrono::duration<double> timespan{1.0 / sample_frequency_};
        timer_.start(std::chrono::duration_cast<std::chrono::nanoseconds>(timespan), std::bind(&RosController::periodicUpdate, this));
        
        // 控制线程
        std::thread control_thread(&RosController::periodicControl, this);
        control_thread.detach();

        return true;

    }

    bool RosController::loadParams(){
        try{
            #ifdef USE_SIM
            filter_config_yaml_ = YAML::LoadFile("/home/yat/Mower_env/src/motion_control/params/filter_params.yaml");
            #else
            filter_config_yaml_ = YAML::LoadFile("/home/kickpi/sim_ws/src/motion_control/params/filter_params.yaml");
            #endif
        } catch(const YAML::Exception& e){
            std::cout << "yaml parsing error: " << e.what() << std::endl;
            return false;
        }

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
        if(filter_config_yaml_["sample_frequency"]){
            sample_frequency_ = filter_config_yaml_["sample_frequency"].as<double>();
        }
        else{
            std::cout << "missing param 'sample_frequency' " << std::endl;
            return false;
        }

        if(!mpc_controller_.is_initialized()){
            control_frequency_ = filter_config_yaml_["control_frequency"].as<double>();
            std::cout << "control_frequency set to " << control_frequency_ << std::endl;
        }

        if(filter_config_yaml_["sensor_timeout"]){
            sensor_timeout_ = filter_config_yaml_["sensor_timeout"].as<double>();
        }
        else{
            std::cout << "missing param 'sensor_timeout'" << std::endl;
            return false;
        }
        if(filter_config_yaml_["local_path_timeout"]){
            local_path_timeout_ = filter_config_yaml_["local_path_timeout"].as<double>();
        }
        else{
            std::cout << "missing param 'local_path_timeout'" << std::endl;
            return false;
        }
        if(filter_config_yaml_["sensor_dead"]){
            sensor_dead_ = filter_config_yaml_["sensor_dead"].as<double>();
        }
        else{
            std::cout << "missing param 'sensor_dead'" << std::endl;
            return false;
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

        std::cout << "now: " << toSec(this->now()) << std::endl;

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

        /*-------------------------------- zmq subscribe-----------------------------------------*/
        std::vector<SubscriberConfig> zmq_sub_cfgs;
        SubscriberConfig sub_cfg;
        if(filter_config_yaml_["zmq_sub_port"]){
            std::string port = filter_config_yaml_["zmq_sub_port"].as<std::string>();
            sub_cfg.address = port;
            std::cout << "controller node subscriber bind to: " << port << std::endl;
        }
        else{
            std::cout << "missing param 'zmq_sub_port'" << std::endl;
            return false;
        }

        // odom subscribe
        size_t topic_ind = 0;
        bool more_params = false;
        do{
            std::stringstream ss;
            ss << "odom" << topic_ind++;
            std::string odom_name = ss.str();
            std::string odom_topic;
            if(filter_config_yaml_[odom_name]){
                more_params = true;
                odom_topic = filter_config_yaml_[odom_name].as<std::string>();
            }
            else{
                more_params = false;
            }

            if(more_params){
                double pose_mahalanobis_threshold = filter_config_yaml_[odom_name + "_pose_rejection_threshold"].as<double>();
                double twist_mahalanobis_threshold = filter_config_yaml_[odom_name + "_twist_rejection_threshold"].as<double>();

                std::vector<bool> update_vec = loadUpdateConfig(odom_name);
                std::vector<bool> pose_update_vec = update_vec;
                std::fill(pose_update_vec.begin() + POSITION_V_OFFSET, pose_update_vec.begin() + POSITION_V_OFFSET + TWIST_SIZE, 0);
                std::vector<bool> twist_update_vec = update_vec;
                std::fill(twist_update_vec.begin() + POSITION_OFFSET, twist_update_vec.begin() + POSITION_OFFSET + POSE_SIZE, 0);
                int pose_update_num = std::accumulate(pose_update_vec.begin(), pose_update_vec.end(), 0);
                int twist_update_num = std::accumulate(twist_update_vec.begin(), twist_update_vec.end(), 0);
                std::cout << pose_update_num << " , " << std::endl;

                if(pose_update_num + twist_update_num > 0){
                    const CallBackInfo pose_callback_info(odom_name + "_pose", pose_update_vec, pose_update_num, pose_mahalanobis_threshold);
                    const CallBackInfo twist_callback_info(odom_name + "_twist", twist_update_vec, twist_update_num, twist_mahalanobis_threshold);
                    topic_callbackinfo_map_[odom_name + "_pose"] = pose_callback_info;
                    topic_callbackinfo_map_[odom_name + "_twist"] = twist_callback_info;
                    topic_name_map_[odom_topic] = odom_name;

                    sub_cfg.topics.push_back(odom_topic);
                }
                else{
                    std::cout << odom_topic << " all update variables are false " << std::endl;
                }
            }

        }while(more_params);

        // pose subscribe
        topic_ind = 0;
        more_params = false;
        do{
            std::stringstream ss;
            ss << "pose" << topic_ind++;
            std::string pose_name = ss.str();
            std::string pose_topic;
            if(filter_config_yaml_[pose_name]){
                more_params = true;
            }
            else{
                more_params = false;
            }

            if(more_params){
                pose_topic = filter_config_yaml_[pose_name].as<std::string>();
                double pose_rejection_threshold = filter_config_yaml_[pose_name + "_rejection_threshold"].as<double>();
                std::vector<bool> pose_update_mask = loadUpdateConfig(pose_name);

                std::fill(pose_update_mask.begin() + POSITION_V_OFFSET, pose_update_mask.begin() + POSITION_V_OFFSET + TWIST_SIZE, 0);
                std::fill(pose_update_mask.begin() + POSITION_A_OFFSET, pose_update_mask.begin() + POSITION_A_OFFSET + ACCELERATION_SIZE, 0);

                int pose_update_sum = std::accumulate(pose_update_mask.begin(), pose_update_mask.end(), 0);
                if(pose_update_sum > 0){
                    const CallBackInfo callback_info(pose_name, pose_update_mask, pose_update_sum, pose_rejection_threshold);
                    topic_callbackinfo_map_[pose_name] = callback_info;
                    topic_name_map_[pose_topic] = pose_name;

                    sub_cfg.topics.push_back(pose_topic);
                }
            }
            else{
                std::cout << pose_topic << " all update variables are false " << std::endl;
            }

        }while(more_params);

        // twist subscribe
        topic_ind = 0;
        more_params = false;
        do{
            std::stringstream ss;
            ss << "twist" << topic_ind++;
            std::string twist_name = ss.str();
            std::string twist_topic;
            if(filter_config_yaml_[twist_name]){
                more_params = true;
                twist_topic = filter_config_yaml_[twist_name].as<std::string>();
            }
            else{
                more_params = false;
            }

            if(more_params){
                double twist_mahalanobis_threshold = filter_config_yaml_[twist_name + "_twist_rejection_threshold"].as<double>();
                std::vector<bool> update_vec = loadUpdateConfig(twist_name);
                std::fill(update_vec.begin() + POSITION_OFFSET, update_vec.begin() + POSITION_OFFSET + POSE_SIZE, 0);
                int twist_update_sum = std::accumulate(update_vec.begin(), update_vec.end(), 0);

                if(twist_update_sum > 0){
                    const CallBackInfo callback_info(twist_topic, update_vec, twist_update_sum, twist_mahalanobis_threshold);
                    topic_callbackinfo_map_[twist_name] = callback_info;
                    topic_name_map_[twist_topic] = twist_name;

                    sub_cfg.topics.push_back(twist_topic);
                }
                else{
                    std::cout << twist_topic << " all update variables are false " << std::endl;
                }
            }

        }while(more_params);

        // local path subscribe
        topic_ind = 0;
        more_params = false;
        do{
            std::stringstream ss;
            ss << "localpath" << topic_ind++;
            std::string localpath_name = ss.str();
            std::string localpath_topic;
            if(filter_config_yaml_[localpath_name]){
                more_params = true;
                localpath_topic = filter_config_yaml_[localpath_name].as<std::string>();
            }
            else{
                more_params = false;
            }

            if(more_params){
                topic_name_map_[localpath_topic] = localpath_name;

                sub_cfg.topics.push_back(localpath_topic);
            }

        }while(more_params);


        // mpc parameters load
        topic_ind = 0;
        more_params = false;
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
                std::vector<double> q, r, f;
                loadParamMatrix(controller_name, q, r, f);
                matrix_params_q_[param_name] = q;
                matrix_params_r_[param_name] = r;
                std::cout << param_name << std::endl;
                std::cout << "Q" << std::endl;
                printVector(q);
                std::cout << "R" << std::endl;
                printVector(r);
            }

        }while(more_params);

        zmq_sub_cfgs.push_back(sub_cfg);
        zmq_subscriber_.initialize(zmq_sub_cfgs);
        zmq_subscriber_.setMessageCallback([this](const std::string& msg, const std::string& topic) {
            this->zmq_message_callback(msg, topic);}
        );
        zmq_subscriber_.start();
        std::cout << "param load success." << std::endl;
        return true;
    }

    void RosController::odometryCallback(
        const std::shared_ptr<nav_msgs::Odometry> & msg,
        const std::string & topic_name,
        const CallBackInfo & pose_callback_info,
        const CallBackInfo & twist_callback_info
    ){
        std::shared_ptr<nav_msgs::Odometry> odom_ptr = std::make_shared<nav_msgs::Odometry>();

        // 处理callback中的pose类数据（xyz，rpy）
        std::shared_ptr<geometry_msgs::PoseWithCovarianceStamped> pos_ptr = std::make_shared<geometry_msgs::PoseWithCovarianceStamped>();
        if(pose_callback_info.update_sum_ > 0){
            pos_ptr->mutable_header()->CopyFrom(msg->header());
            pos_ptr->mutable_pose()->CopyFrom(msg->pose());
            // std::cout << "odometryCallback " << pos_ptr->pose().pose().position().x() << " - " << pos_ptr->pose().pose().position().y() << std::endl;

            poseCallback(pos_ptr, pose_callback_info, world_frame_id_, base_link_frame_id_, false);
        }

        std::shared_ptr<geometry_msgs::TwistWithCovarianceStamped> twist_ptr = std::make_shared<geometry_msgs::TwistWithCovarianceStamped>();
        if(twist_callback_info.update_sum_ > 0){
            twist_ptr->mutable_header()->CopyFrom(msg->header());
            twist_ptr->mutable_twist()->CopyFrom(msg->twist());
            twist_ptr->mutable_header()->set_frame_id(msg->child_frame_id());
            std::cout << "odometryCallback" << twist_ptr->twist().twist().linear().x() << " - " << twist_ptr->twist().twist().angular().z() << std::endl;

            twistCallback(twist_ptr, twist_callback_info, base_link_frame_id_);
        }
    }

    void RosController::poseCallback(
        const std::shared_ptr<geometry_msgs::PoseWithCovarianceStamped> & msg,
        const CallBackInfo & callback_info,
        const std::string & target_frame,
        const std::string & source_frame,
        const bool imu_data
    ){
        const std::string topic_name = callback_info.topic_name_;
        SysTimePoint msg_sys_time = protoToSystime(msg->header().stamp());

        if(last_message_times_.count(topic_name) == 0){
            last_message_times_.insert(std::pair<std::string, SysTimePoint>(topic_name, this->now()));
        }

        if(last_message_times_[topic_name] <= msg_sys_time){
            Eigen::VectorXd measurement(STATE_SIZE);
            Eigen::MatrixXd measurement_covariance(STATE_SIZE, STATE_SIZE);
            measurement.setZero();
            measurement_covariance.setZero();
            Eigen::MatrixXd covariance_rotated(POSE_SIZE, POSE_SIZE);
            covariance_rotated.setZero();
            std::vector<bool> update_vector = callback_info.update_mask_;

            measurement(StateMemberX) = msg->pose().pose().position().x();//msg->pose.pose.position.x;
            measurement(StateMemberY) = msg->pose().pose().position().y();
            measurement(StateMemberZ) = msg->pose().pose().position().z();
            double roll, pitch, yaw;
            quatToRPY(msg->pose().pose().orientation(), roll, pitch, yaw);
            measurement(StateMemberRoll) = roll;
            measurement(StateMemberPitch) = pitch;
            measurement(StateMemberYaw) = yaw;
            measurement_covariance.block(0, 0, POSE_SIZE, POSE_SIZE) = covariance_rotated.block(0, 0, POSE_SIZE, POSE_SIZE);

            copyCovariance(msg->pose().covariance().data(), covariance_rotated, topic_name, update_vector, POSITION_OFFSET, POSE_SIZE);

            pushQueueMeasurement(topic_name, 
                measurement, 
                measurement_covariance, 
                update_vector, 
                callback_info.rejection_threshold_, 
                msg_sys_time
            );

            last_message_times_[topic_name] = protoToSystime(msg->header().stamp());
            double last_update_delta = toSec(this->now() - msg_sys_time);
            if(last_update_delta > sensor_dead_){
                sensor_delayed_.store(2);
                std::cout << topic_name << " time out: " << last_update_delta << std::endl;
            }
            else if(last_update_delta > sensor_timeout_){
                sensor_delayed_.store(1);
                std::cout << topic_name << " time out: " << last_update_delta << std::endl;
            }
            else{
                sensor_delayed_.store(0);
            }
            
        }
        else{
            std::cout << "Message is too old. Last message time for " << topic_name << " is " <<
                toSec(last_message_times_[topic_name]) << ", current message time is " << toSec(msg_sys_time) << std::endl;
        }
    }

    void RosController::twistCallback(
        const std::shared_ptr<geometry_msgs::TwistWithCovarianceStamped> & msg,
        const CallBackInfo & callback_info,
        const std::string & target_frame
    ){
        const std::string & topic_name = callback_info.topic_name_;
        SysTimePoint msg_sys_time = protoToSystime(msg->header().stamp());

        if(last_message_times_.count(topic_name) == 0){
            last_message_times_.insert(std::pair<std::string, SysTimePoint>(topic_name, msg_sys_time));
        }

        if(last_message_times_[topic_name] <= msg_sys_time){
            Eigen::VectorXd measurement(STATE_SIZE);
            Eigen::MatrixXd measurement_covariance(STATE_SIZE, STATE_SIZE);
            measurement.setZero();
            measurement_covariance.setZero();

            std::vector<bool> update_vector = callback_info.update_mask_;
            measurement(StateMemberVx) = msg->twist().twist().linear().x();
            measurement(StateMemberVy) = msg->twist().twist().linear().y();
            measurement(StateMemberVz) = msg->twist().twist().linear().z();
            measurement(StateMemberGx) = msg->twist().twist().angular().x();
            measurement(StateMemberGy) = msg->twist().twist().angular().y();
            measurement(StateMemberGz) = msg->twist().twist().angular().z();

            Eigen::MatrixXd covariance_rotated(TWIST_SIZE, TWIST_SIZE);
            covariance_rotated.setZero();
            copyCovariance(msg->twist().covariance().data(), covariance_rotated, topic_name, update_vector, POSITION_V_OFFSET, TWIST_SIZE);
            measurement_covariance.block(POSITION_V_OFFSET, POSITION_V_OFFSET, TWIST_SIZE, TWIST_SIZE) = covariance_rotated.block(0, 0, TWIST_SIZE, TWIST_SIZE);

            pushQueueMeasurement(
                callback_info.topic_name_, 
                measurement, 
                measurement_covariance, 
                update_vector, 
                callback_info.rejection_threshold_, 
                msg_sys_time
            );

            last_message_times_[topic_name] = msg_sys_time;
            double last_update_delta = toSec(this->now() - msg_sys_time);
            if(last_update_delta > sensor_timeout_){
                sensor_delayed_.store(1);
                std::cout << topic_name << " time out: " << last_update_delta << std::endl;
            }
            else{
                sensor_delayed_.store(0);
            }
        }
        else{
            std::cout << "Message is too old. Last message time for " << topic_name << " is " <<
                toSec(last_message_times_[topic_name]) << ", current message time is " << toSec(msg_sys_time) << std::endl;
        }
    }
    
    void RosController::trajectoryCallback(
        const std::shared_ptr<pnc_msgs::PlanningTrajectory> & msg,
        const std::string & topic_name,
        const std::string & target_frame
    ){
        SysTimePoint msg_sys_time = protoToSystime(msg->header().stamp());

        if(msg->trajectory_size() < 1){
            std::cout << "topic_name: " << topic_name << " get loaclTraj empty." << std::endl;
            return ;
        }

        if(last_message_times_.count(topic_name) == 0){
            last_message_times_.insert(std::pair<std::string, SysTimePoint>(topic_name, msg_sys_time));
        }

        if(last_message_times_[topic_name] <= msg_sys_time){
            std::vector<TrajPoint> local_optimal_trajectory;
            TrajPoint traj_point;
            for(int i=0; i<msg->trajectory_size(); ++i){
                const pnc_msgs::PointVec8f& point = msg->trajectory(i);
                traj_point.path_point.x = point.x();
                traj_point.path_point.y = point.y();
                traj_point.path_point.yaw = point.yaw();
                traj_point.path_point.kappa = point.kappa();
                traj_point.path_point.s = point.s();
                traj_point.t = point.t();
                traj_point.v = point.v();
                traj_point.w = point.w();
                local_optimal_trajectory.push_back(traj_point);
            }

            trajectory_queue_.push(local_optimal_trajectory);

            last_message_times_[topic_name] = msg_sys_time;
            double last_update_delta = toSec(this->now() - msg_sys_time);
            if(last_update_delta > local_path_timeout_){
                local_path_delayed_.store(1);
                std::cout << topic_name << " time out: " << last_update_delta << std::endl;
            }
            else{
                local_path_delayed_.store(0);
            }

        }
        else{
            std::cout << "Message is too old. Last message time for " << topic_name << " is " <<
                toSec(last_message_times_[topic_name]) << ", current message time is " << toSec(msg_sys_time) << std::endl;
        }

    }

    void RosController::zmq_message_callback(const std::string& message, const std::string& topic){
        std::string topic_name = topic_name_map_[topic];
        
        if(topic_name.compare(0, 4, "odom") == 0){
            CallBackInfo call_backinfo_pose = topic_callbackinfo_map_[topic_name + "_pose"];
            CallBackInfo call_backinfo_twist = topic_callbackinfo_map_[topic_name + "_twist"];
            std::shared_ptr<nav_msgs::Odometry> odom_ptr = std::make_shared<nav_msgs::Odometry>();
            if(odom_ptr->ParseFromArray(message.data(), message.size())){
                odometryCallback(odom_ptr, topic_name, call_backinfo_pose, call_backinfo_twist);
            }
            else{
                std::cout << topic_name << " process failed" << std::endl;
            }
        }
        else if(topic_name.compare(0, 4, "pose") == 0){
            CallBackInfo call_backinfo = topic_callbackinfo_map_[topic_name];
            std::shared_ptr<geometry_msgs::PoseWithCovarianceStamped> pose_ptr = std::make_shared<geometry_msgs::PoseWithCovarianceStamped>();
            if(pose_ptr->ParseFromArray(message.data(), message.size())){
                poseCallback(pose_ptr, call_backinfo, world_frame_id_, base_link_frame_id_, false);
            }
            else{
                std::cout << topic_name << " process failed" << std::endl;
            }
        }
        else if(topic_name.compare(0, 5, "twist") == 0){
            CallBackInfo call_backinfo = topic_callbackinfo_map_[topic_name];
            std::shared_ptr<geometry_msgs::TwistWithCovarianceStamped> twist_ptr = std::make_shared<geometry_msgs::TwistWithCovarianceStamped>();
            if(twist_ptr->ParseFromArray(message.data(), message.size())){
                twistCallback(twist_ptr, call_backinfo, base_link_frame_id_);
            }
            else{
                std::cout << topic_name << " process failed" << std::endl;
            }
        }
        else if(topic_name.compare(0, 5, "local") == 0){
            std::shared_ptr<pnc_msgs::PlanningTrajectory> traj_ptr = std::make_shared<pnc_msgs::PlanningTrajectory>();
            if(traj_ptr->ParseFromArray(message.data(), message.size())){
                trajectoryCallback(traj_ptr, topic_name, base_link_frame_id_);
            }
            else{
                std::cout << topic_name << " process failed" << std::endl;
            }
        }

    }

    void RosController::periodicUpdate(){
        // std::cout << " measurement process running " << std::endl;

        SysTimePoint current_time = this->now();

        if(!measurement_queue_.empty()){
            while(!measurement_queue_.empty()){
                MeasurementPtr measurement = measurement_queue_.front();
                if(current_time < measurement->time_){
                    break;
                }
                measurement_queue_.pop();
                size_t measurement_length = measurement->update_mask_.size();
                std::array<double, STATE_SIZE> state_vec = getState();
                for(size_t i=0; i< measurement_length; ++i){
                    state_vec[i] = measurement->update_mask_[i] ? measurement->measurement_[i] : state_vec[i];
                }
                setState(state_vec);
                last_measurement_time_ = measurement->time_;
            }
        }
        else{
            double last_update_delta = toSec(current_time - last_measurement_time_);
            if(last_update_delta >= sensor_dead_){
                std::cout << "all sensor dead" << std::endl;
                sensor_delayed_.store(2);
            }
        }

    }

    void RosController::periodicControl(){

        std::array<double, STATE_SIZE> robot_state;
        controller_run_status_ = CONTROL_FREE;
        stratagy_status_ = 0x00;
        double control_duration = 0;
        bool update_trajectory_flag = true;
        int motor_timeout = 0;
        double cmd_vel_v = 0.0;
        double cmd_vel_w = 0.0;
        WallRate rate_control_(control_frequency_);

        while(true){

            clock_t time_begin_cpu = clock();

            last_stratagy_status_ = stratagy_status_;

            /*--------------------------------get robot state-----------------------------------------*/

            // check sensor data
            int sensor_delayed = sensor_delayed_.load();
            bool local_path_delayed = local_path_delayed_.load();
            
            last_cmd_twist_ = cmd_twist_;
            SysTimePoint ctrl_sys_now = this->now();
            cmd_twist_.mutable_header()->mutable_stamp()->CopyFrom(systimeToProto(ctrl_sys_now));
            // std::cout << "sensor_delayed: " << sensor_delayed << std::endl;
            if(sensor_delayed || local_path_delayed){
                if(sensor_delayed > 1){
                    std::cout << "localization dead, stop" << std::endl;
                    stratagy_status_ |= 0x80;
                }
                else{
                    std::cout << "local path or odom or twist delayed, slowly stop" << std::endl;
                    robot_state = getState();
                    cmd_twist_.mutable_twist()->mutable_linear()->set_x(robot_state[StateMemberVx] - 0.1);
                    if(cmd_twist_.twist().linear().x() < 0){
                        cmd_twist_.mutable_twist()->mutable_linear()->set_x(0.0);
                    }
                    cmd_twist_.mutable_twist()->mutable_angular()->set_z(0.0);
                    std::string serialized_data;
                    cmd_twist_.SerializeToString(&serialized_data);
                    zmq_publisher_.publishMessage("cmd_vel", serialized_data);

                    stratagy_status_ |= 0x40;

                    rate_control_.sleep();
                    continue;
                }
            }
            else{ // normal situation
                // robot state
                robot_state = getState();
                std::cout << "robot_x: " << robot_state[StateMemberX] 
                    << " , robot_y: " << robot_state[StateMemberY] 
                    << " , robot_yaw: " << robot_state[StateMemberYaw] 
                    << " , robot_v: " << robot_state[StateMemberVx] 
                    << " , robot_w: " << robot_state[StateMemberGz] 
                    << std::endl;
                robot_pose_.set_x(robot_state[StateMemberX]);
                robot_pose_.set_y(robot_state[StateMemberY]);
                robot_pose_.set_theta(robot_state[StateMemberYaw]);
                robot_twist_.mutable_linear()->set_x(robot_state[StateMemberVx]);
                robot_twist_.mutable_angular()->set_z(robot_state[StateMemberGz]);

                // local trajectory
                std::vector<TrajPoint>().swap(local_optimal_trajectory_);
                if(!trajectory_queue_.empty()){
                    local_optimal_trajectory_ = trajectory_queue_.back();
                }
                
                if(local_optimal_trajectory_.size() <= 1){
                    std::cout<<"get localpath empty"<<std::endl;
                    cmd_twist_.mutable_twist()->mutable_linear()->set_x(0.0);
                    cmd_twist_.mutable_twist()->mutable_linear()->set_y(0.0);
                    cmd_twist_.mutable_twist()->mutable_angular()->set_z(0.0);
                    std::string serialized_data;
                    cmd_twist_.SerializeToString(&serialized_data);
                    zmq_publisher_.publishMessage("cmd_vel", serialized_data);
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
                    if(std::fabs(robot_state[StateMemberVx]) < 0.02 && fabs(robot_state[StateMemberGz]) < 0.02){
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
                    if(std::fabs(robot_state[StateMemberVx]) < 0.02 && fabs(robot_state[StateMemberGz]) < 0.02){
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

            double t_now = toSec(this->now());
            
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

                    if(std::fabs(robot_state[StateMemberVx]) < 0.005 && std::fabs(robot_state[StateMemberGz]) < 0.005 && controller_run_status_ == CONTROL_BUSY){
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
                double t_delta_min = (robot_state[StateMemberVx] <= 1.01) ? t_delta_max_ : t_delta_min_; //  seconds;
                double t_delta = t_delta_min;
                target_t_ = toSec(this->now()) + t_delta;
                mpc_controller_.findTargetTrajTimepoint(target_traj_point_, target_traj_, target_t_);
                target_t_ = toSec(this->now()) + t_delta_min_;
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
                zmq_publisher_.publishMessage("cmd_vel", serialized_data);
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

    std::array<double, STATE_SIZE> RosController::getState() const{
        return state_.load();
    }

    void RosController::setState(const std::array<double, STATE_SIZE> & state){
        state_.store(state);
    }

    void RosController::pushQueueMeasurement(
        const std::string & topic_name, 
        const Eigen::MatrixXd & measurement,
        const Eigen::MatrixXd & covariance,
        const std::vector<bool> & update_mask,
        const double mahalanobis_thresh,
        const SysTimePoint & time
    ){
        MeasurementPtr meas = std::make_shared<Measurement>();
        // std::cout << toSec(time) << std::endl;
        meas->time_ = time;
        meas->topic_name_ = topic_name;
        meas->mahalanobis_thresh_ = mahalanobis_thresh;
        meas->measurement_ = measurement;
        meas->covariance_ = covariance;
        meas->update_mask_ = update_mask;
        measurement_queue_.push(meas);
    }

    void RosController::copyCovariance(
        const double * covariance_in,
        Eigen::MatrixXd & covariance_out,
        const std::string & topic_name,
        const std::vector<bool> & update_vector,
        const size_t offset, const size_t dimension
    ){
        for(size_t i=0; i<dimension; ++i){
            for(size_t j=0; j<dimension; ++j){
                covariance_out(i, j) = covariance_in[dimension * i + j];

            }
        }
    }

    std::vector<bool> RosController::loadUpdateConfig(const std::string & topic_name){
        std::vector<bool> update_vector(STATE_SIZE, 0);
        const std::string topic_config_name = topic_name + "_config";
        const YAML::Node& cfg_array = filter_config_yaml_[topic_config_name];
        if(cfg_array.IsSequence() && cfg_array.size()==STATE_SIZE){
            for(int i=0; i<STATE_SIZE; ++i){
                update_vector[i] = cfg_array[i].as<bool>();
            }
        }
        return update_vector;
    }

    bool RosController::loadParamMatrix(
        const std::string & param_name, 
        std::vector<double> & q, 
        std::vector<double> & r, 
        std::vector<double> & f
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

        config_name = param_name + "_terminal_weight";
        cfg_array = filter_config_yaml_[config_name];
        if(cfg_array.IsSequence()){
            for(int i=0; i<cfg_array.size(); ++i){
                f.push_back(cfg_array[i].as<double>());
            }
        }

        return true;
    }
}