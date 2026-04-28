#include "all_subscriber.h"

namespace allsubscriber{
    AllSubscriber::AllSubscriber():
        sensor_timeout_(0.1),
        sensor_dead_(3.0),
        last_measurement_time_(std::chrono::seconds(0)),
        running_(true)
    {
        
    }

    AllSubscriber::~AllSubscriber(){
        running_ = false;
        timer_.stop();
        cv_.notify_one();
        while(!measurement_queue_.empty()){
            measurement_queue_.pop();
        }
    }

    void AllSubscriber::reset(){
        std::array<double, STATE_SIZE> state_vec;
        state_vec.fill(0.0);
        setState(state_vec);

        sensor_delayed_.store(2);

        last_message_times_.clear();
    }

    bool AllSubscriber::initialize(){
        reset();

        if(!loadParams()){
            return false;
        }

        // 定时器线程，处理传感器
        const std::chrono::duration<double> timespan{1.0 / sample_frequency_};
        timer_.start(std::chrono::duration_cast<std::chrono::nanoseconds>(timespan), std::bind(&AllSubscriber::periodicUpdate, this));
        
        return true;
    }

    void AllSubscriber::spin(){
        std::unique_lock<std::mutex> lock(mtx_);
        cv_.wait(lock, [this]() { return !running_; });
    }

    bool AllSubscriber::loadParams(){
        try{
            filter_config_yaml_ = YAML::LoadFile("/home/rpdzkj/Mower_env/src/local_planner/params/subscribe_params.yaml");
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

        if(filter_config_yaml_["sensor_timeout"]){
            sensor_timeout_ = filter_config_yaml_["sensor_timeout"].as<double>();
        }
        else{
            std::cout << "missing param 'sensor_timeout'" << std::endl;
            return false;
        }
        if(filter_config_yaml_["sensor_dead"]){
            sensor_dead_ = filter_config_yaml_["sensor_dead"].as<double>();
        }
        else{
            std::cout << "missing param 'sensor_dead'" << std::endl;
            return false;
        }
        if(filter_config_yaml_["sample_frequency"]){
            sample_frequency_ = filter_config_yaml_["sample_frequency"].as<double>();
        }
        else{
            std::cout << "missing param 'sample_frequency' " << std::endl;
            return false;
        }

        std::cout << "now: " << toSec(this->now()) << std::endl;

        /*-------------------------------- zmq subscribe-----------------------------------------*/
        std::vector<SubscriberConfig> zmq_sub_cfgs;

        size_t port_ind = 0;
        bool more_params = false;
        do{
            std::stringstream ss;
            ss << "zmq_sub_port" << port_ind++;
            std::string port_id = ss.str();
            std::string port_addr;
            if(filter_config_yaml_[port_id]){
                more_params = true;
                port_addr = filter_config_yaml_[port_id].as<std::string>();
            }
            else{
                more_params = false;
            }

            if(more_params){
                SubscriberConfig sub_cfg;
                sub_cfg.address = port_addr;
                zmq_sub_cfgs.push_back(sub_cfg);
                std::cout << "controller node subscriber " << port_id << " bind to: " << port_addr << std::endl;
            }
        }while(more_params);

        // odom subscribe
        size_t topic_ind = 0;
        more_params = false;
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
                int port_idx = 0;
                std::string str = filter_config_yaml_[odom_name + "_port"].as<std::string>().substr(4);
                try {
                    port_idx = std::stoi(str);
                    if(port_idx > zmq_sub_cfgs.size()){
                        std::cout << odom_name << " port id out of range" << std::endl;
                        return false;
                    }
                } catch (const std::exception& e) {
                    std::cerr << str <<" parse error: " << e.what() << std::endl;
                }

                double pose_mahalanobis_threshold = filter_config_yaml_[odom_name + "_pose_rejection_threshold"].as<double>();
                double twist_mahalanobis_threshold = filter_config_yaml_[odom_name + "_twist_rejection_threshold"].as<double>();

                std::vector<bool> update_vec = loadUpdateConfig(odom_name);
                std::vector<bool> pose_update_vec = update_vec;
                std::fill(pose_update_vec.begin() + POSITION_V_OFFSET, pose_update_vec.begin() + POSITION_V_OFFSET + TWIST_SIZE, 0);
                std::vector<bool> twist_update_vec = update_vec;
                std::fill(twist_update_vec.begin() + POSITION_OFFSET, twist_update_vec.begin() + POSITION_OFFSET + POSE_SIZE, 0);
                int pose_update_num = std::accumulate(pose_update_vec.begin(), pose_update_vec.end(), 0);
                int twist_update_num = std::accumulate(twist_update_vec.begin(), twist_update_vec.end(), 0);

                if(pose_update_num + twist_update_num > 0){
                    const CallBackInfo pose_callback_info(odom_name + "_pose", pose_update_vec, pose_update_num, pose_mahalanobis_threshold);
                    const CallBackInfo twist_callback_info(odom_name + "_twist", twist_update_vec, twist_update_num, twist_mahalanobis_threshold);
                    topic_callbackinfo_map_[odom_name + "_pose"] = pose_callback_info;
                    topic_callbackinfo_map_[odom_name + "_twist"] = twist_callback_info;
                    topic_name_map_[odom_topic] = odom_name;

                    zmq_sub_cfgs[port_idx].topics.push_back(odom_topic);
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
                pose_topic = filter_config_yaml_[pose_name].as<std::string>();
            }
            else{
                more_params = false;
            }

            if(more_params){
                int port_idx = 0;
                std::string str = filter_config_yaml_[pose_name + "_port"].as<std::string>().substr(4);
                try {
                    port_idx = std::stoi(str);
                    if(port_idx > zmq_sub_cfgs.size()){
                        std::cout << pose_name << " port id out of range" << std::endl;
                        return false;
                    }
                } catch (const std::exception& e) {
                    std::cerr << str << " parse error: " << e.what() << std::endl;
                }

                double pose_rejection_threshold = filter_config_yaml_[pose_name + "_rejection_threshold"].as<double>();
                std::vector<bool> pose_update_mask = loadUpdateConfig(pose_name);
                std::fill(pose_update_mask.begin() + POSITION_V_OFFSET, pose_update_mask.begin() + POSITION_V_OFFSET + TWIST_SIZE, 0);
                std::fill(pose_update_mask.begin() + POSITION_A_OFFSET, pose_update_mask.begin() + POSITION_A_OFFSET + ACCELERATION_SIZE, 0);

                int pose_update_sum = std::accumulate(pose_update_mask.begin(), pose_update_mask.end(), 0);
                if(pose_update_sum > 0){
                    const CallBackInfo callback_info(pose_name, pose_update_mask, pose_update_sum, pose_rejection_threshold);
                    topic_callbackinfo_map_[pose_name] = callback_info;
                    topic_name_map_[pose_topic] = pose_name;

                    zmq_sub_cfgs[port_idx].topics.push_back(pose_topic);
                }
                else{
                    std::cout << pose_topic << " all update variables are false " << std::endl;
                }
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
                int port_idx = 0;
                std::string str = filter_config_yaml_[twist_name + "_port"].as<std::string>().substr(4);
                try {
                    port_idx = std::stoi(str);
                    if(port_idx > zmq_sub_cfgs.size()){
                        std::cout << twist_name << " port id out of range" << std::endl;
                        return false;
                    }
                } catch (const std::exception& e) {
                    std::cerr << str << " parse error: " << e.what() << std::endl;
                }

                double twist_mahalanobis_threshold = filter_config_yaml_[twist_name + "_twist_rejection_threshold"].as<double>();
                std::vector<bool> update_vec = loadUpdateConfig(twist_name);
                std::fill(update_vec.begin() + POSITION_OFFSET, update_vec.begin() + POSITION_OFFSET + POSE_SIZE, 0);
                int twist_update_sum = std::accumulate(update_vec.begin(), update_vec.end(), 0);

                if(twist_update_sum > 0){
                    const CallBackInfo callback_info(twist_topic, update_vec, twist_update_sum, twist_mahalanobis_threshold);
                    topic_callbackinfo_map_[twist_name] = callback_info;
                    topic_name_map_[twist_topic] = twist_name;

                    zmq_sub_cfgs[port_idx].topics.push_back(twist_topic);
                }
                else{
                    std::cout << twist_topic << " all update variables are false " << std::endl;
                }
            }

        }while(more_params);

        zmq_subscriber_.initialize(zmq_sub_cfgs);
        zmq_subscriber_.setMessageCallback([this](const std::string& msg, const std::string& topic) {
            this->zmq_message_callback(msg, topic);}
        );
        zmq_subscriber_.start();
        std::cout << "param load success." << std::endl;

        return true;
    }

    void AllSubscriber::odometryCallback(
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

    void AllSubscriber::poseCallback(
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

    void AllSubscriber::twistCallback(
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
            if(!msg->twist().covariance().data()){
                std::cout << topic_name << " covariance empty" << std::endl;
                return ;
            }
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

    void AllSubscriber::zmq_message_callback(const std::string& message, const std::string& topic){
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
        else{
            std::cout << topic_name << " not defined " << std::endl;
        }
    }

    void AllSubscriber::periodicUpdate(){
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

    std::array<double, STATE_SIZE> AllSubscriber::getState(){
        std::lock_guard<std::mutex> lock(state_mtx_);
        return state_;
    }

    void AllSubscriber::setState(const std::array<double, STATE_SIZE> & state){
        std::lock_guard<std::mutex> lock(state_mtx_);
        state_ = state;
    }

    std::vector<bool> AllSubscriber::loadUpdateConfig(const std::string & topic_name){
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

    void AllSubscriber::pushQueueMeasurement(
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

    void AllSubscriber::copyCovariance(
        const double * covariance_in,
        Eigen::MatrixXd & covariance_out,
        const std::string & topic_name,
        const std::vector<bool> & update_vector,
        const size_t offset, const size_t dimension
    ){
        if(!covariance_in){
            std::cout << topic_name << " covariance empty" << std::endl;
        }
        for(size_t i=0; i<dimension; ++i){
            for(size_t j=0; j<dimension; ++j){
                covariance_out(i, j) = covariance_in[dimension * i + j];

            }
        }
    }

}