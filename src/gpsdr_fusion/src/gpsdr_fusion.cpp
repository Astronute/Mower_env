#include "gpsdr_fusion.h"
#include "glog/logging.h"
#include "Settings.h"


using namespace ORB_SLAM3;

namespace gpsdrfusion{
    GPSDRFusion::GPSDRFusion(): 
        running_(true)
    {
        reset();
    }

    GPSDRFusion::~GPSDRFusion(){
        running_ = false;
        cv_.notify_one();
        delete mpEstimater;
        mpEstimater = nullptr;
        std::cout << "gpsdr_fusion thread exit." << std::endl;
    }

    void GPSDRFusion::reset(){
        Eigen::VectorXd ekfInitstd = Settings::ekfInitStd();
        Eigen::VectorXd ekfDynamicNoise = Settings::ekfDynamicNoise();
        double wheelplus_sf = Settings::ekfWheelplusScalefactor();
        Eigen::Matrix3d Riv = Settings::Riv();
        Eigen::Vector3d piv = Settings::piv();
        Eigen::Vector3d pgv = Settings::pgv();

        mpEstimater = new Estimater(wheelplus_sf, Riv, piv, pgv);
    }

    void GPSDRFusion::spin(){
        std::unique_lock<std::mutex> lock(mtx_);
        cv_.wait(lock, [this]() { return !running_; });
    }

    bool GPSDRFusion::initialize(const std::string & yaml_cfg_dir){
        try{
            filter_config_yaml_ = YAML::LoadFile(yaml_cfg_dir);
        } catch(const YAML::Exception& e){
            LOG(ERROR) << "yaml parsing error: " << e.what();
        }
        reset();

        if(!loadParams(filter_config_yaml_)){
            return false;
        }

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
                std::cout << "allsubscriber subscriber " << port_id << " bind to: " << port_addr << std::endl;
            }
        }while(more_params);

        // twist subscribe
        {
            std::string twist_name = "twist0";
            std::string twist_topic;
            if(filter_config_yaml_[twist_name]){
                twist_topic = filter_config_yaml_[twist_name].as<std::string>();

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

                topic_name_map_[twist_topic] = twist_name;
                zmq_sub_cfgs[port_idx].topics.push_back(twist_topic);
            }
        }

        // imu subscribe
        {
            std::string imu_name = "imu0";
            std::string imu_topic;
            if(filter_config_yaml_[imu_name]){
                imu_topic = filter_config_yaml_[imu_name].as<std::string>();

                int port_idx = 0;
                std::string str = filter_config_yaml_[imu_name + "_port"].as<std::string>().substr(4);
                try {
                    port_idx = std::stoi(str);
                    if(port_idx > zmq_sub_cfgs.size()){
                        std::cout << imu_name << " port id out of range" << std::endl;
                        return false;
                    }
                } catch (const std::exception& e) {
                    std::cerr << str << " parse error: " << e.what() << std::endl;
                }

                topic_name_map_[imu_topic] = imu_name;
                zmq_sub_cfgs[port_idx].topics.push_back(imu_topic);
            }
        }

        // gps subscribe
        {
            std::string gps_name = "gps0";
            std::string gps_topic;
            if(filter_config_yaml_[gps_name]){
                gps_topic = filter_config_yaml_[gps_name].as<std::string>();

                int port_idx = 0;
                std::string str = filter_config_yaml_[gps_name + "_port"].as<std::string>().substr(4);
                try {
                    port_idx = std::stoi(str);
                    if(port_idx > zmq_sub_cfgs.size()){
                        std::cout << gps_name << " port id out of range" << std::endl;
                        return false;
                    }
                } catch (const std::exception& e) {
                    std::cerr << str << " parse error: " << e.what() << std::endl;
                }

                topic_name_map_[gps_topic] = gps_name;
                zmq_sub_cfgs[port_idx].topics.push_back(gps_topic);
            }
        }

        zmq_subscriber_.initialize(zmq_sub_cfgs);
        zmq_subscriber_.setMessageCallback([this](const std::string& msg, const std::string& topic) {
            this->zmq_message_callback(msg, topic);}
        );
        zmq_subscriber_.start();
        std::cout << "param load success." << std::endl;

        return true;
    }

    bool GPSDRFusion::loadParams(const YAML::Node & yaml_cfg){
        
        return true;
    }

    void GPSDRFusion::imuRawCallback(
        const std::shared_ptr<sensor_msgs::Imu> & msg,
        const std::string & topic_name
    ){
        double timestamp = common::toSec(msg->header().stamp());//msg->header.stamp.toSec() + Settings::imudT_;
        Eigen::Vector3d acc(msg->linear_acceleration().x(), msg->linear_acceleration().y(), msg->linear_acceleration().z());
        Eigen::Vector3d gyr(msg->angular_velocity().x(), msg->angular_velocity().y(), msg->angular_velocity().z());

        // // 误差补偿：
        // // 1. 先补偿比例因子和轴交叉耦合误差
        // acc = Settings::Accelcalibparams() * acc;
        // gyr = Settings::Gyrocalibparams() * gyr;
        // // 2. 再从imu物理坐标系转换到虚拟坐标系
        // acc = Settings::RbVitualPhysical() * acc;
        // gyr = Settings::RbVitualPhysical() * gyr;
        // // 3. 再补偿常值零偏
        // acc -= Settings::accelBias();
        // gyr -= Settings::gyroBias();

        SENSOR_DATA::RawImu rawImu;
        rawImu.t = timestamp;
        rawImu.a = acc.cast<float>();
        rawImu.w = gyr.cast<float>();
        mpEstimater->AddRawImu(rawImu);
    }

    // 轮速直接采用反馈上来的线速度
    void GPSDRFusion::wheelRawCallback(
        const std::shared_ptr<geometry_msgs::TwistWithCovarianceStamped> & msg,
        const std::string & topic_name
    ){
        // 传入轮速数据
        WheelOdometer wheelOdometer;
        wheelOdometer.t = common::toSec(msg->header().stamp());
        wheelOdometer.whlplus_l = msg->twist().twist().linear().x();
        wheelOdometer.whlplus_r = msg->twist().twist().linear().x();
        
        SENSOR_DATA::RawWheel rawWheel;
        rawWheel.t = wheelOdometer.t;
        rawWheel.whlplus_l = wheelOdometer.whlplus_l;
        rawWheel.whlplus_r = wheelOdometer.whlplus_r;
        mpEstimater->AddRawWheel(rawWheel);

    }

    void GPSDRFusion::gpsRawCallback(
        const std::shared_ptr<geometry_msgs::PoseWithCovarianceStamped> & msg,
        const std::string & topic_name
    ){
        GpsOdometry gpsOdom;
        gpsOdom.t = common::toSec(msg->header().stamp());
        gpsOdom.lon = msg->pose().pose().position().x();
        gpsOdom.lat = msg->pose().pose().position().y();
        gpsOdom.h = msg->pose().pose().position().z();
        // gpsOdom.vel << gps_msg->twist.twist.linear.x, gps_msg->twist.twist.linear.y, gps_msg->twist.twist.linear.z;
        // gpsOdom.pos_cov(0, 0) = gps_msg->pose.covariance[0];
        // gpsOdom.pos_cov(1, 1) = gps_msg->pose.covariance[7];
        // gpsOdom.pos_cov(2, 2) = gps_msg->pose.covariance[14];

        // 传入GPS数据
        static double last_gps_time = 0;
        if(!mpEstimater->Inited())
        {
            // mpEstimater->AddGpsOdometer(gpsOdom);

            SENSOR_DATA::RawGps rawGps;
            rawGps.t = gpsOdom.t;
            rawGps.lon = gpsOdom.lon;
            rawGps.lat = gpsOdom.lat;
            rawGps.h = gpsOdom.h;
            // rawGps.satnum_used = gpsOdom.satnum_used;
            rawGps.hdop = gpsOdom.hdop;
            mpEstimater->AddRawGps(rawGps);

            last_gps_time = gpsOdom.t;
        }
    }

    void GPSDRFusion::zmq_message_callback(const std::string& message, const std::string& topic){
        std::string topic_name = topic_name_map_[topic];
        
        if(topic.compare("/codbot/twist") == 0){
            std::shared_ptr<geometry_msgs::TwistWithCovarianceStamped> twist_ptr = std::make_shared<geometry_msgs::TwistWithCovarianceStamped>();
            if(twist_ptr->ParseFromArray(message.data(), message.size())){
                wheelRawCallback(twist_ptr, topic_name);
            }
            else{
                std::cout << topic_name << " process failed" << std::endl;
            }
        }
        else if(topic.compare("/codbot/gps") == 0){
            std::shared_ptr<geometry_msgs::PoseWithCovarianceStamped> pose_ptr = std::make_shared<geometry_msgs::PoseWithCovarianceStamped>();
            if(pose_ptr->ParseFromArray(message.data(), message.size())){
                gpsRawCallback(pose_ptr, topic_name);
            }
            else{
                std::cout << topic_name << " process failed" << std::endl;
            }
        }
        else if(topic.compare("/codbot/imu") == 0){
            std::shared_ptr<sensor_msgs::Imu> imu_ptr = std::make_shared<sensor_msgs::Imu>();
            if(imu_ptr->ParseFromArray(message.data(), message.size())){
                imuRawCallback(imu_ptr, topic_name);
            }
            else{
                std::cout << topic_name << " process failed" << std::endl;
            }
        }
        else{
            std::cout << topic_name << " not defined " << std::endl;
        }
    }

}
