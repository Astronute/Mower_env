#pragma once

#include <iostream>
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

#include "yaml-cpp/yaml.h"
#include "Estimater.h"
#include "common.h"
#include "ImuTypes.h"
#include "zmq_subscriber.h"
#include "zmq_publisher.h"

#include "geometry_msgs/twist.pb.h"
#include "geometry_msgs/pose.pb.h"
#include "sensor_msgs/Imu.pb.h"
#include "sensor_msgs/wheel.pb.h"
#include "hardware/hardware.pb.h"


using namespace ORB_SLAM3;

namespace gpsdrfusion{

    struct CallBackInfo{
		CallBackInfo():
			topic_name_(""),
			update_sum_(0),
			rejection_threshold_(0.0)
		{

		}
        CallBackInfo(const std::string & topic_name, const std::vector<bool> update_mask,
        const int update_sum, const double rejection_threshold): 
        topic_name_(topic_name), update_mask_(update_mask),
        update_sum_(update_sum), rejection_threshold_(rejection_threshold){
        }

        std::string topic_name_; // 话题来源
        std::vector<bool> update_mask_; // 更新mask
        int update_sum_; // 更新维数
        double rejection_threshold_; // 异常值过滤阈值
    };

    class GPSDRFusion{
    public:
        GPSDRFusion();
        ~GPSDRFusion();

        bool initialize(const std::string & yaml_cfg_dir);

        void reset();

		void spin();

        bool loadParams(const std::string & yaml_cfg_dir);

        void imuRawCallback(
            const std::shared_ptr<sensor_msgs::Imu> & msg,
            const std::string & topic_name
        );

        // 没有使用距离，直接采用底盘上传的轮速
        void wheelRawCallback(
            const std::shared_ptr<geometry_msgs::TwistWithCovarianceStamped> & msg,
            const std::string & topic_name
        );

        void gpsRawCallback(
            const std::shared_ptr<hardware_message::gps> & msg,
            const std::string & topic_name
        );

        void zmq_message_callback(const std::string& message, const std::string& topic);

		auto now() const {
			return std::chrono::system_clock::now();
		}

    private:
		YAML::Node filter_config_yaml_;

		std::atomic<bool> running_;

		std::condition_variable cv_;

		std::mutex mtx_;

		ZmqSubscriber zmq_subscriber_;

		ZmqPublisher zmq_publisher_;

        Estimater *mpEstimater;

        std::unordered_map<std::string, std::string> topic_name_map_;

    };

}