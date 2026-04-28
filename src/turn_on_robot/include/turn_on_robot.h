#pragma once

#include <google/protobuf/timestamp.pb.h>
#include "std_msgs/header.pb.h"
#include "nav_msgs/odometry.pb.h"
#include "geometry_msgs/pose.pb.h"
#include "geometry_msgs/twist.pb.h"
#include "geometry_msgs/quaternion.pb.h"
#include "pnc_msgs/point_vec.pb.h"
#include "pnc_msgs/planning_trajectory.pb.h"
#include "zmq_subscriber.h"
#include "zmq_publisher.h"
#include "robot_serial.h"
#include "timer.h"

#include "yaml-cpp/yaml.h"

#include <vector>
#include <string>
#include <queue>
#include <map>
#include <memory>
#include <chrono>
#include <mutex>
#include <atomic>
#include <condition_variable>

#if(0)
#define USE_SIM
#endif

using SysTimePoint = std::chrono::time_point<std::chrono::system_clock>;
using SysTimeDuration = std::chrono::system_clock::duration;

namespace turn_on_robot{

    struct EgoVelocityData{
        uint8_t header;
        uint8_t length;
        uint8_t func;
        uint8_t x_dir;
        uint16_t x_speed;
        uint8_t y_dir;
        uint16_t y_speed;
        uint8_t turn_dir;
        uint16_t gyro_speed;
        uint8_t xor_check;
    };

    class TurnOnRobot{
    public:
        explicit TurnOnRobot();

        ~TurnOnRobot();

        bool initialize();

        void reset();

		void spin();

        bool loadParams();

        std::vector<double> loadCovariance(const std::string & sensor_name);

        void zmq_message_callback(const std::string& message, const std::string& topic);

        void packet_unpack(uint8_t _buf);

        void carInfoParse(uint8_t *_buf, uint8_t _len);

        void TimerCallback();

		auto now() const {
			return std::chrono::system_clock::now();
		}

		google::protobuf::Timestamp systimeToProto(const SysTimePoint & time){
			google::protobuf::Timestamp stamp;
			
			auto duration = time.time_since_epoch();
			stamp.set_seconds(std::chrono::duration_cast<std::chrono::seconds>(duration).count());
			stamp.set_nanos(std::chrono::duration_cast<std::chrono::nanoseconds>(duration % std::chrono::seconds(1)).count());

			return stamp;
		}

		void quatToRPY(const geometry_msgs::Quaternion & quat, double & roll, double & pitch, double & yaw){
			// roll (x-axis rotation)
			double sinr_cosp = 2 * (quat.w() * quat.x() + quat.y() * quat.z());
			double cosr_cosp = 1 - 2 * (quat.x() * quat.x() + quat.y() * quat.y());
			roll = std::atan2(sinr_cosp, cosr_cosp);
		
			// pitch (y-axis rotation)
			double sinp = 2 * (quat.w() * quat.y() - quat.z() * quat.x());
			if (std::abs(sinp) >= 1)
				pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
			else
				pitch = std::asin(sinp);
		
			// yaw (z-axis rotation)
			double siny_cosp = 2 * (quat.w() * quat.z() + quat.x() * quat.y());
			double cosy_cosp = 1 - 2 * (quat.y() * quat.y() + quat.z() * quat.z());
			yaw = std::atan2(siny_cosp, cosy_cosp);
		}

		void RPYToQuat(const double & roll, const double & pitch, const double & yaw, geometry_msgs::Quaternion & quat){
			double cy = cos(yaw * 0.5);
			double sy = sin(yaw * 0.5);
			double cp = cos(pitch * 0.5);
			double sp = sin(pitch * 0.5);
			double cr = cos(roll * 0.5);
			double sr = sin(roll * 0.5);
		
			quat.set_w(cy * cp * cr + sy * sp * sr);
			quat.set_x(cy * cp * sr - sy * sp * cr);
			quat.set_y(sy * cp * sr + cy * sp * cr);
			quat.set_z(sy * cp * cr - cy * sp * sr);
		}

    private:
		std::atomic<bool> running_;

        YAML::Node robot_config_yaml_;

        std::string device_tty_;

        double tty_frequency_;

        Timer timer_;

        std::mutex ctrl_mtx_;

        int16_t cmd_vel_x_, cmd_vel_y_, cmd_vel_w_;

        std::unordered_map<std::string, std::string> topic_name_map_;

        ZmqSubscriber zmq_subscriber_;

        ZmqPublisher zmq_publisher_;

        std::string base_link_frame_id_;

        std::string world_frame_id_;

        int* serial_fd_;

        std::unordered_map<std::string, std::vector<double>> sensor_covariance_map_; 

        /*底盘反馈数据缓冲区*/
        carMoveInfo_t g_tCarMoveInfo;

        carMotorInfo_t g_tCarMotorInfo;

        carBatteryInfo_t g_tCarBatteryInfo;

        carImuAttitude_t g_tCarImuAttitudeInfo;

        carImuRaw_t g_tCarImuRawInfo;

        carTypeInfo_t g_tCarTypeInfo;

    };

}
