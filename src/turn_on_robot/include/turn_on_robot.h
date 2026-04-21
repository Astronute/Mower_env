#pragma once

#include <google/protobuf/timestamp.pb.h>
#include "std_msgs/header.pb.h"
#include "nav_msgs/odometry.pb.h"
#include "geometry_msgs/pose.pb.h"
#include "geometry_msgs/twist.pb.h"
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

#if(1)
#define USE_SIM
#endif

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

        void TurnOnRobot::zmq_message_callback(const std::string& message, const std::string& topic);

        void TimerCallback();

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

        int* serial_fd_;

    };

}