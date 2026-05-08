#pragma once

#if(0)
#define USE_SIM
#endif

#include <google/protobuf/timestamp.pb.h>
#include "std_msgs/header.pb.h"
#include "nav_msgs/odometry.pb.h"
#include "geometry_msgs/pose.pb.h"
#include "geometry_msgs/twist.pb.h"
#include "pnc_msgs/point_vec.pb.h"
#include "pnc_msgs/planning_trajectory.pb.h"
#include "zmq_subscriber.h"
#include "zmq_publisher.h"
// #ifdef USE_SIM
// #include "robot_msgs_utils/msg/planning_trajectory.hpp"
// #else
// #include "robot_msgs/msg/planning_output.hpp"
// #endif
#include "common.h"
#include "yaml-cpp/yaml.h"
#include "Eigen/Dense"
#include <Eigen/Core>
#include "mpc.h"
#include "timer.h"
#include "all_subscriber.h"

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



namespace CB{

	const double PI = 3.141592653589793;
	const double TAU = 6.283185307179587;

    class RosController{
    public:
        explicit RosController();

        ~RosController();

        bool initialize(const YAML::Node & yaml_cfg);

        void reset();

		void spin();

        bool loadParams(const YAML::Node & yaml_cfg);

        void setPtr(std::shared_ptr<allsubscriber::AllSubscriber> _all_subscriber){
            this->all_subscriber_ = _all_subscriber;
        }

		void periodicControl();

		bool loadParamMatrix(
			const std::string & param_name, 
			std::vector<double> & q, 
			std::vector<double> & r
		);

		auto now() const {
			return std::chrono::system_clock::now();
		}

    private:
		enum ControllerRunStatus{
			CONTROL_FREE,
			CONTROL_BUSY,
			CONTROL_ERROR,
			CONTROL_END,
			CONTROL_TIMEOUT
		};

		enum ControllerMode{
			SLOWLY_STOP_CMD,
			RE_START_CMD,

			LINE_MOVE_CMD,
			SELF_ROTATE_CMD,
			CIRCLE_MOVE_CMD,

			TO_POINT_CMD,
			TO_POSE_CMD,

			BACKWARD_TO_POSE_CMD,
			FORWARD_TO_POSE_CMD,

			UNDEFINED_CMD
		};

		enum ControllerStratagy{
			NORMAL_MODE_STR,
			EMERGENCY_BRAKING_STR,
			SLOWLY_STOP_STR,
			TEST_STR,
			TO_POINT_STR,
			TO_POSE_STR,
			GO_BACKWORD_STR,
			GO_FORWARD_STR,
			NO_MOTION_STR
		};

		std::string array_controller_stratagy_[9] = {
			"NORMAL_MODE_STR",
			"EMERGENCY_BRAKING_STR",
			"SLOWLY_STOP_STR",
			"TEST_STR",
			"TO_POINT_STR",
			"TO_POSE_STR",
			"GO_BACKWORD_STR",
			"GO_FORWARD_STR",
			"NO_MOTION_STR"
		};

		enum FollowedTrajectory{
			LOCAL_TRAJ,
			EMERG_BRAKE_TRAJ,
			NO_TRAJ
		};

		YAML::Node filter_config_yaml_;

		std::atomic<bool> running_;

		std::thread control_thread_;

		std::condition_variable cv_;

		std::mutex mtx_;

		MPC mpc_controller_;

		std::shared_ptr<allsubscriber::AllSubscriber> all_subscriber_;

		ZmqSubscriber zmq_subscriber_;

		ZmqPublisher zmq_publisher_;

		std::unordered_map<std::string, std::vector<double>> matrix_params_q_;

		std::unordered_map<std::string, std::vector<double>> matrix_params_r_;

		geometry_msgs::Pose2D robot_pose_, last_robot_pose_;

		geometry_msgs::Twist robot_twist_, last_robot_twist_;

		std::vector<TrajPoint> target_traj_; // 控制器最终跟踪的轨迹

		double target_t_; // 前视时间

		TrajPoint target_traj_point_, target_traj_point2_; // 预瞄点

		std::vector<TrajPoint> local_optimal_trajectory_; // 当前局部轨迹

		ControllerRunStatus controller_run_status_;

		ControllerMode controller_run_mode_;

		ControllerStratagy controller_stratagy_;

		FollowedTrajectory pick_followed_trajectory_;

		double t_delta_max_, t_delta_min_;

		double s_delta_max_, s_delta_min_;

		unsigned char stratagy_status_, last_stratagy_status_;

		bool status_change_flag_;

		double control_frequency_;

        std::string map_frame_id_;

        std::string odom_frame_id_;

        std::string base_link_frame_id_;

        std::string world_frame_id_;

		geometry_msgs::TwistStamped cmd_twist_, last_cmd_twist_;
    
    };

}
