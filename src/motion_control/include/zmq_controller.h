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
#include "pnc_msgs/motion_outsignal.pb.h"
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

	enum ControllerRunStatus{
		CONTROL_FREE,
		CONTROL_BUSY,
		CONTROL_ERROR,
		CONTROL_END,
		CONTROL_TIMEOUT
	};

	// 控制器外部命令模式 default: UNDEFINED_CMD
	enum ControllerMode{
		SLOWLY_STOP_CMD,

		RE_START_CMD,

		LINE_MOVE_CMD,
		ROTATE_MOVE_CMD,
		CIRCLE_MOVE_CMD,

		FORWARD_TO_POSE_CMD,
		BACKWARD_TO_POSE_CMD,
		ROTATE_TO_POSE_CMD,

		UNDEFINED_CMD
	};

	enum ControllerStratagy{
		EMERGENCY_BRAKING_STR,
		SLOWLY_STOP_STR,
		TEST_STR,
		FORWARD_TO_POSE_STR,
		BACKWARD_TO_POSE_STR,
		ROTATE_TO_POSE_STR,
		NORMAL_MODE_STR,
		NO_MOTION_STR
	};

	enum FollowedTrajectory{
		LOCAL_TRAJ,
		EMERG_BRAKE_TRAJ,
		TEST_TRAJ,
		FORWARD_TO_GOAL_TRAJ,
		BACKWARD_TO_GOAL_TRAJ,
		ROTATE_TO_GOAL_TRAJ,
		NO_TRAJ
	};

	enum ControlAlgorithm{
		ABS_ALGO,
		MPC_ALGO,
		LQR_ALGO,
		OPENLOOP_ALGO,
		NO_ALGO
	};

	extern std::string array_controller_status_[5];
	extern std::string array_controller_mode_[9];
	extern std::string array_controller_stratagy_[8];
	extern std::string array_followed_trajectory_[7];
	extern std::string array_picked_controller_[5];

	struct OutStratagyInfo{
		ControllerMode out_stratagy;
		pnc_msgs::MotionOutSignal data;
	};

    class RosController{
    public:
        explicit RosController();

        ~RosController();

        bool initialize(const YAML::Node & yaml_cfg);

        void reset();

		void spin();

        bool loadParams(const YAML::Node & yaml_cfg);

		void set_outstratagy(const ControllerMode& mode);

		OutStratagyInfo get_outStratagyInfo();

		std::string zmq_server_callback(const std::string& request);

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

		YAML::Node filter_config_yaml_;

		std::atomic<bool> running_;

		std::thread control_thread_;

		std::condition_variable cv_;

		std::mutex mtx_;

		std::mutex out_stratagy_mtx_;

		MPC mpc_controller_;

		std::shared_ptr<allsubscriber::AllSubscriber> all_subscriber_;

		ZmqSubscriber zmq_subscriber_;

		ZmqPublisher zmq_publisher_, zmq_server_;

		std::unordered_map<std::string, std::vector<double>> matrix_params_q_;

		std::unordered_map<std::string, std::vector<double>> matrix_params_r_;

		geometry_msgs::Pose2D robot_pose_, last_robot_pose_;

		geometry_msgs::Twist robot_twist_, last_robot_twist_;

		std::vector<TrajPoint> target_traj_; // 控制器最终跟踪的轨迹

		double target_t_; // 前视时间

		TrajPoint target_traj_point_, target_traj_point2_; // 预瞄点

		std::vector<TrajPoint> local_optimal_trajectory_; // 当前局部轨迹

		geometry_msgs::Pose2D out_goal_pose_;

		ControllerRunStatus controller_run_status_;

		OutStratagyInfo controller_out_stratagy_;

		ControllerStratagy controller_stratagy_;

		FollowedTrajectory pick_followed_trajectory_;

		ControlAlgorithm pick_control_algorithm_;

		double t_delta_max_, t_delta_min_;

		double s_delta_max_, s_delta_min_;

		unsigned char stratagy_param_, last_stratagy_param_;

		bool status_change_flag_;

		double control_frequency_;

        std::string map_frame_id_;

        std::string odom_frame_id_;

        std::string base_link_frame_id_;

        std::string world_frame_id_;

		geometry_msgs::TwistStamped cmd_twist_, last_cmd_twist_;
    
    };

}
