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
#include "measurement.h"
#include "yaml-cpp/yaml.h"
#include "Eigen/Dense"
#include <Eigen/Core>
#include "mpc.h"
#include "timer.h"

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

	enum StateMembers {
		// pose terms
		StateMemberX = 0,
		StateMemberY,
		StateMemberZ,
		StateMemberRoll,
		StateMemberPitch,
		StateMemberYaw,
		// twist terms
		StateMemberVx,
		StateMemberVy,
		StateMemberVz,
		StateMemberGx,
		StateMemberGy,
		StateMemberGz,
		// 
		StateMemberAx,
		StateMemberAy,
		StateMemberAz
	};

	// control vector
	const int STATE_SIZE = 15;
	const int TWIST_SIZE = 6;
	const int POSE_SIZE = 6;
	const int POSITION_SIZE = 3;
	const int ORIENTATION_SIZE = 3;
	const int LINEAR_VELOCITY_SIZE = 3;
	const int ACCELERATION_SIZE = 3;
	const int POSITION_OFFSET = StateMembers::StateMemberX;
	const int ORIENTATION_OFFSET = StateMembers::StateMemberRoll;
	const int POSITION_V_OFFSET = StateMembers::StateMemberVx;
	const int ORIENTATION_V_OFFSET = StateMembers::StateMemberGx;
	const int POSITION_A_OFFSET = StateMembers::StateMemberAx;

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

    class RosController{
    public:
        explicit RosController();

        ~RosController();

        bool initialize();

        void reset();

		void spin();

        bool loadParams();

        void odometryCallback(
            const std::shared_ptr<nav_msgs::Odometry> & msg,
            const std::string & topic_name,
            const CallBackInfo & pose_callback_info,
            const CallBackInfo & twist_callback_info
        );

        void poseCallback(
            const std::shared_ptr<geometry_msgs::PoseWithCovarianceStamped> & msg,
            const CallBackInfo & callback_info,
            const std::string & target_frame,
            const std::string & source_frame,
            const bool imu_data
        );

        void twistCallback(
            const std::shared_ptr<geometry_msgs::TwistWithCovarianceStamped> & msg,
            const CallBackInfo & callback_info,
            const std::string & target_frame
        );

		void trajectoryCallback(
			const std::shared_ptr<pnc_msgs::PlanningTrajectory> & msg,
			const std::string & topic_name,
			const std::string & target_frame
		);

		void zmq_message_callback(const std::string& message, const std::string& topic);

		void periodicControl();

		void periodicUpdate();

		std::array<double, STATE_SIZE> getState() const;

		void setState(const std::array<double, STATE_SIZE> & state);

		void pushQueueMeasurement(
			const std::string & topic_name, 
			const Eigen::MatrixXd & measurement,
			const Eigen::MatrixXd & covariance,
			const std::vector<bool> & update_mask,
			const double mahalanobis_thresh,
			const SysTimePoint & time
		);

		void copyCovariance(
			const double * covariance_in,
			Eigen::MatrixXd & covariance_out,
			const std::string & topic_name,
			const std::vector<bool> & update_vector,
			const size_t offset, const size_t dimension
		);

        std::vector<bool> loadUpdateConfig(const std::string & topic_name);

		bool loadParamMatrix(
			const std::string & param_name, 
			std::vector<double> & q, 
			std::vector<double> & r, 
			std::vector<double> & f
		);

		void printVector(const std::vector<double> & vec){
			if (vec.empty()) {
				std::cout << "empty param" << std::endl;
				return;
			}
			for (size_t i = 0; i < vec.size(); ++i) {
				std::cout << vec[i];
				if (i != vec.size() - 1) std::cout << ", ";
			}
			std::cout << "\n" << std::endl;
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

		auto now() const {
			return std::chrono::system_clock::now();
		}

		double nanosecToSec(const int64_t nanoseconds){
			return static_cast<double>(nanoseconds) * 1e-9;
		}

		SysTimePoint protoToSystime(const google::protobuf::Timestamp & stamp){
			return std::chrono::system_clock::time_point(
				std::chrono::seconds(stamp.seconds()) +
				std::chrono::nanoseconds(stamp.nanos())
			);
		}

		google::protobuf::Timestamp systimeToProto(const SysTimePoint & time){
			google::protobuf::Timestamp stamp;
			
			auto duration = time.time_since_epoch();
			stamp.set_seconds(std::chrono::duration_cast<std::chrono::seconds>(duration).count());
			stamp.set_nanos(std::chrono::duration_cast<std::chrono::nanoseconds>(duration % std::chrono::seconds(1)).count());

			return stamp;
		}

		double toSec(const SysTimePoint & time){
			return std::chrono::duration<double>(time.time_since_epoch()).count();
		}

		double toSec(const std::chrono::system_clock::duration & duration){
			return std::chrono::duration<double>(duration).count();
		}

		double toSec(const google::protobuf::Timestamp & stamp){
			return static_cast<double>(stamp.seconds()) + nanosecToSec(stamp.nanos());
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

		std::atomic<bool> running_;

		std::condition_variable cv_;

		std::mutex mtx_;

		MPC mpc_controller_;

		ZmqSubscriber zmq_subscriber_;

		ZmqPublisher zmq_publisher_;

		std::unordered_map<std::string, std::string> topic_name_map_;

		std::unordered_map<std::string, CallBackInfo> topic_callbackinfo_map_;

        YAML::Node filter_config_yaml_;

		std::unordered_map<std::string, std::vector<double>> matrix_params_q_;

		std::unordered_map<std::string, std::vector<double>> matrix_params_r_;

		std::unordered_map<std::string, std::vector<double>> matrix_params_f_;

		std::atomic<std::array<double, STATE_SIZE>> state_;

		geometry_msgs::Pose2D robot_pose_, last_robot_pose_;

		geometry_msgs::Twist robot_twist_, last_robot_twist_;

		std::vector<TrajPoint> target_traj_; // 控制器最终跟踪的轨迹

		double target_t_; // 前视时间

		TrajPoint target_traj_point_, target_traj_point2_; // 预瞄点

		std::vector<TrajPoint> local_optimal_trajectory_; // 当前局部轨迹

		std::atomic<int> sensor_delayed_; // 0:无延迟 1:毫秒延迟 2:秒延迟或无数据

		std::atomic<bool> local_path_delayed_;

		ControllerRunStatus controller_run_status_;

		ControllerMode controller_run_mode_;

		ControllerStratagy controller_stratagy_;

		FollowedTrajectory pick_followed_trajectory_;

		double sensor_timeout_; // 传感器延迟阈值

		double sensor_dead_; // 传感器失灵阈值

		double local_path_timeout_; // 局部轨迹延迟阈值

		double t_delta_max_;

		double t_delta_min_;

		unsigned char stratagy_status_, last_stratagy_status_;

		bool status_change_flag_;

        // std::vector<rclcpp::SubscriptionBase::SharedPtr> topic_subs_;

		double sample_frequency_;

		double control_frequency_;

		SysTimePoint last_measurement_time_; // 最近一次measurement时间戳

		SysTimePoint last_localtraj_time_; // 最近一次取得的局部轨迹时间戳

		std::map<std::string, SysTimePoint> last_message_times_; // 前一次测量值时间戳(单独sensor)

        std::string map_frame_id_;

        std::string odom_frame_id_;

        std::string base_link_frame_id_;

        std::string world_frame_id_;

        std::queue<MeasurementPtr> measurement_queue_;

		fixedQueue<std::vector<TrajPoint>> trajectory_queue_;

		// rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr ctrl_cmd_pub_;

		geometry_msgs::TwistStamped cmd_twist_, last_cmd_twist_;

		// rclcpp::TimerBase::SharedPtr timer_;
		Timer timer_;
    
    };

}
