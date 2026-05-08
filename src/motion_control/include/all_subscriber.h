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
#include "common.h"
#include "timer.h"

namespace allsubscriber
{
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

	struct Measurement {
		Measurement() {}

		SysTimePoint time_ = SysTimePoint{std::chrono::seconds{0}};

		std::string topic_name_;

		double mahalanobis_thresh_;

		std::vector<bool> update_mask_;

		Eigen::VectorXd measurement_; // 测量结果
		
		Eigen::MatrixXd covariance_; // 协方差

		bool operator()(const std::shared_ptr<Measurement> & a, const std::shared_ptr<Measurement> & b){
			return (*this)(*(a.get()), *(b.get()));
		}

		bool operator()(const Measurement & a, const Measurement & b){
			return a.time_ > b.time_;
		}
	};
	using MeasurementPtr = std::shared_ptr<Measurement>;

	struct PathPoint{
		PathPoint() : x(0), y(0), z(0), yaw(0), kappa(0), s(0){}

		PathPoint(double x_, double y_, double z_, double yaw_, double kappa_, double s_){
			x = x_;
			y = y_;
			z = z_;
			yaw = yaw_;
			kappa = kappa_;
			s = s_;
		}

		double x, y, z;

		double yaw, kappa, s;
	};

	struct TrajPoint{
		TrajPoint() : path_point(0, 0, 0, 0, 0, 0), v(0), w(0), t(0){}

		TrajPoint(PathPoint path_point_, double v_, double w_, double t_){
			path_point = path_point_;
			v = v_;
			w = w_;
			t = t_;
		}

		PathPoint path_point;

		double v, w, t;
	};

    class AllSubscriber
    {
    public:
        AllSubscriber();

        ~AllSubscriber();

        void reset();

        bool initialize(const YAML::Node & yaml_cfg);

        bool loadParams(const YAML::Node & yaml_cfg);

        void periodicUpdate();

        void spin();

		bool getState(std::array<double, STATE_SIZE> & state);

		void setState(const std::array<double, STATE_SIZE> & state);

		bool getLocalTrajectory(std::vector<TrajPoint>& traj);

        void zmq_message_callback(const std::string& message, const std::string& topic);

        std::vector<bool> loadUpdateConfig(const std::string & topic_name);

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

		auto now() const {
			return std::chrono::system_clock::now();
		}
    
    private:
		std::atomic<bool> running_;

		std::condition_variable cv_;

		std::mutex mtx_;

		std::mutex state_mtx_, trajectory_mtx_;

        YAML::Node filter_config_yaml_;

		ZmqSubscriber zmq_subscriber_;

		ZmqPublisher zmq_publisher_;

        std::string map_frame_id_;

        std::string odom_frame_id_;

        std::string base_link_frame_id_;

        std::string world_frame_id_;

		double sensor_timeout_; // 传感器延迟阈值

		double sensor_dead_; // 传感器失灵阈值

        std::atomic<int> sensor_delayed_; // 0:无延迟 1:毫秒延迟 2:秒延迟或无数据

		std::atomic<int> local_path_delayed_; // 局部轨迹延迟

		std::unordered_map<std::string, std::string> topic_name_map_;

		std::unordered_map<std::string, CallBackInfo> topic_callbackinfo_map_;

        std::queue<MeasurementPtr> measurement_queue_;

        std::array<double, STATE_SIZE> state_;

        Timer timer_;

        double sample_frequency_;

        SysTimePoint last_measurement_time_; // 最近一次measurement时间戳

		common::fixedQueue<std::vector<TrajPoint>> trajectory_queue_;

        std::map<std::string, SysTimePoint> last_message_times_; // 前一次测量值时间戳(单独sensor)

    };
}