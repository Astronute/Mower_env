#pragma once

#include <Eigen/Core>
#include "Eigen/Dense"

#include "geometry_msgs/twist.pb.h"
#include "geometry_msgs/pose2d.pb.h"
#include "geometry_msgs/twist.pb.h"

#include "common.h"
#include "all_subscriber.h"
#include "yaml-cpp/yaml.h"

#include <iostream>
#include <vector>
#include <string>
#include <unordered_map>

namespace CB{

	using namespace allsubscriber;
	
    class ControllerBase{
	public:
		ControllerBase() : 
			initialized_(false),
			sensor_timeout_(0.0)
        {

		}
		~ControllerBase() {}

        bool initialize();

        void reset();

        bool loadParams();

		bool loadParamMatrix(const std::string & param_name, std::vector<double> & q, std::vector<double> & r);

        bool findTargetTrajTimepoint(TrajPoint & target_point, const std::vector<TrajPoint> & target_traj, double target_t);

		bool findTargetPathPoint(TrajPoint & target_point, const std::vector<TrajPoint> & target_traj, double target_s);

		void planLinearTrajectory(std::vector<TrajPoint> & traj, const geometry_msgs::Pose2D & robot_pose, const geometry_msgs::Twist & robot_twist, const double line_s);

		void planRotateTrajectory(std::vector<TrajPoint> & traj, const geometry_msgs::Pose2D & robot_pose, const geometry_msgs::Twist & robot_twist, const double rotate_angle);

		void planLinearToPoseTrajectory(std::vector<TrajPoint> & traj, const geometry_msgs::Pose2D & robot_pose, const geometry_msgs::Twist & robot_twist, const geometry_msgs::Pose2D & goal_pose);

		void planBrakeTrajectory(std::vector<TrajPoint> & traj, const geometry_msgs::Pose2D & robot_pose, const geometry_msgs::Twist & robot_twist);

		void fivetimesPlanTraj(std::array<double, 6> & coeff, double t0, double s0, double v0, double a0, double t1, double s1, double v1, double a1);

		// virtual bool run_controller(
        //     const std::vector<double> & param_q,
        //     const std::vector<double> & param_r,
        //     const geometry_msgs::msg::Pose2D & robot_pose,
        //     const geometry_msgs::msg::Twist & robot_twist,
        //     const TrajPoint & target_point, 
        //     const std::vector<TrajPoint> & target_traj,
        //     double & cmd_vel_v, 
        //     double & cmd_vel_w
        // ) = 0;

		double get_control_rate();

		bool is_initialized();

		auto now() const {
			return std::chrono::system_clock::now();
		}

	protected:
	
		bool initialized_;

		YAML::Node controller_config_yaml_;

		double wheel_base_;

		double sensor_timeout_;

		std::vector<bool> control_update_mask_;

		double control_rate_;

		double max_vel_, min_vel_;

		double max_w_, min_w_;

		double max_a_, max_aw_;

		double tau_w_;
		
		double tau_v_;
		
    };

}