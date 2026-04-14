#pragma once

#include <Eigen/Core>
#include "Eigen/Dense"

#include "geometry_msgs/twist.pb.h"

#include "measurement.h"
#include "yaml-cpp/yaml.h"

#include <iostream>
#include <vector>
#include <string>
#include <unordered_map>

namespace CB{
	
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

		double normalize_angle(double angle);

		double get_control_rate();

		bool is_initialized();

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

	protected:
	
		bool initialized_;

		YAML::Node controller_config_yaml_;

		double wheel_base_;

		double sensor_timeout_;

		std::vector<bool> control_update_mask_;

		double control_rate_;

		double max_vel_, min_vel_;

		double max_w_, min_w_;

		double tau_w_;
		
		double tau_v_;
		
    };

}