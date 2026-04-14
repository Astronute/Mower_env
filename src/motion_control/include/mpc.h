#pragma once

#include "controller_base.h"
#include "geometry_msgs/pose2d.pb.h"
#include "geometry_msgs/twist.pb.h"

#include "Eigen/Dense"
#include "OsqpEigen/OsqpEigen.h"
#include "measurement.h"

#include <vector>

namespace CB{

    class MPC : public ControllerBase{
        public:

        MPC();

        ~MPC();

        bool dlqr(
            Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> &A,
            Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> &B,
            Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> &Q,
            Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> &R,
            Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> &K,
            Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> &P,
            int n
        );

        bool mpc_delay_controller(
            const std::vector<double> & param_q,
            const std::vector<double> & param_r,
            const geometry_msgs::Pose2D & robot_pose,
            const geometry_msgs::Twist & robot_twist,
            const TrajPoint & target_point, 
            const std::vector<TrajPoint> & target_traj,
            double & cmd_vel_v, 
            double & cmd_vel_w
        );

        bool run_controller(
            const std::vector<double> & param_q,
            const std::vector<double> & param_r,
            const geometry_msgs::Pose2D & robot_pose,
            const geometry_msgs::Twist & robot_twist,
            const TrajPoint & target_point, 
            const std::vector<TrajPoint> & target_traj,
            double & cmd_vel_v, 
            double & cmd_vel_w
        );

        private:
        bool solver_initialized_;
    };

}