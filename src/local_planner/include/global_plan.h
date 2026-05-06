#pragma once

#include <vector>
#include <string>
#include <queue>
#include <memory>
#include <atomic>
#include <chrono>
#include <map>
#include <thread>
#include <fstream>
#include <mutex>

#include "geometry_msgs/pose.pb.h"
#include "pnc_msgs/point_vec.pb.h"
#include "pnc_msgs/planning_trajectory.pb.h"
#include "nav_msgs/path.pb.h"

#include "zmq_publisher.h"
#include "yaml-cpp/yaml.h"
#include "nlohmann/json.hpp"
#include "wall_rate.h"
#include "common.h"
#include "params.h"
#include "all_subscriber.h"
#include "Eigen/Dense"
#include "Eigen/Sparse"
#include "OsqpEigen/OsqpEigen.h"

namespace globalplanner
{
    class GlobalPlanner{
    public:
        GlobalPlanner();

        ~GlobalPlanner();

        bool loadParams(const YAML::Node & yaml_cfg);

        bool init(const YAML::Node & yaml_cfg);

        bool execute();

        bool ReadJson(const std::string file_dir);

        int factorial(int n);

        std::string zmq_server_callback(const std::string& request);

        void GetCostFunction(
            const int n_segment, 
            const int n_order, 
            const int cost_snap_jerk, 
            Eigen::SparseMatrix<double> &H
        );

        void GetConstraint(
            const int n_segment, 
            const int n_order, 
            const std::vector<naviparams::point> way_points, 
            Eigen::SparseMatrix<double> &A, 
            Eigen::VectorXd &lowerBound, 
            Eigen::VectorXd &upperBound, Eigen::VectorXd &f
        );

        bool GeneratorPointPlan(const std::vector<naviparams::point> &points_, nav_msgs::Path &traj_);

        nav_msgs::Path GetGlobalPath(){
            std::lock_guard<std::recursive_mutex> data_out_lock(globalpath_in_mutex_);
            return global_traj_;
        }

        void setPtr(std::shared_ptr<allsubscriber::AllSubscriber> _all_subsrciber){
            this->all_subsrciber_ = _all_subsrciber;
        }

		auto now() const {
			return std::chrono::system_clock::now();
		}

    private:
        std::atomic<bool> running_;

        YAML::Node filter_config_yaml_;

        mutable std::recursive_mutex globalpath_in_mutex_;

        std::shared_ptr<allsubscriber::AllSubscriber> all_subsrciber_;

        ZmqPublisher zmq_publisher_, zmq_server_;

        nav_msgs::Path global_path_pub_;

        nav_msgs::Path global_traj_;

        std::vector<geometry_msgs::Pose> global_way_points_;

        std::vector<naviparams::point> map_points_;

        bool task_point_flag_;

        int goal_point_index_;

    };


}