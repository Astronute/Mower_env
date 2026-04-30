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

#include "geometry_msgs/pose.pb.h"
#include "pnc_msgs/point_vec.pb.h"
#include "pnc_msgs/planning_trajectory.pb.h"
#include "nav_msgs/path.pb.h"

#include "zmq_publisher.h"
#include "yaml-cpp/yaml.h"
#include "nlohmann/json.hpp"
#include "wall_rate.h"
#include "common.h"
#include "all_subscriber.h"

namespace globalplanner
{
    class GlobalPlanner{
    public:
        GlobalPlanner();

        ~GlobalPlanner();

        bool loadParams();

        bool init();

        bool execute();

        bool ReadJson(const std::string file_dir);

        std::string zmq_server_callback(const std::string& request);

        bool GeneratorPointPlan(const std::vector<geometry_msgs::Pose> &points_, nav_msgs::Path &traj_);

        void setPtr(std::shared_ptr<allsubscriber::AllSubscriber> _all_subsrciber){
            this->all_subsrciber_ = _all_subsrciber;
        }

		auto now() const {
			return std::chrono::system_clock::now();
		}

    private:
        std::atomic<bool> running_;

        std::shared_ptr<allsubscriber::AllSubscriber> all_subsrciber_;

        YAML::Node filter_config_yaml_;

        ZmqPublisher zmq_publisher_;

        nav_msgs::Path global_path_pub_;

        std::vector<geometry_msgs::Pose> global_way_points_;

        std::vector<geometry_msgs::Pose> map_points_;

        bool task_point_flag_;

        int goal_point_index_;

    };


}