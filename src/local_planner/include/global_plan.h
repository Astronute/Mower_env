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

		auto now() const {
			return std::chrono::system_clock::now();
		}

    private:
        std::atomic<bool> running_;

        YAML::Node filter_config_yaml_;

        ZmqPublisher zmq_publisher_;

        nav_msgs::Path global_path_pub_;

    };


}