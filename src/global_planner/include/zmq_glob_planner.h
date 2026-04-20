#pragma once

#include <vector>
#include <string>
#include <queue>
#include <memory>
#include <chrono>
#include <mutex>
#include <atomic>
#include <condition_variable>

#include "geometry_msgs/pose.pb.h"
#include "zmq_publisher.h"
#include "zmq_subscriber.h"

#include "fields_cover_planner.h"
#include "nlohmann/json.hpp"

namespace global_planner{

    struct MissionInfo{
        size_t task_type;

        double working_rotate;

        double working_width;

        geometry_msgs::Point start_point;

        geometry_msgs::Point home_point;

        std::vector<geometry_msgs::Point> boundary;

        std::vector<std::vector<geometry_msgs::Point>> obstacles;

        std::vector<geometry_msgs::Point> waypoints;

        std::string mission_dir;

    };

    class GlobalPlanner{
    public:
        explicit GlobalPlanner();

        ~GlobalPlanner();

        bool initialize();

        void spin();

        bool getInfoFromJSON(const std::string &file_dir);

        bool writeWaypointsToJSON(const std::string &file_dir, const std::vector<std::vector<double>> &waypoints);

        void writePathToJSON(const std::string &file_dir, const std::vector<std::vector<double>> &path);

        void runGlobalMission(const MissionInfo &mission);

    private:
		std::atomic<bool> running_;

		std::condition_variable cv_;

        std::mutex mtx_;

        std::mutex mtx_json_;

        std::mutex mtx_service_;
        
        MissionInfo mission_info_;
    
    };

}

