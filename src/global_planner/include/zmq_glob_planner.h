#pragma once

#include <vector>
#include <string>
#include <queue>
#include <memory>
#include <chrono>
#include <mutex>

#include "geometry_msgs/pose.pb.h"
#include "zmq_publisher.h"
#include "zmq_subscriber.h"

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

    };

    class GlobalPlanner{
    public:
        explicit GlobalPlanner();

        ~GlobalPlanner();

        bool initialize();

        bool getInfoFromJSON(const std::string &file_dir);

        void writeWaypointToJSON(const std::string &file_dir, const std::vector<std::vector<double>> &waypoints);

        bool runGlobalMission(const MissionInfo &mission);

    private:
        std::mutex mtx_json_;

        std::mutex mtx_service_;
        
        MissionInfo mission_info_;
    
    };

}

