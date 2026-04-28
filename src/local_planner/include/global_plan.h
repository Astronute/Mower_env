#pragma once

#include <vector>
#include <string>
#include <queue>
#include <memory>
#include <chrono>
#include <map>
#include <thread>

#include "zmq_publisher.h"
#include "yaml-cpp/yaml.h"
#include "wall_rate.h"

namespace globalplanner
{
    class GlobalPlanner{
    public:
        GlobalPlanner();

        ~GlobalPlanner();

        bool loadParams();

        bool init();

        bool execute();

        std::string zmq_server_callback(const std::string& request);

    private:
        YAML::Node filter_config_yaml_;

        ZmqPublisher zmq_publisher_;

    };


}