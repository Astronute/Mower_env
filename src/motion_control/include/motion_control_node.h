#pragma once

#include "zmq_controller.h"
#include "all_subscriber.h"
#include "yaml-cpp/yaml.h"

#include <thread>
#include <atomic>
#include <numeric>
#include <condition_variable>

class MotionControlNode{
    public:
        MotionControlNode();

        ~MotionControlNode();

        bool initialize();

        void SubscribeThread();

        void ControllerThread();

        void spin();

    private:
		std::atomic<bool> running_;

		std::condition_variable cv_;

		std::mutex mtx_;

        std::mutex config_mutex_;

        YAML::Node filter_config_yaml_;

        std::shared_ptr<allsubscriber::AllSubscriber> all_subscriber_;

        std::shared_ptr<CB::RosController> controller_;

        // thread
        std::thread subscribe_thread_;

        std::thread controller_thread_;

};