#ifndef ZMQ_SUBSCRIBER_H
#define ZMQ_SUBSCRIBER_H

#include <zmq.hpp>
#include <string>
#include <vector>
#include <thread>
#include <atomic>
#include <functional>
#include <iostream>

// 定义消息回调函数类型
using MessageCallback = std::function<void(const std::string& message, const std::string& topic)>;

struct SubscriberConfig {
    std::string address;
    std::vector<std::string> topics;
};

class ZmqSubscriber {
public:
    ZmqSubscriber();

    ~ZmqSubscriber();
    
    bool initialize(const std::vector<SubscriberConfig>& configs);

    void start();

    void stop();

    void setMessageCallback(MessageCallback callback);
    
    // 请求-回复模式方法
    bool initializeRequestClient(const std::string& server_address = "tcp://localhost:5558");
    std::string sendRequest(const std::string& request, int timeout_ms = 5000);
    
    void shutdown();

private:
    void subscriberWorker();

    void processMessage(zmq::socket_t& socket, int subscriber_id);

    // void handleDifferentMessageTypes(const std::string& message, int subscriber_id);

private:
    zmq::context_t context_;

    std::vector<SubscriberConfig> configs_;

    std::vector<zmq::socket_t> sub_sockets_;

    zmq::socket_t req_socket_;  // 用于请求-回复模式

    std::thread sub_thread_;

    std::atomic<bool> running_;

    MessageCallback message_callback_;

    bool initialized_;

    std::string server_address_;
};

#endif // ZMQ_SUBSCRIBER_H