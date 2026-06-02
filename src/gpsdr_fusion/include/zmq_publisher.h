#ifndef ZMQ_PUBLISHER_H
#define ZMQ_PUBLISHER_H

#include <string>
#include <vector>
#include <iostream>
#include <zmq.hpp>
#include <thread>
#include <atomic>
#include <functional>

// 定义请求处理回调函数类型
using RequestHandler = std::function<std::string(const std::string& request)>;

class ZmqPublisher {
public:
    ZmqPublisher();
    ~ZmqPublisher();
    
    bool initialize(const std::string& bind_address = "tcp://*:5557");
    void publishMessage(const std::string& topic, const std::string& message);
    void publishMultipartMessage(const std::vector<std::string>& parts);
    
    // 请求-回复模式方法
    bool initializeRequestHandler(const std::string& reply_address = "tcp://*:5558");
    void setRequestHandler(RequestHandler handler);
    void startRequestHandler();
    void stopRequestHandler();
    
    void shutdown();

private:
    bool createIpcDirectory(const std::string& ipc_address);
    std::string extractIpcPath(const std::string& ipc_address);
    void requestHandlerWorker();
    
    zmq::context_t context_;
    zmq::socket_t pub_socket_;
    zmq::socket_t rep_socket_;  // 用于请求-回复模式
    std::string bind_address_;
    std::string reply_address_;
    bool initialized_;
    
    // 请求-回复模式相关成员
    std::thread request_thread_;
    std::atomic<bool> request_handler_running_;
    RequestHandler request_handler_;
};

#endif // ZMQ_PUBLISHER_H