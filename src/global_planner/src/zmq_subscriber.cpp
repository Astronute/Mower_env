#include "zmq_subscriber.h"
#include <chrono> // 添加这个头文件

ZmqSubscriber::ZmqSubscriber() 
    : context_(1), 
      req_socket_(context_, ZMQ_REQ),
      running_(false),
      initialized_(false),
      server_address_("tcp://localhost:5558") {
}

ZmqSubscriber::~ZmqSubscriber() {
    stop();
    shutdown();
}

bool ZmqSubscriber::initialize(const std::vector<SubscriberConfig>& configs) {
    try {
        // 为每个配置创建socket并设置订阅
        for (size_t i = 0; i < configs.size(); ++i) {
            sub_sockets_.emplace_back(context_, ZMQ_SUB);
            sub_sockets_[i].connect(configs[i].address);
            
            // 订阅该地址的所有主题 - 使用新的API
            for (const auto& topic : configs[i].topics) {
                sub_sockets_[i].set(zmq::sockopt::subscribe, topic);
            }
            
            std::cout << "Subscriber " << (i + 1) << " connected to: " << configs[i].address << std::endl;
            std::cout << "Topics: ";
            for (const auto& topic : configs[i].topics) {
                std::cout << topic << " ";
            }
            std::cout << std::endl;
        }
        
        initialized_ = true;
        return true;
    } catch (const zmq::error_t& e) {
        std::cerr << "Subscriber initialization failed: " << e.what() << std::endl;
        return false;
    }
}

bool ZmqSubscriber::initializeRequestClient(const std::string& server_address) {
    try {
        server_address_ = server_address;
        req_socket_.connect(server_address_);
        std::cout << "Request client connected to: " << server_address_ << std::endl;
        return true;
    } catch (const zmq::error_t& e) {
        std::cerr << "Request client connection failed: " << e.what() << std::endl;
        return false;
    }
}

std::string ZmqSubscriber::sendRequest(const std::string& request, int timeout_ms) {
    try {
        // 设置超时 - 使用新的API
        req_socket_.set(zmq::sockopt::rcvtimeo, timeout_ms);
        req_socket_.set(zmq::sockopt::sndtimeo, timeout_ms);
        
        // 发送请求
        zmq::message_t req_msg(request.size());
        memcpy(req_msg.data(), request.data(), request.size());
        req_socket_.send(req_msg, zmq::send_flags::none);
        
        std::cout << "[Request Client] Sent request: " << request << std::endl;
        
        // 接收回复
        zmq::message_t reply_msg;
        auto result = req_socket_.recv(reply_msg, zmq::recv_flags::none);
        
        if (result.has_value() && result.value() > 0) {
            std::string reply_str(static_cast<char*>(reply_msg.data()), reply_msg.size());
            std::cout << "[Request Client] Received reply: " << reply_str << std::endl;
            return reply_str;
        } else {
            std::cerr << "[Request Client] Request timeout or error" << std::endl;
            return "";
        }
    } catch (const zmq::error_t& e) {
        std::cerr << "Request failed: " << e.what() << std::endl;
        return "";
    }
}

void ZmqSubscriber::start() {
    if (!initialized_) {
        std::cerr << "Subscriber not initialized!" << std::endl;
        return;
    }
    
    running_ = true;
    sub_thread_ = std::thread(&ZmqSubscriber::subscriberWorker, this);
    std::cout << "ZMQ Subscriber started..." << std::endl;
}

void ZmqSubscriber::stop() {
    running_ = false;
    if (sub_thread_.joinable()) {
        sub_thread_.join();
    }
    std::cout << "ZMQ Subscriber stopped." << std::endl;
}

void ZmqSubscriber::setMessageCallback(MessageCallback callback) {
    message_callback_ = callback;
}

void ZmqSubscriber::subscriberWorker() {
    std::vector<zmq::pollitem_t> poll_items;
    
    // 设置轮询项
    for (auto& socket : sub_sockets_) {
        zmq::pollitem_t item = { 
            static_cast<void*>(socket), 
            0, 
            ZMQ_POLLIN, 
            0 
        };
        poll_items.push_back(item);
    }
    
    while (running_) {
        // 轮询所有订阅socket，等待100ms - 使用新的API
        zmq::poll(poll_items, std::chrono::milliseconds(100));
        
        // 检查每个socket是否有消息
        for (size_t i = 0; i < sub_sockets_.size(); ++i) {
            if (poll_items[i].revents & ZMQ_POLLIN) {
                processMessage(sub_sockets_[i], i + 1);
            }
        }
    }
}

void ZmqSubscriber::processMessage(zmq::socket_t& socket, int subscriber_id) {
    zmq::message_t topic_msg;
    zmq::message_t content_msg;

    try {
        // 接收消息
        auto topic_recv_result = socket.recv(topic_msg, zmq::recv_flags::none);
        auto content_recv_result = socket.recv(content_msg, zmq::recv_flags::none);
        if (!topic_recv_result || !content_recv_result) {
            std::cout << "Failed to receive message" << std::endl;
        }
        else{
            std::string topic(static_cast<char*>(topic_msg.data()), topic_msg.size());
            std::string msg_content(static_cast<char*>(content_msg.data()), content_msg.size());
            
            // 显示接收到的消息
            // std::cout << "[Subscriber " << subscriber_id << "] Received: " << topic << std::endl;
            
            // 调用用户自定义回调
            if (message_callback_) {
                message_callback_(msg_content, topic);
            }
        }
    } catch (const zmq::error_t& e) {
        std::cerr << "Message receive failed: " << e.what() << std::endl;
    }
}

void ZmqSubscriber::shutdown() {
    if (initialized_) {
        for (auto& socket : sub_sockets_) {
            socket.close();
        }
        req_socket_.close();
        context_.close();
        initialized_ = false;
        std::cout << "Subscriber shutdown completed." << std::endl;
    }
}