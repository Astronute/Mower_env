#include "zmq_publisher.h"
#include <cstring> // 对于 memcpy

// 传统目录操作方法
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>

ZmqPublisher::ZmqPublisher() 
    : context_(1), 
      pub_socket_(context_, ZMQ_PUB), 
      rep_socket_(context_, ZMQ_REP),
      bind_address_("tcp://*:5557"),
      reply_address_("tcp://*:5558"),
      initialized_(false),
      request_handler_running_(false) {
}

ZmqPublisher::~ZmqPublisher() {
    shutdown();
}

bool ZmqPublisher::createIpcDirectory(const std::string& ipc_address) {
    std::string ipc_path = extractIpcPath(ipc_address);
    if (ipc_path.empty()) {
        std::cerr << "Invalid IPC address: " << ipc_address << std::endl;
        return false;
    }
    
    // 找到最后一个斜杠的位置
    size_t last_slash = ipc_path.find_last_of('/');
    if (last_slash == std::string::npos) {
        // 没有斜杠，说明在当前目录，不需要创建
        std::cout << "IPC file in current directory, no need to create parent directory" << std::endl;
        return true;
    }
    
    // 提取目录路径
    std::string dir_path = ipc_path.substr(0, last_slash);
    
    // 检查目录是否已存在
    struct stat info;
    if (stat(dir_path.c_str(), &info) == 0) {
        if (S_ISDIR(info.st_mode)) {
            std::cout << "IPC directory already exists: " << dir_path << std::endl;
            return true;
        } else {
            std::cerr << "Path exists but is not a directory: " << dir_path << std::endl;
            return false;
        }
    }
    
    // 创建目录（递归创建）
    std::string current_path;
    size_t pos = 0;
    
    // 处理绝对路径
    if (dir_path[0] == '/') {
        current_path = "/";
        pos = 1;
    }
    
    while (pos < dir_path.length()) {
        size_t next_slash = dir_path.find('/', pos);
        std::string segment;
        
        if (next_slash == std::string::npos) {
            segment = dir_path.substr(pos);
            pos = dir_path.length();
        } else {
            segment = dir_path.substr(pos, next_slash - pos);
            pos = next_slash + 1;
        }
        
        if (segment.empty()) continue;
        
        if (current_path.empty() || current_path == "/") {
            current_path += segment;
        } else {
            current_path += "/" + segment;
        }
        
        // 检查并创建目录
        if (stat(current_path.c_str(), &info) != 0) {
            if (mkdir(current_path.c_str(), 0755) != 0 && errno != EEXIST) {
                std::cerr << "Failed to create directory " << current_path 
                         << ": " << strerror(errno) << std::endl;
                return false;
            }
            std::cout << "Created directory: " << current_path << std::endl;
        }
    }
    
    return true;
}

std::string ZmqPublisher::extractIpcPath(const std::string& ipc_address) {
    // IPC地址格式: "ipc:///path/to/file.ipc"
    const std::string ipc_prefix = "ipc://";
    
    if (ipc_address.find(ipc_prefix) != 0) {
        return ""; // 不是IPC地址
    }
    
    return ipc_address.substr(ipc_prefix.length());
}

bool ZmqPublisher::initialize(const std::string& bind_address) {
    bind_address_ = bind_address;
    try {
        // 如果是IPC地址，确保目录存在
        if (bind_address_.find("ipc://") == 0) {
            if (!createIpcDirectory(bind_address_)) {
                std::cerr << "Failed to create IPC directory for: " << bind_address_ << std::endl;
                return false;
            }
        }
        
        pub_socket_.bind(bind_address_);
        std::cout << "Publisher bound to: " << bind_address_ << std::endl;
        initialized_ = true;
        return true;
    } catch (const zmq::error_t& e) {
        std::cerr << "Publisher bind failed: " << e.what() << std::endl;
        return false;
    } catch (const std::exception& e) {
        std::cerr << "Publisher initialization failed: " << e.what() << std::endl;
        return false;
    }
}

bool ZmqPublisher::initializeRequestHandler(const std::string& reply_address) {
    try {
        reply_address_ = reply_address;
        
        // 如果是IPC地址，确保目录存在
        if (reply_address_.find("ipc://") == 0) {
            if (!createIpcDirectory(reply_address_)) {
                std::cerr << "Failed to create IPC directory for: " << reply_address_ << std::endl;
                return false;
            }
        }
        
        rep_socket_.bind(reply_address_);
        std::cout << "Request handler bound to: " << reply_address_ << std::endl;
        return true;
    } catch (const zmq::error_t& e) {
        std::cerr << "Request handler bind failed: " << e.what() << std::endl;
        return false;
    }
}

void ZmqPublisher::setRequestHandler(RequestHandler handler) {
    request_handler_ = handler;
}

void ZmqPublisher::startRequestHandler() {
    if (!request_handler_) {
        std::cerr << "No request handler set!" << std::endl;
        return;
    }
    
    request_handler_running_ = true;
    request_thread_ = std::thread(&ZmqPublisher::requestHandlerWorker, this);
    std::cout << "Request handler started..." << std::endl;
}

void ZmqPublisher::stopRequestHandler() {
    request_handler_running_ = false;
    if (request_thread_.joinable()) {
        request_thread_.join();
    }
    std::cout << "Request handler stopped." << std::endl;
}

void ZmqPublisher::requestHandlerWorker() {
    while (request_handler_running_) {
        try {
            zmq::message_t request;
            
            // 等待请求
            auto result = rep_socket_.recv(request, zmq::recv_flags::none);
            if (result.has_value() && result.value() > 0) {
                std::string request_str(static_cast<char*>(request.data()), request.size());
                std::cout << "[Request Handler] Received request: " << request_str << std::endl;
                
                // 处理请求
                std::string response = request_handler_(request_str);
                
                // 发送回复
                zmq::message_t reply(response.size());
                memcpy(reply.data(), response.data(), response.size());
                rep_socket_.send(reply, zmq::send_flags::none);
                
                std::cout << "[Request Handler] Sent response: " << response << std::endl;
            }
        } catch (const zmq::error_t& e) {
            if (request_handler_running_) {
                std::cerr << "Request handling failed: " << e.what() << std::endl;
            }
        }
    }
}

void ZmqPublisher::publishMessage(const std::string& topic, const std::string& message) {
    if (!initialized_) {
        std::cerr << "Publisher not initialized!" << std::endl;
        return;
    }
    
    try {
        // std::string full_message = topic + message;
        pub_socket_.send(zmq::buffer(topic), zmq::send_flags::sndmore);
        pub_socket_.send(zmq::buffer(message));
        // std::cout << "[Publisher] Sent: " << full_message << std::endl;
    } catch (const zmq::error_t& e) {
        std::cerr << "Publish failed: " << e.what() << std::endl;
    }
}

void ZmqPublisher::publishMultipartMessage(const std::vector<std::string>& parts) {
    if (!initialized_) {
        std::cerr << "Publisher not initialized!" << std::endl;
        return;
    }
    
    try {
        for (size_t i = 0; i < parts.size(); ++i) {
            zmq::message_t part(parts[i].size());
            memcpy(part.data(), parts[i].data(), parts[i].size());
            
            zmq::send_flags flags = (i < parts.size() - 1) ? 
                                   zmq::send_flags::sndmore : 
                                   zmq::send_flags::none;
            
            pub_socket_.send(part, flags);
        }
        std::cout << "[Publisher] Sent multipart message with " << parts.size() << " parts" << std::endl;
    } catch (const zmq::error_t& e) {
        std::cerr << "Multipart publish failed: " << e.what() << std::endl;
    }
}

void ZmqPublisher::shutdown() {
    stopRequestHandler();
    
    if (initialized_) {
        pub_socket_.close();
        rep_socket_.close();
        context_.close();
        initialized_ = false;
        std::cout << "Publisher shutdown completed." << std::endl;
    }
}