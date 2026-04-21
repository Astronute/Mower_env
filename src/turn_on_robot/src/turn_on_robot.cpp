#include "turn_on_robot.h"

#include <iostream>

namespace turn_on_robot{

    TurnOnRobot::TurnOnRobot():
        running_(true)
    {

    }

    TurnOnRobot::~TurnOnRobot(){
        running_ = false;
        timer_.stop();
        delete serial_fd_;
    }

    void TurnOnRobot::spin(){
        while(running_){
            char buf;
            int res = read(*serial_fd_, &buf, 1);
            if (res > 0){
                packet_unpack(buf);
                
            }
        }
    }

    bool TurnOnRobot::initialize(){

        if(!loadParams()){
            return false;
        }

        // init serial
        int fd = open(device_tty_.c_str(), O_RDWR); //获取串口设备描述符
        serial_fd_ = new int(fd);
        if (fd < 0){
            std::cout << "Fail to Open " << device_tty_ << " device" << std::endl;
            return false;
        }
        else{
            struct termios opt;
            tcflush(fd, TCIOFLUSH); //清空串口接收缓冲区
            tcgetattr(fd, &opt); // 获取串口参数 opt
            cfsetospeed(&opt, B115200); //设置串口输出波特率
            cfsetispeed(&opt, B115200); //设置串口输入波特率
            //设置数据位数
            opt.c_cflag &= ~CSIZE;
            opt.c_cflag |= CS8;
            //校验位
            opt.c_cflag &= ~PARENB;
            opt.c_iflag &= ~INPCK;
            //设置停止位
            opt.c_cflag &= ~CSTOPB;
            
            tcsetattr(fd, TCSANOW, &opt); //更新配置

            opt.c_iflag &= ~(INLCR); /*禁止将输入中的换行符NL映射为回车-换行CR*/
            opt.c_iflag &= ~(IXON | IXOFF | IXANY); //不要软件流控制
            opt.c_oflag &= ~OPOST;
            opt.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); //原始模式

            tcsetattr(fd, TCSANOW, &opt); //更新终端配置

            std::cout << "device" << device_tty_ <<" is set to 115200bps, 8N1" << std::endl;
        }

        // 定时器线程，串口下发控制信号
        const std::chrono::duration<double> timespan{1.0 / tty_frequency_};
        timer_.start(std::chrono::duration_cast<std::chrono::nanoseconds>(timespan), std::bind(&TurnOnRobot::TimerCallback, this));

    }

    bool TurnOnRobot::loadParams(){
        try{
            #ifdef USE_SIM
            robot_config_yaml_ = YAML::LoadFile("/home/tom/Mower_env/src/turn_on_robot/params/robot_params.yaml");
            #else
            robot_config_yaml_ = YAML::LoadFile("/home/kickpi/sim_ws/src/motion_control/params/filter_params.yaml");
            #endif
        } catch(const YAML::Exception& e){
            std::cout << "yaml parsing error: " << e.what() << std::endl;
            return false;
        }

        if(robot_config_yaml_["device_tty"]){
            device_tty_ = robot_config_yaml_["device_tty"].as<std::string>();
        }
        else{
            std::cout << "missing param 'device_tty'" << std::endl;
            return false;
        }
        if(robot_config_yaml_["tty_frequency"]){
            tty_frequency_ = robot_config_yaml_["tty_frequency"].as<double>();
        }
        else{
            std::cout << "missing param 'tty_frequency'" << std::endl;
            return false;
        }

        /*-------------------------------- zmq subscribe-----------------------------------------*/
        std::vector<SubscriberConfig> zmq_sub_cfgs;

        size_t port_ind = 0;
        bool more_params = false;
        do{
            std::stringstream ss;
            ss << "zmq_sub_port" << port_ind++;
            std::string port_id = ss.str();
            std::string port_addr;
            if(robot_config_yaml_[port_id]){
                more_params = true;
                port_addr = robot_config_yaml_[port_id].as<std::string>();
            }
            else{
                more_params = false;
            }

            if(more_params){
                SubscriberConfig sub_cfg;
                sub_cfg.address = port_addr;
                zmq_sub_cfgs.push_back(sub_cfg);
                std::cout << "turn_on_robot node subscriber " << port_id << " bind to: " << port_addr << std::endl;
            }
        }while(more_params);

        // twist subscribe
        {
            std::string twist_name = "ztwist";
            std::string twist_topic;
            if(robot_config_yaml_[twist_name]){
                twist_topic = robot_config_yaml_[twist_name].as<std::string>();
            }
            else{
                std::cout << "missing param 'ztwist'" << std::endl;
                return false;
            }

            int port_idx = 0;
            std::string str = robot_config_yaml_[twist_name + "_sub_port"].as<std::string>().substr(4);
            try {
                port_idx = std::stoi(str);
                if(port_idx > zmq_sub_cfgs.size()){
                    std::cout << twist_name << " port id out of range" << std::endl;
                    return false;
                }
            } catch (const std::exception& e) {
                std::cerr << str << " parse error: " << e.what() << std::endl;
            }

            topic_name_map_[twist_topic] = twist_name;
            zmq_sub_cfgs[port_idx].topics.push_back(twist_topic);
        }

        zmq_subscriber_.initialize(zmq_sub_cfgs);
        zmq_subscriber_.setMessageCallback([this](const std::string& msg, const std::string& topic) {
            this->zmq_message_callback(msg, topic);}
        );
        zmq_subscriber_.start();
        std::cout << "param load success." << std::endl;
        return true;
    }

    void TurnOnRobot::zmq_message_callback(const std::string& message, const std::string& topic){
        std::string topic_id = topic_name_map_[topic];
        
        if(topic.compare(0, 7, "/cmd_vel") == 0){
            std::shared_ptr<geometry_msgs::Twist> twist_ptr = std::make_shared<geometry_msgs::Twist>();
            if(twist_ptr->ParseFromArray(message.data(), message.size())){
                // serial
                std::lock_guard<std::mutex> lock(ctrl_mtx_);
                cmd_vel_x_ = twist_ptr->linear().x() * 1000; // mm/s
                cmd_vel_y_ = twist_ptr->linear().y() * 1000;
                cmd_vel_w_ = twist_ptr->angular().z() * 100; // *100rad/s
            }
            else{
                std::cout << topic_id << "(/cmd_vel) process failed" << std::endl;
            }
        }

    }

    void TurnOnRobot::TimerCallback(){
        {
            std::lock_guard<std::mutex> lock(ctrl_mtx_);
            sendCarControlCmd(serial_fd_, cmd_vel_x_, cmd_vel_y_, cmd_vel_w_);
        }
    }

}