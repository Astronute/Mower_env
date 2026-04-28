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
            std::cout << "TurnOnRobot loadParams failed" << std::endl;
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

        return true;
    }

    bool TurnOnRobot::loadParams(){
        try{
            #ifdef USE_SIM
            robot_config_yaml_ = YAML::LoadFile("/home/tom/Mower_env/src/turn_on_robot/params/robot_params.yaml");
            #else
            robot_config_yaml_ = YAML::LoadFile("/home/rpdzkj/Mower_env/src/turn_on_robot/params/robot_params.yaml");
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
        if(robot_config_yaml_["base_link_frame_id"]){
            base_link_frame_id_ = robot_config_yaml_["base_link_frame_id"].as<std::string>();
        }
        else{
            std::cout << "missing param 'base_link_frame_id' " << std::endl;
            return false;
        }
        if(robot_config_yaml_["world_frame_id"]){
            world_frame_id_ = robot_config_yaml_["world_frame_id"].as<std::string>();
        }
        else{
            std::cout << "missing param 'world_frame_id' " << std::endl;
            return false;
        }
        // load covariance
        sensor_covariance_map_["odom_twist"] = loadCovariance("odom_twist");
        sensor_covariance_map_["imu_odom"] = loadCovariance("imu_odom");

        /*-------------------------------- zmq publisher-----------------------------------------*/

        if(robot_config_yaml_["zmq_pub_port"]){
            std::string port = robot_config_yaml_["zmq_pub_port"].as<std::string>();
            zmq_publisher_.initialize(port);
            std::cout << "turn on robot node publisher bind to: " << port << std::endl;
        }
        else{
            std::cout << "missing param 'zmq_pub_port' " << std::endl;
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

    std::vector<double> TurnOnRobot::loadCovariance(const std::string & sensor_name){
        std::vector<double> sensor_cov(36, 0);
        const std::string sensor_cov_name = sensor_name + "_covariance";
        const YAML::Node& cfg_array = robot_config_yaml_[sensor_cov_name];
        if(cfg_array.IsSequence() && cfg_array.size()==36){
            for(int i=0; i<36; ++i){
                sensor_cov[i] = cfg_array[i].as<double>();
            }
        }
        return sensor_cov;
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

    /*缓存每一帧数据，并缓存下来*/
    void TurnOnRobot::packet_unpack(uint8_t _buf)
    {
        static uint8_t uart_flag = 1;
        static uint8_t s_uartBuf[100];
        static uint8_t s_len = 0;
        if (_buf == 0xFD)
        {
            s_uartBuf[0] = 0xFD;
            uart_flag = 1;
            s_len++;
        }
        else
        {
            if (uart_flag == 1)
            {
                if (s_len > s_uartBuf[1] + 1)
                {
                    s_uartBuf[s_len] = _buf;
                    s_len++;

                    carInfoParse(s_uartBuf, s_len);
                    uart_flag = 0;
                    s_len = 0;
                    return;
                }
                else
                {
                    s_uartBuf[s_len] = _buf;
                    s_len++;
                }
            }
        }
    }

    /*根据缓存的数据，分别存入各个缓冲区*/
    void TurnOnRobot::carInfoParse(uint8_t *_buf, uint8_t _len)
    {
        //根据异或校验判断数据是否存在接收错误
        if (_buf[_len - 1] == xor_check(&_buf[2], _len - 3))
        {
            switch (_buf[2]){
                case 0x02:{ // 速度、转向
                    g_tCarMoveInfo.x_dir = _buf[3];
                    g_tCarMoveInfo.x_lineSpeed = _buf[4] << 8 | _buf[5];
                    g_tCarMoveInfo.y_dir = _buf[6];
                    g_tCarMoveInfo.y_lineSpeed = _buf[7] << 8 | _buf[8];
                    g_tCarMoveInfo.steerDir = _buf[9];
                    g_tCarMoveInfo.steerAngle = _buf[10] << 8 | _buf[11];

                    geometry_msgs::TwistWithCovarianceStamped twist;
                    twist.mutable_header()->set_frame_id(base_link_frame_id_);
                    SysTimePoint sys_now = this->now();
                    twist.mutable_header()->mutable_stamp()->CopyFrom(systimeToProto(sys_now));
                    double vel_x = g_tCarMoveInfo.x_lineSpeed * 0.001;
                    double vel_y = g_tCarMoveInfo.y_lineSpeed * 0.001;
                    double gyro_z = g_tCarMoveInfo.steerAngle * 0.01;
                    if(g_tCarMoveInfo.x_dir == 0x01){
                        vel_x = -1.0 * vel_x;
                    }
                    if(g_tCarMoveInfo.y_dir == 0x01){
                        vel_y = -1.0 * vel_y;
                    }
                    if(g_tCarMoveInfo.steerDir == 0x01){
                        gyro_z = -1.0 * gyro_z;
                    }
                    twist.mutable_twist()->mutable_twist()->mutable_linear()->set_x(vel_x);
                    twist.mutable_twist()->mutable_twist()->mutable_linear()->set_y(vel_y);
                    twist.mutable_twist()->mutable_twist()->mutable_linear()->set_z(0.0);
                    twist.mutable_twist()->mutable_twist()->mutable_angular()->set_x(0.0);
                    twist.mutable_twist()->mutable_twist()->mutable_angular()->set_y(0.0);
                    twist.mutable_twist()->mutable_twist()->mutable_angular()->set_z(gyro_z);
                    
                    if(!sensor_covariance_map_["odom_twist"].empty()){
                        for(double val: sensor_covariance_map_["odom_twist"]){
                            twist.mutable_twist()->mutable_covariance()->Add(val);
                        }
                        std::string serialized_data;
                        twist.SerializeToString(&serialized_data);
                        zmq_publisher_.publishMessage("/codbot/twist", serialized_data);
                        // std::cout << "vel_x: " << vel_x << " gyro_z: " << gyro_z << std::endl;
                    }
                    else{
                        std::cout << "odom_twist covariance empty message not published" << std::endl;
                    }

                    
                    break;
                }
                case 0x03:{ //电机转速
                    g_tCarMotorInfo.motor1Speed = _buf[3] << 8 | _buf[4];
                    g_tCarMotorInfo.motor2Speed = _buf[5] << 8 | _buf[6];
                    g_tCarMotorInfo.motor3Speed = _buf[7] << 8 | _buf[8];
                    g_tCarMotorInfo.motor4Speed = _buf[9] << 8 | _buf[10];

                    break;
                }
                case 0x04:{ //电压
                    g_tCarBatteryInfo.voltage = _buf[3];
                    //printf("%x\n", g_tCarBatteryInfo.voltage );
                    break;
                }
                case 0x05:{ //IMU pitch roll yaw
                    g_tCarImuAttitudeInfo.pitchSymbol = _buf[3];
                    g_tCarImuAttitudeInfo.pitch = _buf[4] << 8 | _buf[5];
                    g_tCarImuAttitudeInfo.rollSymbol = _buf[6];
                    g_tCarImuAttitudeInfo.roll = _buf[7] << 8 | _buf[8];
                    g_tCarImuAttitudeInfo.yawSymbol = _buf[9];
                    g_tCarImuAttitudeInfo.yaw = _buf[10] << 8 | _buf[11];

                    geometry_msgs::PoseWithCovarianceStamped pose_msg;
                    pose_msg.mutable_header()->set_frame_id(base_link_frame_id_);
                    SysTimePoint sys_now = this->now();
                    pose_msg.mutable_header()->mutable_stamp()->CopyFrom(systimeToProto(sys_now));
                    double pitch = g_tCarImuAttitudeInfo.pitch * 0.01 * M_PI / 180.0;
                    double roll = g_tCarImuAttitudeInfo.roll * 0.01 * M_PI / 180.0;
                    double yaw = g_tCarImuAttitudeInfo.yaw * 0.01 * M_PI / 180.0;
                    if(g_tCarImuAttitudeInfo.pitchSymbol == 0x01){
                        pitch = -1.0 * pitch;
                    }
                    if(g_tCarImuAttitudeInfo.rollSymbol == 0x01){
                        roll = -1.0 * roll;
                    }
                    if(g_tCarImuAttitudeInfo.yawSymbol == 0x01){
                        yaw = -1.0 * yaw;
                    }

                    geometry_msgs::Quaternion quat;
                    RPYToQuat(roll, pitch, yaw, quat);
                    pose_msg.mutable_pose()->mutable_pose()->mutable_orientation()->CopyFrom(quat);
                    if(!sensor_covariance_map_["imu_odom"].empty()){
                        for(double val: sensor_covariance_map_["imu_odom"]){
                            pose_msg.mutable_pose()->mutable_covariance()->Add(val);
                        }
                        std::string serialized_data;
                        pose_msg.SerializeToString(&serialized_data);
                        zmq_publisher_.publishMessage("/codbot/pose", serialized_data);
                        // std::cout << "yaw: " << yaw << std::endl;
                    }
                    else{
                        std::cout << "imu_odom covariance empty message not published" << std::endl;
                    }
                }
                case 0x06:{ //imu 原始数据
                    g_tCarImuRawInfo.gyroxSymbol = _buf[3];
                    g_tCarImuRawInfo.gyrox = _buf[4] << 8 | _buf[5];
                    g_tCarImuRawInfo.gyroySymbol = _buf[6];
                    g_tCarImuRawInfo.gyroy = _buf[7] << 8 | _buf[8];
                    g_tCarImuRawInfo.gyrozSymbol = _buf[9];
                    g_tCarImuRawInfo.gyroz = _buf[10] << 8 | _buf[11];
                    g_tCarImuRawInfo.accelxSymbol = _buf[12];
                    g_tCarImuRawInfo.accelx = _buf[13] << 8 | _buf[14];
                    g_tCarImuRawInfo.accelySymbol = _buf[15];
                    g_tCarImuRawInfo.accely = _buf[16] << 8 | _buf[17];
                    g_tCarImuRawInfo.accelzSymbol = _buf[18];
                    g_tCarImuRawInfo.accelz = _buf[19] << 8 | _buf[20];
                    g_tCarImuRawInfo.quatwSymbol = _buf[21];
                    g_tCarImuRawInfo.quatw = _buf[22] << 8 | _buf[23];
                    g_tCarImuRawInfo.quatxSymbol = _buf[24];
                    g_tCarImuRawInfo.quatx = _buf[25] << 8 | _buf[26];
                    g_tCarImuRawInfo.quatySymbol = _buf[27];
                    g_tCarImuRawInfo.quaty = _buf[28] << 8 | _buf[29];
                    g_tCarImuRawInfo.quatzSymbol = _buf[30];
                    g_tCarImuRawInfo.quatz = _buf[31] << 8 | _buf[32];
                    break;
                }
                case 0x07:{ //车辆类型
                    g_tCarTypeInfo.carType = _buf[3];
                    break;
                }
                default:{
                    break;
                }
            }
        }

    }

    void TurnOnRobot::TimerCallback(){
        {
            std::lock_guard<std::mutex> lock(ctrl_mtx_);
            // sendCarControlCmd(serial_fd_, cmd_vel_x_, cmd_vel_y_, cmd_vel_w_);
        }
    }

}
