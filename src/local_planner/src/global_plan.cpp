#include "global_plan.h"

namespace globalplanner
{
    GlobalPlanner::GlobalPlanner():
        task_point_flag_(false),
        goal_point_index_(0),
        running_(true)
    {

    }

    GlobalPlanner::~GlobalPlanner(){
        running_ = false;
    }

    bool GlobalPlanner::loadParams(){
        try{
            filter_config_yaml_ = YAML::LoadFile("/home/rpdzkj/Mower_env/src/local_planner/params/subscribe_params.yaml");
        } catch(const YAML::Exception& e){
            std::cout << "yaml parsing error: " << e.what() << std::endl;
            return false;
        }

        /*-------------------------------- zmq publisher-----------------------------------------*/

        if(filter_config_yaml_["zmq_server_port"]){
            std::string port = filter_config_yaml_["zmq_server_port"].as<std::string>();
            zmq_publisher_.initializeRequestHandler(port);
            zmq_publisher_.setRequestHandler([this](const std::string& request) -> std::string {
                return this->zmq_server_callback(request);
            });
            zmq_publisher_.startRequestHandler();
            std::cout << "mission execute server bind to: " << port << std::endl;
        }
        else{
            std::cout << "missing param 'zmq_server_port' " << std::endl;
            return false;
        }

        return true;
    }

    bool GlobalPlanner::init(){

        if(!loadParams()){
            std::cout << "Failed to load parameters!" << std::endl;
            return false;
        }

        return true;
    }

    bool GlobalPlanner::ReadJson(const std::string file_dir){
        // MissionInfo mission_info;

        std::ifstream load_file(file_dir);
        nlohmann::json json_tree;
        if(load_file.is_open()) {
            try {
                json_tree = nlohmann::json::parse(load_file);
                load_file.close();
                
                for(auto &feature :json_tree.at("features")){
                    if (feature.contains("properties")&&feature["properties"].contains("featureType") // 加载任务类型
                        && feature["properties"]["featureType"]=="taskConfig")
                    {
                        std::string task_type_name = feature["properties"]["taskTypeName"];
                        std::cout << task_type_name << std::endl;
                    }
                    else if(feature.contains("properties")&&feature["properties"].contains("featureType") // 加载全局路径
                        && feature["properties"]["featureType"]=="workingPath")
                    {
                        global_path_pub_.mutable_header()->set_frame_id("map");
                        global_path_pub_.mutable_header()->mutable_stamp()->CopyFrom(navicommon::systimeToProto(this->now()));
                        const auto& coords = feature["geometry"]["coordinates"][0];
                        global_path_pub_.clear_poses();
                        for(const auto& p: coords){
                            geometry_msgs::PoseStamped pos;
                            pos.mutable_pose()->mutable_position()->set_x(p[0]);
                            pos.mutable_pose()->mutable_position()->set_y(p[1]);
                            pos.mutable_pose()->mutable_position()->set_z(0.0);
                            *global_path_pub_.add_poses() = pos;
                        }
                        std::cout << "global path size: " << global_path_pub_.poses_size() << std::endl;
                    }
                    else if(feature.contains("properties")&&feature["properties"].contains("featureType") // 加载全局路径
                        && feature["properties"]["featureType"]=="waypoints")
                    {
                        const auto& coords = feature["geometry"]["coordinates"];
                        global_way_points_.clear();
                        for(const auto& p: coords){
                            geometry_msgs::Pose pos;
                            pos.mutable_position()->set_x(p[0]);
                            pos.mutable_position()->set_y(p[1]);
                            pos.mutable_position()->set_z(0.0);
                            global_way_points_.push_back(pos);
                        }

                    }
                }
            } catch (nlohmann::json::parse_error& e) {
                std::cerr << "解析错误: " << e.what() << std::endl;
                load_file.close();
            }
        }
        else {
            std::cerr << "cannot open: " << file_dir << std::endl;
            return false;
        }
        // mission_info.mission_dir = file_dir;
        // mission_info_ = mission_info;

        return true;
    }

    bool GlobalPlanner::GeneratorPointPlan(const std::vector<geometry_msgs::Pose> &points_, nav_msgs::Path &traj_){
        
    }

    std::string GlobalPlanner::zmq_server_callback(const std::string& request){
        std::cout << "Received request: " << request << std::endl;

        int subsrciber_flag = all_subsrciber_->getSensorState();
        subsrciber_flag = 0;
        std::string mission_dir = request;
        std::string response;
        task_point_flag_ = false;
        // 根据request选择起始目标点
        goal_point_index_ = 0;
        if(subsrciber_flag == 0){
            if(!ReadJson(request)){
                response = "Failed to read mission file: " + mission_dir;
            }
            else{
                response = "Mission file read successfully: " + mission_dir;
            }
        }
        else{
            response = "Subscriber state is not good, cannot execute mission: " + mission_dir;
        }


        std::cout << "Sending response: " << response << std::endl;
        return response;
    }

    bool GlobalPlanner::execute(){
        WallRate rate(10);
        double robot_x, robot_y, robot_v, robot_w, robot_yaw, s, robot2global_dis;
        nav_msgs::Path global_traj;

        while(running_){
            int subsrciber_flag = all_subsrciber_->getSensorState();
            subsrciber_flag = 0;
            if(subsrciber_flag == 0 && global_path_pub_.poses_size()){
                std::array<double, allsubscriber::STATE_SIZE> robot_state = all_subsrciber_->getState();
                robot_x = robot_state[allsubscriber::StateMemberX];
                robot_y = robot_state[allsubscriber::StateMemberY];
                robot_v = robot_state[allsubscriber::StateMemberVx];
                robot_w = robot_state[allsubscriber::StateMemberGz];
                robot_yaw = robot_state[allsubscriber::StateMemberYaw];
                robot2global_dis = hypot(robot_x - global_path_pub_.poses(goal_point_index_).pose().position().x(), 
                                        robot_y - global_path_pub_.poses(goal_point_index_).pose().position().y());

                map_points_.clear();
                if(robot2global_dis > 0.3){
                    geometry_msgs::Pose pos;
                    pos.mutable_position()->set_x(robot_x);
                    pos.mutable_position()->set_y(robot_y);
                    pos.mutable_position()->set_z(0.0);
                    map_points_.push_back(pos);

                    pos.mutable_position()->set_x(global_path_pub_.poses(goal_point_index_).pose().position().x());
                    pos.mutable_position()->set_y(global_path_pub_.poses(goal_point_index_).pose().position().y());
                    pos.mutable_position()->set_z(0.0);
                    map_points_.push_back(pos);
                }
                else{
                    geometry_msgs::Pose pos;
                    pos.mutable_position()->set_x(global_path_pub_.poses(goal_point_index_).pose().position().x());
                    pos.mutable_position()->set_y(global_path_pub_.poses(goal_point_index_).pose().position().y());
                    pos.mutable_position()->set_z(0.0);
                    map_points_.push_back(pos);

                    if(goal_point_index_ + 1 < global_path_pub_.poses_size()){
                        goal_point_index_++;
                        pos.mutable_position()->set_x(global_path_pub_.poses(goal_point_index_).pose().position().x());
                        pos.mutable_position()->set_y(global_path_pub_.poses(goal_point_index_).pose().position().y());
                        pos.mutable_position()->set_z(0.0);
                        map_points_.push_back(pos);
                    }
                    else{
                        map_points_.clear();
                    }
                }

                if(!map_points_.empty() && GeneratorPointPlan(map_points_, global_traj)){
                    
                }
                else{
                    global_traj.clear_poses();
                }

            }

            rate.sleep();
        }

        return true;
    }

}