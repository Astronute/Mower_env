#include "global_plan.h"

namespace globalplanner
{
    GlobalPlanner::GlobalPlanner():
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
                        for(const auto& p: coords){
                            geometry_msgs::PoseStamped pos;
                            pos.mutable_pose()->mutable_position()->set_x(p[0]);
                            pos.mutable_pose()->mutable_position()->set_y(p[1]);
                            pos.mutable_pose()->mutable_position()->set_z(0.0);
                            *global_path_pub_.add_poses() = pos;
                        }
                        std::cout << "global path size: " << global_path_pub_.poses_size() << std::endl;
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

    std::string GlobalPlanner::zmq_server_callback(const std::string& request){
        std::cout << "Received request: " << request << std::endl;

        std::string mission_dir = request;
        if(!ReadJson(request)){
            std::cout << "Failed to read mission file: " << mission_dir << std::endl;
            return "Failed to read mission file: " + mission_dir;
        }

        std::string response = "Response to: " + request;
        std::cout << "Sending response: " << response << std::endl;
        return response;
    }

    bool GlobalPlanner::execute(){
        WallRate rate(10);
        double robot2global_dis;
        
        while(running_){
            std::cout << "global planner running ..." << std::endl;
            
            rate.sleep();
        }

        return true;
    }

}