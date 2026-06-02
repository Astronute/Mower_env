#include "zmq_glob_planner.h"

#include <fstream>
#include <iostream>
#include <algorithm>



namespace global_planner{

    GlobalPlanner::GlobalPlanner():
        running_(true)
    {

    }

    GlobalPlanner::~GlobalPlanner(){
        running_ = false;
    }

    bool GlobalPlanner::initialize(){

        if(!loadParams()){
            std::cout << "Failed to load parameters!" << std::endl;
            return false;
        }

        return true;
    }

    bool GlobalPlanner::loadParams(){

        try{
            filter_config_yaml_ = YAML::LoadFile("/home/rpdzkj/Mower_env/src/global_planner/params/global_params.yaml");
        } catch(const YAML::Exception& e){
            std::cout << "yaml parsing error: " << e.what() << std::endl;
            return false;
        }

        if(filter_config_yaml_["zmq_server_port"]){
            std::string port = filter_config_yaml_["zmq_server_port"].as<std::string>();
            zmq_publisher_.initializeRequestHandler(port);
            zmq_publisher_.setRequestHandler([this](const std::string& request) -> std::string {
                return this->zmq_server_callback(request);
            });
            zmq_publisher_.startRequestHandler();
            std::cout << "mission create server bind to: " << port << std::endl;
        }
        else{
            std::cout << "missing param 'zmq_server_port'" << std::endl;
            return false;
        }

        return true;
    }

    void GlobalPlanner::spin(){
        std::unique_lock<std::mutex> lock(mtx_);
        cv_.wait(lock, [this]() { return !running_; });
    }

    bool GlobalPlanner::getInfoFromJSON(const std::string &file_dir, MissionInfo &mission_info){
        std::lock_guard<std::mutex> lock(mtx_json_);

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
                        mission_info.task_type = feature["properties"]["taskType"];
                        mission_info.working_rotate = feature["properties"]["workingRotate"];
                        mission_info.working_width = feature["properties"]["workingWidth"];
                        std::cout << task_type_name << std::endl;
                    }
                    else if(feature.contains("properties")&&feature["properties"].contains("featureType") // 起始点
                        && feature["properties"]["featureType"]=="startPoint")
                    {
                        geometry_msgs::Point start_point;
                        start_point.set_x(feature["geometry"]["coordinates"][0]);
                        start_point.set_y(feature["geometry"]["coordinates"][1]);
                        start_point.set_z(feature["geometry"]["coordinates"][2]);
                        mission_info.start_point = start_point;
                    }
                    else if(feature.contains("properties")&&feature["properties"].contains("featureType") // 返航点
                        && feature["properties"]["featureType"]=="homePoint")
                    {
                        geometry_msgs::Point home_point;
                        home_point.set_x(feature["geometry"]["coordinates"][0]);
                        home_point.set_y(feature["geometry"]["coordinates"][1]);
                        home_point.set_z(feature["geometry"]["coordinates"][2]);
                        mission_info.home_point = home_point;
                    }
                    else if(feature.contains("geometry")&&feature["geometry"].contains("type") // 作业边界
                        && feature["geometry"]["type"]=="boundary")
                    {
                        std::vector<geometry_msgs::Point> boundary;
                        for(auto cods: feature["geometry"]["coordinates"]){
                            if(cods.size() < 3){
                                std::cout << "getInfoFromJSON Error: boundary waypoint size < 3" << std::endl;
                                return false;
                            }
                            geometry_msgs::Point p;
                            p.set_x(cods[0].get<double>());
                            p.set_y(cods[1].get<double>());
                            p.set_z(cods[2].get<double>());
                            boundary.push_back(p);
                        }
                        mission_info.boundary = boundary;
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
        mission_info.mission_dir = file_dir;
        mission_info_ = mission_info;

        return true;
    }

    bool GlobalPlanner::writeWaypointsToJSON(const std::string &file_dir, const std::vector<std::vector<double>> &waypoints){
        std::lock_guard<std::mutex> lock(mtx_json_);
        bool w_flag = false;
        std::vector<std::vector<double>> writepath;
        writepath = waypoints;

        std::ifstream load_file(file_dir);
        nlohmann::json json_tree;
        if(load_file.is_open()){
            try {
                json_tree = nlohmann::json::parse(load_file);
                load_file.close();
                // std::string mapid = json_tree.at("features").at("mapId").get<std::string>();
                for(auto &feature :json_tree.at("features")){
                    if (feature.contains("properties")&&feature["properties"].contains("featureType")
                        && feature["properties"]["featureType"]=="waypoints"){
                        feature["geometry"]["coordinates"] = writepath;
                        w_flag = true;
                        break;
                    }
                }
                if(!w_flag){
                    nlohmann::json j_properties = {
                        {"featureType", "waypoints"},
                        {"name", "途径点"},
                        {"count", writepath.size()},
                        {"remarks", "关键路径点"}
                    };
                    nlohmann::json j_geometry = {
                        {"type", "MultiPoint"},
                        {"coordinates", writepath}
                    };
                    nlohmann::json j_waypoints = {
                        {"type", "Feature"},
                        {"properties", j_properties},
                        {"geometry", j_geometry},
                        {"bbox", nlohmann::json::array()}
                    };
                    json_tree["features"].push_back(j_waypoints);
                    w_flag = true;
                }
            } catch (nlohmann::json::parse_error& e) {
                std::cerr << "解析错误: " << e.what() << std::endl;
            }
        }
        else {
            std::cerr << "cannot open: " << file_dir << std::endl;
            return false;
        }

        // 写回文件
        std::ofstream out_file(file_dir);
        out_file << std::setw(4) << json_tree << std::endl;
        out_file.close();

        return true;
    }

    void GlobalPlanner::writePathToJSON(const std::string &file_dir, const std::vector<std::vector<double>> &path){
        std::lock_guard<std::mutex> lock(mtx_json_);

        std::vector<std::vector<std::vector<double>>> writepath;
        writepath.push_back(path);

        std::ifstream load_file(file_dir);
        nlohmann::json json_tree;
        if(load_file.is_open()) {
            try {
                json_tree = nlohmann::json::parse(load_file);
                load_file.close();
                // std::string mapid = json_tree.at("features").at("mapId").get<std::string>();
                for(auto &feature :json_tree.at("features")){
                    if (feature.contains("properties")&&feature["properties"].contains("featureType")
                        && feature["properties"]["featureType"]=="workingPath"){
                        feature["geometry"]["coordinates"] = writepath;
                    }
                }
            } catch (nlohmann::json::parse_error& e) {
                std::cerr << "解析错误: " << e.what() << std::endl;
            }
        }
        else {
            std::cerr << "cannot open: " << file_dir << std::endl;
        }
        // 写回文件
        std::ofstream out_file(file_dir);
        out_file << std::setw(4) << json_tree << std::endl;
        out_file.close();
        std::cout << "path write down" << file_dir << std::endl;
    }

    void GlobalPlanner::runGlobalMission(const MissionInfo &mission){

        if(mission.task_type == 1){
            std::cout << "Create fields cover task" << std::endl;
            std::shared_ptr<FieldsCoverPlanner> fc_planner = std::make_shared<FieldsCoverPlanner>();
            fc_planner->setSwathWidth(mission.working_width);
            fc_planner->setStartPoint(mission.start_point.x(), mission.start_point.y());

            std::vector<GU::Point> boundary;
            for(int i=0; i<mission.boundary.size(); ++i){
                GU::Point p(mission.boundary[i].x(), mission.boundary[i].y());
                boundary.push_back(p);
            }
            std::vector<std::vector<GU::Point>> obstacles;
            for(int n=0; n<mission.obstacles.size(); ++n){
                std::vector<GU::Point> obs;
                for(int i=0; i<mission.obstacles[n].size(); ++i){
                    GU::Point p(mission.obstacles[n][i].x(), mission.obstacles[n][i].y());
                    obs.push_back(p);
                }
                obstacles.push_back(obs);
            }

            std::vector<GU::Point> opt_path;
            fc_planner->planLPath(boundary, obstacles, mission.working_rotate, opt_path);
            if(!opt_path.empty()){
                std::vector<std::vector<double>> waypoints_w;
                for(GU::Point p:opt_path){
                    std::vector<double> pos{p(0), p(1)};
                    waypoints_w.push_back(pos);
                }

                writeWaypointsToJSON(mission.mission_dir, waypoints_w);
                writePathToJSON(mission.mission_dir, waypoints_w);
                std::cout << "GONG-任务创建成功: " << waypoints_w.size() << std::endl;
            }
            else{
                std::cout << "GONG-任务创建失败: 无有效路径" << std::endl;
            }
        }
        else if(mission.task_type == 2){

        }

    }

    std::string GlobalPlanner::zmq_server_callback(const std::string& request){
        std::cout << "Received request: " << request << std::endl;

        std::string mission_dir = request;
        MissionInfo mission_info;

        if(!getInfoFromJSON(mission_dir, mission_info)){
            std::cout << "get info from json failed. " << std::endl;
        }
        else{
            std::cout << "get info from json success. " << std::endl;
            runGlobalMission(mission_info);
        }

        std::string response = "Response to: " + request;
        std::cout << "Sending response: " << response << std::endl;
        return response;
    }

}