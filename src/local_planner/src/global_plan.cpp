#include "global_plan.h"

namespace globalplanner
{
    GlobalPlanner::GlobalPlanner(){

    }

    GlobalPlanner::~GlobalPlanner(){

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

    bool GlobalPlanner::execute(){
        WallRate rate(20);
        while(true){
            std::cout << "global planner running ..." << std::endl;
            
            rate.sleep();
        }

        return true;
    }

    std::string GlobalPlanner::zmq_server_callback(const std::string& request){
        std::cout << "Received request: " << request << std::endl;

        std::string response = "Response to: " + request;
        std::cout << "Sending response: " << response << std::endl;
        return response;
    }

}