#include "gpsdr_fusion.h"
#include "glog/logging.h"

int main(int argc, char* argv[]){

    google::InitGoogleLogging(argv[0]);
    FLAGS_logbufsecs = 0;
    std::string filename = "/home/rpdzkj/Mower_env/log/gpsdr_fusion/";
    std::cout << "log dir: " << filename << std::endl;
    google::SetLogDestination(google::GLOG_INFO, filename.c_str());
    google::SetLogDestination(google::GLOG_WARNING, filename.c_str());
    google::SetLogDestination(google::GLOG_ERROR, filename.c_str());
    google::SetLogDestination(google::GLOG_FATAL, filename.c_str());
    LOG(INFO) << "GPSDRFusion node start";

    gpsdrfusion::GPSDRFusion gpsdr_fusion_node;
    std::string yaml_path = "/home/rpdzkj/Mower_env/src/gpsdr_fusion/params/filter_params.yaml";
    if(!gpsdr_fusion_node.initialize(yaml_path)){
        LOG(ERROR) << "GPSDRFusion node initialization failed";
        return -1;
    }
    gpsdr_fusion_node.spin();

    google::ShutdownGoogleLogging();
    
    return 0;
}