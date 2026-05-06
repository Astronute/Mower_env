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

        /*-------------------------------- zmq server-----------------------------------------*/

        if(filter_config_yaml_["zmq_server_port"]){
            std::string port = filter_config_yaml_["zmq_server_port"].as<std::string>();
            zmq_server_.initializeRequestHandler(port);
            zmq_server_.setRequestHandler([this](const std::string& request) -> std::string {
                return this->zmq_server_callback(request);
            });
            zmq_server_.startRequestHandler();
            std::cout << "mission execute server bind to: " << port << std::endl;
        }
        else{
            std::cout << "missing param 'zmq_server_port' " << std::endl;
            return false;
        }

        /*-------------------------------- zmq publisher-----------------------------------------*/

        if(filter_config_yaml_["zmq_pub_port"]){
            std::string port = filter_config_yaml_["zmq_pub_port"].as<std::string>();
            zmq_publisher_.initialize(port);
            std::cout << "controller node publisher bind to: " << port << std::endl;
        }
        else{
            std::cout << "missing param 'zmq_pub_port' " << std::endl;
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

    int GlobalPlanner::factorial(int n){
        if(n == 0 || n == 1)
        {
            return 1;
        }
        else
        {
            return n * factorial(n - 1);
        }
    }

    void GlobalPlanner::GetCostFunction(
        const int n_segment, 
        const int n_order, 
        const int cost_snap_jerk, 
        Eigen::SparseMatrix<double> &H
    ){
        double dt = 0.1;
        Eigen::SparseMatrix<double> M, Q;
        M.resize(n_segment*(n_order+1)*2, n_segment*(n_order+1)*2);
        Q.resize(n_segment*(n_order+1)*2, n_segment*(n_order+1)*2);
        
        for(int k = 0; k < n_segment; k++)
        {
            //for(int i = 0; i < (n_order+1)*2; i++)
            {
                //for(int j = 0; j < (n_order+1)*2; j++)
                {
                    M.insert((k*(n_order+1)*2), (k*(n_order+1)*2)) = 1.0;
                    M.insert(1+(k*(n_order+1)*2), 1+(k*(n_order+1)*2)) = 1.0;
                    M.insert(2+(k*(n_order+1)*2), (k*(n_order+1)*2)) = -5.0;M.insert(2+(k*(n_order+1)*2), 2+(k*(n_order+1)*2)) = 5.0;
                    M.insert(3+(k*(n_order+1)*2), 1+(k*(n_order+1)*2)) = -5.0;M.insert(3+(k*(n_order+1)*2), 3+(k*(n_order+1)*2)) = 5.0;
                    M.insert(4+(k*(n_order+1)*2), (k*(n_order+1)*2)) = 10.0;M.insert(4+(k*(n_order+1)*2), 2+(k*(n_order+1)*2)) = -20.0;M.insert(4+(k*(n_order+1)*2), 4+(k*(n_order+1)*2)) = 10.0;
                    M.insert(5+(k*(n_order+1)*2), 1+(k*(n_order+1)*2)) = 10.0;M.insert(5+(k*(n_order+1)*2), 3+(k*(n_order+1)*2)) = -20.0;M.insert(5+(k*(n_order+1)*2), 5+(k*(n_order+1)*2)) = 10.0;
                    M.insert(6+(k*(n_order+1)*2), (k*(n_order+1)*2)) = -10.0;M.insert(6+(k*(n_order+1)*2), 2+(k*(n_order+1)*2)) = 30.0;M.insert(6+(k*(n_order+1)*2), 4+(k*(n_order+1)*2)) = -30.0;M.insert(6+(k*(n_order+1)*2), 6+(k*(n_order+1)*2)) = 10.0;
                    M.insert(7+(k*(n_order+1)*2), 1+(k*(n_order+1)*2)) = -10.0;M.insert(7+(k*(n_order+1)*2), 3+(k*(n_order+1)*2)) = 30.0;M.insert(7+(k*(n_order+1)*2), 5+(k*(n_order+1)*2)) = -30.0;M.insert(7+(k*(n_order+1)*2), 7+(k*(n_order+1)*2)) = 10.0;
                    M.insert(8+(k*(n_order+1)*2), (k*(n_order+1)*2)) = 5.0;M.insert(8+(k*(n_order+1)*2), 2+(k*(n_order+1)*2)) = -20.0;M.insert(8+(k*(n_order+1)*2), 4+(k*(n_order+1)*2)) = 30.0;M.insert(8+(k*(n_order+1)*2), 6+(k*(n_order+1)*2)) = -20.0;M.insert(8+(k*(n_order+1)*2), 8+(k*(n_order+1)*2)) = 5.0;
                    M.insert(9+(k*(n_order+1)*2), 1+(k*(n_order+1)*2)) = 5.0;M.insert(9+(k*(n_order+1)*2), 3+(k*(n_order+1)*2)) = -20.0;M.insert(9+(k*(n_order+1)*2), 5+(k*(n_order+1)*2)) = 30.0;M.insert(9+(k*(n_order+1)*2), 7+(k*(n_order+1)*2)) = -20.0;M.insert(9+(k*(n_order+1)*2), 9+(k*(n_order+1)*2)) = 5.0;
                    M.insert(10+(k*(n_order+1)*2), (k*(n_order+1)*2)) = -1.0;M.insert(10+(k*(n_order+1)*2), 2+(k*(n_order+1)*2)) = 5.0;M.insert(10+(k*(n_order+1)*2), 4+(k*(n_order+1)*2)) = -10.0;M.insert(10+(k*(n_order+1)*2), 6+(k*(n_order+1)*2)) = 10.0;M.insert(10+(k*(n_order+1)*2), 8+(k*(n_order+1)*2)) = -5.0;M.insert(10+(k*(n_order+1)*2), 10+(k*(n_order+1)*2)) = 1.0;
                    M.insert(11+(k*(n_order+1)*2), 1+(k*(n_order+1)*2)) = -1.0;M.insert(11+(k*(n_order+1)*2), 3+(k*(n_order+1)*2)) = 5.0;M.insert(11+(k*(n_order+1)*2), 5+(k*(n_order+1)*2)) = -10.0;M.insert(11+(k*(n_order+1)*2), 7+(k*(n_order+1)*2)) = 10.0;M.insert(11+(k*(n_order+1)*2), 9+(k*(n_order+1)*2)) = -5.0;M.insert(11+(k*(n_order+1)*2), 11+(k*(n_order+1)*2)) = 1.0;
                }
            }
        }

        for(int k = 0; k < n_segment; k++)
        {
            for(int i = cost_snap_jerk; i <= (n_order); i++)
            {
                for(int j = cost_snap_jerk; j <= (n_order); j++)
                {
                    if((i+j) != cost_snap_jerk*2-1)
                    {
                        Q.insert(2*i+(k*(n_order+1)*2), 2*j+(k*(n_order+1)*2)) = (factorial(i) / factorial(i - (cost_snap_jerk - 1))) * (factorial(j) / factorial(j - (cost_snap_jerk - 1))) *
                                                                            pow(dt, i + j - (cost_snap_jerk*2 - 1)) / (i + j - (cost_snap_jerk*2 - 1));
                        Q.insert(2*i+1+(k*(n_order+1)*2), 2*j+1+(k*(n_order+1)*2)) = (factorial(i) / factorial(i - (cost_snap_jerk - 1))) * (factorial(j) / factorial(j - (cost_snap_jerk - 1))) *
                                                                            pow(dt, i + j - (cost_snap_jerk*2 - 1)) / (i + j - (cost_snap_jerk*2 - 1));
                    }
                }
            }
        }
        H = M.transpose() * Q * M;
        // std::cout<<"H: "<<H<<std::endl;
        // std::cout<<"Q: "<<Q<<std::endl;
        return ;
    }

    void GlobalPlanner::GetConstraint(
        const int n_segment, 
        const int n_order, 
        const std::vector<naviparams::point> way_points, 
        Eigen::SparseMatrix<double> &A, 
        Eigen::VectorXd &lowerBound, 
        Eigen::VectorXd &upperBound, Eigen::VectorXd &f
    ){
        //Eigen::SparseMatrix<double> A;
        //Eigen::VectorXd lowerBound, upperBound, f;
            
        double dt = 0.1;
        int way_points_num = way_points.size();
        ////start
        A.insert(0, 0) = 1.0; 
        A.insert(1, 1) = 1.0;
        A.insert(2, 0) = -1.0 * n_order; A.insert(2, 2) = 1.0 * n_order;
        A.insert(3, 1) = -1.0 * n_order; A.insert(3, 3) = 1.0 * n_order;
        A.insert(4, 0) = 1.0 * n_order*(n_order - 1) / dt; A.insert(4, 2) = -2.0 * n_order*(n_order - 1) / dt; A.insert(4, 4) = 1.0 * n_order*(n_order - 1) / dt;
        A.insert(5, 1) = 1.0 * n_order*(n_order - 1) / dt; A.insert(5, 3) = -2.0 * n_order*(n_order - 1) / dt; A.insert(5, 5) = 1.0 * n_order*(n_order - 1) / dt;
        lowerBound[0] = way_points.front().x; upperBound[0] = way_points.front().x;;
        lowerBound[1] = way_points.front().y; upperBound[1] = way_points.front().y;
        lowerBound[2] = 0.0; upperBound[2] = 0.0;
        lowerBound[3] = 0.0; upperBound[3] = 0.0;
        lowerBound[4] = 0.0; upperBound[4] = 0.0;
        lowerBound[5] = 0.0; upperBound[5] = 0.0;
        //std::cout<<"A11: "<<A<<std::endl;
        ////end
        A.insert(6, n_segment*(n_order+1)*2 - 2) = 1.0; 
        A.insert(7, n_segment*(n_order+1)*2 - 1) = 1.0;
        A.insert(8, n_segment*(n_order+1)*2 - 2) = 1.0 * n_order; A.insert(8, n_segment*(n_order+1)*2 - 4) = -1.0 * n_order;
        A.insert(9, n_segment*(n_order+1)*2 - 1) = 1.0 * n_order; A.insert(9, n_segment*(n_order+1)*2 - 3) = -1.0 * n_order;
        A.insert(10, n_segment*(n_order+1)*2 - 2) = 1.0 * n_order*(n_order - 1) / dt; A.insert(10, n_segment*(n_order+1)*2 - 4) = -2.0 * n_order*(n_order - 1) / dt; A.insert(10, n_segment*(n_order+1)*2 - 6) = 1.0 * n_order*(n_order - 1) / dt;
        A.insert(11, n_segment*(n_order+1)*2 - 1) = 1.0 * n_order*(n_order - 1) / dt; A.insert(11, n_segment*(n_order+1)*2 - 3) = -2.0 * n_order*(n_order - 1) / dt; A.insert(11, n_segment*(n_order+1)*2 - 5) = 1.0 * n_order*(n_order - 1) / dt;
        lowerBound[6] = way_points.back().x; upperBound[6] = way_points.back().x;
        lowerBound[7] = way_points.back().y; upperBound[7] = way_points.back().y;
        lowerBound[8] = 0.0; upperBound[8] = 0.0;
        lowerBound[9] = 0.0; upperBound[9] = 0.0;
        lowerBound[10] = 0.0; upperBound[10] = 0.0;
        lowerBound[11] = 0.0; upperBound[11] = 0.0;
        //std::cout<<"A1: "<<A<<std::endl;
        ////
        for(int i = 1; i < n_segment; i++)
        {
            if(i != n_segment)
            {
                A.insert((n_order + 1) * 2 + 6 * (i - 1), (i)*(n_order+1)*2 - 2) = 1.0; A.insert((n_order + 1) * 2 + 6 * (i - 1), (i)*(n_order+1)*2) = -1.0; 
                A.insert((n_order + 1) * 2 + 6 * (i - 1) + 1, (i)*(n_order+1)*2 - 1) = 1.0; A.insert((n_order + 1) * 2 + 6 * (i - 1) + 1, (i)*(n_order+1)*2 + 1) = -1.0;
                
                A.insert((n_order + 1) * 2 + 6 * (i - 1) + 2, (i)*(n_order+1)*2 - 2) = -1.0 * n_order; A.insert((n_order + 1) * 2 + 6 * (i - 1) + 2, (i)*(n_order+1)*2 - 4) = 1.0 * n_order;
                A.insert((n_order + 1) * 2 + 6 * (i - 1) + 2, (i)*(n_order+1)*2) = -1.0 * n_order; A.insert((n_order + 1) * 2 + 6 * (i - 1) + 2, (i)*(n_order+1)*2 + 2) = 1.0 * n_order;
                A.insert((n_order + 1) * 2 + 6 * (i - 1) + 3, (i)*(n_order+1)*2 - 1) = -1.0 * n_order; A.insert((n_order + 1) * 2 + 6 * (i - 1) + 3, (i)*(n_order+1)*2 - 3) = 1.0 * n_order;
                A.insert((n_order + 1) * 2 + 6 * (i - 1) + 3, (i)*(n_order+1)*2 + 1) = -1.0 * n_order; A.insert((n_order + 1) * 2 + 6 * (i - 1) + 3, (i)*(n_order+1)*2 + 3) = 1.0 * n_order;
                
                A.insert((n_order + 1) * 2 + 6 * (i - 1) + 4, (i)*(n_order+1)*2 - 2) = 1.0 * n_order*(n_order - 1) / dt; 
                A.insert((n_order + 1) * 2 + 6 * (i - 1) + 4, (i)*(n_order+1)*2 - 4) = -2.0 * n_order*(n_order - 1) / dt;
                A.insert((n_order + 1) * 2 + 6 * (i - 1) + 4, (i)*(n_order+1)*2 - 6) = 1.0 * n_order*(n_order - 1) / dt;

                A.insert((n_order + 1) * 2 + 6 * (i - 1) + 4, (i)*(n_order+1)*2) = -1.0 * n_order*(n_order - 1) / dt; 
                A.insert((n_order + 1) * 2 + 6 * (i - 1) + 4, (i)*(n_order+1)*2 + 2) = 2.0 * n_order*(n_order - 1) / dt;
                A.insert((n_order + 1) * 2 + 6 * (i - 1) + 4, (i)*(n_order+1)*2 + 4) = -1.0 * n_order*(n_order - 1) / dt;

                A.insert((n_order + 1) * 2 + 6 * (i - 1) + 5, (i)*(n_order+1)*2 - 1) = 1.0 * n_order*(n_order - 1) / dt; 
                A.insert((n_order + 1) * 2 + 6 * (i - 1) + 5, (i)*(n_order+1)*2 - 3) = -2.0 * n_order*(n_order - 1) / dt;
                A.insert((n_order + 1) * 2 + 6 * (i - 1) + 5, (i)*(n_order+1)*2 - 5) = 1.0 * n_order*(n_order - 1) / dt;

                A.insert((n_order + 1) * 2 + 6 * (i - 1) + 5, (i)*(n_order+1)*2 + 1) = -1.0 * n_order*(n_order - 1) / dt; 
                A.insert((n_order + 1) * 2 + 6 * (i - 1) + 5, (i)*(n_order+1)*2 + 3) = 2.0 * n_order*(n_order - 1) / dt;
                A.insert((n_order + 1) * 2 + 6 * (i - 1) + 5, (i)*(n_order+1)*2 + 5) = -1.0 * n_order*(n_order - 1) / dt;

                lowerBound[(n_order + 1) * 2 + 6 * (i - 1)] = 0.0; upperBound[(n_order + 1) * 2 + 6 * (i - 1)] = 0.0;
                lowerBound[(n_order + 1) * 2 + 6 * (i - 1) + 1] = 0.0; upperBound[(n_order + 1) * 2 + 6 * (i - 1) + 1] = 0.0;
                lowerBound[(n_order + 1) * 2 + 6 * (i - 1) + 2] = 0.0; upperBound[(n_order + 1) * 2 + 6 * (i - 1) + 2] = 0.0;
                lowerBound[(n_order + 1) * 2 + 6 * (i - 1) + 3] = 0.0; upperBound[(n_order + 1) * 2 + 6 * (i - 1) + 3] = 0.0;
                lowerBound[(n_order + 1) * 2 + 6 * (i - 1) + 4] = 0.0; upperBound[(n_order + 1) * 2 + 6 * (i - 1) + 4] = 0.0;
                lowerBound[(n_order + 1) * 2 + 6 * (i - 1) + 5] = 0.0; upperBound[(n_order + 1) * 2 + 6 * (i - 1) + 5] = 0.0;
            }
        }
        // std::cout<<"A2: "<<A<<std::endl;
        ////
        for(int  i = 1; i < n_segment; i++)
        {
            A.insert((n_order + 1) * 2 + 6 * (n_segment - 1) + 2 * (i - 1), i*(n_order+1)*2 - 2) = 1.0; 
            A.insert((n_order + 1) * 2 + 6 * (n_segment - 1) + 2 * (i - 1) + 1, i*(n_order+1)*2 - 1) = 1.0; 

            lowerBound[(n_order + 1) * 2 + 6 * (n_segment - 1) + 2 * (i - 1)] = way_points[i].x; upperBound[(n_order + 1) * 2 + 6 * (n_segment - 1) + 2 * (i - 1)] = way_points[i].x;
            lowerBound[(n_order + 1) * 2 + 6 * (n_segment - 1) + 2 * (i - 1) + 1] = way_points[i].y; upperBound[(n_order + 1) * 2 + 6 * (n_segment - 1) + 2 * (i - 1) + 1] = way_points[i].y;
        }
        // std::cout<<"A3: "<<A<<std::endl;
        /////
        for(int i = 0; i < n_segment; i++)
        {
            for(int j = 0; j < (n_order + 1); j++)
            {
                A.insert((n_order + 1) * 2 + 6 * (n_segment - 1) + 2 * (n_segment - 1) + 2 * (i) * (n_order + 1) + 2*j, 2 * (i)  * (n_order + 1) + 2*j) = 1.0; 
                A.insert((n_order + 1) * 2 + 6 * (n_segment - 1) + 2 * (n_segment - 1) + 2 * (i) * (n_order + 1) + 2*j + 1, 2 * (i)  * (n_order + 1) + 2*j+1) = 1.0; 
                //if(j <= (n_order + 1))
                {

                    if(way_points[i].x < way_points[i+1].x)
                    {
                        lowerBound[(n_order + 1) * 2 + 6 * (n_segment - 1) + 2 * (n_segment - 1) + 2 * (i)  * (n_order + 1) + 2*j] = way_points[i].x; upperBound[(n_order + 1) * 2 + 6 * (n_segment - 1) + 2 * (n_segment - 1) + 2 * (i)  * (n_order + 1) + 2*j] = way_points[i + 1].x;
                    }
                    else
                    {
                        lowerBound[(n_order + 1) * 2 + 6 * (n_segment - 1) + 2 * (n_segment - 1) + 2 * (i)  * (n_order + 1) + 2*j] = way_points[i + 1].x; upperBound[(n_order + 1) * 2 + 6 * (n_segment - 1) + 2 * (n_segment - 1) + 2 * (i)  * (n_order + 1) + 2*j] = way_points[i].x;
                    }

                    if(way_points[i].y < way_points[i+1].y)
                    {
                        lowerBound[(n_order + 1) * 2 + 6 * (n_segment - 1) + 2 * (n_segment - 1) + 2 * (i)  * (n_order + 1) + 2*j + 1] = way_points[i].y; upperBound[(n_order + 1) * 2 + 6 * (n_segment - 1) + 2 * (n_segment - 1) + 2 * (i)  * (n_order + 1) + 2*j + 1] = way_points[i + 1].y;
                    }
                    else
                    {
                        lowerBound[(n_order + 1) * 2 + 6 * (n_segment - 1) + 2 * (n_segment - 1) + 2 * (i)  * (n_order + 1) + 2*j + 1] = way_points[i+1].y; upperBound[(n_order + 1) * 2 + 6 * (n_segment - 1) + 2 * (n_segment - 1) + 2 * (i)  * (n_order + 1) + 2*j + 1] = way_points[i].y;
                    }

                }

            }
        }
        ///////
        #if(1)
        int n = n_segment * (n_order+1)*2;
        //dt = 1.0;
        for(int i = 0; i < (n_segment - 1); i++)
        {
            //for(int j = 0; j < (n_order + 1); j++)
            {
                //A.insert((n_order + 1) * 2 + 2 * (n_segment - 2) + 2 * n_segment + 2 * (i) * (n_order + 1) + 6*j, 2 * (i)  * (n_order + 1) + 2*j) = 1.0; 
                //A.insert((n_order + 1) * 2 + 2 * (n_segment - 2) + 2 * n_segment + 2 * (i) * (n_order + 1) + 6*j + 1, 2 * (i)  * (n_order + 1) + 2*j+1) = 1.0; 
                
                A.insert((n_order + 1) * 2 + 6 * (n_segment - 1) + 2 * (n_segment - 1) + n + 4*i, 2 * (i+1)  * (n_order + 1) - 4) = 1.0 * n_order; 
                A.insert((n_order + 1) * 2 + 6 * (n_segment - 1) + 2 * (n_segment - 1) + n + 4*i, 2 * (i+1)  * (n_order + 1) - 2) = -1.0 * n_order; 
                A.insert((n_order + 1) * 2 + 6 * (n_segment - 1) + 2 * (n_segment - 1) + n + 4*i + 1, 2 * (i+1)  * (n_order + 1) - 3) = 1.0 * n_order; 
                A.insert((n_order + 1) * 2 + 6 * (n_segment - 1) + 2 * (n_segment - 1) + n + 4*i + 1, 2 * (i+1)  * (n_order + 1) - 1) = -1.0 * n_order; 

                A.insert((n_order + 1) * 2 + 6 * (n_segment - 1) + 2 * (n_segment - 1) + n + 4*i + 2, 2 * (i+1)  * (n_order + 1) - 6) = 1.0 * n_order*(n_order - 1) / dt; 
                A.insert((n_order + 1) * 2 + 6 * (n_segment - 1) + 2 * (n_segment - 1) + n + 4*i + 2, 2 * (i+1)  * (n_order + 1) - 4) = -2.0 * n_order*(n_order - 1) / dt; 
                A.insert((n_order + 1) * 2 + 6 * (n_segment - 1) + 2 * (n_segment - 1) + n + 4*i + 2, 2 * (i+1)  * (n_order + 1) - 2) = 1.0 * n_order*(n_order - 1) / dt; 
                A.insert((n_order + 1) * 2 + 6 * (n_segment - 1) + 2 * (n_segment - 1) + n + 4*i + 3, 2 * (i+1)  * (n_order + 1) - 5) = 1.0 * n_order*(n_order - 1) / dt; 
                A.insert((n_order + 1) * 2 + 6 * (n_segment - 1) + 2 * (n_segment - 1) + n + 4*i + 3, 2 * (i+1)  * (n_order + 1) - 3) = -2.0 * n_order*(n_order - 1) / dt; 
                A.insert((n_order + 1) * 2 + 6 * (n_segment - 1) + 2 * (n_segment - 1) + n + 4*i + 3, 2 * (i+1)  * (n_order + 1) - 1) = 1.0 * n_order*(n_order - 1) / dt; 

                lowerBound[(n_order + 1) * 2 + 6 * (n_segment - 1) + 2 * (n_segment - 1) + n + 4*i] = -0.01; upperBound[(n_order + 1) * 2 + 6 * (n_segment - 1) + 2 * (n_segment - 1) + n + 4*i] = 0.01;
                lowerBound[(n_order + 1) * 2 + 6 * (n_segment - 1) + 2 * (n_segment - 1) + n + 4*i + 1] = -0.01; upperBound[(n_order + 1) * 2 + 6 * (n_segment - 1) + 2 * (n_segment - 1) + n + 4*i + 1] = 0.01;
                lowerBound[(n_order + 1) * 2 + 6 * (n_segment - 1) + 2 * (n_segment - 1) + n + 4*i + 2] = -0.01; upperBound[(n_order + 1) * 2 + 6 * (n_segment - 1) + 2 * (n_segment - 1) + n + 4*i + 2] = 0.01;
                lowerBound[(n_order + 1) * 2 + 6 * (n_segment - 1) + 2 * (n_segment - 1) + n + 4*i + 3] = -0.01; upperBound[(n_order + 1) * 2 + 6 * (n_segment - 1) + 2 * (n_segment - 1) + n + 4*i + 3] = 0.01;
            }
        }
        #endif
        // std::cout<<"A4: "<<A<<std::endl;
        // std::cout<<"lowerBound: "<<lowerBound<<std::endl;
        // std::cout<<"upperBound: "<<upperBound<<std::endl;
        //////
        for(int i = 0; i < (n_order + 1) * 2 * n_segment; i++)
        { 
            f[i] = 0.0; 
        }
    }

    bool GlobalPlanner::GeneratorPointPlan(const std::vector<naviparams::point> &points_, nav_msgs::Path &traj_){
        Eigen::SparseMatrix<double> A, H;
        Eigen::VectorXd lowerBound, upperBound, f;
        std::vector<naviparams::point> way_points;
        Eigen::VectorXd x;
        int n_order = 5;
        int cost_snap_jerk = 3; 

        for(naviparams::point point: points_){
            naviparams::point p;
            p.x = point.x;
            p.y = point.y;
            way_points.push_back(p);
            // log->infoStream() << "GeneratorPointPlan-x: "<<p.x<<" y: "<<p.y;
        }

        if(way_points.size() > 1)
        {
            int n_segment = way_points.size() - 1;
            int n = n_segment * (n_order+1)*2; // num of para
            H.resize(n_segment*(n_order+1)*2, n_segment*(n_order+1)*2);
            #if(1)
            {
                A.resize((n_order + 1) * 2 + 6 * (n_segment - 1) + 2 * (n_segment - 1) + n + (n_segment - 1) * 4, n);
                lowerBound.resize((n_order + 1) * 2 + 6 * (n_segment - 1) + 2 * (n_segment - 1) + n + (n_segment - 1) * 4);
                upperBound.resize((n_order + 1) * 2 + 6 * (n_segment - 1) + 2 * (n_segment - 1) + n + (n_segment - 1) * 4);
            }
            #else
            {
                A.resize((n_order + 1) * 2 + 6 * (n_segment - 1) + 2 * (n_segment - 1) + n, n);
                lowerBound.resize((n_order + 1) * 2 + 6 * (n_segment - 1) + 2 * (n_segment - 1) + n);
                upperBound.resize((n_order + 1) * 2 + 6 * (n_segment - 1) + 2 * (n_segment - 1) + n);
            }
            #endif
            f.resize(n); 
            // log->infoStream() << "GeneratorPointPlan1";
            GetCostFunction(n_segment, n_order, cost_snap_jerk, H);
            // log->infoStream() << "GeneratorPointPlan--GetCostFunction";
            GetConstraint(n_segment, n_order, way_points, A, lowerBound, upperBound, f);
            // log->infoStream() << "GeneratorPointPlan--GetConstraint";
            OsqpEigen::Solver solver;
            //solver.settings()->setVerbosity(false);
            solver.settings()->setAlpha(1.6);
            solver.data()->setNumberOfVariables(n);
            #if(1)
            {
                solver.data()->setNumberOfConstraints((n_order + 1) * 2 + 6 * (n_segment - 1) + 2 * (n_segment - 1) + n + (n_segment - 1) * 4);
            }
            #else
            {
                solver.data()->setNumberOfConstraints((n_order + 1) * 2 + 6 * (n_segment - 1) + 2 * (n_segment - 1) + n);
            }
            #endif
            if (!solver.data()->setHessianMatrix(H))
                return false;
            if (!solver.data()->setGradient(f))
                return false;
            if (!solver.data()->setLinearConstraintsMatrix(A))
                return false;
            if (!solver.data()->setLowerBound(lowerBound))
                return false;
            if (!solver.data()->setUpperBound(upperBound))
                return false;
            solver.initSolver();
            solver.solveProblem();

            x = solver.getSolution();
            // for(int i = 0; i < x.size(); i++){
            //     std::cout<<"x: "<<x[i]<<std::endl;
            // }
            if (solver.getStatus() == OsqpEigen::Status::Solved){
                std::vector<double> bx, by;
                int n_pam = (n_order+1)*2;
                for (int i = 1; i <= n_segment; i++)
                {
                    double t = 0.01;
                    for(int j = 0; j < 100; j++)
                    {
                        //bx.push_back(pow((1-t),5.0)*x(1+(i-1)*n_pam,1)+5.0*t*pow((1-t),4.0)*x(3+(i-1)*n_pam,1)+10.0*pow(t,2.0)*pow((1-t),3.0)*x(5+(i-1)*n_pam,1)+10.0*pow(t,3.0)*pow((1-t),2.0)*x(7+(i-1)*n_pam,1)+5.0*pow(t,4.0)*(1-t)*x(9+(i-1)*n_pam,1)+pow(t,5.0)*x(11+(i-1)*n_pam,1));
                        //by.push_back(pow((1-t),5.0)*x(2+(i-1)*n_pam,1)+5.0*t*pow((1-t),4.0)*x(4+(i-1)*n_pam,1)+10.0*pow(t,2.0)*pow((1-t),3.0)*x(6+(i-1)*n_pam,1)+10.0*pow(t,3.0)*pow((1-t),2.0)*x(8+(i-1)*n_pam,1)+5.0*pow(t,4.0)*(1-t)*x(10+(i-1)*n_pam,1)+pow(t,5.0)*x(12+(i-1)*n_pam,1));
                        bx.push_back(pow((1-t),5.0)*x((i-1)*n_pam)+5.0*t*pow((1-t),4.0)*x(2+(i-1)*n_pam)+10.0*pow(t,2.0)*pow((1-t),3.0)*x(4+(i-1)*n_pam)+10.0*pow(t,3.0)*pow((1-t),2.0)*x(6+(i-1)*n_pam)+5.0*pow(t,4.0)*(1-t)*x(8+(i-1)*n_pam)+pow(t,5.0)*x(10+(i-1)*n_pam));
                        by.push_back(pow((1-t),5.0)*x(1+(i-1)*n_pam)+5.0*t*pow((1-t),4.0)*x(3+(i-1)*n_pam)+10.0*pow(t,2.0)*pow((1-t),3.0)*x(5+(i-1)*n_pam)+10.0*pow(t,3.0)*pow((1-t),2.0)*x(7+(i-1)*n_pam)+5.0*pow(t,4.0)*(1-t)*x(9+(i-1)*n_pam)+pow(t,5.0)*x(11+(i-1)*n_pam));
                        t = t + 0.01;
                    }
                }
                // log->infoStream() << "GeneratorPointPlan--2";
                // solver_getStatus_ = 0;
                traj_.mutable_header()->mutable_stamp()->CopyFrom(navicommon::systimeToProto(this->now()));
                traj_.mutable_header()->set_frame_id("map");
                traj_.clear_poses();
                for(int i = 0; i < bx.size(); i++){
                    geometry_msgs::PoseStamped pos;
                    pos.mutable_header()->mutable_stamp()->CopyFrom(navicommon::systimeToProto(this->now()));
                    pos.mutable_header()->set_frame_id("map");
                    pos.mutable_pose()->mutable_position()->set_x(bx[i]);
                    pos.mutable_pose()->mutable_position()->set_y(by[i]);
                    pos.mutable_pose()->mutable_position()->set_z(0.0);
                    *traj_.add_poses() = pos;
                }
                
                // log->infoStream() << "osqp success: "<<bx.size();
            }
            else{
                return false;
                // log->infoStream() << "osqp failed";
            }
        }

        return true;
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
                    naviparams::point pos;
                    pos.x = robot_x;
                    pos.y = robot_y;
                    map_points_.push_back(pos);

                    pos.x = global_path_pub_.poses(goal_point_index_).pose().position().x();
                    pos.y = global_path_pub_.poses(goal_point_index_).pose().position().y();
                    map_points_.push_back(pos);
                }
                else{
                    naviparams::point pos;
                    pos.x = global_path_pub_.poses(goal_point_index_).pose().position().x();
                    pos.y = global_path_pub_.poses(goal_point_index_).pose().position().y();
                    map_points_.push_back(pos);

                    if(goal_point_index_ + 1 < global_path_pub_.poses_size()){
                        goal_point_index_++;
                        pos.x = global_path_pub_.poses(goal_point_index_).pose().position().x();
                        pos.y = global_path_pub_.poses(goal_point_index_).pose().position().y();
                        map_points_.push_back(pos);
                    }
                    else{
                        map_points_.clear();
                    }
                }

                if(!map_points_.empty() && GeneratorPointPlan(map_points_, global_traj)){
                    std::string serialized_data;
                    global_traj.SerializeToString(&serialized_data);
                    zmq_publisher_.publishMessage("/global_planner/path", serialized_data);
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