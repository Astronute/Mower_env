#include "controller_base.h"

namespace CB{

    bool ControllerBase::initialize(){

        if(!loadParams()){
            std::cout << "controller base load param failed" << std::endl;
            return false;
        }

        initialized_ = true;
        return true;
    }

    void ControllerBase::reset(){
        
    }

    bool ControllerBase::loadParams(){

        try{
            // controller_config_yaml_ = YAML::LoadFile("/home/tom/Mower_env/src/motion_control/params/controller_params.yaml");
            controller_config_yaml_ = YAML::LoadFile("/home/rpdzkj/Mower_env/src/motion_control/params/controller_params.yaml");
        } catch(const YAML::Exception& e){
            std::cout << "yaml parsing error: " << e.what() << std::endl;
            return false;
        }

        if(controller_config_yaml_["control_rate"]){
            control_rate_ = controller_config_yaml_["control_rate"].as<double>();
        }
        else{
            std::cout << "missing param 'control_rate' " << std::endl;
            return false;
        }
        
        if(controller_config_yaml_["wheel_base"]){
            wheel_base_ = controller_config_yaml_["wheel_base"].as<double>();
        }
        else{
            std::cout << "missing param 'wheel_base' " << std::endl;
            return false;
        }
        if(controller_config_yaml_["max_vel"]){
            max_vel_ = controller_config_yaml_["max_vel"].as<double>();
        }
        else{
            std::cout << "missing param 'max_vel' " << std::endl;
            return false;
        }
        if(controller_config_yaml_["min_vel"]){
            min_vel_ = controller_config_yaml_["min_vel"].as<double>();
        }
        else{
            std::cout << "missing param 'min_vel' " << std::endl;
            return false;
        }
        if(controller_config_yaml_["max_w"]){
            max_w_ = controller_config_yaml_["max_w"].as<double>();
        }
        else{
            std::cout << "missing param 'max_w' " << std::endl;
            return false;
        }
        if(controller_config_yaml_["min_w"]){
            min_w_ = controller_config_yaml_["min_w"].as<double>();
        }
        else{
            std::cout << "missing param 'min_w' " << std::endl;
            return false;
        }
        if(controller_config_yaml_["max_a"]){
            max_a_ = controller_config_yaml_["max_a"].as<double>();
        }
        else{
            std::cout << "missing param 'max_a' " << std::endl;
            return false;
        }
        if(controller_config_yaml_["min_a"]){
            min_a_ = controller_config_yaml_["min_a"].as<double>();
        }
        else{
            std::cout << "missing param 'min_a' " << std::endl;
            return false;
        }
        if(controller_config_yaml_["max_aw"]){
            max_aw_ = controller_config_yaml_["max_aw"].as<double>();
        }
        else{
            std::cout << "missing param 'max_aw' " << std::endl;
            return false;
        }
        if(controller_config_yaml_["tau_w"]){
            tau_w_ = controller_config_yaml_["tau_w"].as<double>();
        }
        else{
            std::cout << "missing param 'tau_w' " << std::endl;
            return false;
        }
        if(controller_config_yaml_["tau_v"]){
            tau_v_ = controller_config_yaml_["tau_v"].as<double>();
        }
        else{
            std::cout << "missing param 'tau_v' " << std::endl;
            return false;
        }

        return true;
    }

    bool ControllerBase::findTargetTrajTimepoint(
        TrajPoint & target_point, 
        const std::vector<TrajPoint> & target_traj, 
        double target_t
    ){
        double target_x, target_y, target_yaw, target_kappa, target_s, target_v, target_w;
        int target_idx = 0;
        bool searched = false;

        if(target_traj.size() > 1){
            for(int i=0; i<target_traj.size(); ++i){
                if(target_t <= target_traj.at(i).t){
                    target_idx = i;
                    searched = true;
                    break;
                }
            }

            if(target_idx == 0){
                if(searched){
                    TrajPoint p(
                        target_traj.at(0).path_point,
                        target_traj.at(0).v,
                        target_traj.at(0).w,
                        target_traj.at(0).t
                    );
                    target_point = p;
                }
                else{
                    TrajPoint p(
                        target_traj.back().path_point,
                        target_traj.back().v,
                        target_traj.back().w,
                        target_traj.back().t
                    );
                    target_point = p;
                }
                return true;
            }
            else{
                TrajPoint prev_point(
                    target_traj.at(target_idx - 1).path_point,
                    target_traj.at(target_idx - 1).v,
                    target_traj.at(target_idx - 1).w,
                    target_traj.at(target_idx - 1).t
                );
                TrajPoint next_point(
                    target_traj.at(target_idx).path_point,
                    target_traj.at(target_idx).v,
                    target_traj.at(target_idx).w,
                    target_traj.at(target_idx).t
                );
                target_x = common::linearEquation(prev_point.t, prev_point.path_point.x, next_point.t, next_point.path_point.x, target_t);
                target_y = common::linearEquation(prev_point.t, prev_point.path_point.y, next_point.t, next_point.path_point.y, target_t);
                target_yaw = common::linearEquationAngle(prev_point.t, prev_point.path_point.yaw, next_point.t, next_point.path_point.yaw, target_t);
                target_kappa = common::linearEquation(prev_point.t, prev_point.path_point.kappa, next_point.t, next_point.path_point.kappa, target_t);
                target_s = common::linearEquation(prev_point.t, prev_point.path_point.s, next_point.t, next_point.path_point.s, target_t);
                target_v = common::linearEquation(prev_point.t, prev_point.v, next_point.t, next_point.v, target_t);
                target_w = common::linearEquation(prev_point.t, prev_point.w, next_point.t, next_point.w, target_t);

            }
        }
        else{
            std::cout << "findTargetTrajTimepoint: target_trajectory size < 1" << std::endl;
            return false;
        }

        PathPoint tar_p(target_x, target_y, 0.0, target_yaw, target_kappa, target_s);
        target_point.path_point = tar_p;
        target_point.t = target_t;
        target_point.v = target_v;
        target_point.w = target_w;

        return true;
    }

    bool ControllerBase::findTargetPathPoint(
        TrajPoint & target_point, 
        const std::vector<TrajPoint> & target_traj, 
        double target_s
    ){
        double target_x, target_y, target_yaw, target_kappa, target_t, target_v, target_w;
        int target_index = 0;
        bool searched = false;

        if(target_traj.size() > 1){
            for(int i=0; i<target_traj.size(); ++i){
                if(target_s <= target_traj.at(i).path_point.s){
                    target_index = i;
                    searched = true;
                    break;
                }
            }

            if(target_index == 0){
                if(searched){
                    TrajPoint p(
                        target_traj.at(0).path_point,
                        target_traj.at(0).v,
                        target_traj.at(0).w,
                        target_traj.at(0).t
                    );
                    target_point = p;
                }
                else{
                    TrajPoint p(
                        target_traj.back().path_point,
                        target_traj.back().v,
                        target_traj.back().w,
                        target_traj.back().t
                    );
                    target_point = p;
                }
                return true;
            }

            TrajPoint prev_point(
                target_traj.at(target_index - 1).path_point,
                target_traj.at(target_index - 1).v,
                target_traj.at(target_index - 1).w,
                target_traj.at(target_index - 1).t
            );
            TrajPoint next_point(
                target_traj.at(target_index).path_point,
                target_traj.at(target_index).v,
                target_traj.at(target_index).w,
                target_traj.at(target_index).t
            );
            target_x = common::linearEquation(prev_point.path_point.s, prev_point.path_point.x, next_point.path_point.s, next_point.path_point.x, target_s);
            target_y = common::linearEquation(prev_point.path_point.s, prev_point.path_point.y, next_point.path_point.s, next_point.path_point.y, target_s);
            target_yaw = common::linearEquationAngle(prev_point.path_point.s, prev_point.path_point.yaw, next_point.path_point.s, next_point.path_point.yaw, target_s);
            target_kappa = common::linearEquation(prev_point.path_point.s, prev_point.path_point.kappa, next_point.path_point.s, next_point.path_point.kappa, target_s);
            target_t = common::linearEquation(prev_point.path_point.s, prev_point.t, next_point.path_point.s, next_point.t, target_s);
            target_v = common::linearEquation(prev_point.path_point.s, prev_point.v, next_point.path_point.s, next_point.v, target_s);
            target_w = common::linearEquation(prev_point.path_point.s, prev_point.w, next_point.path_point.s, next_point.w, target_s);
        }
        else{
            std::cout << "findTargetPathPoint: target_trajectory size < 1" << std::endl;
            return false;
        }

        PathPoint tar_p(target_x, target_y, 0.0, target_yaw, target_kappa, target_s);
        target_point.path_point = tar_p;
        target_point.t = target_t;
        target_point.v = target_v;
        target_point.w = target_w;
        return true;
    }

    // 规划当前朝向的固定长度直线轨迹
    void ControllerBase::planLinearTrajectory(
        std::vector<TrajPoint> & traj, 
        const geometry_msgs::Pose2D & robot_pose, 
        const geometry_msgs::Twist & robot_twist, 
        const double line_s
    ){
        double line_max_v = max_vel_;
        double line_max_a = max_a_;
        double acc_time = line_max_v / line_max_a; // 加速时间
        double acc_dis = 0.5 * line_max_v * acc_time; // 加速距离
        double dec_time = acc_time; // 减速时间
        double dec_dis = acc_dis; // 减速距离
        double uni_dis = line_s - acc_dis - dec_dis; // 匀速距离
        double uni_time = uni_dis / line_max_v; // 匀速时间
        if(uni_dis < 0){
            acc_time = std::sqrt(line_s / line_max_a);
            acc_dis = line_s / 2.0;
            dec_time = acc_time;
            dec_dis = acc_dis;
            uni_time = 0;
            uni_dis = 0;
        }
        double plan_total_time = acc_time + uni_time + dec_time;
        double plan_v_peak = acc_time * line_max_a;

        std::array<double, 6> acc_coeff;
        std::array<double, 6> uni_coeff;
        std::array<double, 6> dec_coeff;
        fivetimesPlanTraj(acc_coeff, 0, 0, 0, 0, acc_time, acc_dis, plan_v_peak, 0);
        fivetimesPlanTraj(uni_coeff, acc_time, acc_dis, plan_v_peak, 0, acc_time + uni_time, acc_dis + uni_dis, plan_v_peak, 0);
        fivetimesPlanTraj(dec_coeff, acc_time + uni_time, acc_dis + uni_dis, plan_v_peak, 0, plan_total_time, line_s, 0, 0);

        TrajPoint target_point;
        double delta_t = 1.0 / control_rate_;
        unsigned int num_points = plan_total_time / delta_t;
        double plan_t = 0.0;
        double plan_s, plan_v;
        double target_x, target_y, target_yaw, target_kappa, target_t, target_v, target_w;

        // initialize pose and time
        target_x = robot_pose.x();
        target_y = robot_pose.y();
        target_yaw = robot_pose.theta();
        double begin_t = common::toSec(this->now()) + 5 * delta_t;
        double calc_t;
        for(int i=0; i<num_points; ++i){
            if(plan_t < acc_time){
                plan_s = acc_coeff.at(0) + acc_coeff.at(1) * plan_t + acc_coeff.at(2) * pow(plan_t, 2) + acc_coeff.at(3) * pow(plan_t, 3) + acc_coeff.at(4) * pow(plan_t, 4) + acc_coeff.at(5) * pow(plan_t, 5);
                plan_v = acc_coeff.at(1) + 2 * acc_coeff.at(2) * plan_t + 3 * acc_coeff.at(3) * pow(plan_t, 2) + 4 * acc_coeff.at(4) * pow(plan_t, 3) + 5 * acc_coeff.at(5) * pow(plan_t, 4);
            }
            else if(plan_t <= (acc_time + uni_time)){
                calc_t = plan_t - acc_time;
                plan_s = uni_coeff.at(0) + uni_coeff.at(1) * calc_t + uni_coeff.at(2) * pow(calc_t, 2) + uni_coeff.at(3) * pow(calc_t, 3) + uni_coeff.at(4) * pow(calc_t, 4) + uni_coeff.at(5) * pow(calc_t, 5);
                plan_v = uni_coeff.at(1) + 2 * uni_coeff.at(2) * calc_t + 3 * uni_coeff.at(3) * pow(calc_t, 2) + 4 * uni_coeff.at(4) * pow(calc_t, 3) + 5 * uni_coeff.at(5) * pow(calc_t, 4);
            }
            else{
                calc_t = plan_t - acc_time - uni_time;
                plan_s = dec_coeff.at(0) + dec_coeff.at(1) * calc_t + dec_coeff.at(2) * pow(calc_t, 2) + dec_coeff.at(3) * pow(calc_t, 3) + dec_coeff.at(4) * pow(calc_t, 4) + dec_coeff.at(5) * pow(calc_t, 5);
                plan_v = dec_coeff.at(1) + 2 * dec_coeff.at(2) * calc_t + 3 * dec_coeff.at(3) * pow(calc_t, 2) + 4 * dec_coeff.at(4) * pow(calc_t, 3) + 5 * dec_coeff.at(5) * pow(calc_t, 4);
            }

            target_x = plan_v * cos(target_yaw) * delta_t + target_x;
            target_y = plan_v * sin(target_yaw) * delta_t + target_y;
            target_v = plan_v;
            target_w = 0.0;
            target_yaw = common::normalize_angle(target_yaw + target_w * delta_t);
            target_kappa = (plan_v != 0) ? target_w / plan_v : 0.0;
            target_t = begin_t + plan_t;

            target_point.path_point = PathPoint(target_x, target_y, 0.0, target_yaw, target_kappa, plan_s);
            target_point.t = target_t;
            target_point.v = target_v;
            target_point.w = target_w;
            traj.push_back(target_point);

            plan_t += delta_t;
        }
        /*--------------end-------------------*/
        target_point.v = 0.0;
        target_point.w = 0.0;
        for(int i=0; i<20; ++i){
            target_t = begin_t + plan_t;
            target_point.t = target_t;
            traj.push_back(target_point);
            plan_t += delta_t;
        }
    }

    // 规划逆时针旋转固定角度轨迹
    void ControllerBase::planRotateTrajectory(
        std::vector<TrajPoint> & traj, 
        const geometry_msgs::Pose2D & robot_pose, 
        const geometry_msgs::Twist & robot_twist, 
        const double rotate_angle
    ){
        double rotate_yaw = std::fabs(rotate_angle);
        double rotate_max_w = max_w_;
        double rotate_max_aw = max_aw_ / 1.8;
        double w_acc_time = rotate_max_w / rotate_max_aw; // 加速时间
        double w_acc_yaw = 0.5 * w_acc_time * rotate_max_w; // 加速角度
        double w_dec_time = w_acc_time;
        double w_dec_yaw = w_acc_yaw;
        double w_uni_yaw = rotate_yaw - w_acc_yaw - w_dec_yaw;
        double w_uni_time = w_uni_yaw / rotate_max_w;
        if (w_uni_yaw < 0) {
            w_acc_time = sqrt(rotate_yaw / rotate_max_aw);
            w_acc_yaw = 0.5 * rotate_yaw;
            w_uni_time = 0;
            w_uni_yaw = 0;
            w_dec_time = w_acc_time;
            w_dec_yaw = w_acc_yaw;
        }

        double plan_total_time = w_acc_time + w_uni_time + w_dec_time;
        double plan_w_peak = w_acc_time * rotate_max_aw;

        std::array<double, 6> acc_coeff;
        std::array<double, 6> uni_coeff;
        std::array<double, 6> dec_coeff;
        fivetimesPlanTraj(acc_coeff, 0, 0, 0, 0, w_acc_time, w_acc_yaw, plan_w_peak, 0);
        fivetimesPlanTraj(uni_coeff, w_acc_time, w_acc_yaw, plan_w_peak, 0, w_acc_time + w_uni_time, w_acc_yaw + w_uni_yaw, plan_w_peak, 0);
        fivetimesPlanTraj(dec_coeff, w_acc_time + w_uni_time, w_acc_yaw + w_uni_yaw, plan_w_peak, 0, plan_total_time, rotate_yaw, 0, 0);

        TrajPoint target_point;
        double delta_t = 1.0 / control_rate_;
        unsigned int num_points = plan_total_time / delta_t;
        double plan_t = 0.0;
        double plan_yaw, plan_w;
        double target_x, target_y, target_yaw, target_s, target_t, target_w;

        // initialize pose and time
        target_x = robot_pose.x();
        target_y = robot_pose.y();
        target_yaw = robot_pose.theta();
        double begin_t = common::toSec(this->now()) + 5 * delta_t;
        double calc_t;
        for(int i=0; i<num_points; ++i){
            if(plan_t < w_acc_time){
                plan_yaw = acc_coeff.at(0) + acc_coeff.at(1) * plan_t + acc_coeff.at(2) * pow(plan_t, 2) + acc_coeff.at(3) * pow(plan_t, 3) + acc_coeff.at(4) * pow(plan_t, 4) + acc_coeff.at(5) * pow(plan_t, 5);
                plan_w = acc_coeff.at(1) + 2 * acc_coeff.at(2) * plan_t + 3 * acc_coeff.at(3) * pow(plan_t, 2) + 4 * acc_coeff.at(4) * pow(plan_t, 3) + 5 * acc_coeff.at(5) * pow(plan_t, 4);
            }
            else if(plan_t <= (w_acc_time + w_uni_time)){
                calc_t = plan_t - w_acc_time;
                plan_yaw = uni_coeff.at(0) + uni_coeff.at(1) * calc_t + uni_coeff.at(2) * pow(calc_t, 2) + uni_coeff.at(3) * pow(calc_t, 3) + uni_coeff.at(4) * pow(calc_t, 4) + uni_coeff.at(5) * pow(calc_t, 5);
                plan_w = uni_coeff.at(1) + 2 * uni_coeff.at(2) * calc_t + 3 * uni_coeff.at(3) * pow(calc_t, 2) + 4 * uni_coeff.at(4) * pow(calc_t, 3) + 5 * uni_coeff.at(5) * pow(calc_t, 4);
            }
            else{
                calc_t = plan_t - w_acc_time - w_uni_time;
                plan_yaw = dec_coeff.at(0) + dec_coeff.at(1) * calc_t + dec_coeff.at(2) * pow(calc_t, 2) + dec_coeff.at(3) * pow(calc_t, 3) + dec_coeff.at(4) * pow(calc_t, 4) + dec_coeff.at(5) * pow(calc_t, 5);
                plan_w = dec_coeff.at(1) + 2 * dec_coeff.at(2) * calc_t + 3 * dec_coeff.at(3) * pow(calc_t, 2) + 4 * dec_coeff.at(4) * pow(calc_t, 3) + 5 * dec_coeff.at(5) * pow(calc_t, 4);
            }

            target_yaw = common::normalize_angle(target_yaw + plan_w * delta_t);
            target_t = begin_t + plan_t;

            target_point.path_point = PathPoint(target_x, target_y, 0.0, target_yaw, 0.0, plan_yaw);
            target_point.t = target_t;
            target_point.v = 0.0;
            target_point.w = plan_w;
            traj.push_back(target_point);

            plan_t += delta_t;
        }
        /*--------------end-------------------*/
        target_point.w = 0.0;
        for(int i=0; i<20; ++i){
            target_t = begin_t + plan_t;
            target_point.t = target_t;
            traj.push_back(target_point);
            plan_t += delta_t;
        }
    }

    // 规划到目标点的轨迹（为什么不用五次多项式？）
    void ControllerBase::planLinearToPoseTrajectory(
        std::vector<TrajPoint> & traj, 
        const geometry_msgs::Pose2D & robot_pose, 
        const geometry_msgs::Twist & robot_twist, 
        const geometry_msgs::Pose2D & goal_pose
    ){

        double line_s = hypot(goal_pose.x() - robot_pose.x(), goal_pose.y() - robot_pose.y());
        double line_yaw = atan2(goal_pose.y() - robot_pose.y(), goal_pose.x() - robot_pose.x());

        double line_vertical_s = line_s;
        double line_max_v = max_vel_ * 0.5;
        double line_max_a = max_a_;
        // 加速段（缓慢加速）
        double acc_time = 2.0 * line_max_v / line_max_a;
        double acc_dis = 0.5 * line_max_v * acc_time;
        // 减速段（快速减速）
        double dec_time = acc_time / 2.0;
        double dec_dis = acc_dis / 2.0;
        // 匀速段
        double uni_dis = line_vertical_s - acc_dis - dec_dis;
        double uni_time = uni_dis / line_max_v;
        if(uni_dis < 0){
            acc_time = 2.0 * std::sqrt(2.0 / 3.0 * line_vertical_s / line_max_a);
            acc_dis = line_vertical_s * 2.0 / 3.0;
            uni_time = 0.0;
            uni_dis = 0.0;
            dec_time = acc_time / 2.0;
            dec_dis = acc_dis / 2.0;
        }
        double plan_total_time = acc_time + uni_time + dec_time;
        
        TrajPoint target_point;
        double delta_t = 1.0 / control_rate_;
        unsigned int num_points = plan_total_time / delta_t;
        double begin_t = common::toSec(this->now()) + 5 * delta_t;
        double plan_t = 0.0;
        double plan_s = 0.0;
        double plan_v;
        double plan_v_peak = acc_time * line_max_a / 2.0;
        double target_x, target_y, target_yaw, target_kappa, target_s, target_t, target_v, target_w;
        target_x = robot_pose.x();
        target_y = robot_pose.y();
        target_yaw = line_yaw;
        
        for(int i=0; i<num_points; ++i){
            if(plan_t < acc_time){
                plan_v = line_max_a / 2.0 * plan_t;
            }
            else if(plan_t < (acc_time + uni_time)){
                plan_v = line_max_v;
            }
            else if(plan_t < plan_total_time){
                plan_v = plan_v_peak - line_max_a * (plan_t - acc_time - uni_time);
            }

            plan_s += plan_v * delta_t;

            target_x += plan_v * cos(line_yaw) * delta_t;
            target_y += plan_v * sin(line_yaw) * delta_t;
            target_yaw = line_yaw;
            target_kappa = 0.0;
            target_s = plan_s;
            target_t = begin_t + plan_t;
            target_v = plan_v;
            target_w = 0.0;

            target_point.path_point = PathPoint(target_x, target_y, 0.0, target_yaw, target_kappa, target_s);
            target_point.t = target_t;
            target_point.v = target_v;
            target_point.w = target_w;
            traj.push_back(target_point);

            plan_t += delta_t;
        }
        /*--------------end-------------------*/
        target_point.v = 0.0;
        target_point.w = 0.0;
        for(int i=0; i<20; ++i){
            target_t = begin_t + plan_t;
            target_point.t = target_t;
            traj.push_back(target_point);
            plan_t += delta_t;
        }
    }

    void ControllerBase::planRotateToPoseTrajectory(
        std::vector<TrajPoint> & traj, 
        const geometry_msgs::Pose2D & robot_pose, 
        const geometry_msgs::Twist & robot_twist, 
        const double goal_yaw
    ){
        int direction_flag;
        double rotate_theta = common::shortest_angular_distance(robot_pose.theta(), goal_yaw);
        if(rotate_theta > 0){
            direction_flag = 1.0;
        }
        else{
            direction_flag = -1.0;
        }
        double rotate_yaw = std::fabs(rotate_theta);
        double rotate_max_w = max_w_;
        double rotate_max_aw = max_aw_ / 1.8;
        double w_acc_time = rotate_max_w / rotate_max_aw; // 加速时间
        double w_acc_yaw = 0.5 * w_acc_time * rotate_max_w; // 加速角度
        double w_dec_time = w_acc_time;
        double w_dec_yaw = w_acc_yaw;
        double w_uni_yaw = rotate_yaw - w_acc_yaw - w_dec_yaw;
        double w_uni_time = w_uni_yaw / rotate_max_w;
        if (w_uni_yaw < 0) {
            w_acc_time = sqrt(rotate_yaw / rotate_max_aw);
            w_acc_yaw = 0.5 * rotate_yaw;
            w_uni_time = 0;
            w_uni_yaw = 0;
            w_dec_time = w_acc_time;
            w_dec_yaw = w_acc_yaw;
        }

        double plan_total_time = w_acc_time + w_uni_time + w_dec_time;
        double plan_w_peak = w_acc_time * rotate_max_aw;

        std::array<double, 6> acc_coeff;
        std::array<double, 6> uni_coeff;
        std::array<double, 6> dec_coeff;
        fivetimesPlanTraj(acc_coeff, 0, 0, 0, 0, w_acc_time, w_acc_yaw, plan_w_peak, 0);
        fivetimesPlanTraj(uni_coeff, w_acc_time, w_acc_yaw, plan_w_peak, 0, w_acc_time + w_uni_time, w_acc_yaw + w_uni_yaw, plan_w_peak, 0);
        fivetimesPlanTraj(dec_coeff, w_acc_time + w_uni_time, w_acc_yaw + w_uni_yaw, plan_w_peak, 0, plan_total_time, rotate_yaw, 0, 0);

        TrajPoint target_point;
        double delta_t = 1.0 / control_rate_;
        unsigned int num_points = plan_total_time / delta_t;
        double plan_t = 0.0;
        double plan_yaw, plan_w;
        double target_x, target_y, target_yaw, target_s, target_t, target_w;

        // initialize pose and time
        target_x = robot_pose.x();
        target_y = robot_pose.y();
        target_yaw = robot_pose.theta();
        double begin_t = common::toSec(this->now()) + 5 * delta_t;
        double calc_t;
        for(int i=0; i<num_points; ++i){
            if(plan_t < w_acc_time){
                plan_yaw = acc_coeff.at(0) + acc_coeff.at(1) * plan_t + acc_coeff.at(2) * pow(plan_t, 2) + acc_coeff.at(3) * pow(plan_t, 3) + acc_coeff.at(4) * pow(plan_t, 4) + acc_coeff.at(5) * pow(plan_t, 5);
                plan_w = acc_coeff.at(1) + 2 * acc_coeff.at(2) * plan_t + 3 * acc_coeff.at(3) * pow(plan_t, 2) + 4 * acc_coeff.at(4) * pow(plan_t, 3) + 5 * acc_coeff.at(5) * pow(plan_t, 4);
            }
            else if(plan_t <= (w_acc_time + w_uni_time)){
                calc_t = plan_t - w_acc_time;
                plan_yaw = uni_coeff.at(0) + uni_coeff.at(1) * calc_t + uni_coeff.at(2) * pow(calc_t, 2) + uni_coeff.at(3) * pow(calc_t, 3) + uni_coeff.at(4) * pow(calc_t, 4) + uni_coeff.at(5) * pow(calc_t, 5);
                plan_w = uni_coeff.at(1) + 2 * uni_coeff.at(2) * calc_t + 3 * uni_coeff.at(3) * pow(calc_t, 2) + 4 * uni_coeff.at(4) * pow(calc_t, 3) + 5 * uni_coeff.at(5) * pow(calc_t, 4);
            }
            else{
                calc_t = plan_t - w_acc_time - w_uni_time;
                plan_yaw = dec_coeff.at(0) + dec_coeff.at(1) * calc_t + dec_coeff.at(2) * pow(calc_t, 2) + dec_coeff.at(3) * pow(calc_t, 3) + dec_coeff.at(4) * pow(calc_t, 4) + dec_coeff.at(5) * pow(calc_t, 5);
                plan_w = dec_coeff.at(1) + 2 * dec_coeff.at(2) * calc_t + 3 * dec_coeff.at(3) * pow(calc_t, 2) + 4 * dec_coeff.at(4) * pow(calc_t, 3) + 5 * dec_coeff.at(5) * pow(calc_t, 4);
            }

            target_w = direction_flag * plan_w;
            target_yaw = common::normalize_angle(target_yaw + target_w * delta_t);
            target_t = begin_t + plan_t;

            target_point.path_point = PathPoint(target_x, target_y, 0.0, target_yaw, 0.0, plan_yaw);
            target_point.t = target_t;
            target_point.v = 0.0;
            target_point.w = target_w;
            traj.push_back(target_point);

            plan_t += delta_t;
        }
        /*--------------end-------------------*/
        target_point.w = 0.0;
        target_point.path_point.yaw = goal_yaw;
        for(int i=0; i<20; ++i){
            target_t = begin_t + plan_t;
            target_point.t = target_t;
            traj.push_back(target_point);
            plan_t += delta_t;
        }
    }

    void ControllerBase::planBrakeTrajectory(
        std::vector<TrajPoint> & traj, 
        double robot_vel
    ){
        double robot_v = robot_vel;
        double line_max_a = std::fabs(min_a_);
        double dec_time = robot_v / line_max_a;
        double dec_dis = 0.5 * robot_v * dec_time;
        double plan_total_time = dec_time;

        TrajPoint target_point;
        double delta_t = 1.0 / control_rate_;
        unsigned int num_points = plan_total_time / delta_t;
        double begin_t = common::toSec(this->now()) + 5 * delta_t;
        double plan_t = 0.0;
        double plan_v;
        double target_t, target_v;

        for(int i=0; i<num_points; ++i){
            if(plan_t <= plan_total_time){
                plan_v = robot_v - line_max_a * plan_t;
            }

            target_v = plan_v;
            target_t = begin_t + plan_t;

            target_point.path_point = PathPoint(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
            target_point.t = target_t;
            target_point.v = target_v;
            target_point.w = 0.0;
            traj.push_back(target_point);

            plan_t += delta_t;
        }
        /*--------------end-------------------*/
        target_point.v = 0.0;
        for(int i=0; i<20; ++i){
            target_t = begin_t + plan_t;
            target_point.t = target_t;
            traj.push_back(target_point);
            plan_t += delta_t;
        }
    }

    void ControllerBase::fivetimesPlanTraj(
        std::array<double, 6> & coeff, 
        double t0, double s0, double v0, double a0, 
        double t1, double s1, double v1, double a1
    ){
        double dt = t1 - t0;
        double ds = s1 - s0;
        coeff.at(0) = s0;
        coeff.at(1) = v0;
        coeff.at(2) = a0 / 2.0;
        coeff.at(3) = 1 / (2 * pow(dt, 3)) * (20 * ds - (8 * v1 + 12 * v0) * dt - (3 * a0 - a1) * pow(dt, 2));
        coeff.at(4) = 1 / (2 * pow(dt, 4)) * (-30 * ds + (14 * v1 + 16 * v0) * dt + (3 * a0 - 2 * a1) * pow(dt, 2));
        coeff.at(5) = 1 / (2 * pow(dt, 5)) * (12 * ds - 6 * (v1 + v0) * dt + (a1 - a0) * pow(dt, 2));
    }

    double ControllerBase::get_control_rate(){
        return control_rate_;
    }

    bool ControllerBase::is_initialized(){
        return initialized_;
    }

}
