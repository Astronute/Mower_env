#include "controller_base.h"

namespace CB{

    const double PI = 3.141592653589793;

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

    bool ControllerBase::findTargetTrajTimepoint(TrajPoint & target_point, const std::vector<TrajPoint> & target_traj, double target_t){
        double target_x, target_y, target_yaw, target_kappa, target_s, target_v, target_w;

        auto linearEquation = [&](double x1, double y1, double x2, double y2, double x) -> double{
            double dx = x2 - x1;
            if(std::fabs(dx) < 1e-9){
                return y1;
            }

            return (x - x1) * (y2 - y1) / dx + y1;
        };

        auto linearEquationAngle = [&](double x1, double y1, double x2, double y2, double x) -> double{
            double dx = x2 - x1;
            if(std::fabs(dx) < 1e-9){
                return y1;
            }

            double angle = normalize_angle(y2 - y1);
            return normalize_angle((x - x1) * angle / dx + y1);
        };

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
                target_x = linearEquation(prev_point.t, prev_point.path_point.x, next_point.t, next_point.path_point.x, target_t);
                target_y = linearEquation(prev_point.t, prev_point.path_point.y, next_point.t, next_point.path_point.y, target_t);
                target_yaw = linearEquationAngle(prev_point.t, prev_point.path_point.yaw, next_point.t, next_point.path_point.yaw, target_t);
                target_kappa = linearEquation(prev_point.t, prev_point.path_point.kappa, next_point.t, next_point.path_point.kappa, target_t);
                target_s = linearEquation(prev_point.t, prev_point.path_point.s, next_point.t, next_point.path_point.s, target_t);
                target_v = linearEquation(prev_point.t, prev_point.v, next_point.t, next_point.v, target_t);
                target_w = linearEquation(prev_point.t, prev_point.w, next_point.t, next_point.w, target_t);

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

    double ControllerBase::normalize_angle(double angle){
        double TWO_PI = 2.0 * PI;
        angle = std::fmod(angle, TWO_PI);
        if(angle > PI){
            angle = angle - TWO_PI;
        }
        else if(angle <= -PI){
            angle = angle + TWO_PI;
        }
        return angle;
    }

    double ControllerBase::get_control_rate(){
        return control_rate_;
    }

    bool ControllerBase::is_initialized(){
        return initialized_;
    }

}
