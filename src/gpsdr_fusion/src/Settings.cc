/**
 * This file is part of ORB-SLAM3
 *
 * Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
 * Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
 *
 * ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
 * the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with ORB-SLAM3.
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include "Settings.h"
#include "Common.h"

#include <opencv2/core/persistence.hpp>
#include <opencv2/core/eigen.hpp>

#include <iostream>

using namespace std;

namespace ORB_SLAM3
{
    // Inertial stuff
    Eigen::Vector3d Settings::accel_bias_;
    Eigen::Matrix3d Settings::Accel_calib_params_;
    Eigen::Vector3d Settings::gyro_bias_;
    Eigen::Matrix3d Settings::Gyro_calib_params_;
    Sophus::SE3f Settings::Tb_vitual_physical_;
    Eigen::Matrix3d Settings::Rb_vitual_physical_;
    Eigen::VectorXd Settings::ekfInitStd_(10);
    Eigen::VectorXd Settings::ekfDynamicNoise_(10);
    float Settings::ekfWheelplusScalefactor_;
    bool Settings::ekfUseWheelspeed_;
    float Settings::ekfInitAlignDist_;
    float Settings::rtkInterval_;
    Eigen::Vector3d Settings::piv_;
    Eigen::Matrix3d Settings::Riv_;
    Eigen::Vector3d Settings::pgv_;
    float Settings::imudT_;

    // other parameters
    std::string Settings::logRoot_ = "";
    std::string Settings::ProjectRoot_ = "";

    std::string Settings::gpsFile_;
    std::string Settings::ekfFile_;

    int Settings::viewRtkpathSize_;
    int Settings::viewEkfpathSize_;

    bool Settings::bViewer_;

    float Settings::preintegNoisePos_;
    float Settings::preintegNoiseYaw_;
    float Settings::preintegNoiseWheelSf_;

    bool Settings::bWheelSlipProcess_ = false;

    Eigen::Matrix3d Settings::T_N0_E0_ ;

    template <>
    float Settings::readParameter<float>(cv::FileStorage &fSettings, const std::string &name, bool &found, const bool required)
    {
        cv::FileNode node = fSettings[name];
        if (node.empty())
        {
            if (required)
            {
                std::cerr << name << " required parameter does not exist, aborting..." << std::endl;
                exit(-1);
            }
            else
            {
                std::cerr << name << " optional parameter does not exist..." << std::endl;
                found = false;
                return 0.0f;
            }
        }
        else if (!node.isReal())
        {
            std::cerr << name << " parameter must be a real number, aborting..." << std::endl;
            exit(-1);
        }
        else
        {
            found = true;
            return node.real();
        }
    }

    template <>
    int Settings::readParameter<int>(cv::FileStorage &fSettings, const std::string &name, bool &found, const bool required)
    {
        cv::FileNode node = fSettings[name];
        if (node.empty())
        {
            if (required)
            {
                std::cerr << name << " required parameter does not exist, aborting..." << std::endl;
                exit(-1);
            }
            else
            {
                std::cerr << name << " optional parameter does not exist..." << std::endl;
                found = false;
                return 0;
            }
        }
        else if (!node.isInt())
        {
            std::cerr << name << " parameter must be an integer number, aborting..." << std::endl;
            exit(-1);
        }
        else
        {
            found = true;
            return node.operator int();
        }
    }

    template <>
    string Settings::readParameter<string>(cv::FileStorage &fSettings, const std::string &name, bool &found, const bool required)
    {
        cv::FileNode node = fSettings[name];
        if (node.empty())
        {
            if (required)
            {
                std::cerr << name << " required parameter does not exist, aborting..." << std::endl;
                exit(-1);
            }
            else
            {
                std::cerr << name << " optional parameter does not exist..." << std::endl;
                found = false;
                return string();
            }
        }
        else if (!node.isString())
        {
            std::cerr << name << " parameter must be a string, aborting..." << std::endl;
            exit(-1);
        }
        else
        {
            found = true;
            return node.string();
        }
    }

    template <>
    cv::Mat Settings::readParameter<cv::Mat>(cv::FileStorage &fSettings, const std::string &name, bool &found, const bool required)
    {
        cv::FileNode node = fSettings[name];
        if (node.empty())
        {
            if (required)
            {
                std::cerr << name << " required parameter does not exist, aborting..." << std::endl;
                exit(-1);
            }
            else
            {
                std::cerr << name << " optional parameter does not exist..." << std::endl;
                found = false;
                return cv::Mat();
            }
        }
        else
        {
            found = true;
            return node.mat();
        }
    }

    void Settings::ParameterLoader(const std::string &configFile)
    {
        // Open settings file
        cv::FileStorage fSettings(configFile, cv::FileStorage::READ);

        if (!fSettings.isOpened())
        {
            cerr << "[ERROR]: could not open configuration file at: " << configFile << endl;
            cerr << "Aborting..." << endl;

            exit(-1);
        }
        else
        {
            cout << "Loading settings from " << configFile << endl;
        }

        readIMU(fSettings);

        readOtherParameters(fSettings);
    }

    void Settings::readIMU(cv::FileStorage &fSettings)
    {
        bool found;

        // set the default value
        accel_bias_.setZero();
        Accel_calib_params_.setIdentity();
        gyro_bias_.setZero();
        Gyro_calib_params_.setIdentity();

        double accel_bx = readParameter<double>(fSettings, "IMU.ba_x", found, false);
        double accel_by = readParameter<double>(fSettings, "IMU.ba_y", found, false);
        double accel_bz = readParameter<double>(fSettings, "IMU.ba_z", found, false);
        accel_bias_ << accel_bx, accel_by, accel_bz;

        double gyro_bx = readParameter<double>(fSettings, "IMU.bg_x", found, false);
        double gyro_by = readParameter<double>(fSettings, "IMU.bg_y", found, false);
        double gyro_bz = readParameter<double>(fSettings, "IMU.bg_z", found, false);
        gyro_bias_ << gyro_bx, gyro_by, gyro_bz;

        std::cout << "constant accel bias raw : " << accel_bias_.transpose() << std::endl;
        std::cout << "constant gyro bias raw : " << gyro_bias_.transpose() << std::endl;

        cv::Mat cvAccelcalibparams = readParameter<cv::Mat>(fSettings, "IMU.accel_calib_params", found, false);
        if (found)
        {
            Accel_calib_params_ = toMatrix3d(cvAccelcalibparams);
        }

        cv::Mat cvGyrocalibparams = readParameter<cv::Mat>(fSettings, "IMU.gyro_calib_params", found, false);
        if (found)
        {
            Gyro_calib_params_ = toMatrix3d(cvGyrocalibparams);
        }

        cv::Mat cvTbb = readParameter<cv::Mat>(fSettings, "IMU.b_vitual_phycical", found, false);
        if (found)
        {
            Tb_vitual_physical_ = toSophus(cvTbb);
        }

        cv::Mat cvTEN = readParameter<cv::Mat>(fSettings, "Output.Coordinate", found, false);
        if (found)
        {
            cv::cv2eigen(cvTEN, T_N0_E0_);
        }

        Rb_vitual_physical_ = Tb_vitual_physical_.rotationMatrix().cast<double>();

        accel_bias_ = Rb_vitual_physical_ * accel_bias_;
        gyro_bias_ = Rb_vitual_physical_ * gyro_bias_;

        // Ekf paramters
        cv::Mat cvEkfInitStd = readParameter<cv::Mat>(fSettings, "EKF.init_std", found);
        ekfInitStd_ << cvEkfInitStd.at<float>(0), cvEkfInitStd.at<float>(1), cvEkfInitStd.at<float>(2),
            cvEkfInitStd.at<float>(3), cvEkfInitStd.at<float>(4), cvEkfInitStd.at<float>(5),
            cvEkfInitStd.at<float>(6), cvEkfInitStd.at<float>(7), cvEkfInitStd.at<float>(8),
            cvEkfInitStd.at<float>(9);

        cv::Mat cvEkfDynamicNoise = readParameter<cv::Mat>(fSettings, "EKF.dynamic_noise", found);
        ekfDynamicNoise_ << cvEkfDynamicNoise.at<float>(0), cvEkfDynamicNoise.at<float>(1), cvEkfDynamicNoise.at<float>(2),
            cvEkfDynamicNoise.at<float>(3), cvEkfDynamicNoise.at<float>(4), cvEkfDynamicNoise.at<float>(5),
            cvEkfDynamicNoise.at<float>(6), cvEkfDynamicNoise.at<float>(7), cvEkfDynamicNoise.at<float>(8),
            cvEkfDynamicNoise.at<float>(9);

        ekfWheelplusScalefactor_ = readParameter<float>(fSettings, "EKF.WheelplusScalefactor", found, false);
        if (!found)
        {
            ekfWheelplusScalefactor_ = 1.0;
        }

        ekfUseWheelspeed_ = readParameter<int>(fSettings, "EkF.UseWheelspeed", found, false);
        if (!found)
        {
            ekfUseWheelspeed_ = false;
        }

        ekfInitAlignDist_ = readParameter<float>(fSettings, "EKF.InitAlignDist", found, false);
        if (!found)
        {
            ekfInitAlignDist_ = 10.0;
        }

        rtkInterval_ = readParameter<float>(fSettings, "RTK.Interval", found, false);
        if (!found)
        {
            rtkInterval_ = 0.1;
        }

        imudT_ = readParameter<float>(fSettings, "Imu.dT", found, false);
        if (!found)
        {
            imudT_ = 0;
        }

        cv::Mat cv_imu_piv = readParameter<cv::Mat>(fSettings, "IMU.piv", found);
        piv_ << cv_imu_piv.at<float>(0), cv_imu_piv.at<float>(1), cv_imu_piv.at<float>(2);

        cv::Mat cv_imu_riv = readParameter<cv::Mat>(fSettings, "IMU.riv", found);
        Eigen::Vector3d imu_riv;
        imu_riv << cv_imu_riv.at<float>(0), cv_imu_riv.at<float>(1), cv_imu_riv.at<float>(2);
        Riv_ = Attitude2R(imu_riv * M_PI / 180);

        cv::Mat cv_imu_pgv = readParameter<cv::Mat>(fSettings, "IMU.pgv", found);
        pgv_ << cv_imu_pgv.at<float>(0), cv_imu_pgv.at<float>(1), cv_imu_pgv.at<float>(2);

        preintegNoisePos_ = readParameter<float>(fSettings, "PreintegNoise_pos", found, true);
        preintegNoiseYaw_ = readParameter<float>(fSettings, "PreintegNoise_yaw", found, true);
        preintegNoiseYaw_ *= M_PI / 180;
        preintegNoiseWheelSf_ = readParameter<float>(fSettings, "PreintegNoise_wheelsf", found, true);

        std::cout << "constant accel bias in vitual imu frame: " << accel_bias_.transpose() << std::endl;
        std::cout << "constant gyro bias in vitual imu frame: " << gyro_bias_.transpose() << std::endl;

        std::cout << "Gyro_calib_params_ : " << Gyro_calib_params_ << std::endl;
        std::cout << "Accel_calib_params_ : " << Accel_calib_params_ << std::endl;

        std::cout << "Ekf InitStd : " << ekfInitStd_.transpose() << std::endl;
        std::cout << "Ekf Dynamic noise : " << ekfDynamicNoise_.transpose() << std::endl;
        std::cout << "Ekf WheelplusScalefactor : " << ekfWheelplusScalefactor_ << std::endl;
        std::cout << "EKF UseWheelspeed : " << ekfUseWheelspeed_ << std::endl;
        std::cout << "EKF InitAlignDist : " << ekfInitAlignDist_ << std::endl;
        std::cout << "Rkt Interval : " << rtkInterval_ << std::endl;
        std::cout << "Imu dT : " << imudT_ << std::endl;

        std::cout << "piv : " << piv_.transpose() << std::endl;
        std::cout << "riv : " << imu_riv.transpose() << std::endl;
        std::cout << "pgv : " << pgv_.transpose() << std::endl;
        std::cout << "Riv : \n"
                  << Riv_ << std::endl;

        std::cout << "preintegNoisePos [m / sqrt(s)] : " << preintegNoisePos_ << std::endl;
        std::cout << "preintegNoiseYaw [deg / sqrt(s)] : " << preintegNoiseYaw_ * 180 / M_PI << std::endl;
        std::cout << "preintegNoiseWheelSf : " << preintegNoiseWheelSf_ << std::endl;

        std::cout << "Output.Coordinate : " << T_N0_E0_ << std::endl;

        std::cout << "WheelSlipProcess : " << bWheelSlipProcess_ << std::endl;
    }

    void Settings::readOtherParameters(cv::FileStorage &fSettings)
    {
        bool found;
        // log root
        ProjectRoot_ = readParameter<string>(fSettings, "System.ProjectRoot", found, true);
        if (ProjectRoot_.empty())
        {
            cerr << "Project root is empty, please check your configuration file." << endl;
            exit(-1);
        }
        if (ProjectRoot_.back() == '/')
        {
            ProjectRoot_.pop_back(); // remove trailing slash if exists
        }

        logRoot_ = ProjectRoot_ + "/logs/";

        gpsFile_ = readParameter<string>(fSettings, "GpsFile", found, false);
        if (!found)
        {
            gpsFile_ = "gps.txt";
        }

        ekfFile_ = readParameter<string>(fSettings, "EkfFile", found, false);
        if (!found)
        {
            ekfFile_ = "ekf.txt";
        }

        viewRtkpathSize_ = readParameter<int>(fSettings, "View.RtkpathSize", found, false);
        if (!found)
        {
            viewRtkpathSize_ = 1000;
        }

        viewEkfpathSize_ = readParameter<int>(fSettings, "View.EkfpathSize", found, false);
        if (!found)
        {
            viewEkfpathSize_ = 1000;
        }

        bViewer_ = readParameter<int>(fSettings, "Viewer.bViewer", found, false);
        if (!found)
        {
            bViewer_ = true;
        }

        bWheelSlipProcess_ = readParameter<int>(fSettings, "WheelSlipProcess", found, false);
        if (!found)
        {
            bWheelSlipProcess_ = false;
        }
    }

    ostream &operator<<(std::ostream &output, const Settings &settings)
    {
        output << "SLAM settings: " << endl;
        return output;
    }

    Eigen::Matrix<double, 3, 3> Settings::toMatrix3d(const cv::Mat &cvMat3)
    {
        Eigen::Matrix<double, 3, 3> M;

        M << cvMat3.at<float>(0, 0), cvMat3.at<float>(0, 1), cvMat3.at<float>(0, 2),
            cvMat3.at<float>(1, 0), cvMat3.at<float>(1, 1), cvMat3.at<float>(1, 2),
            cvMat3.at<float>(2, 0), cvMat3.at<float>(2, 1), cvMat3.at<float>(2, 2);

        return M;
    }

    Eigen::Matrix<double, 3, 1> Settings::toVector3d(const cv::Mat &cvVector)
    {
        Eigen::Matrix<double, 3, 1> v;
        v << cvVector.at<float>(0), cvVector.at<float>(1), cvVector.at<float>(2);

        return v;
    }

    Sophus::SE3<float> Settings::toSophus(const cv::Mat &T)
    {
        Eigen::Matrix<double, 3, 3> eigMat = toMatrix3d(T.rowRange(0, 3).colRange(0, 3));
        Eigen::Quaternionf q(eigMat.cast<float>());

        Eigen::Matrix<float, 3, 1> t = toVector3d(T.rowRange(0, 3).col(3)).cast<float>();

        return Sophus::SE3<float>(q, t);
    }

};
