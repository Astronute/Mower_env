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

#ifndef ORB_SLAM3_SETTINGS_H
#define ORB_SLAM3_SETTINGS_H


#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>

// #include "Thirdparty/Sophus/sophus/se3.hpp"
#include "sophus/se3.hpp"

namespace ORB_SLAM3
{
    class Settings
    {
    public:

        /*
         * Delete default constructor
         */
        Settings() = default;

        /*
         * Constructor from file
         */
        static void ParameterLoader(const std::string &configFile);

        /*
         * Ostream operator overloading to dump settings to the terminal
         */
        friend std::ostream &operator<<(std::ostream &output, const Settings &s);

        /*
         * Getter methods
         */
        static Eigen::Vector3d accelBias() { return accel_bias_; }
        static Eigen::Matrix3d Accelcalibparams() { return Accel_calib_params_; }
        static Eigen::Vector3d gyroBias() { return gyro_bias_; }
        static Eigen::Matrix3d Gyrocalibparams() { return Gyro_calib_params_; }
        static Eigen::Matrix3d RbVitualPhysical() { return Rb_vitual_physical_; }
        static Eigen::VectorXd ekfInitStd() { return ekfInitStd_; }
        static Eigen::VectorXd ekfDynamicNoise() { return ekfDynamicNoise_; }
        static Eigen::Vector3d piv() { return piv_; }
        static Eigen::Matrix3d Riv() { return Riv_;}
        static Eigen::Vector3d pgv() { return pgv_; }
        static float ekfWheelplusScalefactor() {return ekfWheelplusScalefactor_;}
        static bool ekfUseWheelspeed() {return ekfUseWheelspeed_;}
        static float rtkInterval() {return rtkInterval_;}
        static float ekfInitAlignDist() {return ekfInitAlignDist_;}
        static std::string logRoot() { return logRoot_; }
        static Eigen::Matrix3d RotateEN() { return T_N0_E0_; }

    private:
        template <typename T>
        static T readParameter(cv::FileStorage &fSettings, const std::string &name, bool &found, const bool required = true)
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
                    return T();
                }
            }
            else
            {
                found = true;
                return (T)node;
            }
        }

        static void readIMU(cv::FileStorage &fSettings);
        static void readOtherParameters(cv::FileStorage &fSettings);

        static Eigen::Matrix<double, 3, 3> toMatrix3d(const cv::Mat &cvMat3);
        static Eigen::Matrix<double, 3, 1> toVector3d(const cv::Mat &cvVector);
        static Sophus::SE3<float> toSophus(const cv::Mat &T);

    private:
        /*
         * Inertial stuff
         */
        static Eigen::Vector3d accel_bias_;
        static Eigen::Matrix3d Accel_calib_params_;
        static Eigen::Vector3d gyro_bias_;
        static Eigen::Matrix3d Gyro_calib_params_;
        static Sophus::SE3f Tb_vitual_physical_;
        static Eigen::Matrix3d Rb_vitual_physical_;

        static Eigen::VectorXd ekfInitStd_;
        static Eigen::VectorXd ekfDynamicNoise_;
        static Eigen::Vector3d piv_;
        static Eigen::Matrix3d Riv_;
        static Eigen::Vector3d pgv_;
        static float ekfWheelplusScalefactor_;
        static bool ekfUseWheelspeed_;
        static float ekfInitAlignDist_;
        static float rtkInterval_;

        static Eigen::Matrix3d T_N0_E0_;
        
    public:
        static std::string logRoot_;
        static std::string ProjectRoot_;

        static std::string gpsFile_;
        static std::string ekfFile_;   

        static int viewRtkpathSize_;
        static int viewEkfpathSize_;

        static float imudT_;

        static bool bViewer_;

        static float preintegNoisePos_;
        static float preintegNoiseYaw_;
        static float preintegNoiseWheelSf_;

        static bool bWheelSlipProcess_;

    };
};

#endif // ORB_SLAM3_SETTINGS_H
