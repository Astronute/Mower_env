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

#ifndef IMUTYPES_H
#define IMUTYPES_H

#include <vector>
#include <utility>
// #include <opencv2/core/core.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <mutex>
#include <memory>

namespace ORB_SLAM3
{

    struct GpsOdometry
    {
        double t;
        double x3_t;
        double lon;
        double lat;
        double h;
        double ref_east;  // 东
        double ref_north; // 北
        double east_vel;
        double north_vel;
        double speed;
        double direction;
        int numSv;
        double hdop;

        int valid; // 数据是否有效 1 数据有效， 0 数据无效
        int fs;    // 当前定位质量 4

        Eigen::Vector3d pos;
        Eigen::Vector3d vel;
        Eigen::Matrix3d pos_cov;

        Eigen::Matrix3d vel_cov;
        double heading;
    };

    using GpsOdometryPtr = std::shared_ptr<GpsOdometry>;

    enum class EnMotionStatus
    {
        STOP = 0,
        MOVE = 1,
    };

    struct MotionStatus
    {
        double t;
        EnMotionStatus status;
    };

    struct StopImuInfo
    {
        double t0;
        double t;

        double time_len_mean;
        Eigen::Vector2d pitchroll_mean;
        Eigen::Vector3d gyro_bias_mean;
        // double timelen;

        bool available;
    };

    struct WheelOdometer
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        double t;

        // 原始输出的轮速脉冲距离（累计值）
        double whlplus_l;
        double whlplus_r;

        // 上一个数据到当前的轮速脉冲距离
        double pre_t;
        double d_l;
        double d_r;
        bool valid;

        // 与前面一定时间间隔的数据计算速度
        double vel;
        bool vel_valid;

        Eigen::Vector3d vel_3d_from_rtk; // 用rtk速度计算的后轴中心速度，用于处理打滑或抬起的情况
        bool use_rtk_vel;

        Eigen::Vector3d anguler_vel;

        WheelOdometer() : use_rtk_vel(false) {}
    };

    namespace IMU
    {
        const float GRAVITY_VALUE = 9.81;

        // IMU measurement (gyro, accelerometer and timestamp)
        class Point
        {
        public:
            Point(const float &acc_x, const float &acc_y, const float &acc_z,
                  const float &ang_vel_x, const float &ang_vel_y, const float &ang_vel_z, double yaw,
                  const double &timestamp) : a(acc_x, acc_y, acc_z), w(ang_vel_x, ang_vel_y, ang_vel_z), yaw(yaw), t(timestamp) {}
            // Point(const cv::Point3f Acc, const cv::Point3f Gyro, double yaw, double &timestamp)
            //     : a(Acc.x, Acc.y, Acc.z), w(Gyro.x, Gyro.y, Gyro.z), yaw(yaw), t(timestamp) {}
            Point(const Eigen::Vector3f &Acc, const Eigen::Vector3f &Gyro, double yaw, const double &timestamp)
                : a(Acc), w(Gyro), yaw(yaw), t(timestamp) {}

            Point(const float &acc_x, const float &acc_y, const float &acc_z,
                  const float &ang_vel_x, const float &ang_vel_y, const float &ang_vel_z, 
                  const double &timestamp) : a(acc_x, acc_y, acc_z), w(ang_vel_x, ang_vel_y, ang_vel_z), yaw(0.0), t(timestamp) {}
            // Point(const cv::Point3f Acc, const cv::Point3f Gyro, double &timestamp)
            //     : a(Acc.x, Acc.y, Acc.z), w(Gyro.x, Gyro.y, Gyro.z), yaw(0.0), t(timestamp) {}
            Point(const Eigen::Vector3f &Acc, const Eigen::Vector3f &Gyro, const double &timestamp)
                : a(Acc), w(Gyro), yaw(0.0), t(timestamp) {}

        public:
            Eigen::Vector3f a;
            Eigen::Vector3f w;
            double t;
            double x3_t;
            double yaw;
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        };
    }

} // namespace ORB_SLAM2

#endif // IMUTYPES_H
