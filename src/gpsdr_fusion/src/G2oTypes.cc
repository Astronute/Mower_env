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

#include "G2oTypes.h"
#include "ImuTypes.h"
#include "Common.h"
#include "Settings.h"

// #include "Thirdparty/Sophus/sophus/geometry.hpp"
// #include "Thirdparty/Sophus/sophus/sim3.hpp"
// #include "Thirdparty/Sophus/sophus/so3.hpp"
#include "sophus/geometry.hpp"
#include "sophus/sim3.hpp"
#include "sophus/so3.hpp"

namespace ORB_SLAM3
{

    ImuCamPose::ImuCamPose(Eigen::Vector2d &pos_xy, double yaw)
    {
        twb = pos_xy;
        Rwb << cos(yaw), -sin(yaw), sin(yaw), cos(yaw);
    }

    void ImuCamPose::Update(const double *pu)
    {
        Eigen::Vector3d ur, ut;
        ur << 0, 0, pu[0];
        ut << pu[1], pu[2], 0;

        Eigen::Matrix3d Rwb3d = mat2dToMat3d(Rwb);
        Eigen::Vector3d twb3d = Vector2dToVector3d(twb);

        // Update body pose
        twb3d += Rwb3d * ut;
        Rwb3d = Rwb3d * ExpSO3(ur);

        // Normalize rotation after 3 updates
        its++;
        if (its >= 3)
        {
            Rwb3d = NormalizeRotation(Rwb3d);
            its = 0;
        }

        Rwb = Rwb3d.block<2,2>(0,0);
        twb = twb3d.topRows(2);
    }

    // SO3 FUNCTIONS
    Eigen::Matrix3d ExpSO3(const Eigen::Vector3d &w)
    {
        return ExpSO3(w[0], w[1], w[2]);
    }

    Eigen::Matrix3d ExpSO3(const double x, const double y, const double z)
    {
        const double d2 = x * x + y * y + z * z;
        const double d = sqrt(d2);
        Eigen::Matrix3d W;
        W << 0.0, -z, y, z, 0.0, -x, -y, x, 0.0;
        if (d < 1e-5)
        {
            Eigen::Matrix3d res = Eigen::Matrix3d::Identity() + W + 0.5 * W * W;
            return NormalizeRotation(res);
        }
        else
        {
            Eigen::Matrix3d res = Eigen::Matrix3d::Identity() + W * sin(d) / d + W * W * (1.0 - cos(d)) / d2;
            return NormalizeRotation(res);
        }
    }

    Eigen::Vector3d LogSO3(const Eigen::Matrix3d &R)
    {
        const double tr = R(0, 0) + R(1, 1) + R(2, 2);
        Eigen::Vector3d w;
        w << (R(2, 1) - R(1, 2)) / 2, (R(0, 2) - R(2, 0)) / 2, (R(1, 0) - R(0, 1)) / 2;
        const double costheta = (tr - 1.0) * 0.5f;
        if (costheta > 1 || costheta < -1)
            return w;
        const double theta = acos(costheta);
        const double s = sin(theta);
        if (fabs(s) < 1e-5)
            return w;
        else
            return theta * w / s;
    }

    Eigen::Matrix3d InverseRightJacobianSO3(const Eigen::Vector3d &v)
    {
        return InverseRightJacobianSO3(v[0], v[1], v[2]);
    }

    Eigen::Matrix3d InverseRightJacobianSO3(const double x, const double y, const double z)
    {
        const double d2 = x * x + y * y + z * z;
        const double d = sqrt(d2);

        Eigen::Matrix3d W;
        W << 0.0, -z, y, z, 0.0, -x, -y, x, 0.0;
        if (d < 1e-5)
            return Eigen::Matrix3d::Identity();
        else
            return Eigen::Matrix3d::Identity() + W / 2 + W * W * (1.0 / d2 - (1.0 + cos(d)) / (2.0 * d * sin(d)));
    }

    Eigen::Matrix3d RightJacobianSO3(const Eigen::Vector3d &v)
    {
        return RightJacobianSO3(v[0], v[1], v[2]);
    }

    Eigen::Matrix3d RightJacobianSO3(const double x, const double y, const double z)
    {
        const double d2 = x * x + y * y + z * z;
        const double d = sqrt(d2);

        Eigen::Matrix3d W;
        W << 0.0, -z, y, z, 0.0, -x, -y, x, 0.0;
        if (d < 1e-5)
        {
            return Eigen::Matrix3d::Identity();
        }
        else
        {
            return Eigen::Matrix3d::Identity() - W * (1.0 - cos(d)) / d2 + W * W * (d - sin(d)) / (d2 * d);
        }
    }

    Eigen::Matrix3d Skew(const Eigen::Vector3d &w)
    {
        Eigen::Matrix3d W;
        W << 0.0, -w[2], w[1], w[2], 0.0, -w[0], -w[1], w[0], 0.0;
        return W;
    }

    EdgeRelativePose::EdgeRelativePose(const Eigen::Matrix2d &Rb1b2_, const Eigen::Vector2d &tb1b2_)
    {
        Rb1b2 = mat2dToMat3d(Rb1b2_);
        tb1b2 = Vector2dToVector3d(tb1b2_);
    }

    void EdgeRelativePose::computeError()
    {
        const VertexPose *VP1 = static_cast<const VertexPose *>(_vertices[0]);
        const VertexPose *VP2 = static_cast<const VertexPose *>(_vertices[1]);

        Eigen::Matrix3d Rwb1_3d = mat2dToMat3d(VP1->estimate().Rwb);
        Eigen::Matrix3d Rwb2_3d = mat2dToMat3d(VP2->estimate().Rwb);
        Eigen::Vector3d twb1_3d = Vector2dToVector3d(VP1->estimate().twb);
        Eigen::Vector3d twb2_3d = Vector2dToVector3d(VP2->estimate().twb);

        Eigen::Matrix3d Rb1b2_tmp = Rwb1_3d.transpose() * Rwb2_3d;
        Eigen::Vector3d tb1b2_tmp = Rwb1_3d.transpose() * (twb2_3d - twb1_3d);

        Eigen::Vector3d er = LogSO3(Rb1b2.transpose() * Rb1b2_tmp);
        Eigen::Vector3d et = tb1b2_tmp - tb1b2;

        _error << er(2), et(0), et(1);
    }

    void EdgeRelativePose::linearizeOplus()
    {
        const VertexPose *VP1 = static_cast<const VertexPose *>(_vertices[0]);
        const VertexPose *VP2 = static_cast<const VertexPose *>(_vertices[1]);

        Eigen::Matrix3d Rwb1_3d = mat2dToMat3d(VP1->estimate().Rwb);
        Eigen::Matrix3d Rwb2_3d = mat2dToMat3d(VP2->estimate().Rwb);
        Eigen::Vector3d twb1_3d = Vector2dToVector3d(VP1->estimate().twb);
        Eigen::Vector3d twb2_3d = Vector2dToVector3d(VP2->estimate().twb);

        Eigen::Matrix3d Rb1b2_tmp = Rwb1_3d.transpose() * Rwb2_3d;
        Eigen::Vector3d tb1b2_tmp = Rwb1_3d.transpose() * (twb2_3d - twb1_3d);
        Eigen::Vector3d er = LogSO3(Rb1b2.transpose() * Rb1b2_tmp);
        Eigen::Matrix3d invJr = InverseRightJacobianSO3(er);

        _jacobianOplusXi.setZero();
        // 姿态误差对第一帧的姿态
        Eigen::Matrix3d mat3d_tmp;
        mat3d_tmp = -invJr * Rwb2_3d.transpose() * Rwb1_3d;
        _jacobianOplusXi.block<1, 1>(0, 0) = mat3d_tmp.block<1,1>(2,2);
        // 位置误差对第一帧的姿态
        mat3d_tmp = Sophus::SO3d::hat(tb1b2_tmp);
        _jacobianOplusXi.block<2, 1>(1, 0) = mat3d_tmp.block<2,1>(0,2);
        // 位置误差对第一帧的位置
        _jacobianOplusXi.block<2, 2>(1, 1) = -Eigen::Matrix2d::Identity();

        //
        _jacobianOplusXj.setZero();
        // 姿态误差对第二帧的姿态
        _jacobianOplusXj.block<1, 1>(0, 0) = invJr.block<1,1>(2,2);
        // 位置误差对第二帧的位置
        mat3d_tmp = Rwb1_3d.transpose() * Rwb2_3d;
        _jacobianOplusXj.block<2, 2>(1, 1) = mat3d_tmp.block<2,2>(0,0);
    }

    void EdgeGpsPositionXY::computeError()
    {
        const VertexPose *VP = static_cast<const VertexPose *>(_vertices[0]);

        // 修改：把待优化状态的位置定义在后轴中心
        // gps位置转换到后轴中心计算残差
        _error = gps_pos + VP->estimate().Rwb * Settings::pgv().topRows(2) - VP->estimate().twb;
        
        // _error = gps_pos - VP->estimate().twb;
    }

    void EdgeGpsPositionXY::linearizeOplus()
    {
        const VertexPose *VP = static_cast<const VertexPose *>(_vertices[0]);

        _jacobianOplusXi.setZero();

        // 修改：把待优化状态的位置定义在后轴中心
        // 增加对姿态的雅可比（通过杆臂起作用）
        Eigen::Matrix3d Rwb1_3d = mat2dToMat3d(VP->estimate().Rwb);
        Eigen::Matrix3d mat_pgv = Sophus::SO3d::hat(Settings::pgv());
        Eigen::Matrix3d mat3d_tmp = -Rwb1_3d * mat_pgv;
        _jacobianOplusXi.block<2, 1>(0, 0) = mat3d_tmp.block<2,1>(0,2);

        _jacobianOplusXi.block<2, 2>(0, 1) = -VP->estimate().Rwb;
    }

    EdgePriorStatePose::EdgePriorStatePose(const Eigen::Matrix2d &Rwb_, const Eigen::Vector2d &twb_) 
    {
        Rwb = mat2dToMat3d(Rwb_);
        twb = Vector2dToVector3d(twb_);
    }

    void EdgePriorStatePose::computeError()
    {
        const VertexPose *VP = static_cast<const VertexPose *>(_vertices[0]);

        Eigen::Matrix3d Rwb_estim_3d = mat2dToMat3d(VP->estimate().Rwb);
        Eigen::Vector3d twb_estim_3d = Vector2dToVector3d(VP->estimate().twb);

        const Eigen::Vector3d er = LogSO3(Rwb.transpose() * Rwb_estim_3d);
        const Eigen::Vector3d et = Rwb.transpose() * (twb_estim_3d - twb);

        _error << er(2), et(0), et(1);
    }

    void EdgePriorStatePose::linearizeOplus()
    {
        const VertexPose *VP = static_cast<const VertexPose *>(_vertices[0]);

        Eigen::Matrix3d Rwb_estim_3d = mat2dToMat3d(VP->estimate().Rwb);
        Eigen::Vector3d twb_estim_3d = Vector2dToVector3d(VP->estimate().twb);

        const Eigen::Vector3d er = LogSO3(Rwb.transpose() * Rwb_estim_3d);
        _jacobianOplusXi.setZero();
        Eigen::Matrix3d mat3d_tmp;
        mat3d_tmp = InverseRightJacobianSO3(er);
        _jacobianOplusXi.block<1, 1>(0, 0) = mat3d_tmp.block<1,1>(2,2);
        mat3d_tmp = Rwb.transpose() * Rwb_estim_3d;
        _jacobianOplusXi.block<2, 2>(1, 1) = mat3d_tmp.block<2,2>(0,0);
    }

    /* ----------------- GPS速度观测 -----------------
    */
    void EdgeGpsVelocity::computeError()
    {
        const VertexPose *VP1 = static_cast<const VertexPose *>(_vertices[0]);
        const VertexPose *VP2 = static_cast<const VertexPose *>(_vertices[1]);

        Eigen::Matrix3d Rwb1_3d = mat2dToMat3d(VP1->estimate().Rwb);
        Eigen::Matrix3d Rwb2_3d = mat2dToMat3d(VP2->estimate().Rwb);
        Eigen::Vector3d twb1_3d = Vector2dToVector3d(VP1->estimate().twb);
        Eigen::Vector3d twb2_3d = Vector2dToVector3d(VP2->estimate().twb);

        Eigen::Vector3d pgv_in_w_1 = Rwb1_3d * Settings::pgv();
        Eigen::Vector3d pgv_in_w_2 = Rwb2_3d * Settings::pgv();

        Eigen::Vector3d gps_vel_3d(gps_vel(0), gps_vel(1), 0);
        // 位置从后轴中心转到rtk中心
        Eigen::Vector3d et = (twb2_3d - pgv_in_w_2) - (twb1_3d - pgv_in_w_1) - gps_vel_3d * dt;

        _error << et(0), et(1);
    }

    void EdgeGpsVelocity::linearizeOplus()
    {
        const VertexPose *VP1 = static_cast<const VertexPose *>(_vertices[0]);
        const VertexPose *VP2 = static_cast<const VertexPose *>(_vertices[1]);

        Eigen::Matrix3d Rwb1_3d = mat2dToMat3d(VP1->estimate().Rwb);
        Eigen::Matrix3d Rwb2_3d = mat2dToMat3d(VP2->estimate().Rwb);
        Eigen::Vector3d twb1_3d = Vector2dToVector3d(VP1->estimate().twb);
        Eigen::Vector3d twb2_3d = Vector2dToVector3d(VP2->estimate().twb);

        Eigen::Vector3d pgv_in_w_1 = Rwb1_3d * Settings::pgv();
        Eigen::Vector3d pgv_in_w_2 = Rwb2_3d * Settings::pgv();

        Eigen::Matrix3d mat_pgv = Sophus::SO3d::hat(Settings::pgv());
        Eigen::Matrix3d mat3d_tmp;
        
        mat3d_tmp = -Rwb1_3d * mat_pgv;
        _jacobianOplusXi.setZero();
        _jacobianOplusXi.block<2, 1>(0, 0) = mat3d_tmp.block<2,1>(0,2);
        _jacobianOplusXi.block<2, 2>(0, 1) = -Rwb1_3d.block<2, 2>(0, 0);

        mat3d_tmp = Rwb2_3d * mat_pgv;
        _jacobianOplusXj.setZero();         
        _jacobianOplusXj.block<2, 1>(0, 0) = mat3d_tmp.block<2,1>(0,2);
        _jacobianOplusXj.block<2, 2>(0, 1) = Rwb2_3d.block<2, 2>(0, 0);
    }

}
