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

#ifndef G2OTYPES_H
#define G2OTYPES_H

// #include "Thirdparty/g2o/g2o/core/base_vertex.h"
// #include "Thirdparty/g2o/g2o/core/base_binary_edge.h"
// #include "Thirdparty/g2o/g2o/types/types_sba.h"
// #include "Thirdparty/g2o/g2o/core/base_multi_edge.h"
// #include "Thirdparty/g2o/g2o/core/base_unary_edge.h"

#include "g2o/core/base_vertex.h"
#include "g2o/core/base_binary_edge.h"
#include "g2o/types/types_sba.h"
#include "g2o/core/base_multi_edge.h"
#include "g2o/core/base_unary_edge.h"

#include <opencv2/core/core.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <math.h>

namespace ORB_SLAM3
{

    typedef Eigen::Matrix<double, 6, 1> Vector6d;
    typedef Eigen::Matrix<double, 9, 1> Vector9d;
    typedef Eigen::Matrix<double, 12, 1> Vector12d;
    typedef Eigen::Matrix<double, 15, 1> Vector15d;
    typedef Eigen::Matrix<double, 12, 12> Matrix12d;
    typedef Eigen::Matrix<double, 15, 15> Matrix15d;
    typedef Eigen::Matrix<double, 9, 9> Matrix9d;

    Eigen::Matrix3d ExpSO3(const double x, const double y, const double z);
    Eigen::Matrix3d ExpSO3(const Eigen::Vector3d &w);

    Eigen::Vector3d LogSO3(const Eigen::Matrix3d &R);

    Eigen::Matrix3d InverseRightJacobianSO3(const Eigen::Vector3d &v);
    Eigen::Matrix3d RightJacobianSO3(const Eigen::Vector3d &v);
    Eigen::Matrix3d RightJacobianSO3(const double x, const double y, const double z);

    Eigen::Matrix3d Skew(const Eigen::Vector3d &w);
    Eigen::Matrix3d InverseRightJacobianSO3(const double x, const double y, const double z);

    // Eigen::Matrix3d Attitude2R(const Eigen::Vector3d &atti);
    // Eigen::Vector3d getAttitudeFromR(const Eigen::Matrix3d &Rb2L);

    template <typename T = double>
    Eigen::Matrix<T, 3, 3> NormalizeRotation(const Eigen::Matrix<T, 3, 3> &R)
    {
        Eigen::JacobiSVD<Eigen::Matrix<T, 3, 3>> svd(R, Eigen::ComputeFullU | Eigen::ComputeFullV);
        return svd.matrixU() * svd.matrixV().transpose();
    }
    
    class ImuCamPose
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        ImuCamPose() {}
        ImuCamPose(Eigen::Vector2d &pos_xy, double yaw);

        // pu : dtheta, dx, dy
        void Update(const double *pu);

    public:
        Eigen::Matrix2d Rwb;
        Eigen::Vector2d twb;
        int its;
    };

    // Optimizable parameters are IMU pose
    class VertexPose : public g2o::BaseVertex<3, ImuCamPose>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        
        VertexPose() {}
        VertexPose(ImuCamPose &pose)
        {
            setEstimate(pose);
        }

        virtual bool read(std::istream &is) { return false; }
        virtual bool write(std::ostream &os) const { return false; }

        virtual void setToOriginImpl()
        {
            
        }

        virtual void oplusImpl(const double *update_)
        {
            _estimate.Update(update_);
            updateCache();
        }
    };

    // 相邻顶点之间的位姿变化约束
    class EdgeRelativePose : public g2o::BaseBinaryEdge<3, Eigen::Vector3d, VertexPose, VertexPose>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        EdgeRelativePose(const Eigen::Matrix2d &Rb1b2_, const Eigen::Vector2d &tb1b2_);

        virtual bool read(std::istream &is) { return false; }
        virtual bool write(std::ostream &os) const { return false; }

        void computeError();
        virtual void linearizeOplus();

        Eigen::Matrix<double, 6, 6> GetHessian()
        {
            linearizeOplus();
            Eigen::Matrix<double, 3, 6> J;
            J.block<3, 3>(0, 0) = _jacobianOplusXi;
            J.block<3, 3>(0, 3) = _jacobianOplusXj;
            return J.transpose() * information() * J;
        }

        Eigen::Matrix3d Rb1b2;
        Eigen::Vector3d tb1b2;
    };

    // Gps平面位置观测
    class EdgeGpsPositionXY : public g2o::BaseUnaryEdge<2, Eigen::Vector2d, VertexPose>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        EdgeGpsPositionXY(const Eigen::Vector2d &gps_pos_) : gps_pos(gps_pos_) {}

        virtual bool read(std::istream &is) { return false; }
        virtual bool write(std::ostream &os) const { return false; }

        void computeError();
        virtual void linearizeOplus();

        Eigen::Matrix3d GetHessian()
        {
            linearizeOplus();
            return _jacobianOplusXi.transpose() * information() * _jacobianOplusXi;
        }

        Eigen::Vector2d gps_pos;
    };

    // 先验位姿约束
    class EdgePriorStatePose : public g2o::BaseUnaryEdge<3, Eigen::Vector3d, VertexPose>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        EdgePriorStatePose(const Eigen::Matrix2d &Rwb_, const Eigen::Vector2d &twb_);

        virtual bool read(std::istream &is) { return false; }
        virtual bool write(std::ostream &os) const { return false; }

        void computeError();
        virtual void linearizeOplus();

        Eigen::Matrix3d GetHessian()
        {
            linearizeOplus();
            return _jacobianOplusXi.transpose() * information() * _jacobianOplusXi;
        }

        Eigen::Matrix3d Rwb;
        Eigen::Vector3d twb;
    };

    // GPS速度观测
    class EdgeGpsVelocity : public g2o::BaseBinaryEdge<2, Eigen::Vector2d, VertexPose, VertexPose>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        EdgeGpsVelocity(const Eigen::Vector2d &gps_vel_, double dt_) : gps_vel(gps_vel_), dt(dt_) {}

        virtual bool read(std::istream &is) { return false; }
        virtual bool write(std::ostream &os) const { return false; }

        void computeError();
        virtual void linearizeOplus();

        Eigen::Matrix<double, 6, 6> GetHessian()
        {
            linearizeOplus();
            Eigen::Matrix<double, 2, 6> J;
            J.block<2, 3>(0, 0) = _jacobianOplusXi;
            J.block<2, 3>(0, 3) = _jacobianOplusXj;
            return J.transpose() * information() * J;
        }

        Eigen::Vector2d gps_vel;
        double dt;
    };

} // namespace ORB_SLAM2

#endif // G2OTYPES_H
