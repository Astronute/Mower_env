#include "Common.h"
#include <string>
#include <fstream>
#include <vector>

#include "Settings.h"

namespace ORB_SLAM3
{
    std::map<std::string, std::shared_ptr<std::fstream>> Logger::logFiles;
    std::atomic_flag Logger::flag = ATOMIC_FLAG_INIT; // 原子标志，初始为未设置状态

    Logger::Logger(const std::string &fileName)
    {
        const std::string &logFileRoot = Settings::logRoot();
        if (logFileRoot.back() != '/')
        {
            logFileName = logFileRoot + "/" + fileName;
        }
        else
        {
            logFileName = logFileRoot + fileName;
        }
        lock();
        if (logFiles.find(logFileName) == logFiles.end())
        {
            logFiles[logFileName] = std::make_shared<std::fstream>(logFileName, std::ios::out);
        }
        unlock();
    }

    Eigen::Vector3d Vector2dToVector3d(const Eigen::Vector2d &vector2d)
    {
        Eigen::Vector3d vector3d;
        vector3d << vector2d(0), vector2d(1), 0;

        return vector3d;
    }

    Eigen::Matrix3d mat2dToMat3d(const Eigen::Matrix2d &mat2d)
    {
        Eigen::Matrix3d mat3d;
        mat3d.setIdentity();
        mat3d.block<2, 2>(0, 0) = mat2d;

        return mat3d;
    }

    Eigen::Matrix3d Attitude2R(const Eigen::Vector3d &atti)
    {
        double cosp = cos(atti(0));
        double sinp = sin(atti(0));
        double cosr = cos(atti(1));
        double sinr = sin(atti(1));
        double cosy = cos(atti(2));
        double siny = sin(atti(2));

        Eigen::Matrix3d Rb2L;

        Rb2L(0, 0) = cosr * cosy - sinr * siny * sinp;
        Rb2L(0, 1) = -siny * cosp;
        Rb2L(0, 2) = sinr * cosy + cosr * siny * sinp;
        Rb2L(1, 0) = cosr * siny + sinr * cosy * sinp;
        Rb2L(1, 1) = cosy * cosp;
        Rb2L(1, 2) = sinr * siny - cosr * cosy * sinp;
        Rb2L(2, 0) = -sinr * cosp;
        Rb2L(2, 1) = sinp;
        Rb2L(2, 2) = cosr * cosp;

        return Rb2L;
    }

    // pitch roll yall
    Eigen::Vector3d getAttitudeFromR(const Eigen::Matrix3d &Rb2L)
    {
        return {asin(Rb2L(2, 1)), atan2(-Rb2L(2, 0), Rb2L(2, 2)), atan2(-Rb2L(0, 1), Rb2L(1, 1))};
    }

    Eigen::Matrix2d Yaw2R(double yaw)
    {
        Eigen::Matrix2d R;
        R << cos(yaw), -sin(yaw), sin(yaw), cos(yaw);
        return R;
    }

    double getYawFromR(const Eigen::Matrix2d &Rb2L)
    {
        return atan2(Rb2L(1, 0), Rb2L(0, 0));
    }

    // 由经纬度变化量计算东向/北向位置变化量
    void dLondLat2dEdN(double lon0, double lat0, double dlon, double dlat, double &dE, double &dN)
    {
        double WGS84_a = 6378137.0;
        double WGS84_f = 1.0 / 298.257223563;
        double WGS84_e2 = (2.0 - WGS84_f) * WGS84_f;
        double pi = 3.1415926535897;

        double W = sqrt(1 - WGS84_e2 * sin(lat0 * pi / 180) * sin(lat0 * pi / 180));
        double M = WGS84_a * (1 - WGS84_e2) / (W * W * W);
        double N = WGS84_a / W;
        dE = N * dlon * pi / 180 * cos(lat0 * pi / 180);
        dN = M * dlat * pi / 180;
    }

    // 由东向/北向位置变化量计算经纬度变化量
    void dEdN2dLondLat(double lon0, double lat0, double dE, double dN, double &dlon, double &dlat)
    {
        double WGS84_a = 6378137.0;
        double WGS84_f = 1.0 / 298.257223563;
        double WGS84_e2 = (2.0 - WGS84_f) * WGS84_f;
        double pi = 3.1415926535897;

        double W = sqrt(1 - WGS84_e2 * sin(lat0 * pi / 180) * sin(lat0 * pi / 180));
        double M = WGS84_a * (1 - WGS84_e2) / (W * W * W);
        double N = WGS84_a / W;

        dlon = dE / N * 180 / pi / cos(lat0 * pi / 180);
        dlat = dN / M * 180 / pi;
    }

    Eigen::MatrixXd Marginalize(const Eigen::MatrixXd &H, const int &start, const int &end)
    {
        // Goal
        // a  | ab | ac       a*  | 0 | ac*
        // ba | b  | bc  -->  0   | 0 | 0
        // ca | cb | c        ca* | 0 | c*

        // Size of block before block to marginalize
        const int a = start;
        // Size of block to marginalize
        const int b = end - start + 1;
        // Size of block after block to marginalize
        const int c = H.cols() - (end + 1);

        // Reorder as follows:
        // a  | ab | ac       a  | ac | ab
        // ba | b  | bc  -->  ca | c  | cb
        // ca | cb | c        ba | bc | b

        Eigen::MatrixXd Hn = Eigen::MatrixXd::Zero(H.rows(), H.cols());
        if (a > 0)
        {
            Hn.block(0, 0, a, a) = H.block(0, 0, a, a);
            Hn.block(0, a + c, a, b) = H.block(0, a, a, b);
            Hn.block(a + c, 0, b, a) = H.block(a, 0, b, a);
        }
        if (a > 0 && c > 0)
        {
            Hn.block(0, a, a, c) = H.block(0, a + b, a, c);
            Hn.block(a, 0, c, a) = H.block(a + b, 0, c, a);
        }
        if (c > 0)
        {
            Hn.block(a, a, c, c) = H.block(a + b, a + b, c, c);
            Hn.block(a, a + c, c, b) = H.block(a + b, a, c, b);
            Hn.block(a + c, a, b, c) = H.block(a, a + b, b, c);
        }
        Hn.block(a + c, a + c, b, b) = H.block(a, a, b, b);

        // Perform marginalization (Schur complement)
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(Hn.block(a + c, a + c, b, b), Eigen::ComputeThinU | Eigen::ComputeThinV);
        Eigen::JacobiSVD<Eigen::MatrixXd>::SingularValuesType singularValues_inv = svd.singularValues();
        for (int i = 0; i < b; ++i)
        {
            if (singularValues_inv(i) > 1e-6)
                singularValues_inv(i) = 1.0 / singularValues_inv(i);
            else
                singularValues_inv(i) = 0;
        }
        Eigen::MatrixXd invHb = svd.matrixV() * singularValues_inv.asDiagonal() * svd.matrixU().transpose();
        Hn.block(0, 0, a + c, a + c) = Hn.block(0, 0, a + c, a + c) - Hn.block(0, a + c, a + c, b) * invHb * Hn.block(a + c, 0, b, a + c);
        Hn.block(a + c, a + c, b, b) = Eigen::MatrixXd::Zero(b, b);
        Hn.block(0, a + c, a + c, b) = Eigen::MatrixXd::Zero(a + c, b);
        Hn.block(a + c, 0, b, a + c) = Eigen::MatrixXd::Zero(b, a + c);

        // Inverse reorder
        // a*  | ac* | 0       a*  | 0 | ac*
        // ca* | c*  | 0  -->  0   | 0 | 0
        // 0   | 0   | 0       ca* | 0 | c*
        Eigen::MatrixXd res = Eigen::MatrixXd::Zero(H.rows(), H.cols());
        if (a > 0)
        {
            res.block(0, 0, a, a) = Hn.block(0, 0, a, a);
            res.block(0, a, a, b) = Hn.block(0, a + c, a, b);
            res.block(a, 0, b, a) = Hn.block(a + c, 0, b, a);
        }
        if (a > 0 && c > 0)
        {
            res.block(0, a + b, a, c) = Hn.block(0, a, a, c);
            res.block(a + b, 0, c, a) = Hn.block(a, 0, c, a);
        }
        if (c > 0)
        {
            res.block(a + b, a + b, c, c) = Hn.block(a, a, c, c);
            res.block(a + b, a, c, b) = Hn.block(a, a + c, c, b);
            res.block(a, a + b, b, c) = Hn.block(a + c, a, b, c);
        }

        res.block(a, a, b, b) = Hn.block(a + c, a + c, b, b);

        return res;
    }


} // namespace ORB_SLAM3