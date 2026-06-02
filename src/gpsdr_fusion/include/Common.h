#ifndef _COMMON_H_
#define _COMMON_H_
#include <iostream>
#include <cstdlib>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <fstream>
#include <map>
#include <memory>
#include <atomic>
#include <mutex>
#include <thread>
#include "ImuTypes.h"

namespace ORB_SLAM3
{

#define CONDITION_CHECK(condition, message)    \
    do                                         \
    {                                          \
        if (!(condition))                      \
        {                                      \
            std::cerr << "Error: "             \
                      << message << std::endl; \
            std::exit(EXIT_FAILURE);           \
        }                                      \
    } while (0)

    class Logger
    {
    public:
        Logger(const std::string &fileName);
        void lock()
        {
            while (flag.test_and_set(std::memory_order_acquire))
            {
                std::this_thread::yield();
            }
        }
        void unlock()
        {
            flag.clear(std::memory_order_release);
        }
        template <typename T>
        std::fstream &operator<<(const T &message)
        {
            lock();
            std::shared_ptr<std::fstream> file = logFiles[logFileName];
            unlock();
            if (!file->is_open())
            {
                // std::cerr << "Error: Log file " << logFileName << " is not open." << std::endl;
                return *file;
            }

            *file << message;
            return *file;
        }

    private:
        static std::map<std::string, std::shared_ptr<std::fstream>> logFiles;
        static std::atomic_flag flag;
        std::string logFileName;
    };


    Eigen::Vector3d Vector2dToVector3d(const Eigen::Vector2d &vector2d);
    Eigen::Matrix3d mat2dToMat3d(const Eigen::Matrix2d &mat2d);

    Eigen::Matrix3d Attitude2R(const Eigen::Vector3d &atti);
    Eigen::Vector3d getAttitudeFromR(const Eigen::Matrix3d &Rb2L);

 

    Eigen::Matrix2d Yaw2R(double yaw);
    double getYawFromR(const Eigen::Matrix2d &Rb2L);

    void dLondLat2dEdN(double lon0, double lat0, double dlon, double dlat, double &dE, double &dN);
    void dEdN2dLondLat(double lon0, double lat0, double dE, double dN, double &dlon, double &dlat);

    Eigen::MatrixXd Marginalize(const Eigen::MatrixXd &H, const int &start, const int &end);

}
#endif
