#pragma once

#include <Eigen/Core>
#include "Eigen/Dense"

#include <memory>
#include <mutex>
#include <queue>

namespace SENSOR_DATA
{

    enum class RTKQuality
    {
        UNABLE = 0,
        SPS = 1,
        DGPS = 2,
        PPS = 3,
        RTKFIXED = 4,
        RTKFLOT = 5,

        NONE = 999,
    };

    enum class Type
    {
        IMU = 0,
        WHEELODOMETER = 1,
        GPS = 2,

        NONE = 999,
    };

    struct RawData
    {
        virtual ~RawData() {}
    };

    using RawDataPtr = std::shared_ptr<RawData>;

    struct RawImu : RawData
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        double t;
        double x3_t;
        Eigen::Vector3f a;
        Eigen::Vector3f w;
        double yaw;

        ~RawImu() {}
    };

    struct RawWheel : RawData
    {
        double t;
        double x3_t;
        double whlplus_l;
        double whlplus_r;

        ~RawWheel() {}
    };

    struct RawGps : RawData
    {
        double t;
        double x3_t;
        double lon;
        double lat;
        double h;
        double ref_east;
        double ref_north;
        double east_vel;
        double north_vel;
        double speed;
        double direction;
        int numSv;
        double hdop;

        int valid;
        int fs;

        ~RawGps() {}
    };

    struct RawDataWithType
    {
        Type type;
        RawDataPtr pRawData;

        RawDataWithType()
        {
            type = SENSOR_DATA::Type::NONE;
            pRawData = nullptr;
        }
    };

}

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
        Point(const Eigen::Vector3f &Acc, const Eigen::Vector3f &Gyro, double yaw, const double &timestamp)
            : a(Acc), w(Gyro), yaw(yaw), t(timestamp) {}

        Point(const float &acc_x, const float &acc_y, const float &acc_z,
                const float &ang_vel_x, const float &ang_vel_y, const float &ang_vel_z, 
                const double &timestamp) : a(acc_x, acc_y, acc_z), w(ang_vel_x, ang_vel_y, ang_vel_z), yaw(0.0), t(timestamp) {}
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

class Estimater{
public:
    Estimater();

    ~Estimater();

    void AddRawImu(const SENSOR_DATA::RawImu &imu);

    void AddRawWheel(const SENSOR_DATA::RawWheel &wheelOdometer);

    void AddRawGps(const SENSOR_DATA::RawGps &gpsOdom);

    void AddImu(const IMU::Point &imu);

    void FindCalcToImuIndx(double tolerance);

    void FindMotionStatusCalcFromImuIndx(int &fromImuIndx);

    void CalcMotionstatus(double cur_t, MotionStatus &motionstatus);

    void run();

private:
    std::deque<IMU::Point> mImuQueue;
    // std::deque<WheelOdometer> mWheelOdometerQueue;
    // std::deque<GpsOdometry> mGpsOdomQueue;
    // std::deque<EKF_WHEELPLUS::State, Eigen::aligned_allocator<EKF_WHEELPLUS::State>> mStateQueue;
    // std::deque<EKF_WHEELPLUS::SimpleState, Eigen::aligned_allocator<EKF_WHEELPLUS::SimpleState>> mSimpleStateQueue;
    std::mutex mMutexSensordata;
    // std::deque<EKF_WHEELPLUS::rtkState> mRtkStateQueue;

    bool mbInited;
    int mCalcToImuIndx;

    double mWheelplusSF;
    Eigen::Matrix3d mRiv;
    Eigen::Vector3d mPiv;
    Eigen::Vector3d mPgv;

    // std::deque<MotionStatus> mMotionstatusQueue;
    bool mIsStopLasttime;
    // std::vector<StopImuInfo> mvStopImuInfo;
    std::mutex mMutexWheelOdometer;

    // std::mutex mMutexWriteSensordataFile;
    double mLon0;
    double mLat0;

    bool mbDrInited;
    // 记录上一次预积分算到的时间（与GPS数据对应）
    double mLastPreintegTime;
    bool mbGpsAigned;
    // Gps到Dr的位姿变换矩阵（二维平面），用于将Gps坐标转换到Dr参考坐标系
    Eigen::Matrix3d mT_Dr_Gps_fixed;
    Eigen::Matrix3d mT_Dr_Gps_fixed_inv;
    //
    // EKF_WHEELPLUS::PreintegInfo mPreintegInfo;
    // 用于计算Dr的gpsR,gpsPos
    Eigen::Matrix3d mT_Gps_Dr;
    //
    bool mbAddNewDr;
    //
    // EKF_WHEELPLUS::MarginInfo mMarginInfo;
    //
    double mLaseGpsProcessedTime;
    //
    std::deque<SENSOR_DATA::RawDataWithType> mRawDataWithTypeQueue;
    // std::thread mProcessThread;
    bool mbRunning;

    // Viewer *mpViewer;

    // std::deque<ViewData> AlignData;
    std::mutex align_mutex;

    // 将 输出位姿转换到基站坐标
    double mRefE_ob;
    double mRefN_ob;
    bool have_Ref = false;

    Eigen::Matrix3d mR_EN;

    bool _rtk_error = false;
    double _rtk_error_time = 0.0;
    std::mutex rtkerror_mutex;    


    // EKF_WHEELPLUS::State mLatestState;
    bool mbSetLatestState;

    std::atomic<int> split_val; // 0 无打滑 ， 2直行打滑, 3旋转打滑, 4后轮抬起

    double mLastOptimizeTime = 0;

    // std::deque<SimpleGpsData> mSimpleGpsQueue; //保存最近的1000个数据
    // std::deque<RtkSlipInfo> mRtkSlipInfos; // 保存最近的rtk跳变信息，包含跳变开始时间、结束时间和跳变时段内的轨迹长度

    // std::atomic<bool> mbLastRtkSlipped = false; // 记录上一次rtk是否处于跳变状态，用于判断是否可以从跳变状态恢复到正常
    double mLastState0Time = 0;

    double mLatestGpsTime = 0;

    // 最新的rtk跳变持续时长
    // std::atomic<double> mLastRtkSlipDuration = 0;

    // WheelSlipInfo mWheelSlipInfo;

};