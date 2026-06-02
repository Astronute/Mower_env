#ifndef Estimater_H
#define Estimater_H

#include "ImuTypes.h"
#include "Settings.h"
#include <mutex>
#include <vector>
#include <queue>
#include <deque>
#include <thread>

#include "Eigen/Core"

#include "sophus/se3.hpp"
#include "sophus/so3.hpp"

#include "Viewer.h"
#include "Logger.h"

namespace ORB_SLAM3
{
    typedef Eigen::Matrix<double, 5, 1> Vector5d;
    typedef Eigen::Matrix<double, 6, 1> Vector6d;

    typedef Eigen::Matrix<double, 5, 5> Matrix5d;
    typedef Eigen::Matrix<double, 6, 6> Matrix6d;
    typedef Eigen::Matrix<double, 9, 9> Matrix9d;
    typedef Eigen::Matrix<double, 10, 10> Matrix10d;
    typedef Eigen::Matrix<double, 15, 15> Matrix15d;
    typedef Eigen::Matrix<double, 16, 16> Matrix16d;
    typedef Eigen::Matrix<double, 17, 17> Matrix17d;

#define GPS_DATA_MAX_DELAY (5.0) // Gps数据的最大时间延迟

    namespace EKF_WHEELPLUS
    {
        const double gravity = -9.81;

        enum class MeasureDataType
        {
            VISUAL = 1,
            WHEEL_ODOMETER = 2,

            KEYFRAME_VISUAL = 3,

            RTK = 4,

            NONE = 999,
        };

        struct MeasureData
        {
            double t;
            MeasureDataType type;
            void *data;

            static bool Compare(const MeasureData &data1, const MeasureData &data2)
            {
                return data1.t < data2.t;
            }
        };

        struct PreintegInfo
        {
            // double t;
            double yaw0;

            Eigen::Matrix3d cov;

            double dT;
            std::vector<double> ve_dt;
            std::vector<double> ve_d;
            std::vector<double> ve_yaw;
            std::vector<EnMotionStatus> ve_motionStatus;
        };

        struct State
        {
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW

            double t;
            double x3_t;
            Eigen::Vector3d pos;
            Eigen::Vector3d vel;
            Eigen::Matrix3d R;
            Eigen::Vector3d ba;
            Eigen::Vector3d bg;

            Eigen::Matrix2d gpsR;
            Eigen::Vector2d gpsPos;

            // 输出在东北天坐标系下的位姿，坐标原点为初始，后续会修正为RTK基站
            Eigen::Vector3d Position;
            Eigen::Vector3d Attitude;

            // dr_position and dr_attitude
            Eigen::Vector3d Dr_Position;
            Eigen::Vector3d Dr_Attitude;

            // 相邻两个状态之间的预积分（位姿变化）
            bool bPreintegred;
            PreintegInfo preintegInfo;

            // 是否已经做过关联Gps数据的处理
            bool bGpsProcessed;
            // 关联的Gps观测数据
            GpsOdometryPtr gpsOdom;

            bool bAligned = false;

            State()
            {
                bPreintegred = false;
                bGpsProcessed = false;
                gpsOdom = nullptr;
            }
        };

        using StatePtr = std::shared_ptr<State>;

        struct SimpleState
        {
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW

            double t;
            Eigen::Vector3d pos;
            Eigen::Matrix3d R;

            Eigen::Matrix2d gpsR;
            Eigen::Vector2d gpsPos;
        };
        struct rtkState
        {
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW

            double t;
            double x3_t;
            Eigen::Vector3d pos;
            int valid;
            int fs;
        };
        struct MarginInfo
        {
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW

            double t;
            // yaw, x, y
            Eigen::Vector3d state;
            Eigen::Matrix3d info; // 信息矩阵
        };

        static Eigen::Matrix3d getSkewMatrix(Eigen::Vector3d r);

    } // namespace EKF

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

    namespace CONTROL_DATA
    {
        struct RawControl
        {
            double timestamp;
            double vx;
            double wz;
        };
    };

    
    // 保存简要的GPS信息，用于判断是否可以从跳变状态恢复到正常
    struct SimpleGpsData
    {
        double t;
        int fs;    //定位质量（定位类型）

        double x;
        double y; //平面坐标
        double speed; //速度（绝对值）

        double trajLen; //距离起始位置的轨迹长度（根据gps相邻位置计算）
    };

    // rtk跳变信息
    struct RtkSlipInfo
    {
        double t0; // 跳变开始时间
        double t1; // 跳变结束时间

        double trajLen; // [t0,t1]时段内的运行轨迹长度（根据轮速粗略推算）
    };

    // 轮速打滑信息
    struct WheelSlipInfo
    {
        double t0; // 打滑开始时间
        double t1; // 打滑结束时间
        double cur_t; // 当前时间

        bool blastslip; // 上一个时刻是否打滑
        bool bslip; // 是否处于打滑状态（如果前面被判断为打滑，则外部传进来非打滑状态要持续3秒以上才会设置为非打滑）

        bool bprocessed; // 是否处理过了（做了回退处理）

        WheelSlipInfo() : blastslip(false), bslip(false), bprocessed(false) {}
    };

    class Estimater
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        Estimater(double wheelplusSF, Eigen::Matrix3d &Riv, Eigen::Vector3d &piv, Eigen::Vector3d &pgv);

        ~Estimater();

        bool Inited() { return mbInited; }

        void AddRawImu(const SENSOR_DATA::RawImu &imu);

        void AddRawWheel(const SENSOR_DATA::RawWheel &wheelOdometer);

        void AddRawGps(const SENSOR_DATA::RawGps &gpsOdom);

        void AddImu(const IMU::Point &imu);

        void AddWheelOdometer(const WheelOdometer &wheelOdometer);

        void AddGpsOdometer(const GpsOdometry &gpsOdomRaw);

        bool IsStopOntime(double t, double tolerance);

        bool GetImustopinfoOntime(double t, double tolerance, StopImuInfo &stopImuinfo);

        bool GetEKFState(double t, EKF_WHEELPLUS::State &state);

        bool GetLatestEKFState(EKF_WHEELPLUS::State &state);

        bool GetAlignData(ViewData &aligndata);

        void getLon0Lat0(double &lon0, double &lat0);

        void stop();

        bool GetLatestRtkState(EKF_WHEELPLUS::rtkState &state);

        bool GetRef(double &ref_e, double &ref_n);
        
        bool GetRTKErrorState(double t);

        double GetCurrRtkSlipDuration();

        void ChangeSplitState(int state);

        void UpdateWheelSlipInfo(int slip_state, double t, double duration);

    private:
        void run();

        bool GpsDrAlign();

        void LocationUpdate();

        void GetInterpImu(double t, const IMU::Point &imu1, const IMU::Point &imu2, IMU::Point &imuInterp);

        void GetInterpState(double t, const EKF_WHEELPLUS::State &state1, const EKF_WHEELPLUS::State &state2, EKF_WHEELPLUS::State &stateInterp);

        void GetInterpSimpleState(double t, const EKF_WHEELPLUS::SimpleState &simplestate1, const EKF_WHEELPLUS::SimpleState &simplestate2,
                                  EKF_WHEELPLUS::SimpleState &simplestateInterp);

        void RemoveOldQueuedatas();

        // void ClearReverttoTime() {mReverttoTime = DBL_MAX;}

        void DeadReckoning(bool bWheelslipped);

        void ProcessStateGpsSmooth();

        // void Preintegrate();

        // bool CalcRelativeCov(double t1, double t2, Eigen::Matrix3d &relativeCov);
 
        
        void GetRtkdataInterp(double t, GpsOdometry &rtkdataInterp);

        void CalcMotionstatus(double cur_t, MotionStatus &motionstatus);

        void UpdateImustopinfo(double cur_t);

        bool isStopInRecent1s(double t);

        bool CalcPitchrollGyrobias_1s(double cur_t, Eigen::Vector2d &pitchroll, Eigen::Vector3d &gyrobias);

        int getImuIndx(double t, double tolerance);

        int getWheelodometerIndx(double t, double tolerance);

        int getMotionstatusIndx(double t, double tolerance);

        void FindCalcToImuIndx(double tolerance);

        void FindMotionStatusCalcFromImuIndx(int &fromImuIndx);

        bool CalcImuMeanAndStdInfos(double t, double smooth_timelen,
                                    Eigen::Vector3d &accel_mean, Eigen::Vector3d &gyro_mean, Eigen::Vector3d &accel_std);

        bool CalcMeanStatePitchroll(double t, double smooth_timelen, Eigen::Vector2d &state_pitchroll_mean);

        void CalibPitchRollWithAccel(const WheelOdometer &wheelOdometer, EKF_WHEELPLUS::State &state, const Eigen::Vector2d &state_atti_mean,
                                     const Eigen::Vector3d &accel_mean, const Eigen::Vector3d &gyro_mean, const Eigen::Vector3d &accel_std);

        void ResetPreintegInfo(const EKF_WHEELPLUS::State &drState);

        bool SmoothGps(double t1, double t2, double target_t, GpsOdometry &gpsOdomSmooth);

        static void Preintegrate(double dt, double d, double yaw_, EnMotionStatus motionStatus, EKF_WHEELPLUS::PreintegInfo &preintegInfo);

        static bool CalcTransformParams_LS(const std::vector<EKF_WHEELPLUS::State, Eigen::aligned_allocator<EKF_WHEELPLUS::State>> &vDrdata,
                                           const std::vector<GpsOdometry> &vRtkdata,
                                           const Eigen::Vector3d &r_ig, Eigen::Matrix3d &R33, Eigen::Vector3d &t31, double &scale, double &mean_error);
                                        
        //处理RTK跳变的相关函数
        void CheckRtkSlip();
        void CheckRtkRecorved();
        bool CheckRtkRecorvedSelf(double traj_len_thre, double dT_min_thre);
        
        // 处理打滑相关的函数
        bool OnSlideDetected(double slide_t, double time_len);
        void RecalcDeadReckoning(int imuStartIndx);
        void RecalcProcessStateGpsSmooth();
        bool CalcWheelVelWithRtk(double t, Eigen::Vector3d &vel_3d);

        void Optimize(std::vector<EKF_WHEELPLUS::State, Eigen::aligned_allocator<EKF_WHEELPLUS::State>> &veState,
                      EKF_WHEELPLUS::MarginInfo *pMarginInfo, bool bSlidewindow, double &chi2, double &robust_chi2);

    private:
        std::deque<IMU::Point> mImuQueue;
        std::deque<WheelOdometer> mWheelOdometerQueue;
        std::deque<GpsOdometry> mGpsOdomQueue;
        std::deque<EKF_WHEELPLUS::State, Eigen::aligned_allocator<EKF_WHEELPLUS::State>> mStateQueue;
        std::deque<EKF_WHEELPLUS::SimpleState, Eigen::aligned_allocator<EKF_WHEELPLUS::SimpleState>> mSimpleStateQueue;
        std::mutex mMutexSensordata;
        std::deque<EKF_WHEELPLUS::rtkState> mRtkStateQueue;

        bool mbInited;
        int mCalcToImuIndx;

        double mWheelplusSF;
        Eigen::Matrix3d mRiv;
        Eigen::Vector3d mPiv;
        Eigen::Vector3d mPgv;

        std::deque<MotionStatus> mMotionstatusQueue;
        bool mIsStopLasttime;
        std::vector<StopImuInfo> mvStopImuInfo;
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
        EKF_WHEELPLUS::PreintegInfo mPreintegInfo;
        // 用于计算Dr的gpsR,gpsPos
        Eigen::Matrix3d mT_Gps_Dr;
        //
        bool mbAddNewDr;
        //
        EKF_WHEELPLUS::MarginInfo mMarginInfo;
        //
        double mLaseGpsProcessedTime;
        //
        std::deque<SENSOR_DATA::RawDataWithType> mRawDataWithTypeQueue;
        std::thread mProcessThread;
        bool mbRunning;

        Viewer *mpViewer;

        std::deque<ViewData> AlignData;
        std::mutex align_mutex;

        // 将 输出位姿转换到基站坐标
        double mRefE_ob;
        double mRefN_ob;
        bool have_Ref = false;

        Eigen::Matrix3d mR_EN;

        bool _rtk_error = false;
        double _rtk_error_time = 0.0;
        std::mutex rtkerror_mutex;    


        EKF_WHEELPLUS::State mLatestState;
        bool mbSetLatestState;

        std::atomic<int> split_val; // 0 无打滑 ， 2直行打滑, 3旋转打滑, 4后轮抬起

        double mLastOptimizeTime = 0;

        std::deque<SimpleGpsData> mSimpleGpsQueue; //保存最近的1000个数据
        std::deque<RtkSlipInfo> mRtkSlipInfos; // 保存最近的rtk跳变信息，包含跳变开始时间、结束时间和跳变时段内的轨迹长度

        std::atomic<bool> mbLastRtkSlipped = false; // 记录上一次rtk是否处于跳变状态，用于判断是否可以从跳变状态恢复到正常
        double mLastState0Time = 0;

        double mLatestGpsTime = 0;

        // 最新的rtk跳变持续时长
        std::atomic<double> mLastRtkSlipDuration = 0;

        WheelSlipInfo mWheelSlipInfo;
        
    };

} // namespace ORB_SLAM

#endif // EKF_FUSION_H
