#include "Estimater.h"
#include "Common.h"
#include "G2oTypes.h"
// #include "g2o/core/sparse_block_matrix.h"
// #include "g2o/core/block_solver.h"
// #include "g2o/core/optimization_algorithm_levenberg.h"
// #include "g2o/core/optimization_algorithm_gauss_newton.h"
// #include "g2o/solvers/linear_solver_eigen.h"
// #include "g2o/types/types_six_dof_expmap.h"
// #include "g2o/core/robust_kernel_impl.h"
// #include "g2o/solvers/linear_solver_dense.h"
#include "g2o/core/sparse_block_matrix.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/solvers/linear_solver_eigen.h"
#include "g2o/types/types_six_dof_expmap.h"
#include "g2o/core/robust_kernel_impl.h"
#include "g2o/solvers/linear_solver_dense.h"

#include <algorithm>
#include <iostream>
#include <iomanip>
#include <mutex>

namespace ORB_SLAM3
{
    Eigen::Matrix3d EKF_WHEELPLUS::getSkewMatrix(Eigen::Vector3d r)
    {
        Eigen::Matrix3d mat;
        mat << 0, -r(2), r(1),
            r(2), 0, -r(0),
            -r(1), r(0), 0;
        return mat;
    }

    Estimater::Estimater(double wheelplusSF, Eigen::Matrix3d &Riv, Eigen::Vector3d &piv, Eigen::Vector3d &pgv)
    {
        mWheelplusSF = wheelplusSF;
        mRiv = Riv;
        mPiv = piv;
        mPgv = pgv;

        mT_Dr_Gps_fixed.setIdentity();

        mIsStopLasttime = false;
        mbInited = false;
        mLon0 = 0;
        mLat0 = 0;
        mbDrInited = false;
        mLastPreintegTime = 0;
        mbGpsAigned = false;
        mT_Gps_Dr = Eigen::Matrix3d::Identity();
        mLaseGpsProcessedTime = 0;
        mR_EN = Settings::RotateEN();
        mbSetLatestState = false;
        split_val.store(0);
        clink::Logger("debug_log.txt") << "mR_EN: " << mR_EN(0, 0) << " " << mR_EN(0, 1) << " " << mR_EN(0, 2)
                                       << mR_EN(1, 0) << " " << mR_EN(1, 1) << " " << mR_EN(1, 2)
                                       << mR_EN(2, 0) << " " << mR_EN(2, 1) << " " << mR_EN(2, 2) << std::endl;

        mProcessThread = std::move(std::thread(&Estimater::run, this));
    }

    Estimater::~Estimater()
    {
    }

    void Estimater::run()
    {
        mbRunning = true;

        while (mbRunning)
        {
            SENSOR_DATA::RawDataWithType rawDataWithType;
            bool have_data = false;

            {
                std::lock_guard<std::mutex> lock(mMutexSensordata);
                if (!mRawDataWithTypeQueue.empty())
                {
                    rawDataWithType = mRawDataWithTypeQueue.front();
                    mRawDataWithTypeQueue.pop_front();
                    have_data = true;
                }
            }

            if (have_data)
            {
                if (rawDataWithType.type == SENSOR_DATA::Type::IMU)
                {
                    auto pImu = std::dynamic_pointer_cast<SENSOR_DATA::RawImu>(rawDataWithType.pRawData);

                    IMU::Point imu(pImu->a, pImu->w, pImu->yaw, pImu->t);
                    imu.x3_t = pImu->x3_t;
                    AddImu(imu);
                }
                else if (rawDataWithType.type == SENSOR_DATA::Type::WHEELODOMETER)
                {
                    auto pWheel = std::dynamic_pointer_cast<SENSOR_DATA::RawWheel>(rawDataWithType.pRawData);

                    WheelOdometer wheelOdometer;
                    wheelOdometer.t = pWheel->t;
                    wheelOdometer.whlplus_l = pWheel->whlplus_l;
                    wheelOdometer.whlplus_r = pWheel->whlplus_r;
                    AddWheelOdometer(wheelOdometer);
                }
                else if (rawDataWithType.type == SENSOR_DATA::Type::GPS)
                {
                    auto pGps = std::dynamic_pointer_cast<SENSOR_DATA::RawGps>(rawDataWithType.pRawData);

                    GpsOdometry gpsOdom;
                    gpsOdom.t = pGps->t;
                    gpsOdom.x3_t = pGps->x3_t;
                    gpsOdom.lon = pGps->lon;
                    gpsOdom.lat = pGps->lat;
                    gpsOdom.h = pGps->h;
                    gpsOdom.ref_east = pGps->ref_east;
                    gpsOdom.ref_north = pGps->ref_north;
                    gpsOdom.east_vel = pGps->east_vel;
                    gpsOdom.north_vel = pGps->north_vel;
                    gpsOdom.speed = pGps->speed;
                    gpsOdom.direction = pGps->direction;
                    gpsOdom.numSv = pGps->numSv;
                    gpsOdom.hdop = pGps->hdop;

                    gpsOdom.valid = pGps->valid;
                    gpsOdom.fs = pGps->fs;
                    AddGpsOdometer(gpsOdom);
                }
            }
            else
            {
                std::this_thread::sleep_for(std::chrono::microseconds(500));
            }
        }
    }

    void Estimater::stop()
    {
        mbRunning = false;
    }

    void Estimater::AddRawImu(const SENSOR_DATA::RawImu &imu)
    {
        std::lock_guard<std::mutex> lock(mMutexSensordata);
        SENSOR_DATA::RawDataWithType rawDataWithType;
        rawDataWithType.type = SENSOR_DATA::Type::IMU;
        rawDataWithType.pRawData = std::make_shared<SENSOR_DATA::RawImu>(imu);
        mRawDataWithTypeQueue.push_back(rawDataWithType);
    }

    void Estimater::AddRawWheel(const SENSOR_DATA::RawWheel &wheelOdometer)
    {
        std::lock_guard<std::mutex> lock(mMutexSensordata);
        SENSOR_DATA::RawDataWithType rawDataWithType;
        rawDataWithType.type = SENSOR_DATA::Type::WHEELODOMETER;
        rawDataWithType.pRawData = std::make_shared<SENSOR_DATA::RawWheel>(wheelOdometer);
        mRawDataWithTypeQueue.push_back(rawDataWithType);
    }

    void Estimater::AddRawGps(const SENSOR_DATA::RawGps &gpsOdom)
    {
        std::lock_guard<std::mutex> lock(mMutexSensordata);
        SENSOR_DATA::RawDataWithType rawDataWithType;
        rawDataWithType.type = SENSOR_DATA::Type::GPS;
        rawDataWithType.pRawData = std::make_shared<SENSOR_DATA::RawGps>(gpsOdom);
        mRawDataWithTypeQueue.push_back(rawDataWithType);
    }

    void Estimater::AddImu(const IMU::Point &imu)
    {
        static double last_time = 0;
        if (imu.t <= last_time)
        {
            clink::Logger("debug_log.txt") << "error, imu data disorder [last_time, cur_time] : " << last_time << ", " << imu.t << std::endl;
            return;
        }
        last_time = imu.t;

        {
            // std::lock_guard<std::mutex> lock(mMutexSensordata);
            mImuQueue.push_back(imu);
            // 先确定可以计算到哪个IMU数据（运动状态/DR推算）
            FindCalcToImuIndx(0.05);
            if (mCalcToImuIndx < 0)
            {
                std::cout << "error mCalcToImuIndx < 0" << std::endl;
                return;
            }

            // 计算运动状态
            int motionStatusCalcFromImuIndx;
            FindMotionStatusCalcFromImuIndx(motionStatusCalcFromImuIndx);
            if (motionStatusCalcFromImuIndx >= 0)
            {
                for (int i = motionStatusCalcFromImuIndx; i <= mCalcToImuIndx; i++)
                {
                    double t = mImuQueue.at(i).t;

                    MotionStatus motionStatus;
                    CalcMotionstatus(t, motionStatus);
                    clink::Logger("motionStatus.txt") << "motionStatus : " << motionStatus.t << ", " << static_cast<int>(motionStatus.status) << std::endl;
                    mMotionstatusQueue.push_back(motionStatus);
                    UpdateImustopinfo(t);
                }
            }
        }
        LocationUpdate();
    }

    void Estimater::AddWheelOdometer(const WheelOdometer &wheelOdometer)
    {
        static double last_time = 0;
        if (wheelOdometer.t <= last_time)
        {
            std::cout << "error, wheel Odometer data disorder [last_time, cur_time] : " << last_time << ", " << wheelOdometer.t << std::endl;
            return;
        }
        last_time = wheelOdometer.t;

        WheelOdometer wheelOdometer_update;
        {
            // std::lock_guard<std::mutex> lock(mMutexSensordata);

            wheelOdometer_update.t = wheelOdometer.t;
            wheelOdometer_update.whlplus_l = wheelOdometer.whlplus_l;
            wheelOdometer_update.whlplus_r = wheelOdometer.whlplus_r;

            // 计算上一个数据到当前的轮速脉冲距离
            if (mWheelOdometerQueue.size() == 0)
            {
                wheelOdometer_update.pre_t = wheelOdometer_update.t - 0.1;
                wheelOdometer_update.d_l = 0;
                wheelOdometer_update.d_r = 0;
                wheelOdometer_update.valid = false;
            }
            else
            {
                WheelOdometer &wheelOdometer_pre = mWheelOdometerQueue.back();
                wheelOdometer_update.pre_t = wheelOdometer_pre.t;
                wheelOdometer_update.d_l = wheelOdometer_update.whlplus_l - wheelOdometer_pre.whlplus_l;
                wheelOdometer_update.d_r = wheelOdometer_update.whlplus_r - wheelOdometer_pre.whlplus_r;
                wheelOdometer_update.valid = true;
            }

            // 与前面一定时间间隔的数据计算速度
            int pre_indx = -1;
            int indx;
            int size = mWheelOdometerQueue.size();
            for (int i = 0; i < size; i++)
            {
                indx = size - 1 - i;
                if (wheelOdometer_update.t - mWheelOdometerQueue.at(indx).t >= 0.5)
                {
                    pre_indx = indx;
                    break;
                }
            }
            //
            if (pre_indx == -1)
            {
                wheelOdometer_update.vel = 0;
                wheelOdometer_update.vel_valid = false;
            }
            else
            {
                double d_l = wheelOdometer_update.whlplus_l - mWheelOdometerQueue.at(pre_indx).whlplus_l;
                double d_r = wheelOdometer_update.whlplus_r - mWheelOdometerQueue.at(pre_indx).whlplus_r;

                // if(std::abs(d_l) > 1 || std::abs(d_r) > 1) // 防止轨迹数据异常
                // {
                //     return;
                // }

                double d = (d_l + d_r) * 0.5;
                double dt = wheelOdometer_update.t - mWheelOdometerQueue.at(pre_indx).t;
                wheelOdometer_update.vel = d / dt;

                if(fabs(wheelOdometer_update.vel) > 2.0)
                {
                    std::cout << "wheelOdometer data error, [l_pre, r_pre, l, r] : " 
                        << mWheelOdometerQueue.at(pre_indx).whlplus_l << ", " << mWheelOdometerQueue.at(pre_indx).whlplus_r << ", "
                        << wheelOdometer_update.whlplus_l << ", " << wheelOdometer_update.whlplus_r << std::endl;
                }

                // 为了处理轮速脉冲异常的情况：有时候脉冲数跳变
                if(fabs(wheelOdometer_update.vel) > 0.6)
                {
                    wheelOdometer_update.vel = 0.6 * wheelOdometer_update.vel / fabs(wheelOdometer_update.vel);
                }
                wheelOdometer_update.vel_valid = true;
            }


            // // 测试：模拟静止时打滑，数据：yat_rtk2_algorithm_fusion(error2)) ，时段：[6257778, 6291383], [6372614, 6442795]
            // // 模拟抬起，数据：26_04_20 : [5045646, 5075686]
            // static double last_update_time = 0;
            // static bool bfirst_slip = true;
            // // if((wheelOdometer_update.t >= 6257.778 && wheelOdometer_update.t <= 6291.383)
            // //     || (wheelOdometer_update.t >= 6372.614 && wheelOdometer_update.t <= 6442.795) )
            // if((wheelOdometer_update.t >= 5045.646 && wheelOdometer_update.t <= 5075.686))
            // {
            //     wheelOdometer_update.vel = 1.0;
            //     double dt = wheelOdometer_update.t - wheelOdometer_update.pre_t;
            //     wheelOdometer_update.d_l = wheelOdometer_update.vel * dt;
            //     wheelOdometer_update.d_r = wheelOdometer_update.vel * dt;

            //     // if((wheelOdometer_update.t >= 6257.778 + 3.0 && wheelOdometer_update.t <= 6291.383)
            //     //     || (wheelOdometer_update.t >= 6372.614 + 3.0 && wheelOdometer_update.t <= 6442.795))
            //     if((wheelOdometer_update.t >= 5045.646 + 3.0 && wheelOdometer_update.t <= 5075.686))
            //     {
            //         int slip_state = 2;
            //         double t = wheelOdometer_update.t; 
            //         double duration;
            //         if(bfirst_slip)
            //         {
            //             duration = 3.0;
            //         }
            //         else
            //         {
            //             duration = 1.0;
            //         }
            //         bfirst_slip = false;

            //         if(t - last_update_time > 0.95)
            //         {
            //             UpdateWheelSlipInfo(slip_state, t, duration);
            //             last_update_time = t;

            //             std::cout << "wheel slipped : " << t << ", " << duration << std::endl;
            //         }

            //     }

            // }
            // else
            // {
            //     bfirst_slip = true;

            //     int slip_state = 0;
            //     double t = wheelOdometer_update.t; 
            //     double duration = 1.0;
            //     if(t - last_update_time > 0.95)
            //     {
            //         UpdateWheelSlipInfo(slip_state, t, duration);
            //         last_update_time = t;
            //     }
                
            // }


            mWheelOdometerQueue.push_back(wheelOdometer_update);
        }

        // 注意：这里没有计算角速度 wheelOdometer_update.anguler_vel，文件写入0
        // Logger("wheelodometer.txt") << std::fixed << std::setprecision(3) << wheelOdometer_update.t << " "
        //                             << wheelOdometer_update.whlplus_l << " " << wheelOdometer_update.whlplus_r << " "
        //                             << wheelOdometer_update.d_l << " " << wheelOdometer_update.d_r << " "
        //                             << wheelOdometer_update.vel << " " << Eigen::Vector3d::Zero().transpose()
        //                             << std::endl;
    }

    void Estimater::AddGpsOdometer(const GpsOdometry &gpsOdomRaw)
    {
        // 第一个rtk必须为固定解
        if (mGpsOdomQueue.empty() && gpsOdomRaw.fs != 4) 
        {
            return;
        }

        if (gpsOdomRaw.fs != 4 && gpsOdomRaw.fs != 5) // 只用4和5的RTK数据做融合
        {
            return;
        }

        if(std::abs(gpsOdomRaw.lon) < 1e-9 || std::abs(gpsOdomRaw.lat) < 1e-9) 
        {
            return;
        }

        static double last_time = 0;
        if (gpsOdomRaw.t <= last_time)
        {
            std::cout << "error, gps odom data disorder [last_time, cur_time] : " << last_time << ", " << gpsOdomRaw.t;
            return;
        }
        last_time = gpsOdomRaw.t;

        // 判断经纬度是否有效
        if (fabs(gpsOdomRaw.lon) < 1e-3 || fabs(gpsOdomRaw.lon) > 180 ||
            fabs(gpsOdomRaw.lat) < 1e-3 || fabs(gpsOdomRaw.lat) > 90 || (gpsOdomRaw.valid == 0))
        {
            return;
        }

        if (mLon0 == 0 && mLat0 == 0)
        {
            mLon0 = gpsOdomRaw.lon;
            mLat0 = gpsOdomRaw.lat;
            clink::Logger("debug_log.txt") << "first data : " << mLon0 << " " << mLat0 << std::endl;
        }

        if (!have_Ref && gpsOdomRaw.fs == 4) // 4是RTK差分模式
        {
            mRefE_ob = gpsOdomRaw.ref_east;
            mRefN_ob = gpsOdomRaw.ref_north;
            have_Ref = true;
            clink::Logger("debug_log.txt") << "init ref: " << mRefE_ob << " " << mRefN_ob << std::endl;
        }

        GpsOdometry gpsOdom = gpsOdomRaw;

        // 经纬度转成平面坐标
        double dlon = gpsOdom.lon - mLon0;
        double dlat = gpsOdom.lat - mLat0;
        double dE, dN;
        dLondLat2dEdN(mLon0, mLat0, dlon, dlat, dE, dN);
        gpsOdom.pos << dE, dN, 0;

        // 测试：取rtk的高度
        gpsOdom.pos << dE, dN, gpsOdom.h;

        // 千米/小时 转换成 米/秒
        gpsOdom.vel << gpsOdomRaw.east_vel /3.6, gpsOdomRaw.north_vel /3.6, 0;

        Logger(Settings::gpsFile_) << std::fixed << std::setprecision(3) << gpsOdom.t << " " << gpsOdom.pos.transpose() << std::endl;
        {
            std::lock_guard<std::mutex> lock(mMutexSensordata);
            EKF_WHEELPLUS::rtkState rtkState;
            rtkState.t = gpsOdom.t;
            rtkState.x3_t = gpsOdom.x3_t;
            rtkState.pos = gpsOdom.pos;
            rtkState.valid = gpsOdom.valid;
            rtkState.fs = rtkState.fs;
            mRtkStateQueue.push_back(rtkState);
        }
        // Gps坐标转换到Dr参考系
        if (mbGpsAigned)
        {
            gpsOdom.pos.topRows(2) = mT_Dr_Gps_fixed.block<2, 2>(0, 0) * gpsOdom.pos.topRows(2) + mT_Dr_Gps_fixed.block<2, 1>(0, 2);
            gpsOdom.vel.topRows(2) = mT_Dr_Gps_fixed.block<2, 2>(0, 0) * gpsOdom.vel.topRows(2);
        }

        {
            std::lock_guard<std::mutex> lock(mMutexSensordata);
            mGpsOdomQueue.push_back(gpsOdom);
        }

        CheckRtkSlip();

        ViewData viewData;
        viewData.pos = gpsOdom.pos;
        viewData.atti = Eigen::Vector3d::Zero();
        viewData.bRtkslip = mbLastRtkSlipped;

#ifndef SUNRISE_X3
        if (mpViewer)
        {
            mpViewer->AddRtkViewdata(viewData);
        }
#endif
    }

    void Estimater::LocationUpdate()
    {
        WheelSlipInfo wheelSlipInfo;
        {
            std::lock_guard<std::mutex> lock(mMutexSensordata);

            // 清除打滑或抬起状态
            if(mStateQueue.size() > 0 && mWheelSlipInfo.bslip)
            {
                double dt = mStateQueue.back().t - mWheelSlipInfo.t1;
                if(dt > 3.0)
                {
                    mWheelSlipInfo.bslip = false;
                }
            }

            wheelSlipInfo = mWheelSlipInfo;
            mWheelSlipInfo.bprocessed = true;
        }

        mbAddNewDr = false;
        bool bWheelslipped = Settings::bWheelSlipProcess_ && wheelSlipInfo.bslip && wheelSlipInfo.blastslip;
        DeadReckoning(bWheelslipped);
        ProcessStateGpsSmooth();

        // 处理打滑
        if(Settings::bWheelSlipProcess_ && wheelSlipInfo.bslip && !wheelSlipInfo.blastslip && !wheelSlipInfo.bprocessed)
        {
            OnSlideDetected(wheelSlipInfo.cur_t, wheelSlipInfo.t1 - wheelSlipInfo.t0);
        }

        if (!mbGpsAigned)
        {
            GpsDrAlign();
        }
        else
        {
            double dt;
            if(mLastOptimizeTime == 0)
            {
                dt = 0;
            }
            else
            {
                dt = mStateQueue.at(mStateQueue.size() - 1).t - mLastOptimizeTime;
            }
             
            // 静止则每秒执行一次优化 
            if (mbAddNewDr || dt > 1.0)
            {
                int state_size = mStateQueue.size();

                bool bSlidewindow = false;
                // 计算轨迹长度
                double traj_len = 0;
                for (int i = 0; i < state_size - 1; i++)
                {
                    Eigen::Vector3d rtk_pos_i1 = mStateQueue.at(i + 1).pos - mStateQueue.at(i + 1).R * mPgv;
                    Eigen::Vector3d rtk_pos_i = mStateQueue.at(i).pos - mStateQueue.at(i).R * mPgv;
                    Eigen::Vector3d dpos = rtk_pos_i1 - rtk_pos_i; // pose 位移
                    traj_len += dpos.topRows(2).norm(); // 轨迹长度

                    // if (traj_len > Settings::ekfInitAlignDist() * 2)
                    if (traj_len > 6.0)
                    {
                        bSlidewindow = true;
                        break;
                    }
                }

                std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

                std::vector<EKF_WHEELPLUS::State, Eigen::aligned_allocator<EKF_WHEELPLUS::State>> veState;
                veState.insert(veState.end(), mStateQueue.begin(), mStateQueue.end());
                double chi2;
                double robust_chi2;


                // 测试：写log，记录优化前后的位置和方向角
                Eigen::Vector2d pos_before = veState[veState.size()-1].gpsPos;
                double yaw_before = getYawFromR(veState[veState.size()-1].gpsR);

                Optimize(veState, &mMarginInfo, bSlidewindow, chi2, robust_chi2);

                Eigen::Vector2d pos_after = veState[veState.size()-1].gpsPos;
                double yaw_after = getYawFromR(veState[veState.size()-1].gpsR);
                Eigen::Vector2d dpos = pos_after - pos_before;
                double dyaw = yaw_after - yaw_before;
                dyaw = sqrt(2*(1-cos(dyaw)));

                // Logger("optimize_change.txt") << veState[veState.size()-1].t << " " << pos_before.transpose() << " "
                //     << yaw_before*57.3 << " " << pos_after.transpose() << " " << yaw_after*57.3 << " " 
                //     << dpos.transpose() << " " << dyaw*57.3 << std::endl;
                

                mLastOptimizeTime = mStateQueue.at(mStateQueue.size() - 1).t;

                std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
                double time_ms = std::chrono::duration<double, std::milli>(t2 - t1).count();
                // std::cout << "Optimize time: " << time_ms << " ms" << std::endl;
                // 更新状态
                for (int i = 0; i < veState.size(); i++)
                {
                    EKF_WHEELPLUS::State &state = mStateQueue[i];
                    state.gpsPos = veState[i].gpsPos;
                    state.gpsR = veState[i].gpsR;
                }

                // 更新 mT_Gps_Dr
                Eigen::Matrix3d T_Dr_b = Eigen::Matrix3d::Identity();
                Eigen::Vector3d atti = getAttitudeFromR(veState[veState.size() - 1].R);
                T_Dr_b.block<2, 2>(0, 0) = Yaw2R(atti(2));
                T_Dr_b.block<2, 1>(0, 2) = veState[veState.size() - 1].pos.topRows(2);
                Eigen::Matrix3d T_Gps_b = Eigen::Matrix3d::Identity();
                T_Gps_b.block<2, 2>(0, 0) = veState[veState.size() - 1].gpsR;
                T_Gps_b.block<2, 1>(0, 2) = veState[veState.size() - 1].gpsPos;
                // gps到dr的变换矩阵
                mT_Gps_Dr = T_Gps_b * T_Dr_b.inverse();

                // std::cout << "bSlidewindow : " << bSlidewindow << std::endl;
                // std::cout << "mT_Gps_Dr : " << mT_Gps_Dr << std::endl;
                Eigen::Vector2d pos = mT_Gps_Dr.block<2, 1>(0, 2);
                double yaw = getYawFromR(mT_Gps_Dr.block<2, 2>(0, 0));
                // clink::Logger("T_gps_dr.txt") << mStateQueue.back().t << " " << pos.transpose() << " " << yaw * 57.3 << std::endl;
                // 滑窗（边缘化）
                if (bSlidewindow)
                {
                    mLastState0Time = mStateQueue.front().t;
                    mStateQueue.pop_front();
                }
            }
        }
        // clink::Logger("add_imu.log") << " RemoveOldQueuedatas before "  << std::endl;
        RemoveOldQueuedatas();
        // clink::Logger("add_imu.log") << " RemoveOldQueuedatas back "  << std::endl;

        CheckRtkRecorved();

        {
            std::lock_guard<std::mutex> lock(mMutexSensordata);

            if (mbInited && mStateQueue.size() > 0)
            {
                mLatestState = mStateQueue.back();
                mbSetLatestState = true;
            }

            if(mGpsOdomQueue.size() > 0)
            {
                mLatestGpsTime = mGpsOdomQueue.back().t;
            }

            if(mbLastRtkSlipped && mRtkSlipInfos.size() > 0)
            {
                mLastRtkSlipDuration = mRtkSlipInfos.back().t1 - mRtkSlipInfos.back().t0;
            }
            else
            {
                mLastRtkSlipDuration = 0;
            }

        }

        return;
    }

    void Estimater::DeadReckoning(bool bWheelslipped)
    {
        if (mImuQueue.size() < 100 || mWheelOdometerQueue.size() < 10) // 数据太少
        {
            return;
        }

        // 1、初始化
        if (!mbDrInited)
        {
            double t = mImuQueue.at(mCalcToImuIndx).t;
            double x3_t = mImuQueue.at(mCalcToImuIndx).x3_t;
            double yaw = mImuQueue.at(mCalcToImuIndx).yaw;
            StopImuInfo stopImuInfo;
            if (!GetImustopinfoOntime(t, 1e-3, stopImuInfo)) // 判断imu数据是否静止
            {
                return;
            }

            EKF_WHEELPLUS::State drState;

            drState.t = t;
            drState.x3_t = x3_t;
            drState.pos = Eigen::Vector3d::Zero();
            drState.vel = Eigen::Vector3d::Zero();
            Eigen::Vector3d init_atti;
            init_atti << stopImuInfo.pitchroll_mean(0), stopImuInfo.pitchroll_mean(1), yaw; // 获取初始 roll pitch 角度
            drState.R = Attitude2R(init_atti);
            drState.ba = Eigen::Vector3d::Zero();
            drState.bg = stopImuInfo.gyro_bias_mean;
            mStateQueue.push_back(drState);

            clink::Logger("debug.txt") << drState.t  << " " << drState.pos.x() << " " << drState.pos.y() << " " << drState.pos.z() << std::endl;

            ResetPreintegInfo(drState); // 设置预积分的初始值

            EKF_WHEELPLUS::SimpleState simpleState;
            simpleState.t = drState.t;
            simpleState.pos = drState.pos;
            simpleState.R = drState.R;
            simpleState.gpsPos = drState.gpsPos;
            simpleState.gpsR = drState.gpsR;
            mSimpleStateQueue.push_back(simpleState);

            mbDrInited = true;
            mbInited = true;
            return;
        }

        // 2、DR正常计算
        double t = mStateQueue.back().t;
        int matchImuIndx = getImuIndx(t, 1e-6); // 根据时间获取imu的 id
        if (matchImuIndx < 0)
        {
            // should not happen
            std::cout << "Can't get Imu indx in DeadReckoning [time, min_t, max_t] : "
                      << t << " " << mImuQueue.front().t << " " << mImuQueue.back().t << std::endl;

            return;
        }

        if (matchImuIndx == 0)
        {
            std::cout << "matchImuIndx == 0" << std::endl;
            return;
        }

        int imuStartIndx = matchImuIndx + 1;
        for (int i = imuStartIndx; i <= mCalcToImuIndx; i++)
        {
            IMU::Point &imuPre = mImuQueue.at(i - 1); // 获取之前时刻的imu数据
            IMU::Point &imuCur = mImuQueue.at(i);     // 获取 当前时刻的imu数据
            double cur_t = imuCur.t;

            EKF_WHEELPLUS::State &drStatePre = mStateQueue.back();
            EKF_WHEELPLUS::State drState;
            drState.t = cur_t;
            drState.x3_t = imuCur.x3_t;

            // 提前获取到cur_t时刻的轮速
            WheelOdometer wheelOdometer;
            int wheelIndx = getWheelodometerIndx(cur_t, 0.15);
            if (wheelIndx < 0)
            {
                std::cout << "Can't get wheeelodometer indx in DeadReckoning [time, min_t, max_t] : "
                          << cur_t << " " << mWheelOdometerQueue.front().t << " " << mWheelOdometerQueue.back().t << std::endl;
            }
            if (wheelIndx <= 0)
            {
                wheelIndx = 1;
            }
            wheelOdometer = mWheelOdometerQueue.at(wheelIndx);
            wheelOdometer.use_rtk_vel = false;

            if(bWheelslipped)
            {
                Eigen::Vector3d rtk_vel;
                if(CalcWheelVelWithRtk(wheelOdometer.t, rtk_vel))
                {
                    wheelOdometer.vel_3d_from_rtk = rtk_vel;
                    wheelOdometer.use_rtk_vel = true;
                }
            }

            // DR计算：静止/运动两种情况
            double dt = imuCur.t - imuPre.t;
            if (isStopInRecent1s(cur_t))
            {
                // 静止
                drState.pos = drStatePre.pos;
                drState.vel = Eigen::Vector3d::Zero();
                drState.R = drStatePre.R;
                drState.ba = drStatePre.ba;
                drState.bg = drStatePre.bg;

                // 计算预积分
                Eigen::Vector3d atti = getAttitudeFromR(drState.R);
                Preintegrate(dt, 0, atti(2), EnMotionStatus::STOP, mPreintegInfo);

                // 更新陀螺零偏
                StopImuInfo stopImuInfo;
                if (GetImustopinfoOntime(cur_t, 1e-3, stopImuInfo))
                {
                    drState.bg = stopImuInfo.gyro_bias_mean;
                }
            }
            else
            {
                // 运动
                Eigen::Vector3f gyroRaw = (imuPre.w + imuCur.w) * 0.5;
                Eigen::Vector3d gyro = gyroRaw.cast<double>() - drStatePre.bg;
                // 姿态更新
                Eigen::Vector3d r = gyro * dt;
                Eigen::AngleAxisd rv(r.norm(), r.normalized());
                Eigen::Matrix3d incrR(rv);
                drState.R = drStatePre.R * incrR;

                double d;
                if(bWheelslipped && wheelOdometer.use_rtk_vel)
                {
                    // 速度
                    drState.vel = drState.R * wheelOdometer.vel_3d_from_rtk;

                    // 位置更新
                    Eigen::Vector3d vehicle_d(wheelOdometer.vel_3d_from_rtk * dt);
                    drState.pos = drStatePre.pos + (drState.R + drStatePre.R) * 0.5 * vehicle_d;

                    d = (wheelOdometer.vel_3d_from_rtk * dt).norm();
                }
                else
                {
                    // 速度
                    Eigen::Vector3d vel(0, wheelOdometer.vel * mWheelplusSF, 0);
                    drState.vel = drState.R * vel;

                    // 位置更新
                    double dl = wheelOdometer.d_l;
                    double dr = wheelOdometer.d_r;
                    double dT = wheelOdometer.t - wheelOdometer.pre_t;
                    d = (dl + dr) * 0.5 * dt / dT;
                    Eigen::Vector3d vehicle_d(0, d, 0);
                    drState.pos = drStatePre.pos + (drState.R + drStatePre.R) * 0.5 * vehicle_d * mWheelplusSF;
                }

                drState.ba = drStatePre.ba;
                drState.bg = drStatePre.bg;

                // 计算预积分
                Eigen::Vector3d atti = getAttitudeFromR(drState.R);
                Preintegrate(dt, d, atti(2), EnMotionStatus::MOVE, mPreintegInfo);
            }

            // 修正pitch/roll
            static double last_pr_calib_time = 0;
            if (cur_t - last_pr_calib_time > 0.1 - 0.005)
            {
                Eigen::Vector2d state_atti_mean;
                Eigen::Vector3d accel_mean;
                Eigen::Vector3d gyro_mean;
                Eigen::Vector3d accel_std;
                double smooth_timelen = 0.5;

                if (CalcImuMeanAndStdInfos(cur_t, smooth_timelen, accel_mean, gyro_mean, accel_std) && CalcMeanStatePitchroll(cur_t, smooth_timelen, state_atti_mean))
                {
                    CalibPitchRollWithAccel(wheelOdometer, drState, state_atti_mean, accel_mean, gyro_mean, accel_std);
                }

                last_pr_calib_time = cur_t;
            }

            // DR结果添加到队列：避免添加距离很近的轨迹点
            int drStateSize = mStateQueue.size();
            if (drStateSize < 2)
            {
                drState.bPreintegred = true;
                drState.preintegInfo = mPreintegInfo;
                mStateQueue.push_back(drState);
            }
            else
            {
                Eigen::Vector3d dpos = mStateQueue.at(drStateSize - 1).pos - mStateQueue.at(drStateSize - 2).pos;
                double d = dpos.topRows(2).norm();
                Eigen::Vector3d atti1 = getAttitudeFromR(mStateQueue.at(drStateSize - 1).R);
                Eigen::Vector3d atti0 = getAttitudeFromR(mStateQueue.at(drStateSize - 2).R);
                double dyaw = atti1(2) - atti0(2);
                while (dyaw > M_PI)
                {
                    dyaw -= 2 * M_PI;
                }
                while (dyaw < -M_PI)
                {
                    dyaw += 2 * M_PI;
                }
                double dt = mStateQueue.at(drStateSize-1).t - mStateQueue.at(drStateSize-2).t;
                // 相邻点之间的距离小于阈值，且角度变化小于阈值
                if (d < 0.2 && fabs(dyaw) < 15 * M_PI / 180 && dt < 3.0)
                {
                    EKF_WHEELPLUS::State &state = mStateQueue.back();
                    state.t = drState.t;
                    state.x3_t = drState.x3_t;
                    state.pos = drState.pos;
                    state.vel = drState.vel;
                    state.R = drState.R;
                    state.ba = drState.ba;
                    state.bg = drState.bg;
                    state.bPreintegred = true;
                    for (int k = 0; k < mPreintegInfo.ve_dt.size(); k++)
                    {
                        Preintegrate(mPreintegInfo.ve_dt[k], mPreintegInfo.ve_d[k], mPreintegInfo.ve_yaw[k],
                                     mPreintegInfo.ve_motionStatus[k], state.preintegInfo);
                    }
                }
                else
                {
                    drState.bPreintegred = true;
                    drState.preintegInfo = mPreintegInfo;
                    mStateQueue.push_back(drState);

                    mbAddNewDr = true;
                }
            }

            if(mStateQueue.size() > 10)
            {
                bool combined = false;
                for(int i=0; i<3; i++)
                {
                    int indx = mStateQueue.size() - 7 - i;
                    Eigen::Vector3d dpos_1 = mStateQueue.at(indx+1).pos - mStateQueue.at(indx).pos;
                    Eigen::Vector3d datti_1 = getAttitudeFromR(mStateQueue.at(indx+1).R) - getAttitudeFromR(mStateQueue.at(indx).R);
                    double dyaw1 = datti_1(2);
                    while(dyaw1 > M_PI)
                    {
                        dyaw1 -= 2 * M_PI;
                    }
                    while(dyaw1 < -M_PI)
                    {
                        dyaw1 += 2 * M_PI;
                    }

                    Eigen::Vector3d dpos_2 = mStateQueue.at(indx).pos - mStateQueue.at(indx-1).pos;
                    Eigen::Vector3d datti_2 = getAttitudeFromR(mStateQueue.at(indx).R) - getAttitudeFromR(mStateQueue.at(indx-1).R);
                    double dyaw2 = datti_2(2);
                    while(dyaw2 > M_PI)
                    {
                        dyaw2 -= 2 * M_PI;
                    }
                    while(dyaw2 < -M_PI)
                    {
                        dyaw2 += 2 * M_PI;
                    }

                    // 删掉indx对应的state
                    if(dpos_1.norm() < 1.0e-3 && dyaw1 < 1.0e-4 && dpos_2.norm() < 1.0e-3 && dyaw2 < 1.0e-4)
                    {
                        std::deque<EKF_WHEELPLUS::State, Eigen::aligned_allocator<EKF_WHEELPLUS::State>> tmpqueue = 
                            mStateQueue;

                        mStateQueue.clear();
                        for(int i=0; i<tmpqueue.size(); i++)
                        {
                            if(i != indx)
                            {
                                mStateQueue.push_back(tmpqueue.at(i));
                            }
                        }
                        combined = true;
                    }

                    if(combined)
                    {
                        break;
                    }
                }

            }


            EKF_WHEELPLUS::State temp_state;
            std::lock_guard<std::mutex> lock(mMutexSensordata);
            {
                EKF_WHEELPLUS::State &lastest_state = mStateQueue.back();
                lastest_state.Dr_Position = lastest_state.pos;
                Eigen::Vector3d dr_atti = getAttitudeFromR(lastest_state.R);
                lastest_state.Dr_Attitude = dr_atti;
                // 计算drState在Gps坐标系下的位姿
                if (mbGpsAigned)
                {
                    lastest_state.bAligned = mbGpsAigned;
                    lastest_state.gpsPos = mT_Gps_Dr.block<2, 2>(0, 0) * lastest_state.pos.topRows(2) + mT_Gps_Dr.block<2, 1>(0, 2);
                    Eigen::Vector3d atti = getAttitudeFromR(lastest_state.R);
                    lastest_state.gpsR = mT_Gps_Dr.block<2, 2>(0, 0) * Yaw2R(atti(2));

                    lastest_state.Position.topRows(2) = mT_Dr_Gps_fixed_inv.block<2, 2>(0, 0) * lastest_state.gpsPos + mT_Dr_Gps_fixed_inv.block<2, 1>(0, 2);
                    lastest_state.Position.z() = 0.0;
                    Eigen::Matrix2d R = mT_Dr_Gps_fixed_inv.block<2, 2>(0, 0) * lastest_state.gpsR;

                    lastest_state.Attitude.x() = atti.x();       // pitch
                    lastest_state.Attitude.y() = atti.y();       // roll
                    lastest_state.Attitude.z() = getYawFromR(R); // yaw

                    Eigen::Matrix3d R_temp = mR_EN * Attitude2R(lastest_state.Attitude);
                    Eigen::Vector3d Atti_end = getAttitudeFromR(R_temp);
                    lastest_state.Attitude.x() = Atti_end.x();
                    lastest_state.Attitude.y() = Atti_end.y();
                    lastest_state.Attitude.z() = Atti_end.z();

                    // clink::Logger("yaw_debug.txt") << lastest_state.t << " " << mT_Dr_Gps_fixed_inv(0, 0) << " " << mT_Dr_Gps_fixed_inv(0, 1) << " " << mT_Dr_Gps_fixed_inv(1, 0) << " " << mT_Dr_Gps_fixed_inv(1, 1) << " " << atti.z() << " " << lastest_state.Attitude.z() << std::endl;
                }
                temp_state = lastest_state;
            }
            ResetPreintegInfo(temp_state);

            EKF_WHEELPLUS::SimpleState simpleState;
            simpleState.t = temp_state.t;
            simpleState.pos = temp_state.pos;
            simpleState.R = temp_state.R;
            simpleState.gpsPos = temp_state.gpsPos;
            simpleState.gpsR = temp_state.gpsR;
            mSimpleStateQueue.push_back(simpleState);


            // // 测试：写log
            // Logger("locresult.txt") << simpleState.t << " " << simpleState.pos.transpose() << " "
            //     << getAttitudeFromR(simpleState.R).transpose() *57.3 << " " 
            //     << simpleState.gpsPos.transpose() << " "
            //     << getYawFromR(simpleState.gpsR) *57.3 << std::endl;

            //
            static double last_pub_time = 0;
            // if (lastest_state.t - last_pub_time > 0.005)
            // {
            ViewData viewData;
            viewData.t = temp_state.t;
            if (!mbGpsAigned)
            {
                viewData.pos = temp_state.pos;
                viewData.atti = getAttitudeFromR(temp_state.R);
                // 从后轴中心转换到rtk中心再输出，方便与rtk比较
                viewData.pos -= temp_state.R * mPgv;
            }
            else
            {
                viewData.pos << temp_state.gpsPos, 0; // gpsPos 是修正誤差後的位姿吧
                Eigen::Vector3d atti = getAttitudeFromR(temp_state.R);
                atti(2) = getYawFromR(temp_state.gpsR);
                viewData.atti = atti;
                // 从后轴中心转换到rtk中心再输出，方便与rtk比较
                viewData.pos -= mat2dToMat3d(temp_state.gpsR) * mPgv;
            }
            {
                std::lock_guard<std::mutex> lock(align_mutex);
                AlignData.push_back(viewData);
            }

            if (temp_state.t - last_pub_time > 0.005)
            {
#ifndef SUNRISE_X3
                if (mpViewer)
                {
                    mpViewer->AddEkfViewdata(viewData);
                }
#endif

                last_pub_time = temp_state.t;

                // 写文件
                Eigen::Vector3d pos = temp_state.pos;
                Eigen::Vector3d vel = temp_state.vel;
                Eigen::Vector3d atti = getAttitudeFromR(temp_state.R);
                Eigen::Vector3d ba = temp_state.ba;
                Eigen::Vector3d bg = temp_state.bg;
                
                                            
            }
        }
    }

    /*
     * 处理State关联的Gps数据：
     * 1、如果待处理的State在mStateQueue中的索引为indx, 则取[indx-1, indx+1]范围内(对应[t1,t2])的Gps数据做平滑
     * 2、先确定能处理到哪个State（calc_to_state_indx）
     *   1.1 因为mStateQueue里面最后一个数据是动态更新的，不可用。
     *   1.2 倒数第二个数据已经稳定了，但考虑到平滑参考点应该位于中间，因此最多算到倒数第三个数据（indx = size-3）
     *   1.3 再根据最新的Gps时刻调整，要求indx+1对应的时间戳 <= 最新的Gps时间戳
     */
    void Estimater::ProcessStateGpsSmooth()
    {

        if(mStateQueue.size() < 4 || mGpsOdomQueue.size() == 0)
        {
            if(mGpsOdomQueue.size() == 0 && mStateQueue.size() >= 4)
            {
                int state_size = mStateQueue.size();
                double ref_t = mStateQueue.at(state_size - 2).t;
                for(int i=state_size-1; i>=0; i--)
                {
                    EKF_WHEELPLUS::State &state = mStateQueue.at(i);
                    if(state.bGpsProcessed)
                    {
                        break;
                    }

                    if(ref_t - state.t >= GPS_DATA_MAX_DELAY)
                    {
                        state.bGpsProcessed = true;
                        mLaseGpsProcessedTime = state.t;
                    }
                }
            }

            return;
        }
        
        int state_size = mStateQueue.size();

        int calc_to_state_indx = state_size - 3;

        // 再根据最新的Gps时刻调整，要求indx+1对应的时间戳 <= 最新的Gps时间戳
        double latest_gpst = mGpsOdomQueue.back().t;
        int gps_ready_state_indx = -1;
        for (int i = state_size - 1; i >= 0; i--)
        {
            if (mStateQueue.at(i).t <= latest_gpst)
            {
                gps_ready_state_indx = i;
                break;
            }
        }
        if (gps_ready_state_indx < 2)
        {
            return;
        }
        if (calc_to_state_indx > gps_ready_state_indx - 1)
        {
            calc_to_state_indx = gps_ready_state_indx - 1;
        }
        // 将所有距离倒数第二个状态 GPS_DATA_MAX_DELAY 之前的全部状态置为已经处理
        double ref_t = mStateQueue.at(state_size - 2).t;
        for (int i = state_size - 1; i >= 0; i--)
        {
            EKF_WHEELPLUS::State &state = mStateQueue.at(i);
            if (state.bGpsProcessed)
            {
                break;
            }

            if (ref_t - state.t >= GPS_DATA_MAX_DELAY)
            {
                state.bGpsProcessed = true;
                mLaseGpsProcessedTime = state.t;
            }
        }
        // 再确定calc_from_state_indx
        int calc_from_state_indx = -1;
        for (int i = state_size - 1; i >= 0; i--)
        {
            if (mStateQueue.at(i).bGpsProcessed)
            {
                calc_from_state_indx = i + 1;
                break;
            }
        }
        if (calc_from_state_indx < 1)
        {
            calc_from_state_indx = 1;
        }
        // 下面开始处理
        for (int i = calc_from_state_indx; i <= calc_to_state_indx; i++)
        {

            EKF_WHEELPLUS::State &state = mStateQueue.at(i);
            double t1 = mStateQueue.at(i - 1).t;
            double t2 = mStateQueue.at(i + 1).t;
            GpsOdometry gpsOdomSmooth;
            if (SmoothGps(t1, t2, state.t, gpsOdomSmooth))
            {
                state.gpsOdom = std::make_shared<GpsOdometry>(gpsOdomSmooth);
            }
            state.bGpsProcessed = true;
            mLaseGpsProcessedTime = state.t;
        }
    }

    /*
     * 对[t1,t2]时间段内的Gps数据计算平滑值
     * 结合Dr数据做平滑，把两个时刻之间的Dr位置增量（从后轴中心转换到Gps中心）补偿进去
     */
    bool Estimater::SmoothGps(double t1, double t2, double target_t, GpsOdometry &gpsOdomSmooth)
    {
        int gps_size = mGpsOdomQueue.size();
        if (gps_size == 0)
        {
            return false;
        }
        // 先找到[t1,t2]时间段内的Gps数据索引[gps_indx_min, gps_indx_max]
        int gps_indx_min = -1;
        int gps_indx_max = -1;
        
        for (int i = gps_size - 1; i >= 0; i--)
        {
            GpsOdometry &gpsOdom = mGpsOdomQueue[i];
            if (gpsOdom.t > t2)
            {
                continue;
            }
            if (gpsOdom.t < t1)
            {
                break;
            }

            if (gps_indx_max == -1)
            {
                gps_indx_max = i;
            }
            gps_indx_min = i;
        }
        if (gps_indx_min < 0 || gps_indx_max < 0)
        {
            return false;
        }
        Eigen::Vector3d pos_sum = Eigen::Vector3d::Zero();
        Eigen::Vector3d vel_sum = Eigen::Vector3d::Zero();
        int cnt = 0;
        int loctype = 4; // rtk fix
        // Gps/DR没对齐之前，Gps数据直接取平均
        if (!mbGpsAigned)
        {
            for (int i = gps_indx_min; i <= gps_indx_max; i++)
            {
                GpsOdometry &gpsOdom = mGpsOdomQueue[i];
                pos_sum += gpsOdom.pos;
                vel_sum += gpsOdom.vel;

                if(gpsOdom.fs != 4)
                {
                    loctype = gpsOdom.fs;
                }
                cnt++;
            }
        }
        else
        {
            int size = mSimpleStateQueue.size();
            EKF_WHEELPLUS::SimpleState target_state;
            bool find = false;
            for (int i = size - 2; i >= 0; i--)
            {
                if (mSimpleStateQueue.at(i).t <= target_t && mSimpleStateQueue.at(i + 1).t >= target_t)
                {
                    GetInterpSimpleState(target_t, mSimpleStateQueue.at(i), mSimpleStateQueue.at(i + 1), target_state);
                    find = true;
                    break;
                }
            }
            if (!find)
            {
                // should not happen
                std::cout << "error, can't find target_state in SmoothGps [target_t, min_t, max_t] : "
                          << target_t << ", " << mSimpleStateQueue.front().t << ", " << mSimpleStateQueue.back().t << std::endl;
                return false;
            }
            // 从后轴中心转换到gps中心
            Eigen::Vector3d target_gps_pos = Vector2dToVector3d(target_state.gpsPos) - mat2dToMat3d(target_state.gpsR) * mPgv;
            for (int i = gps_indx_min; i <= gps_indx_max; i++)
            {
                GpsOdometry &gpsOdom = mGpsOdomQueue[i];
                EKF_WHEELPLUS::SimpleState state;
                bool find = false;
                for (int i = size - 2; i >= 0; i--)
                {
                    if (mSimpleStateQueue.at(i).t <= gpsOdom.t && mSimpleStateQueue.at(i + 1).t >= gpsOdom.t)
                    {
                        GetInterpSimpleState(gpsOdom.t, mSimpleStateQueue.at(i), mSimpleStateQueue.at(i + 1), state);
                        find = true;
                        break;
                    }
                }
                if (find)
                {
                    Eigen::Vector3d gps_pos = Vector2dToVector3d(state.gpsPos) - mat2dToMat3d(state.gpsR) * mPgv;
                    Eigen::Vector3d dpos = target_gps_pos - gps_pos;
                    pos_sum += gpsOdom.pos + dpos;
                    vel_sum += gpsOdom.vel;

                    if(gpsOdom.fs != 4)
                    {
                        loctype = gpsOdom.fs;
                    }
                    cnt++;
                }
                else
                {
                    // should not happen
                    std::cout << "error, can't find state in SmoothGps [gpsOdom.t, min_t, max_t] : "
                              << gpsOdom.t << ", " << mSimpleStateQueue.front().t << ", " << mSimpleStateQueue.back().t << std::endl;
                }
            }
        }
        if (cnt == 0)
        {
            return false;
        }

        gpsOdomSmooth.t = target_t;
        gpsOdomSmooth.pos = pos_sum / cnt;
        gpsOdomSmooth.vel = vel_sum / cnt;
        gpsOdomSmooth.fs = loctype;
        return true;
    }

    /*
     * Gps与Dr轨迹对齐
     * 条件：
     *  1、轨迹长度，2、运动速度（平均），3、当前点到前面轨迹点的最大距离（看离哪个点最远）
     *  4、优化残差
     */
    bool Estimater::GpsDrAlign()
    {
        static double last_try_time = 0;

        if (!mbDrInited || mStateQueue.size() < 10 || mGpsOdomQueue.size() < 10)
        {
            clink::Logger("debug_log.txt") << " !mbDrInited || mStateQueue.size() < 10 || mGpsOdomQueue.size() < 10 "<< std::endl;
            return false;
        }

        if (mImuQueue.at(mCalcToImuIndx).t - last_try_time < 1.0)
        {
            return false;
        }
        last_try_time = mImuQueue.at(mCalcToImuIndx).t;

        int drSize = mStateQueue.size();

        // 满足长度的轨迹的起始state索引
        int drStartIndx = -1;

        /* 计算轨迹长度（rtk天线的运动轨迹）
         * 注意：状态里面保存的是后轴中心的位置，要转换到rtk天线
         */
        double traj_len = 0;
        for (int i = drSize - 2; i >= 0; i--)
        {
            // Eigen::Vector3d dpos = mStateQueue.at(i+1).pos - mStateQueue.at(i).pos;
            Eigen::Vector3d rtk_pos_i1 = mStateQueue.at(i + 1).pos - mStateQueue.at(i + 1).R * mPgv;
            Eigen::Vector3d rtk_pos_i = mStateQueue.at(i).pos - mStateQueue.at(i).R * mPgv;
            Eigen::Vector3d dpos = rtk_pos_i1 - rtk_pos_i; // pose 位移

            traj_len += dpos.topRows(2).norm(); // 轨迹长度

            if (traj_len > Settings::ekfInitAlignDist() * 1.2) // 2 * 1.2
            {
                drStartIndx = i;
                break;
            }
        }

        std::cout << "GpsDrAlign traj_len step1 : " << traj_len << std::endl;

        if (drStartIndx == -1 || traj_len < Settings::ekfInitAlignDist())
        {
            clink::Logger("debug_log.txt") << " mImuQueue.at(mCalcToImuIndx).t - last_try_time < 1.0 " << std::endl;
            return false;
        }

        double time_len = mStateQueue.at(drSize - 1).t - mStateQueue.at(drStartIndx).t;
        // 运动太慢
        double vel_mean = traj_len / time_len; // 速度信息

        if (vel_mean < 0.15)
        {
            std::cout << "GpsDrAlign vel_mean : " << vel_mean << std::endl;
            clink::Logger("debug_log.txt") << " vel_mean < 0.15 " << std::endl;
            return false;
        }

        // 从当前位置到之前的轨迹点的最大距离的阈值
        // double d_thre = Settings::ekfInitAlignDist() / 3;
        // clink::Logger("debug_log.txt") << " d_thre : " << d_thre << std::endl;
        // if (d_thre < 2.5)
        // {
        //     d_thre = 0.5;
        // }

        double d_thre = 0.5;
        clink::Logger("debug_log.txt") << " d_thre : " << d_thre << std::endl;

        // 从当前点计算到之前每个点的直线距离，选择直线距离最大的
        double d_max = 0;
        Eigen::Vector3d dpos;
        double d;
        for (int i = drSize - 2; i >= drStartIndx; i--)
        {
            // dpos = mStateQueue.at(drSize-1).pos - mStateQueue.at(i).pos;
            Eigen::Vector3d rtk_pos_i1 = mStateQueue.at(drSize - 1).pos - mStateQueue.at(drSize - 1).R * mPgv;
            Eigen::Vector3d rtk_pos_i = mStateQueue.at(i).pos - mStateQueue.at(i).R * mPgv;
            Eigen::Vector3d dpos = rtk_pos_i1 - rtk_pos_i;

            d = dpos.topRows(2).norm();
            if (d > d_max)
            {
                d_max = d;
                // if (d_max > d_thre)
                // {
                //     break;
                // }
            }
        }

        std::cout << "GpsDrAlign d_max step1 : " << d_max << std::endl;

        if (d_max < d_thre)
        {
            clink::Logger("debug_log.txt") << " d_max < d_thre " << std::endl;
            return false;
        }

        std::vector<EKF_WHEELPLUS::State, Eigen::aligned_allocator<EKF_WHEELPLUS::State>> vDrdata;
        bool first = true;
        int gps_cnt = 0; // 关联的gps定位数量
        // mStateQueue里面关联了Gps数据的state索引（min/max）
        int state_min_indx_with_gps = -1;
        int state_max_indx_with_gps = -1;
        for (int i = drStartIndx; i < drSize - 1; i++)
        {
            // 先找到第一个即有预积分又关联了Gps数据的sate
            if (first && (!mStateQueue.at(i).bPreintegred || !mStateQueue.at(i).gpsOdom))
            {
                continue;
            }
            first = false;

            vDrdata.push_back(mStateQueue.at(i));

            if (mStateQueue.at(i).gpsOdom)
            {
                if (state_min_indx_with_gps == -1)
                {
                    state_min_indx_with_gps = i;
                }
                state_max_indx_with_gps = i;
                gps_cnt++;
            }
        }

        // 修改drStartIndx为state_min_indx_with_gps，后面可能会用到
        drStartIndx = state_min_indx_with_gps;

        if (gps_cnt < 10 || gps_cnt < vDrdata.size() / 2)
        {
            std::cout << "GpsDrAlign few gps data [gps_cnt, vDrdata.size] : " << gps_cnt << ", " << vDrdata.size() << std::endl;
            clink::Logger("debug_log.txt") << "[GpsDrAlign]" << "few gps data [gps_cnt, vDrdata.size] : " << gps_cnt << ", " << vDrdata.size() << std::endl;
            return false;
        }

        // 再次计算轨迹长度
        traj_len = 0;
        for (int i = state_min_indx_with_gps; i < state_max_indx_with_gps - 1; i++)
        {
            // Eigen::Vector3d dpos = mStateQueue[i+1].pos - mStateQueue[i].pos;
            Eigen::Vector3d rtk_pos_i1 = mStateQueue.at(i + 1).pos - mStateQueue.at(i + 1).R * mPgv;
            Eigen::Vector3d rtk_pos_i = mStateQueue.at(i).pos - mStateQueue.at(i).R * mPgv;
            Eigen::Vector3d dpos = rtk_pos_i1 - rtk_pos_i;

            traj_len += dpos.topRows(2).norm();
        }

        std::cout << "GpsDrAlign traj_len step2 : " << traj_len << std::endl;
        clink::Logger("debug_log.txt") << "traj_len step2 : " << traj_len << std::endl;

        if (traj_len < Settings::ekfInitAlignDist() * 0.85)
        {
            return false;
        }

        // 再次计算起点到之前轨迹点的最大距离
        d_max = 0;
        int max_dist_indx = -1;
        // int vDrdataSize = vDrdata.size();
        for (int i = state_max_indx_with_gps - 1; i >= state_min_indx_with_gps; i--)
        {
            if (!mStateQueue[i].gpsOdom)
            {
                continue;
            }

            // dpos = mStateQueue[state_max_indx_with_gps].pos - mStateQueue[i].pos;
            Eigen::Vector3d rtk_pos_i1 = mStateQueue.at(state_max_indx_with_gps).pos - mStateQueue.at(state_max_indx_with_gps).R * mPgv;
            Eigen::Vector3d rtk_pos_i = mStateQueue.at(i).pos - mStateQueue.at(i).R * mPgv;
            Eigen::Vector3d dpos = rtk_pos_i1 - rtk_pos_i;

            d = dpos.topRows(2).norm();
            if (d > d_max)
            {
                d_max = d;
                max_dist_indx = i;

                // if(d_max > d_thre)
                // {
                //     break;
                // }
            }
        }

        std::cout << "GpsDrAlign d_max step2 : " << d_max << std::endl;
        clink::Logger("debug_log.txt") << "d_max step2 : " << d_max << std::endl;

        if (d_max < d_thre)
        {
            return false;
        }

        // 先求Dr轨迹对应的GPS坐标，作为优化初值
        // dpos = mStateQueue[state_max_indx_with_gps].pos - mStateQueue[max_dist_indx].pos;
        Eigen::Vector3d rtk_pos_i1 = mStateQueue.at(state_max_indx_with_gps).pos - mStateQueue.at(state_max_indx_with_gps).R * mPgv;
        Eigen::Vector3d rtk_pos_i = mStateQueue.at(max_dist_indx).pos - mStateQueue.at(max_dist_indx).R * mPgv;
        dpos = rtk_pos_i1 - rtk_pos_i;

        double heading_dr = atan2(dpos(0), dpos(1));
        dpos = mStateQueue[state_max_indx_with_gps].gpsOdom->pos - mStateQueue[max_dist_indx].gpsOdom->pos;
        double heading_gps = atan2(dpos(0), dpos(1));
        double dyaw = -(heading_gps - heading_dr); // 方向差值
        Eigen::Matrix2d R_gps_dr;                  // dr 到 gps坐标系下的变换
        R_gps_dr << cos(dyaw), -sin(dyaw), sin(dyaw), cos(dyaw);
        Eigen::Vector2d dr_pos0(mStateQueue[max_dist_indx].pos.topRows(2));
        Eigen::Vector2d gps_pos0(mStateQueue[max_dist_indx].gpsOdom->pos.topRows(2));

        /* 注意：这里计算dr.gpsPos有问题，因为state的pos和gpsPos都定义在后轴中心
        * 下面计算的gpsPos可能是rtk天线对应的gps坐标，没办法只能这样计算，因为没有绝对方向没法消除杆臂的影响
        * 对后面的Optimize()函数的影响是初值不对。因为杆臂值比较小，通过优化可以收敛到后轴中心
        */
        for (int i = 0; i < vDrdata.size(); i++)
        {
            EKF_WHEELPLUS::State &dr = vDrdata[i];
            dr.gpsPos = R_gps_dr * (dr.pos.topRows(2) - dr_pos0) + gps_pos0;
            Eigen::Vector3d atti = getAttitudeFromR(dr.R);
            Eigen::Matrix2d R_tmp = Yaw2R(atti(2));
            dr.gpsR = R_gps_dr * R_tmp;
        }

        // 轨迹对齐（优化）
        EKF_WHEELPLUS::MarginInfo *pMarginInfo = NULL;
        double chi2;
        double robust_chi2;
        Optimize(vDrdata, NULL, false, chi2, robust_chi2);
        double std = sqrt(chi2 / (vDrdata.size() - 1));
        double robust_std = sqrt(robust_chi2 / (vDrdata.size() - 1));

        std::cout << "GpsDrAlign std , robust_std : " << std << ", " << robust_std << std::endl;
        clink::Logger("debug_log.txt") << "std , robust_std : " << std << ", " << robust_std << std::endl;
        // if (std < 0.5 && robust_std < 0.25)
        if (std < 2 && robust_std < 2)
        {
            /*
             * 对所有dr数据做处理：计算gpsR, gpsPos
             *   因为只有部分Dr数据参与了优化，遍历Dr数组，找最接近的参与了优化的，作为参考
             */
            for (int i = 0; i < mStateQueue.size(); i++)
            {
                EKF_WHEELPLUS::State &state = mStateQueue.at(i);
                double t = state.t;

                int match_indx = -1;
                double min_t_diff = 1e10;
                for (int j = 0; j < vDrdata.size(); j++)
                {
                    if (fabs(vDrdata[j].t - t) < min_t_diff)
                    {
                        min_t_diff = fabs(vDrdata[j].t - t);
                        match_indx = j;
                    }
                }

                EKF_WHEELPLUS::State &state_ref = vDrdata[match_indx];
                Eigen::Vector3d atti_ref = getAttitudeFromR(state_ref.R);
                Eigen::Matrix2d R_ref = Yaw2R(atti_ref(2));
                Eigen::Vector3d atti = getAttitudeFromR(state.R);
                Eigen::Matrix2d R = Yaw2R(atti(2));
                Eigen::Matrix2d dR = R_ref.transpose() * R;
                Eigen::Vector2d dPos = R_ref.transpose() * (state.pos.topRows(2) - state_ref.pos.topRows(2));

                state.gpsR = state_ref.gpsR * dR;
                state.gpsPos = state_ref.gpsPos + state_ref.gpsR * dPos;
            }

            /*
             * 计算gps到dr的变换矩阵
             *   第一次轨迹对齐之后，[gpsR, gpsPos] 代表gps坐标系下的位姿，[R,pos]代表Dr坐标系的坐标
             */
            Eigen::Matrix3d T_Dr_b = Eigen::Matrix3d::Identity();
            Eigen::Vector3d atti = getAttitudeFromR(vDrdata[vDrdata.size() - 1].R);
            T_Dr_b.block<2, 2>(0, 0) = Yaw2R(atti(2));
            T_Dr_b.block<2, 1>(0, 2) = vDrdata[vDrdata.size() - 1].pos.topRows(2);
            Eigen::Matrix3d T_Gps_b = Eigen::Matrix3d::Identity();
            T_Gps_b.block<2, 2>(0, 0) = vDrdata[vDrdata.size() - 1].gpsR;
            T_Gps_b.block<2, 1>(0, 2) = vDrdata[vDrdata.size() - 1].gpsPos;
            // gps到dr的变换矩阵
            mT_Dr_Gps_fixed = T_Dr_b * T_Gps_b.inverse();
            mT_Dr_Gps_fixed_inv = mT_Dr_Gps_fixed.inverse();

            // 将DR与GPS的位姿都转换到DR坐标系
            for (int i = 0; i < mStateQueue.size(); i++)
            {
                EKF_WHEELPLUS::State &state = mStateQueue.at(i);
                state.gpsPos = mT_Dr_Gps_fixed.block<2, 2>(0, 0) * state.gpsPos + mT_Dr_Gps_fixed.block<2, 1>(0, 2);
                state.gpsR = mT_Dr_Gps_fixed.block<2, 2>(0, 0) * state.gpsR;
                if (state.gpsOdom)
                {
                    state.gpsOdom->pos.topRows(2) = mT_Dr_Gps_fixed.block<2, 2>(0, 0) * state.gpsOdom->pos.topRows(2) + mT_Dr_Gps_fixed.block<2, 1>(0, 2);
                }
            }
            for (int i = 0; i < mGpsOdomQueue.size(); i++)
            {
                GpsOdometry &gpsOdom = mGpsOdomQueue.at(i);
                gpsOdom.pos.topRows(2) = mT_Dr_Gps_fixed.block<2, 2>(0, 0) * gpsOdom.pos.topRows(2) + mT_Dr_Gps_fixed.block<2, 1>(0, 2);
            }

#ifndef SUNRISE_X3
            // 替换显示数据
            if (mpViewer)
            {
                std::vector<ViewData> veRtkdata;
                std::vector<ViewData> veEkfdata;
                for (int i = 0; i < mStateQueue.size(); i++)
                {
                    EKF_WHEELPLUS::State &state = mStateQueue.at(i);
                    ViewData viewData;
                    viewData.pos << state.gpsPos(0), state.gpsPos(1), 0;
                    viewData.atti = getAttitudeFromR(mat2dToMat3d(state.gpsR));
                    // 从后轴中心转换到rtk中心再输出，方便与rtk比较
                    viewData.pos -= mat2dToMat3d(state.gpsR) * mPgv;

                    veEkfdata.push_back(viewData);
                }
                for (int i = 0; i < mGpsOdomQueue.size(); i++)
                {
                    GpsOdometry &gpsOdom = mGpsOdomQueue.at(i);
                    ViewData viewData;
                    viewData.pos = gpsOdom.pos;
                    viewData.atti = Eigen::Vector3d::Zero();
                    veRtkdata.push_back(viewData);
                }

                mpViewer->ReplaceRtkEkfViewdatas(veRtkdata, veEkfdata);
            }
#endif

            // 清除mStateQueue位于drStartIndx前面的数据
            int i = 0;
            while (i < drStartIndx)
            {
                mLastState0Time = mStateQueue.front().t;
                mStateQueue.pop_front();
                i++;
            }

            // 设置边缘化初值
            mMarginInfo.state << getYawFromR(mStateQueue.front().gpsR), mStateQueue.front().gpsPos;
            Eigen::Matrix3d cov = Eigen::Matrix3d::Identity();
            cov(0, 0) = 1.5 / 57.3 * 1.5 / 57.3; // 先驗標準差
            cov(1, 1) = 0.15 * 0.15;
            cov(2, 2) = 0.15 * 0.15;
            mMarginInfo.info = cov.inverse();

            mbGpsAigned = true;
        }

        return true;
    }

    void Estimater::GetInterpImu(double t, const IMU::Point &imu1, const IMU::Point &imu2, IMU::Point &imuInterp)
    {
        double dT = imu2.t - imu1.t;
        double dt = t - imu1.t;
        Eigen::Vector3f da = imu2.a - imu1.a;
        Eigen::Vector3f dw = imu2.w - imu1.w;

        imuInterp.t = t;
        imuInterp.a = imu1.a + dt / dT * da;
        imuInterp.w = imu1.w + dt / dT * dw;
    }

    void Estimater::GetInterpState(double t, const EKF_WHEELPLUS::State &state1, const EKF_WHEELPLUS::State &state2, EKF_WHEELPLUS::State &stateInterp)
    {
        double dT = state2.t - state1.t;
        double dt = t - state1.t;

        stateInterp.t = t;
        stateInterp.pos = state1.pos + (state2.pos - state1.pos) * dt / dT;
        stateInterp.vel = state1.vel + (state2.vel - state1.vel) * dt / dT;

        Eigen::Vector3d atti1 = getAttitudeFromR(state1.R);
        Eigen::Vector3d atti2 = getAttitudeFromR(state2.R);
        Eigen::Vector3d attiInterp;
        attiInterp(0) = atti1(0) + (atti2(0) - atti1(0)) * dt / dT;
        attiInterp(1) = atti1(1) + (atti2(1) - atti1(1)) * dt / dT;
        double dyaw = atti2(2) - atti1(2);
        while (dyaw > M_PI)
        {
            dyaw -= 2 * M_PI;
        }
        while (dyaw < -M_PI)
        {
            dyaw += 2 * M_PI;
        }
        attiInterp(2) = atti1(2) + dyaw * dt / dT;
        stateInterp.R = Attitude2R(attiInterp);

        stateInterp.ba = state1.ba;
        stateInterp.bg = state1.bg;
        // stateInterp.wheelplus_sf = state1.wheelplus_sf;
        // stateInterp.cov = state1.cov;
    }

    void Estimater::GetInterpSimpleState(double t, const EKF_WHEELPLUS::SimpleState &simplestate1, const EKF_WHEELPLUS::SimpleState &simplestate2,
                                         EKF_WHEELPLUS::SimpleState &simplestateInterp)
    {
        double dT = simplestate2.t - simplestate1.t;
        double dt = t - simplestate1.t;

        simplestateInterp.t = t;
        simplestateInterp.pos = simplestate1.pos + (simplestate2.pos - simplestate1.pos) * dt / dT;

        Eigen::Vector3d atti1 = getAttitudeFromR(simplestate1.R);
        Eigen::Vector3d atti2 = getAttitudeFromR(simplestate2.R);
        Eigen::Vector3d attiInterp;
        attiInterp(0) = atti1(0) + (atti2(0) - atti1(0)) * dt / dT;
        attiInterp(1) = atti1(1) + (atti2(1) - atti1(1)) * dt / dT;
        double dyaw = atti2(2) - atti1(2);
        while (dyaw > M_PI)
        {
            dyaw -= 2 * M_PI;
        }
        while (dyaw < -M_PI)
        {
            dyaw += 2 * M_PI;
        }
        attiInterp(2) = atti1(2) + dyaw * dt / dT;
        simplestateInterp.R = Attitude2R(attiInterp);

        simplestateInterp.gpsPos = simplestate1.gpsPos + (simplestate2.gpsPos - simplestate1.gpsPos) * dt / dT;
        double yaw1 = getYawFromR(simplestate1.gpsR);
        double yaw2 = getYawFromR(simplestate2.gpsR);
        dyaw = yaw2 - yaw1;
        while (dyaw > M_PI)
        {
            dyaw -= 2 * M_PI;
        }
        while (dyaw < -M_PI)
        {
            dyaw += 2 * M_PI;
        }
        double yaw = yaw1 + dyaw * dt / dT;
        simplestateInterp.gpsR = Yaw2R(yaw);
    }

    // 用加速度修正pitch/roll，每0.1秒修正一次
    // 根据加速度标准差和预测残差调整修正系数：[0.003, 0.03]
    void Estimater::CalibPitchRollWithAccel(const WheelOdometer &wheelOdometer, EKF_WHEELPLUS::State &state, const Eigen::Vector2d &state_atti_mean,
                                            const Eigen::Vector3d &accel_mean, const Eigen::Vector3d &gyro_mean, const Eigen::Vector3d &accel_std)
    {
        double stopAccelStdthre = 0.05;
        double pitch_diff_max = 5.0 * M_PI / 180;
        double roll_diff_max = 5.0 * M_PI / 180;
        double calib_ratio_max = 0.03;
        double calib_ratio_min = 0.003;

        Eigen::Vector3d gyro_mean_no_bias = gyro_mean - state.bg;
        Eigen::Vector3d wheelspeed;
        wheelspeed << 0, wheelOdometer.vel, 0;
        wheelspeed = wheelspeed * mWheelplusSF;
        // 计算向心加速度 a_centrip = w*v
        Eigen::Vector3d centrip_accel = EKF_WHEELPLUS::getSkewMatrix(gyro_mean_no_bias) * wheelspeed;
        // 补偿零偏，补偿向心加速度：这里不考虑imu坐标系与车身坐标系的差异
        double ax = accel_mean(0) - state.ba(0) - centrip_accel(0);
        double ay = accel_mean(1) - state.ba(1) - centrip_accel(1);

        Eigen::Vector2d accel_pitchroll;
        accel_pitchroll(0) = asin(ay / IMU::GRAVITY_VALUE);
        accel_pitchroll(1) = -asin(ax / (IMU::GRAVITY_VALUE * cos(accel_pitchroll(0))));

        double calib_ratio;
        double accel_xy_std_max = accel_std(0) > accel_std(1) ? accel_std(0) : accel_std(1);
        if (accel_xy_std_max < stopAccelStdthre)
        {
            calib_ratio = calib_ratio_max;
        }
        else
        {
            calib_ratio = calib_ratio_max * stopAccelStdthre / accel_xy_std_max;
        }

        // 修正pitch/roll
        Eigen::Vector3d atti = getAttitudeFromR(state.R);
        // 1. pitch
        double calib_ratio_pitch = calib_ratio;
        double pitch_diff = accel_pitchroll(0) - state_atti_mean(0);
        if (fabs(pitch_diff) > pitch_diff_max)
        {
            calib_ratio_pitch = calib_ratio_pitch * pitch_diff_max / fabs(pitch_diff);
        }

        if (calib_ratio_pitch > calib_ratio_max)
        {
            calib_ratio_pitch = calib_ratio_max;
        }
        if (calib_ratio_pitch < calib_ratio_min)
        {
            calib_ratio_pitch = calib_ratio_min;
        }
        atti(0) += pitch_diff * calib_ratio_pitch;

        // 2. roll
        double calib_ratio_roll = calib_ratio;
        double roll_diff = accel_pitchroll(1) - state_atti_mean(1);
        if (fabs(roll_diff) > roll_diff_max)
        {
            calib_ratio_roll = calib_ratio_roll * roll_diff_max / fabs(roll_diff);
        }

        if (calib_ratio_roll > calib_ratio_max)
        {
            calib_ratio_roll = calib_ratio_max;
        }
        if (calib_ratio_roll < calib_ratio_min)
        {
            calib_ratio_roll = calib_ratio_min;
        }
        atti(1) += roll_diff * calib_ratio_roll;

        //
        state.R = Attitude2R(atti);
    }

    /* 确定可以计算到哪个IMU数据：运动状态和导航计算（DR）都算到同一个IMU
     *    不能超过最新的轮速数据的时刻latest_wheel_t + tolerance
     *    因为计算运动状态/DR推算都需要同时用到imu数据与轮速数据
     *  0.05
     */
    void Estimater::FindCalcToImuIndx(double tolerance)
    {
        mCalcToImuIndx = -1;
        if (mImuQueue.empty() || mWheelOdometerQueue.empty())
        {
            return;
        }

        double latest_wheel_t = mWheelOdometerQueue.back().t; // 轮速最新的数据
        int imu_size = mImuQueue.size();
        for (int i = 0; i < imu_size; i++) // 遍历imu数据
        {
            int indx = imu_size - 1 - i;
            if (mImuQueue.at(indx).t < latest_wheel_t + tolerance) // 获取与最新轮速数据时间最接近的Imu数据idx
            {
                mCalcToImuIndx = indx;
                return;
            }
        }
    }

    /* 确定从哪个IMU数据开始计算运动状态：
     *    如果从来没算过，就从0开始；否则从已经算过的下一个开始
     */
    void Estimater::FindMotionStatusCalcFromImuIndx(int &fromImuIndx)
    {
        fromImuIndx = -1;

        if (mImuQueue.empty() || mWheelOdometerQueue.empty())
        {
            return;
        }

        if (mMotionstatusQueue.empty())
        {
            fromImuIndx = 0;
        }
        else
        {
            double latest_motionstatus_t = mMotionstatusQueue.back().t;
            int indx = getImuIndx(latest_motionstatus_t, 1e-3);
            if (indx < 0)
            {
                return;
            }
            fromImuIndx = indx + 1;
        }
    }

    // 计算t时刻的运动状态，取前面一秒的数据：imu/轮速
    void Estimater::CalcMotionstatus(double cur_t, MotionStatus &motionstatus)
    {
        // 先硬编码阈值
        double stopAccelthre = 0.05;
        double stopAngularthre = 1.0 * M_PI / 180; // 
        double stopVelthre = 0.02;

        motionstatus.t = cur_t;

        int size = mImuQueue.size();
        if (size == 0) // 状态未知先置为MOVE
        {
            motionstatus.status = EnMotionStatus::MOVE;
            return;
        }

        int target_indx = getImuIndx(cur_t, 1e-3);
        if (target_indx == -1) // 获取不到合适的imu数据 状态置为MOVE
        {
            motionstatus.status = EnMotionStatus::MOVE;
            return;
        }
        if (target_indx < 100) // imu数据较少 状态置为MOVE
        {
            motionstatus.status = EnMotionStatus::MOVE;
            return;
        }

        double dt = mImuQueue.at(target_indx).t - mImuQueue.front().t;
        if (dt < 1.0)
        {
            motionstatus.status = EnMotionStatus::MOVE;
            return;
        }

        // 根据轮速判断
        {
            // std::lock_guard<std::mutex> lock(mMutexWheelOdometer);

            int indx_max = getWheelodometerIndx(cur_t, 0.15); // 找到对应时间的轮速数据
            if (indx_max == -1)                               // 找不到对应的轮速数据
            {
                motionstatus.status = EnMotionStatus::MOVE; // 默认MOVE就不初始化
                return;
            }

            double dt = mWheelOdometerQueue.at(indx_max).t - mWheelOdometerQueue.front().t;
            if (dt < 1.0)
            {
                motionstatus.status = EnMotionStatus::MOVE; // 时间较短
                return;
            }

            int indx_min = -1;
            int indx;
            for (int i = 0; i <= indx_max; i++)
            {
                indx = indx_max - i;
                // 考虑到轮速的数据频率、以及与IMU数据到达的先后顺序随机性
                if (mWheelOdometerQueue.at(indx).t < cur_t - 1.0) // 找到第一个小于cur_t - 1.0 时间的数据
                {
                    indx_min = indx;
                    break;
                }
            }
            if (indx_max < 0 || indx_min < 0)
            {
                motionstatus.status = EnMotionStatus::MOVE;
                return;
            }
            if (mWheelOdometerQueue.at(indx_max).t - mWheelOdometerQueue.at(indx_min).t < 0.8)
            {
                motionstatus.status = EnMotionStatus::MOVE;
                return;
            }
            //
            double d_l = mWheelOdometerQueue.at(indx_max).whlplus_l - mWheelOdometerQueue.at(indx_min).whlplus_l;
            double d_r = mWheelOdometerQueue.at(indx_max).whlplus_r - mWheelOdometerQueue.at(indx_min).whlplus_r;
            if (fabs(d_l) > stopVelthre || fabs(d_r) > stopVelthre)
            {
                motionstatus.status = EnMotionStatus::MOVE;
                return;
            }

            for (int i = indx_min; i < indx_max; i++)
            {
                // double d_l = mWheelOdometerQueue.at(i + 1).whlplus_l - mWheelOdometerQueue.at(i).whlplus_l;
                // double d_r = mWheelOdometerQueue.at(i + 1).whlplus_r - mWheelOdometerQueue.at(i).whlplus_r;
                double d_l = mWheelOdometerQueue.at(i + 1).d_l;
                double d_r = mWheelOdometerQueue.at(i + 1).d_r;
                double dt = mWheelOdometerQueue.at(i + 1).t - mWheelOdometerQueue.at(i + 1).pre_t;
                if (fabs(d_l) > stopVelthre * dt || fabs(d_r) > stopVelthre * dt)
                {
                    motionstatus.status = EnMotionStatus::MOVE;
                    return;
                }
            }
        }

        // 根据imu数据判断
        int indx0 = -1;
        for (int i = target_indx; i >= 0; i--)
        {
            if (mImuQueue.at(i).t < cur_t - 1.0)
            {
                indx0 = i;
                break;
            }
        }
        if (indx0 < 0 || target_indx - indx0 < 50)
        {
            motionstatus.status = EnMotionStatus::MOVE;
            return;
        }
        // 计算均值
        Eigen::Vector3d accel_sum;
        Eigen::Vector3d angular_sum;
        Eigen::Vector3d accel_mean;
        Eigen::Vector3d angular_mean;

        accel_sum.setZero();
        angular_sum.setZero();
        for (int i = indx0; i <= target_indx; i++)
        {
            accel_sum += mImuQueue.at(i).a.cast<double>();
            angular_sum += mImuQueue.at(i).w.cast<double>();
        }
        accel_mean = accel_sum / (target_indx - indx0 + 1);
        angular_mean = angular_sum / (target_indx - indx0 + 1);

        // 计算方差
        accel_sum.setZero();
        angular_sum.setZero();
        Eigen::Vector3d accel;
        Eigen::Vector3d angular;
        for (int i = indx0; i <= target_indx; i++)
        {
            accel = mImuQueue.at(i).a.cast<double>() - accel_mean;
            angular = mImuQueue.at(i).w.cast<double>() - angular_mean;
            accel_sum(0) += accel(0) * accel(0);
            accel_sum(1) += accel(1) * accel(1);
            accel_sum(2) += accel(2) * accel(2);
            angular_sum(0) += angular(0) * angular(0);
            angular_sum(1) += angular(1) * angular(1);
            angular_sum(2) += angular(2) * angular(2);
        }

        Eigen::Vector3d accel_std;
        Eigen::Vector3d angular_std;
        for (int i = 0; i < 3; i++)
        {
            accel_std(i) = sqrt(accel_sum(i) / (target_indx - indx0));
            angular_std(i) = sqrt(angular_sum(i) / (target_indx - indx0));
        }
        if (accel_std(0) > stopAccelthre || accel_std(1) > stopAccelthre || accel_std(2) > stopAccelthre ||
            angular_std(0) > stopAngularthre || angular_std(1) > stopAngularthre || angular_std(2) > stopAngularthre)
        {
            motionstatus.status = EnMotionStatus::MOVE;
        }
        else
        {
            motionstatus.status = EnMotionStatus::STOP;
        }
        // std::cout << static_cast<int>(motionstatus.status) << angular_std(0) << " " << angular_std(1) << " " << angular_std(2) << std::endl;
        // clink::Logger("move_state_debug.txt") << cur_t << " " << static_cast<int>(motionstatus.status) << " " << accel_std(0) << " " << accel_std(1) << " " << accel_std(2) << " "
        //                                       << angular_std(0) << " " << angular_std(1) << " " << angular_std(2) << std::endl;
    }

    void Estimater::UpdateImustopinfo(double cur_t)
    {
        int target_indx = getMotionstatusIndx(cur_t, 1e-3);
        if (target_indx < 0)
        {
            return;
        }

        MotionStatus motionstatus;
        motionstatus.t = mMotionstatusQueue.at(target_indx).t;
        if (isStopInRecent1s(motionstatus.t))
        {
            motionstatus.status = EnMotionStatus::STOP;
        }
        else
        {
            motionstatus.status = EnMotionStatus::MOVE;
        }

        if (mvStopImuInfo.size() == 0)
        {
            StopImuInfo stopImuinfoNew;
            stopImuinfoNew.t0 = motionstatus.t;
            stopImuinfoNew.t = motionstatus.t;
            stopImuinfoNew.gyro_bias_mean = Eigen::Vector3d::Zero();
            stopImuinfoNew.pitchroll_mean = Eigen::Vector2d::Zero();
            stopImuinfoNew.time_len_mean = 0;
            stopImuinfoNew.available = false;
            printf("UpdateImustopinfo [1] \n");
            mvStopImuInfo.push_back(stopImuinfoNew);
        }

        int size = mvStopImuInfo.size();
        StopImuInfo &stopImuinfo = mvStopImuInfo[size - 1];

        // 看是否1秒内静止，是则计算陀螺仪零偏，以及根据加速度计算pitch/roll
        if (motionstatus.status == EnMotionStatus::STOP) // 1、当前时刻静止
        {
            if (mIsStopLasttime) // 1.1 上一时刻静止
            {
                // 更新最新的静止时刻
                stopImuinfo.t = motionstatus.t;
                // if(stopImuinfo.t - stopImuinfo.t0 > 3.0)
                // {
                //     stopImuinfo.available = true;
                // }

                // 每秒计算一次零偏和 pitch roll
                if (stopImuinfo.t - stopImuinfo.t0 >= stopImuinfo.time_len_mean + 1.0)
                {
                    Eigen::Vector2d pitchroll;
                    Eigen::Vector3d gyrobias;
                    if (CalcPitchrollGyrobias_1s(motionstatus.t, pitchroll, gyrobias))
                    {
                        stopImuinfo.pitchroll_mean = (stopImuinfo.pitchroll_mean * stopImuinfo.time_len_mean + pitchroll) / (stopImuinfo.time_len_mean + 1.0);
                        stopImuinfo.gyro_bias_mean = (stopImuinfo.gyro_bias_mean * stopImuinfo.time_len_mean + gyrobias) / (stopImuinfo.time_len_mean + 1.0);
                        stopImuinfo.time_len_mean += 1.0;
                        // TODO 后面修改成静止时间不够 Bias 初始化为0
                        if (stopImuinfo.time_len_mean >= 3.0) // 静止时间 3s 最好(gnss 代码修改为 0.1)
                        {
                            stopImuinfo.available = true;
                        }
                    }
                    else
                    {
                        // 归零
                        stopImuinfo.t0 = motionstatus.t;
                        stopImuinfo.t = motionstatus.t;
                        stopImuinfo.gyro_bias_mean = Eigen::Vector3d::Zero();
                        stopImuinfo.pitchroll_mean = Eigen::Vector2d::Zero();
                        stopImuinfo.time_len_mean = 0;
                        stopImuinfo.available = false;
                    }
                }
            }
            else // 1.2 上一时刻运动
            {
                stopImuinfo.t0 = motionstatus.t;
                stopImuinfo.t = motionstatus.t;
                stopImuinfo.gyro_bias_mean = Eigen::Vector3d::Zero();
                stopImuinfo.pitchroll_mean = Eigen::Vector2d::Zero();
                stopImuinfo.time_len_mean = 0;
                stopImuinfo.available = false;
            }
        }
        else // 2、当前时刻运动
        {
            if (mIsStopLasttime) // 2.1 上一时刻静止
            {
                if (stopImuinfo.available)
                {
                    // 保存之前的信息：即生成一个新的对象添加到vector
                    StopImuInfo stopImuinfoNew;
                    stopImuinfoNew.t0 = motionstatus.t;
                    stopImuinfoNew.t = motionstatus.t;
                    stopImuinfoNew.gyro_bias_mean = Eigen::Vector3d::Zero();
                    stopImuinfoNew.pitchroll_mean = Eigen::Vector2d::Zero();
                    stopImuinfoNew.time_len_mean = 0;
                    stopImuinfoNew.available = false;
                    printf("UpdateImustopinfo[2] \n");
                    mvStopImuInfo.push_back(stopImuinfoNew);
                }
                else
                {
                    // 归零
                    stopImuinfo.t0 = motionstatus.t;
                    stopImuinfo.t = motionstatus.t;
                    stopImuinfo.gyro_bias_mean = Eigen::Vector3d::Zero();
                    stopImuinfo.pitchroll_mean = Eigen::Vector2d::Zero();
                    stopImuinfo.time_len_mean = 0;
                    stopImuinfo.available = false;
                }
            }
            else // 2.2 上一时刻运动
            {
                // do nothing
            }
        }

        mIsStopLasttime = (motionstatus.status == EnMotionStatus::STOP);
    }

    bool Estimater::isStopInRecent1s(double t)
    {
        int size = mMotionstatusQueue.size();

        if (size < 100)
        {
            return false;
        }

        int indx_t = -1;
        double t_tmp;
        for (int i = size - 1; i >= 0; i--)
        {
            t_tmp = mMotionstatusQueue.at(i).t;
            if (fabs(t - t_tmp) < 5e-3)
            {
                indx_t = i;
                break;
            }

            if (t_tmp < t - 1.0)
            {
                break;
            }
        }

        if (indx_t < 0)
        {
            return false;
        }

        if (mMotionstatusQueue.at(indx_t).status == EnMotionStatus::MOVE)
        {
            return false;
        }
        else
        {
            // 如果当前时刻静止，再检查1秒内是否都静止
            double dt = mMotionstatusQueue.at(indx_t).t - mMotionstatusQueue.front().t;
            if (dt < 1.0)
            {
                return false;
            }
            else
            {
                for (int i = indx_t; i >= 0; i--)
                {
                    if (mMotionstatusQueue.at(i).status == EnMotionStatus::MOVE)
                    {
                        return false;
                    }

                    dt = mMotionstatusQueue.at(indx_t).t - mMotionstatusQueue.at(i).t;
                    if (dt >= 1.0)
                    {
                        break;
                    }
                }

                return true;
            }
        }
    }

    bool Estimater::CalcPitchrollGyrobias_1s(double cur_t, Eigen::Vector2d &pitchroll, Eigen::Vector3d &gyrobias)
    {
        {
            // 注意：这里不加锁，在外面调用这个函数的地方加锁
            // unique_lock<mutex> lock(mMutexImuQueue);

            // 1秒之内的加速度计数据计算pitch/roll
            int target_indx = getImuIndx(cur_t, 1e-3);
            if (target_indx < 100)
            {
                return false;
            }

            int indx0 = -1;
            double dt;
            for (int i = target_indx; i >= 0; i--)
            {
                dt = mImuQueue.back().t - mImuQueue.at(i).t;
                if (dt >= 1.0)
                {
                    indx0 = i;
                    break;
                }
            }

            if (indx0 < 0)
            {
                return false;
            }

            Eigen::Vector3d accel_sum;
            Eigen::Vector3d gyro_sum;
            Eigen::Vector3d accel_mean;
            Eigen::Vector3d gyro_mean;

            accel_sum.setZero();
            gyro_sum.setZero();
            for (int i = indx0; i <= target_indx; i++)
            {
                accel_sum += mImuQueue.at(i).a.cast<double>();
                gyro_sum += mImuQueue.at(i).w.cast<double>();
            }
            accel_mean = accel_sum / (target_indx - indx0 + 1);
            gyro_mean = gyro_sum / (target_indx - indx0 + 1);

            pitchroll(0) = asin(accel_mean(1) / IMU::GRAVITY_VALUE);
            pitchroll(1) = -asin(accel_mean(0) / (IMU::GRAVITY_VALUE * cos(pitchroll(0))));

            gyrobias = gyro_mean;
        }

        return true;
    }

    int Estimater::getImuIndx(double t, double tolerance)
    {
        int target_indx = -1;

        int size = mImuQueue.size();
        if (size == 0)
        {
            return target_indx;
        }

        double min_time_diff = 1e10;
        for (int i = 0; i < size; i++)
        {
            int indx = size - 1 - i;
            double dt = mImuQueue.at(indx).t - t;

            if (dt > tolerance)
            {
                continue;
            }
            if (dt < -tolerance)
            {
                break;
            }

            if (fabs(dt) < min_time_diff)
            {
                min_time_diff = fabs(dt);
                target_indx = indx;
            }
        }

        return target_indx;
    }

    int Estimater::getWheelodometerIndx(double t, double tolerance)
    {
        int target_indx = -1;

        int size = mWheelOdometerQueue.size();
        if (size == 0)
        {
            return target_indx;
        }

        double min_time_diff = 1e10;
        for (int i = 0; i < size; i++)
        {
            int indx = size - 1 - i;
            double dt = mWheelOdometerQueue.at(indx).t - t;

            if (dt > tolerance)
            {
                continue;
            }
            if (dt < -tolerance)
            {
                break;
            }

            if (fabs(dt) < min_time_diff)
            {
                min_time_diff = fabs(dt);
                target_indx = indx;
            }
        }

        // 如果丢了数据，会导致相邻数据的时间间隔变大，前面可能找不到tolerance范围内的匹配数据，下面就找最接近的
        if (target_indx < 0)
        {
            for (int i = 0; i < size - 1; i++)
            {
                if (mWheelOdometerQueue.at(i).t <= t && mWheelOdometerQueue.at(i + 1).t >= t)
                {
                    double dt1 = fabs(mWheelOdometerQueue.at(i).t - t);
                    double dt2 = fabs(mWheelOdometerQueue.at(i + 1).t - t);
                    if (dt1 <= dt2)
                    {
                        target_indx = i;
                    }
                    else
                    {
                        target_indx = i + 1;
                    }

                    break;
                }
            }
        }

        return target_indx;
    }

    int Estimater::getMotionstatusIndx(double t, double tolerance)
    {
        int target_indx = -1;

        int size = mMotionstatusQueue.size();
        if (size == 0)
        {
            return target_indx;
        }

        double min_time_diff = 1e10;
        for (int i = 0; i < size; i++)
        {
            int indx = size - 1 - i;
            double dt = mMotionstatusQueue.at(indx).t - t;

            if (dt > tolerance)
            {
                continue;
            }
            if (dt < -tolerance)
            {
                break;
            }

            if (fabs(dt) < min_time_diff)
            {
                min_time_diff = fabs(dt);
                target_indx = indx;
            }
        }

        return target_indx;
    }

    bool Estimater::IsStopOntime(double t, double tolerance)
    {
        {
            // std::lock_guard<std::mutex> lock(mMutexSensordata);

            int size = mvStopImuInfo.size();

            if (size == 0)
            {
                return false;
            }

            for (int i = 0; i < size; i++)
            {
                StopImuInfo &stopImuinfo = mvStopImuInfo[i];
                if (!stopImuinfo.available)
                {
                    continue;
                }

                if (t + tolerance >= stopImuinfo.t0 && t - tolerance <= stopImuinfo.t)
                {
                    return true;
                }
            }

            return false;
        }
    }

    bool Estimater::GetImustopinfoOntime(double t, double tolerance, StopImuInfo &stopImuinfo)
    {
        {
            // std::lock_guard<std::mutex> lock(mMutexSensordata);

            int size = mvStopImuInfo.size();
            if (size == 0)
            {
                return false;
            }

            for (int i = 0; i < size; i++)
            {
                StopImuInfo &stopImuinfo_tmp = mvStopImuInfo[i];
                if (!stopImuinfo_tmp.available)
                {
                    continue;
                }
                if (t + tolerance >= stopImuinfo_tmp.t0 && t - tolerance <= stopImuinfo_tmp.t)
                {
                    stopImuinfo = stopImuinfo_tmp;
                    return true;
                }
            }

            return false;
        }
    }

    /*
     * 删除老的数据：
     *   mStateQueue 在滑窗中已经处理了
     * 
     * 修改：都保留10s的数据，为了处理打滑判断后的回退计算
     */
    void Estimater::RemoveOldQueuedatas()
    {
        if (mStateQueue.size() < 5 || mMotionstatusQueue.size() < 100)
        {
            return;
        }

        // 因为对Gps数据做平滑只能计算到：mStateQueue.size() - 2
        double ref_t_sate = mStateQueue.at(mStateQueue.size() - 2).t;
        double ref_t_motionstatus = mMotionstatusQueue.back().t;
        double dt;
        double min_t;

        // 1. imu
        while (mImuQueue.size() > 100)
        {
            dt = mImuQueue.back().t - mImuQueue.front().t;
            min_t = mImuQueue.at(1).t;
            if (dt > 10.0 && min_t < ref_t_sate - 0.1 && min_t < ref_t_motionstatus - 0.1)
            {
                mImuQueue.pop_front();
            }
            else
            {
                break;
            }
        }

        // 2. wheel odometer
        while (mWheelOdometerQueue.size() > 100)
        {
            dt = mWheelOdometerQueue.back().t - mWheelOdometerQueue.front().t;
            min_t = mWheelOdometerQueue.at(1).t;
            if (dt > 10.0 && min_t < ref_t_sate - 0.1 && min_t < ref_t_motionstatus - 0.1)
            {
                mWheelOdometerQueue.pop_front();
            }
            else
            {
                break;
            }
        }

        // 3. gps
        while(mGpsOdomQueue.size() > 0)
        {
            min_t = mGpsOdomQueue.front().t;
            if(min_t < ref_t_sate - GPS_DATA_MAX_DELAY && min_t < mLaseGpsProcessedTime 
                && min_t < ref_t_sate - 10.0)
            {
                mGpsOdomQueue.pop_front();
            }
            else
            {
                break;
            }
        }


        // 4. motion status
        while (mMotionstatusQueue.size() > 100)
        {
            dt = mMotionstatusQueue.back().t - mMotionstatusQueue.front().t;
            if (dt > 10.0 && mMotionstatusQueue.at(1).t < ref_t_sate - GPS_DATA_MAX_DELAY)
            {
                mMotionstatusQueue.pop_front();
            }
            else
            {
                break;
            }
        }

        // 5. simple state
        while(mSimpleStateQueue.size() > 100)
        {
            double gps_ref_t = 1e30;
            if(!mGpsOdomQueue.empty())
            {
                gps_ref_t = mGpsOdomQueue.back().t;
            }

            dt = mSimpleStateQueue.back().t - mSimpleStateQueue.front().t;
            min_t = mSimpleStateQueue.at(1).t;
            if(dt > 10.0 && min_t < ref_t_sate - GPS_DATA_MAX_DELAY 
                && min_t < mLaseGpsProcessedTime && min_t < gps_ref_t)
            {
                mSimpleStateQueue.pop_front();
            }
            else
            {
                break;
            }
        }

        // 6. preintegrate info
        for(int i=0; i<mStateQueue.size()-2; i++)
        {
            EKF_WHEELPLUS::State &state = mStateQueue[i];
            
            // 优化：至少保留5秒的数据
            dt = mStateQueue.back().t - mStateQueue[i+1].t;
            if(dt <= 10.0)
            {
                break;
            }

            state.preintegInfo.ve_dt.clear();
            state.preintegInfo.ve_d.clear();
            state.preintegInfo.ve_yaw.clear();
            state.preintegInfo.ve_motionStatus.clear();
        }

        // 清理 Align data
        while(AlignData.size() > 10) 
        {
            AlignData.pop_front();
        }

        // simple gps queue
        while (mSimpleGpsQueue.size() > 1000)
        {
            if (mSimpleGpsQueue.at(1).t < mStateQueue.front().t)
            {
                mSimpleGpsQueue.pop_front();
            }
            else
            {
                break;
            }
        }

        // rtk slipinfo queue
        while (mRtkSlipInfos.size() > 1)
        {
            if (mRtkSlipInfos.front().t1 < mStateQueue.front().t)
            {
                mRtkSlipInfos.pop_front();
            }
            else
            {
                break;
            }
        }

    }

    // 计算加速度/角速度均值、加速度标准差，用0.5秒平滑（取前面0.5秒内的数据计算）
    bool Estimater::CalcImuMeanAndStdInfos(double t, double smooth_timelen,
                                           Eigen::Vector3d &accel_mean, Eigen::Vector3d &gyro_mean, Eigen::Vector3d &accel_std)
    {
        int imuIndx;

        {
            // std::lock_guard<std::mutex> lock(mMutexSensordata);

            int size = mImuQueue.size();
            imuIndx = -1;
            for (int i = 0; i < size; i++)
            {
                int indx = size - 1 - i;
                if (mImuQueue.at(indx).t < t)
                {
                    imuIndx = indx;
                    break;
                }
            }

            if (imuIndx < 0)
            {
                std::cout << "error get imu index in CalcImuMeanAndStdInfos [time] : "
                          << t << " " << mImuQueue.front().t << " " << mImuQueue.back().t << std::endl;
                return false;
            }

            if (mImuQueue.at(imuIndx).t - mImuQueue.front().t < smooth_timelen || imuIndx < 25)
            {
                return false;
            }

            // 计算加速度/角速度的均值
            Eigen::Vector3d accel_sum = Eigen::Vector3d::Zero();
            Eigen::Vector3d gyro_sum = Eigen::Vector3d::Zero();
            int cnt = 0;
            for (int i = 0; i < size; i++)
            {
                int indx = size - 1 - i;
                if (mImuQueue.at(indx).t < t - smooth_timelen)
                {
                    break;
                }
                accel_sum += mImuQueue.at(indx).a.cast<double>();
                gyro_sum += mImuQueue.at(indx).w.cast<double>();
                cnt++;
            }
            if (cnt < 25)
            {
                return false;
            }
            accel_mean = accel_sum / cnt;
            gyro_mean = gyro_sum / cnt;

            // 计算加速度标准差
            accel_sum = Eigen::Vector3d::Zero();
            cnt = 0;
            for (int i = 0; i < size; i++)
            {
                int indx = size - 1 - i;
                if (mImuQueue.at(indx).t < t - smooth_timelen)
                {
                    break;
                }
                Eigen::Vector3d accel_tmp = mImuQueue.at(indx).a.cast<double>();
                Eigen::Vector3d daccel = accel_tmp - accel_mean;

                accel_sum(0) += daccel(0) * daccel(0);
                accel_sum(1) += daccel(1) * daccel(1);
                accel_sum(2) += daccel(2) * daccel(2);
                cnt++;
            }
            if (cnt < 25)
            {
                return false;
            }
            accel_std(0) = sqrt(accel_sum(0) / (cnt - 1));
            accel_std(1) = sqrt(accel_sum(1) / (cnt - 1));
            accel_std(2) = sqrt(accel_sum(2) / (cnt - 1));
        }

        return true;
    }

    // 计算状态的pitch/roll均值，用0.5秒平滑（取前面0.5秒内的数据计算）
    bool Estimater::CalcMeanStatePitchroll(double t, double smooth_timelen, Eigen::Vector2d &state_pitchroll_mean)
    {
        if (mSimpleStateQueue.size() < 10)
        {
            return false;
        }

        int stateIndx;

        {
            // std::lock_guard<std::mutex> lock(mMutexSensordata);

            int size = mSimpleStateQueue.size();
            stateIndx = -1;
            for (int i = 0; i < size; i++)
            {
                int indx = size - 1 - i;
                if (mSimpleStateQueue.at(indx).t < t)
                {
                    stateIndx = indx;
                    break;
                }
            }

            if (stateIndx < 0)
            {
                std::cout << "error get SimpleStateQueue index in CalcMeanStatePitchroll [time] : "
                          << t << " " << mSimpleStateQueue.front().t << " " << mSimpleStateQueue.back().t << std::endl;
                return false;
            }

            if (mSimpleStateQueue.at(stateIndx).t - mSimpleStateQueue.front().t < smooth_timelen || stateIndx < 25)
            {
                return false;
            }

            Eigen::Vector2d atti_sum = Eigen::Vector2d::Zero();
            int cnt = 0;
            for (int i = 0; i < size; i++)
            {
                int indx = size - 1 - i;
                if (mSimpleStateQueue.at(indx).t < t - smooth_timelen)
                {
                    break;
                }
                Eigen::Vector3d atti = getAttitudeFromR(mSimpleStateQueue.at(indx).R);
                atti_sum(0) += atti(0);
                atti_sum(1) += atti(1);
                cnt++;
            }
            if (cnt < 25)
            {
                return false;
            }
            state_pitchroll_mean(0) = atti_sum(0) / cnt;
            state_pitchroll_mean(1) = atti_sum(1) / cnt;
        }

        return true;
    }

    bool Estimater::GetEKFState(double t, EKF_WHEELPLUS::State &state)
    {
        std::lock_guard<std::mutex> lock(mMutexSensordata);

        if (!mbInited || mStateQueue.size() == 0)
        {
            return false;
        }

        if (mStateQueue.size() == 1)
        {
            // 只有一个数据，这种情况出现在刚初始化完
            double dt = mStateQueue.back().t - t;
            if (fabs(dt) < 0.01)
            {
                state = mStateQueue.back();
                return true;
            }
            else
            {
                return false;
            }
        }
        else
        {
            // 内插或者外推
            int size = mStateQueue.size();
            int target_indx = -1;
            int indx;
            for (int i = 0; i < size; i++)
            {
                indx = size - 1 - i;
                if (mStateQueue.at(indx).t < t)
                {
                    target_indx = indx;
                    break;
                }
            }

            if (target_indx == -1)
            {
                return false;
            }
            else
            {
                if (target_indx == size - 1)
                {
                    target_indx = size - 2;
                }
                EKF_WHEELPLUS::State stateInterp;
                EKF_WHEELPLUS::State state1 = mStateQueue.at(target_indx);
                EKF_WHEELPLUS::State state2 = mStateQueue.at(target_indx + 1);
                GetInterpState(t, state1, state2, stateInterp);
                state = stateInterp;

                return true;
            }
        }
    }

    bool Estimater::GetLatestEKFState(EKF_WHEELPLUS::State &state)
    {
        std::lock_guard<std::mutex> lock(mMutexSensordata);
        {
            if (!mbInited || mStateQueue.size() == 0)
            {
                return false;
            }

            if(!mbSetLatestState)
            {
                return false;
            }
            state = mLatestState;
        }
        return true;
    }


    bool Estimater::GetRTKErrorState(double t) 
    {
        {
            std::lock_guard<std::mutex> lock(mMutexSensordata);

            double dt = 0;
            if(mbSetLatestState)
            {
                dt = mLatestState.t - mLatestGpsTime;
            }
             
            if(mbLastRtkSlipped || dt > 5.0)
            {
                return true;
            }
            else
            {
                return false;
            }

        }
    }

    double Estimater::GetCurrRtkSlipDuration()
    {
        {
            std::lock_guard<std::mutex> lock(mMutexSensordata);

            double duration = mLastRtkSlipDuration;
            double dt = 0;
            if(mbSetLatestState)
            {
                dt = mLatestState.t - mLatestGpsTime;
            }
            
            if(dt > duration)
            {
                duration =dt;
            }
            
            return duration;
        }

    }


    void Estimater::GetRtkdataInterp(double t, GpsOdometry &rtkdataInterp)
    {
        if (mGpsOdomQueue.size() < 2)
        {
            return;
        }

        int indx = -1;
        int size = mGpsOdomQueue.size();
        for (int i = 0; i < size - 1; i++)
        {
            const GpsOdometry &rtk_0 = mGpsOdomQueue.at(i);
            const GpsOdometry &rtk_1 = mGpsOdomQueue.at(i + 1);
            if (rtk_0.t <= t && t <= rtk_1.t)
            {
                indx = i;
                break;
            }
        }

        if (indx == -1)
        {
            if (t <= mGpsOdomQueue.at(0).t)
            {
                indx = 0;
            }
            else
            {
                indx = size - 2;
            }
        }

        double dT = mGpsOdomQueue.at(indx + 1).t - mGpsOdomQueue.at(indx).t;
        double dx = mGpsOdomQueue.at(indx + 1).pos(0) - mGpsOdomQueue.at(indx).pos(0);
        double dy = mGpsOdomQueue.at(indx + 1).pos(1) - mGpsOdomQueue.at(indx).pos(1);
        double dh = mGpsOdomQueue.at(indx + 1).pos(2) - mGpsOdomQueue.at(indx).pos(2);
        double dt = t - mGpsOdomQueue.at(indx).t;
        rtkdataInterp.t = t;
        rtkdataInterp.pos(0) = mGpsOdomQueue.at(indx).pos(0) + dt / dT * dx;
        rtkdataInterp.pos(1) = mGpsOdomQueue.at(indx).pos(1) + dt / dT * dy;
        rtkdataInterp.pos(2) = mGpsOdomQueue.at(indx).pos(2) + dt / dT * dh;
    }

    void Estimater::getLon0Lat0(double &lon0, double &lat0)
    {
        lon0 = mLon0;
        lat0 = mLat0;
    }

    void Estimater::ResetPreintegInfo(const EKF_WHEELPLUS::State &drState)
    {
        // mPreintegInfo.t = drState.t;
        Eigen::Vector3d atti = getAttitudeFromR(drState.R);
        mPreintegInfo.yaw0 = atti(2);
        mPreintegInfo.cov = Eigen::Matrix3d::Zero();
        mPreintegInfo.dT = 0;
        mPreintegInfo.ve_dt.clear();
        mPreintegInfo.ve_d.clear();
        mPreintegInfo.ve_yaw.clear();
        mPreintegInfo.ve_motionStatus.clear();
    }

    void Estimater::Preintegrate(double dt, double d, double yaw_, EnMotionStatus motionStatus, EKF_WHEELPLUS::PreintegInfo &preintegInfo)
    {
        if (preintegInfo.dT > 0.01 && motionStatus == EnMotionStatus::STOP)
        {
            return;
        }

        double noise_pos = Settings::preintegNoisePos_;
        double noise_yaw = Settings::preintegNoiseYaw_;
        double noise_wheelsf = Settings::preintegNoiseWheelSf_;

        Eigen::Matrix3d cov = preintegInfo.cov;
        double yaw0 = preintegInfo.yaw0;

        double yaw = yaw_ - yaw0;

        Eigen::Matrix3d matA;
        matA.setIdentity();
        matA(1, 0) = -cos(yaw) * d;
        matA(2, 0) = -sin(yaw) * d;

        Eigen::MatrixXd matB(3, 1);
        matB << 0, -sin(yaw) * d, cos(yaw) * d;
        Eigen::Matrix<double, 1, 1> covB;
        covB(0, 0) = noise_wheelsf * noise_wheelsf;

        Eigen::Matrix3d matC = Eigen::Matrix3d::Identity();
        Eigen::Matrix3d covC = Eigen::Matrix3d::Identity();
        covC(0, 0) = noise_yaw * noise_yaw;
        covC(1, 1) = noise_pos * noise_pos;
        covC(2, 2) = noise_pos * noise_pos;

        preintegInfo.cov = matA * cov * matA.transpose() + matB * covB * matB.transpose() + matC * covC * matC.transpose() * dt;

        preintegInfo.dT += dt;
        preintegInfo.ve_dt.push_back(dt);
        preintegInfo.ve_d.push_back(d);
        // 注意：这里要保存DR的原始方向角
        preintegInfo.ve_yaw.push_back(yaw_);
        preintegInfo.ve_motionStatus.push_back(motionStatus);
    }

    /* --------- 用最小二乘计算rtk轨迹与Dr轨迹的变换参数：旋转、平移、尺度 ---------
     * Dr轨迹是指IMU中心（或后轴中心）的轨迹
     * r_ig : 从imu（车辆后轴中心）到gps的杆臂值
     * 输出：旋转、平移、尺度， 精度（中误差）
     */
    bool Estimater::CalcTransformParams_LS(const std::vector<EKF_WHEELPLUS::State, Eigen::aligned_allocator<EKF_WHEELPLUS::State>> &vDrdata,
                                           const std::vector<GpsOdometry> &vRtkdata,
                                           const Eigen::Vector3d &r_ig, Eigen::Matrix3d &R33, Eigen::Vector3d &t31, double &scale, double &mean_error)
    {
        mean_error = 1e10;

        double d_thre = Settings::ekfInitAlignDist() / 3;
        if (d_thre < 2.5)
        {
            d_thre = 2.5;
        }

        int size = vDrdata.size();

        Eigen::Vector3d sum_dr_pos;
        Eigen::Vector3d sum_rtk_pos;
        Eigen::Vector3d mean_dr_pos;
        Eigen::Vector3d mean_rtk_pos;

        sum_dr_pos = Eigen::Vector3d::Zero();
        sum_rtk_pos = Eigen::Vector3d::Zero();
        double dist_dr_sum = 0;
        double dist_rtk_sum = 0;
        int target_indx;
        Eigen::Vector3d dpos;
        double d;
        double d_max = 0;
        const EKF_WHEELPLUS::State &drStateCur = vDrdata[size - 1];
        for (int i = 0; i < size; i++)
        {
            const EKF_WHEELPLUS::State &drdata_tmp = vDrdata[i];
            const GpsOdometry &rtkdata_tmp = vRtkdata[i];
            sum_dr_pos += drdata_tmp.pos;
            sum_rtk_pos += rtkdata_tmp.pos;

            // 计算总的轨迹长度
            if (i > 0)
            {
                dpos = vDrdata[i].pos - vDrdata[i - 1].pos;
                dist_dr_sum += dpos.topRows(2).norm();
                dpos = vRtkdata[i].pos - vRtkdata[i - 1].pos;
                dist_rtk_sum += dpos.topRows(2).norm();
            }

            // 找到轨迹里面距离当前点最远的
            dpos = drStateCur.pos - drdata_tmp.pos;
            d = dpos.topRows(2).norm();
            if (d > d_max)
            {
                d_max = d;
                target_indx = i;
            }
        }
        mean_rtk_pos = sum_rtk_pos / size;
        mean_dr_pos = sum_dr_pos / size;

        double vi_dx = vDrdata[size - 1].pos(0) - vDrdata[target_indx].pos(0);
        double vi_dy = vDrdata[size - 1].pos(1) - vDrdata[target_indx].pos(1);
        double vi_dz = vDrdata[size - 1].pos(2) - vDrdata[target_indx].pos(2);

        // 判断起点距离
        double dist_vi_xy = sqrt(vi_dx * vi_dx + vi_dy * vi_dy);
        if (dist_vi_xy < d_thre)
        {
            return false;
        }

        double theta_vi = atan2(vi_dy, vi_dx);
        // double dist_vi = sqrt(vi_dx * vi_dx + vi_dy * vi_dy);
        double rtk_dx = vRtkdata[size - 1].pos(0) - vRtkdata[target_indx].pos(0);
        double rtk_dy = vRtkdata[size - 1].pos(1) - vRtkdata[target_indx].pos(1);
        double rtk_dz = vRtkdata[size - 1].pos(2) - vRtkdata[target_indx].pos(2);
        double theta_rtk = atan2(rtk_dy, rtk_dx);
        // double dist_rtk = sqrt(rtk_dx * rtk_dx + rtk_dy * rtk_dy);

        /* 求解参数：dx, dy, dtheta, s
         * rtk_x - rtk_x0 = s * ( cos(dtheta) * (vi_x - vi_x0) - sin(dtheta) * (vi_y - vi_y0) ) + dx + r_ig_in_rtk_x
         * rtk_y - rtk_y0 = s * ( sin(dtheta) * (vi_x - vi_x0) + cos(dtheta) * (vi_y - vi_y0) ) + dy + r_ig_in_rtk_y
         * =>
         * rtk_x = s * (cos(dtheta) * vi_x - sin(dtheta) * vi_y) +
         *         s * (-cos(dtheta) * vi_x0 + sin(dtheta) * vi_y0) + rtk_x0 + dx + r_ig_in_rtk_x
         * rtk_y = s * (sin(dtheta) * vi_x + cos(dtheta) * vi_y) +
         *         s * (-sin(dtheta) * vi_x0 - cos(dtheta) * vi_y0 + rtk_y0 + dy + r_ig_in_rtk_y
         *
         * 考虑杆臂补偿：r_ig 表达在IMU坐标系，记为 r_ig_in_b;
         *    先转换到DR的参考坐标系 ： r_bi_in_w = Rwb * r_ig_in_b
         *    再转换到rtk坐标系  r_ig_in_rtk = R(dtheta) * r_ig_in_w
         */
        double vi_x0, vi_y0;
        double rtk_x0, rtk_y0;
        double dx, dy, dtheta, s;
        double r_cg_x, r_cg_y; // IMU到GPS的杆臂值
        double vi_x, vi_y;
        double rtk_x, rtk_y;
        Eigen::Matrix2d R_rtk_w; // IMU参考系到rtk参考系的旋转矩阵
        Eigen::Matrix2d R_w_b;   // IMU坐标系（body系）到IMU参考系的旋转矩阵

        vi_x0 = mean_dr_pos(0);
        vi_y0 = mean_dr_pos(1);
        rtk_x0 = mean_rtk_pos(0);
        rtk_y0 = mean_rtk_pos(1);
        dx = 0;
        dy = 0;
        dtheta = theta_rtk - theta_vi;
        // 尺度初值限制一下范围
        s = dist_rtk_sum / dist_dr_sum;
        if (s > 1.25)
        {
            s = 1.25;
        }
        if (s < 0.8)
        {
            s = 0.8;
        }
        // 杆臂不变
        r_cg_x = r_ig(0);
        r_cg_y = r_ig(1);

        Eigen::MatrixXd matA(2 * size, 4);
        Eigen::VectorXd matl(2 * size, 1);
        Eigen::MatrixXd matP(2 * size, 2 * size);
        Eigen::VectorXd matx;
        int iter_cnt = 0;
        while (1)
        {
            matA.setZero();
            matl.setZero();
            matP.setZero();

            R_rtk_w(0, 0) = cos(dtheta);
            R_rtk_w(0, 1) = -sin(dtheta);
            R_rtk_w(1, 0) = sin(dtheta);
            R_rtk_w(1, 1) = cos(dtheta);

            // 构建观测方程
            double l1, l2;
            for (int i = 0; i < size; i++)
            {
                vi_x = vDrdata[i].pos(0);
                vi_y = vDrdata[i].pos(1);
                rtk_x = vRtkdata[i].pos(0);
                rtk_y = vRtkdata[i].pos(1);

                Eigen::Vector3d atti = getAttitudeFromR(vDrdata[i].R);
                double yaw_tmp = atti(2);
                R_w_b(0, 0) = cos(yaw_tmp);
                R_w_b(0, 1) = -sin(yaw_tmp);
                R_w_b(1, 0) = sin(yaw_tmp);
                R_w_b(1, 1) = cos(yaw_tmp);

                Eigen::Matrix2d R_rtk_b = R_rtk_w * R_w_b;
                Eigen::Vector2d r_cg_in_rtk = R_rtk_b * Eigen::Vector2d(r_cg_x, r_cg_y);

                l1 = (rtk_x - rtk_x0) - (s * (cos(dtheta) * (vi_x - vi_x0) - sin(dtheta) * (vi_y - vi_y0)) + dx + r_cg_in_rtk(0));
                l2 = (rtk_y - rtk_y0) - (s * (sin(dtheta) * (vi_x - vi_x0) + cos(dtheta) * (vi_y - vi_y0)) + dy + r_cg_in_rtk(1));

                matl(2 * i) = l1;
                matl(2 * i + 1) = l2;

                matA(2 * i, 0) = 1;
                matA(2 * i, 1) = 0;
                matA(2 * i, 2) = s * (-sin(dtheta) * (vi_x - vi_x0) - cos(dtheta) * (vi_y - vi_y0));
                matA(2 * i, 3) = (cos(dtheta) * (vi_x - vi_x0) - sin(dtheta) * (vi_y - vi_y0));
                matA(2 * i + 1, 0) = 0;
                matA(2 * i + 1, 1) = 1;
                matA(2 * i + 1, 2) = s * (cos(dtheta) * (vi_x - vi_x0) - sin(dtheta) * (vi_y - vi_y0));
                matA(2 * i + 1, 3) = (sin(dtheta) * (vi_x - vi_x0) + cos(dtheta) * (vi_y - vi_y0));

                // 根据残差调整权重
                double weight;
                if (iter_cnt < 2)
                {
                    weight = 1.0;
                }
                else
                {
                    double residual = sqrt(l1 * l1 + l2 * l2);
                    if (residual <= 0.5)
                    {
                        weight = 1.0;
                    }
                    else
                    {
                        weight = 1.0 * (0.5 / residual);
                    }
                }
                matP(2 * i, 2 * i) = weight;
                matP(2 * i + 1, 2 * i + 1) = weight;
            }

            // matx = (matA.transpose() * matP * matA).inverse() * matA.transpose() * matP * matl;
            matx = (matA.transpose() * matP * matA).ldlt().solve(matA.transpose() * matP * matl);

            dx += matx(0);
            dy += matx(1);
            dtheta += matx(2);
            s += matx(3);

            // if(fabs(matx(0)) < 0.01 && fabs(matx(1)) < 0.01 && fabs(matx(2)) < 0.03 / 57.3 && fabs(matx(3)) < 0.001 &&
            //     fabs(matx(4)) < 0.01 && fabs(matx(5)) < 0.01)
            // {
            //     break;
            // }

            iter_cnt++;
            if (iter_cnt >= 5)
            {
                mean_error = sqrt((matl.transpose() * matP * matl)(0, 0) / (2 * size - 4));
                break;
            }
        }

        //
        Eigen::Vector3d atti;
        atti(0) = 0;
        atti(1) = 0;
        atti(2) = dtheta;
        R33 = Attitude2R(atti);

        scale = s;

        t31(0) = s * (-cos(dtheta) * vi_x0 + sin(dtheta) * vi_y0) + rtk_x0 + dx;
        t31(1) = s * (-sin(dtheta) * vi_x0 - cos(dtheta) * vi_y0) + rtk_y0 + dy;
        t31(2) = mean_rtk_pos(2) - (mean_dr_pos(2) + r_ig(2));

        return true;
    }

    /*
     * 优化
     * pMarginInfo：对第一帧的先验约束，NULL表示没有
     *   为了快速使用Gps，用较短的轨迹（10m）做Gps与Dr对齐，对齐之后就有了第一帧的先验约束
     * bSlidewindow：是否滑窗（边缘化）
     *   为了提高精度，窗口内需要保存更长的轨迹（20m），因此在轨迹长度达到20m之前，只做优化，不滑窗
     *   轨迹长度超过20m，就做滑窗（边缘化）
     */
    void Estimater::Optimize(std::vector<EKF_WHEELPLUS::State, Eigen::aligned_allocator<EKF_WHEELPLUS::State>> &veState,
                             EKF_WHEELPLUS::MarginInfo *pMarginInfo, bool bSlidewindow, double &chi2, double &robust_chi2)
    {
        static int optimize_cnt = 0;
        if(mbGpsAigned)
        {
            optimize_cnt++;
        }
        
        g2o::SparseOptimizer optimizer;
        g2o::BlockSolverX::LinearSolverType *linearSolver;

        linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();

        g2o::BlockSolverX *solver_ptr = new g2o::BlockSolverX(linearSolver);

        g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
        // solver->setUserLambdaInit(1e3);

        optimizer.setAlgorithm(solver);

        // 添加顶点(RTK Pose 是顶点)
        for (size_t i = 0; i < veState.size(); i++)
        {
            Eigen::Vector2d pos_xy = veState[i].gpsPos;
            // // 位置从后轴中心转换到rtk天线
            // pos_xy -= veState[i].gpsR * mPgv.topRows(2);

            double yaw = getYawFromR(veState[i].gpsR); // yaw角是否也要转一下
            ImuCamPose pose(pos_xy, yaw);
            VertexPose *VP = new VertexPose(pose);
            VP->setId(i);
            VP->setFixed(false);
            optimizer.addVertex(VP);
        }

        // 添加边
        // 位姿变化约束 信息矩阵 yaw x y的 权重

        EdgeRelativePose *edgeRelativePose_1 = NULL;
        for (size_t i = 1; i < veState.size(); i++)
        {
            Eigen::Matrix3d info;


            Eigen::Vector3d atti1 = getAttitudeFromR(veState[i - 1].R);
            Eigen::Matrix3d Rwb1 = Attitude2R(Eigen::Vector3d(0, 0, atti1(2)));
            Eigen::Vector3d twb1 = veState[i - 1].pos;
            // // 位置从后轴中心转换到rtk天线
            // twb1 -= Rwb1 * mPgv;

            Eigen::Vector3d atti2 = getAttitudeFromR(veState[i].R);
            Eigen::Matrix3d Rwb2 = Attitude2R(Eigen::Vector3d(0, 0, atti2(2)));
            Eigen::Vector3d twb2 = veState[i].pos;
            // // 位置从后轴中心转换到rtk天线
            // twb2 -= Rwb2 * mPgv;

            Eigen::Matrix3d Rb1b2 = Rwb1.transpose() * Rwb2; // 获取相对变换
            Eigen::Vector3d tb1b2 = Rwb1.transpose() * (twb2 - twb1);

            // 边 相对位姿约束
            g2o::HyperGraph::Vertex *VP1 = optimizer.vertex(i - 1);
            g2o::HyperGraph::Vertex *VP2 = optimizer.vertex(i);

            Eigen::Matrix2d Rb1b2_2d;
            Rb1b2_2d(0, 0) = Rb1b2(0, 0); // cost heta
            Rb1b2_2d(0, 1) = Rb1b2(0, 1); // -sin theta
            Rb1b2_2d(1, 0) = Rb1b2(1, 0);
            Rb1b2_2d(1, 1) = Rb1b2(1, 1);

            double yaw_rad = std::atan2(Rb1b2_2d(1, 0), Rb1b2_2d(0, 0));
            // if(std::abs(yaw_rad) < 0.02) 
            // {
            //     yaw_rad = 0.02;
            // }   
            Eigen::Matrix3d InfoRelativePoseDefault;
            InfoRelativePoseDefault.setIdentity();

            // InfoRelativePoseDefault(0, 0) = 1.0 / (yaw_rad * 0.2 * yaw_rad * 0.2); // 0.1 -> 1
            // // 检测到打滑将位置噪声放大
            // double pose_rate = 1;
            // if(split_state == 2 || split_state == 4) {
            //     std::cout << " up mover " << std::endl;
            //     pose_rate = 10;
            // }

            // InfoRelativePoseDefault(1, 1) = 1.0 / (pose_rate * 0.03 * pose_rate * 0.03);   // 5
            // InfoRelativePoseDefault(2, 2) = 1.0 / (pose_rate * 0.03 * pose_rate * 0.03);            
            // if (veState[i].bPreintegred)
            // {
            //     // info = veState[i].preintegInfo.cov.inverse();
            //     info = InfoRelativePoseDefault;
            // }
            // else
            // {
            //     std::cout << "error, not preintegrated gps [time] : " << veState[i].t << std::endl;
            //     info = InfoRelativePoseDefault;
            // }

            if(std::abs(yaw_rad) < 5 * M_PI/180) 
            {
                InfoRelativePoseDefault(0, 0) = 1.0 / (0.15 /57.3 * 0.15 /57.3);
            }   
            else
            {
                double ratio = std::abs(yaw_rad) / (5 * M_PI/180);
                if(ratio > 3)
                {
                    ratio = 3;
                }
                InfoRelativePoseDefault(0, 0) = 1.0 / (0.15 /57.3 * 0.15 /57.3 * ratio * ratio);
            }
            
            InfoRelativePoseDefault(1, 1) = 1.0 / (0.05 * 0.05);   // 5
            InfoRelativePoseDefault(2, 2) = 1.0 / (0.05 * 0.05); 

            info = InfoRelativePoseDefault;

            Eigen::Vector2d tb1b2_2d;
            tb1b2_2d(0) = tb1b2(0);
            tb1b2_2d(1) = tb1b2(1);

            EdgeRelativePose *edgeRelativePose = new EdgeRelativePose(Rb1b2_2d, tb1b2_2d); // b2 到 b1的 旋转和平移
            edgeRelativePose->setVertex(0, VP1);
            edgeRelativePose->setVertex(1, VP2);

            edgeRelativePose->setInformation(info);
            optimizer.addEdge(edgeRelativePose);

            if (i == 1)
            {
                edgeRelativePose_1 = edgeRelativePose;
            }
        }

        // 先验位姿(边缘化？？)
        // 边缘化的先验位姿是个顶点吗
        EdgePriorStatePose *edgePriorStatePose = NULL;
        if (pMarginInfo)
        {
            double yaw = pMarginInfo->state(0);
            Eigen::Matrix2d Rwb;
            Rwb << cos(yaw), -sin(yaw), sin(yaw), cos(yaw);
            Eigen::Vector2d twb;
            twb << pMarginInfo->state(1), pMarginInfo->state(2);

            // // 把位置从后轴中心转换到rtk天线
            // twb -= Rwb * mPgv.topRows(2);

            edgePriorStatePose = new EdgePriorStatePose(Rwb, twb);
            g2o::HyperGraph::Vertex *VP1 = optimizer.vertex(0);
            edgePriorStatePose->setVertex(0, VP1);
            edgePriorStatePose->setInformation(pMarginInfo->info * 1); // * 0.1可調
            optimizer.addEdge(edgePriorStatePose);
        }

        // 以GPS位置融合或者速度融合
        double edgeGps_rk_dalta = 1;
        Eigen::Matrix2d InfoGps;
        InfoGps.setIdentity();
        InfoGps.block<2, 2>(0, 0) = 1.0 / (0.05 * 0.05) * Eigen::Matrix2d::Identity();

        double edgeGpsvel_rk_dalta = 1;
        Eigen::Matrix2d InfoGpsVel;
        InfoGpsVel.setIdentity();
        InfoGpsVel.block<2, 2>(0, 0) = 1.0 / (0.03 * 0.03) * Eigen::Matrix2d::Identity();

        int gps_pos_obs_cnt = 0;
        int gps_vel_obs_cnt = 0;
        for (int i = 0; i < veState.size(); i++)
        {
            if (!veState[i].gpsOdom)
            {
                continue;
            }

            double state_gpstime_0, state_gpstime_1;
            if(i == 0)
            {
                state_gpstime_0 = mLastState0Time;
            }
            else
            {
                state_gpstime_0 = veState[i-1].t;
            }
            if(i == veState.size()-1)
            {
                state_gpstime_1 = veState[i].t + 1e10;
            }
            else
            {
                state_gpstime_1 = veState[i+1].t;
            }

            bool gps_pos_ok = true;
            for(int i=0; i<mRtkSlipInfos.size(); i++)
            {
                double t0 = mRtkSlipInfos.at(i).t0;
                double t1 = mRtkSlipInfos.at(i).t1;

                if(state_gpstime_1 >= t0 && state_gpstime_0 <= t1)
                {
                    gps_pos_ok = false;
                    break;
                }
            }

            if(gps_pos_ok)
            {
                // 先验是约束第一帧的，且边缘化的时候已经考虑了Gps，不能重复使用
                if (pMarginInfo && i == 0)
                {
                    continue;
                }

                gps_pos_obs_cnt++;

                Eigen::Vector2d pos_xy = veState[i].gpsOdom->pos.topRows(2);
                EdgeGpsPositionXY *edgeGpsPosition = new EdgeGpsPositionXY(pos_xy);
                g2o::HyperGraph::Vertex *VP1 = optimizer.vertex(i);
                edgeGpsPosition->setVertex(0, VP1);

                if(veState[i].gpsOdom->fs == 4)
                {
                    edgeGpsPosition->setInformation(InfoGps);
                }
                else
                {
                    edgeGpsPosition->setInformation(InfoGps * 0.1);
                }
                
                // 核函数
                g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                edgeGpsPosition->setRobustKernel(rk);
                rk->setDelta(edgeGps_rk_dalta);

                optimizer.addEdge(edgeGpsPosition);
            }
            else
            {
                if(i == veState.size()-1)
                {
                    continue;
                }

                gps_vel_obs_cnt++;

                Eigen::Vector2d gps_vel = veState[i].gpsOdom->vel.topRows(2);
                double dt = veState[i+1].t - veState[i].t;
                EdgeGpsVelocity *edgeGpsVelocity = new EdgeGpsVelocity(gps_vel, dt);
                g2o::HyperGraph::Vertex *VP1 = optimizer.vertex(i);
                g2o::HyperGraph::Vertex *VP2 = optimizer.vertex(i+1);
                edgeGpsVelocity->setVertex(0, VP1);
                edgeGpsVelocity->setVertex(1, VP2);
                edgeGpsVelocity->setInformation(InfoGpsVel);
                // 核函数
                g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                edgeGpsVelocity->setRobustKernel(rk);
                rk->setDelta(edgeGpsvel_rk_dalta);

                optimizer.addEdge(edgeGpsVelocity);
            }

        }

        std::cout << "optimize, [optimize_cnt, time, state_cnt, gps_pos_obs_cnt, gps_vel_obs_cnt] : " 
            << optimize_cnt << ", " << veState[veState.size() - 1].t << ", " << veState.size() << ", "
            << gps_pos_obs_cnt << ", " << gps_vel_obs_cnt << std::endl;

        std::string s;
        for(int i=0; i<mRtkSlipInfos.size(); i++)
        {
            double t0 = mRtkSlipInfos.at(i).t0;
            double t1 = mRtkSlipInfos.at(i).t1;

            std::string s_t0 = std::to_string(t0);
            int pos = s_t0.find(".");
            if(pos >= 0 && pos < s_t0.length() - 4)
            {
                s += s_t0.substr(0, pos+4);
            }
            else
            {
                s += s_t0;
            }
            s +=  ",";
            std::string s_t1 = std::to_string(t1);
            pos = s_t1.find(".");
            if(pos >= 0 && pos < s_t1.length() - 4)
            {
                s += s_t1.substr(0, pos+4);
            }
            else
            {
                s += s_t1;
            }
            s += "|";
        }
        std::cout << "mRtkSlipInfos : " << s << std::endl;
        std::cout << "veState time range : " << veState[0].t << ", " << veState[veState.size() - 1].t << std::endl;


        int opt_it;
        if (pMarginInfo)
        {
            opt_it = 10;
        }
        else
        {
            opt_it = 20;
        }
        // 初始化优化 → 计算初始误差 → 执行非线性优化 → 计算优化后的误差
        optimizer.initializeOptimization();
        optimizer.computeActiveErrors();
        double err = optimizer.activeRobustChi2();
        optimizer.optimize(opt_it);
        double err_end = optimizer.activeRobustChi2();

        // 计算优化后的残差
        chi2 = optimizer.activeChi2();
        robust_chi2 = optimizer.activeRobustChi2();

        // 获取优化结果
        VertexPose *VP = static_cast<VertexPose *>(optimizer.vertex(0));
        Eigen::Vector2d gpstwb_before = veState[0].gpsPos;
        Eigen::Vector2d gpstwb_after = VP->estimate().twb;
        // // 位置从rtk天线转回到后轴中心
        // gpstwb_after += veState[0].gpsR * mPgv.topRows(2);

        // gpstwb_after - gpstwb_before
        Eigen::Vector2d gpstwb_drift = gpstwb_after - gpstwb_before;
        double yaw_before = getYawFromR(veState[0].gpsR);
        double yaw_after = getYawFromR(VP->estimate().Rwb);
        // yaw_before - yaw_after
        double yaw_drift = yaw_before - yaw_after;
        Eigen::Matrix2d Rwb_drift = Yaw2R(yaw_drift);

        if (!pMarginInfo)
        {
            gpstwb_drift = Eigen::Vector2d::Zero();
            Rwb_drift = Eigen::Matrix2d::Identity();
        }

        gpstwb_drift = Eigen::Vector2d::Zero();
        Rwb_drift = Eigen::Matrix2d::Identity();
        gpstwb_before = gpstwb_after;

        VertexPose *VP0 = static_cast<VertexPose *>(optimizer.vertex(0));
        for (size_t i = 0; i < veState.size(); i++)
        {
            VertexPose *VP = static_cast<VertexPose *>(optimizer.vertex(i));
            veState[i].gpsR = Rwb_drift * VP->estimate().Rwb;
            veState[i].gpsPos = Rwb_drift * (VP->estimate().twb - VP0->estimate().twb) + gpstwb_before;
            // // 位置从rtk天线转回到后轴中心
            // veState[i].gpsPos += veState[i].gpsR * mPgv.topRows(2);
        }

        // 滑窗（边缘化）
        if (bSlidewindow)
        {
            double yaw = getYawFromR(veState[1].gpsR);
            pMarginInfo->state << yaw, veState[1].gpsPos(0), veState[1].gpsPos(1);
            
            if(gps_pos_obs_cnt > gps_vel_obs_cnt)
            {
                pMarginInfo->info.setIdentity();
                pMarginInfo->info(0, 0) = 1.0 / (1.0 / 57.3 * 1.0 / 57.3); // 0.3 -> 3
                pMarginInfo->info(1, 1) = 1.0 / (0.3 * 0.3);
                pMarginInfo->info(2, 2) = 1.0 / (0.3 * 0.3);
            }
            else
            {
                pMarginInfo->info.setIdentity();
                pMarginInfo->info(0, 0) = 1.0 / (0.5 / 57.3 * 0.5 / 57.3); // 0.3 -> 3
                pMarginInfo->info(1, 1) = 1.0 / (0.15 * 0.15);
                pMarginInfo->info(2, 2) = 1.0 / (0.15 * 0.15);
            }

        }
    }

    bool Estimater::GetAlignData(ViewData &aligndata)
    {
        std::lock_guard<std::mutex> lock(align_mutex);
        if (!mbInited || AlignData.size() == 0)
        {
            return false;
        }
        aligndata = AlignData.back();
        return true;
    }

    bool Estimater::GetLatestRtkState(EKF_WHEELPLUS::rtkState &state)
    {
        std::lock_guard<std::mutex> lock(mMutexSensordata);
        if (mRtkStateQueue.size() == 0)
        {
            return false;
        }
        state = mRtkStateQueue.back();
        while (!mRtkStateQueue.empty())
        {
            mRtkStateQueue.pop_front();
        }
        return true;
    }

    bool Estimater::GetRef(double &ref_e, double &ref_n)
    {
        if (have_Ref)
        {
            ref_e = mRefE_ob;
            ref_n = mRefN_ob;
            return true;
        }

        return false;
    }

    void Estimater::ChangeSplitState(int state) 
    {
        split_val.store(state);
    }

    /* 
    * 把最新的gpsodom添加到mSimpleGpsQueue
    * 并判断是否跳变
    */
    void Estimater::CheckRtkSlip()
    {
        if(mGpsOdomQueue.empty() || !mbGpsAigned)
        {
            return;
        }

        // 先把最新的gpsodom添加到mSimpleGpsQueue
        GpsOdometry &lastGpsOdom = mGpsOdomQueue.back();
        SimpleGpsData simpleGps;
        simpleGps.t = lastGpsOdom.t;
        simpleGps.fs = lastGpsOdom.fs;
        simpleGps.x = lastGpsOdom.pos.x();
        simpleGps.y = lastGpsOdom.pos.y();
        simpleGps.speed = lastGpsOdom.vel.norm();
        if(mSimpleGpsQueue.size() == 0)
        {
            simpleGps.trajLen = 0;
        }
        else
        {
            SimpleGpsData &lastSimpleGps = mSimpleGpsQueue.back();
            double dx = simpleGps.x - lastSimpleGps.x;
            double dy = simpleGps.y - lastSimpleGps.y;
            double d = sqrt(dx*dx + dy*dy);
            simpleGps.trajLen = lastSimpleGps.trajLen + d;
        }
        mSimpleGpsQueue.push_back(simpleGps);

        // 判断是否跳变
        int size = mSimpleGpsQueue.size();
        if(size < 2)
        {
            return;
        }

        // 先把轮速取好，计算两个rtk数据之间的距离，统计跳变过程中的轨迹长度
        double vel;
        int wheelIndx = getWheelodometerIndx(mSimpleGpsQueue.back().t, 1e15);
        if (wheelIndx < 0)
        {
            // 用个默认值，粗略计算即可
            vel = 0.2;
        }
        else
        {
            vel = mWheelOdometerQueue.at(wheelIndx).vel;
        }

        // 再算角速度，取最近0.1秒内的imu数据
        Eigen::Vector3d angular_vel;
        if(mImuQueue.size() < 100 || mStateQueue.empty())
        {
            angular_vel = Eigen::Vector3d::Zero();
        }
        else
        {
            Eigen::Vector3d angular_vel_sum = Eigen::Vector3d::Zero();
            int size = mImuQueue.size();
            for(int i=0; i<10; i++)
            {
                int indx = size - 1 - i;
                angular_vel_sum += mImuQueue.at(indx).w.cast<double>();
            }
            angular_vel = angular_vel_sum / 10;
            angular_vel -= mStateQueue.back().bg;
        }
        
        // 如果上一个状态为跳变，则当前状态必然为跳变 (只有通过其它方式判断恢复正常)
        if(mbLastRtkSlipped)
        {
            RtkSlipInfo &rtkSlipInfo = mRtkSlipInfos.back();
            double dt = mSimpleGpsQueue.back().t - rtkSlipInfo.t1;
            rtkSlipInfo.t1 = mSimpleGpsQueue.back().t;

            Eigen::Vector3d vel_3d_tmp(0, vel, 0);
            Eigen::Vector3d vel_3d = vel_3d_tmp + EKF_WHEELPLUS::getSkewMatrix(angular_vel) * (-mPgv);
            rtkSlipInfo.trajLen += vel_3d.topRows(2).norm() * dt;

            // Logger("rtkslipinfo.txt") << "slipped for mbLastRtkSlipped, time : " <<  mSimpleGpsQueue.at(size-1).t << std::endl;

            return;
        }
        else
        {
            bool slipped = false;

            double dt = mSimpleGpsQueue.at(size-1).t - mSimpleGpsQueue.at(size-2).t;
            if(dt > 0.6)
            {
                // Logger("rtkslipinfo.txt") << "slipped for dt [time, dt] : " <<  mSimpleGpsQueue.at(size-1).t 
                //     << ", " << dt << std::endl;

                slipped = true;
            }
            else 
            {
                double vel_0 = mSimpleGpsQueue.at(size-2).speed;
                double vel_1 = mSimpleGpsQueue.at(size-1).speed;
                double vel_mean = (vel_0 + vel_1) / 2;
                if(vel_mean > 1.0)
                {
                    vel_mean = 1.0;
                }
                double dx = mSimpleGpsQueue.at(size-1).x - mSimpleGpsQueue.at(size-2).x;
                double dy = mSimpleGpsQueue.at(size-1).y - mSimpleGpsQueue.at(size-2).y;
                double d = sqrt(dx*dx + dy*dy);
                double s = vel_mean * dt;
                if(d > s * 1.25 + 1.0)
                {
                    // Logger("rtkslipinfo.txt") << "slipped for d,s [time, d, s] : " <<  mSimpleGpsQueue.at(size-1).t 
                    //     << ", " << s << ", " << d << std::endl;

                    slipped = true;
                }
                else
                {
                    slipped = false;
                }
            }

            // 出现了新的跳变信息
            if(slipped)
            {
                RtkSlipInfo rtkSlipInfo;
                rtkSlipInfo.t0 = mSimpleGpsQueue.at(size-2).t;
                rtkSlipInfo.t1 = mSimpleGpsQueue.at(size-1).t;
                double dt = rtkSlipInfo.t1 - rtkSlipInfo.t0;
                // rtkSlipInfo.trajLen += fabs(vel) * dt;
                Eigen::Vector3d vel_3d_tmp(0, vel, 0);
                Eigen::Vector3d vel_3d = vel_3d_tmp + EKF_WHEELPLUS::getSkewMatrix(angular_vel) * (-mPgv);
                rtkSlipInfo.trajLen += vel_3d.topRows(2).norm() * dt;

                mRtkSlipInfos.push_back(rtkSlipInfo);

                mbLastRtkSlipped = true;
            }
            else
            {
                // Logger("rtkslipinfo.txt") << "good, no slip for time : " <<  mSimpleGpsQueue.at(size-1).t << std::endl;
            }

        }
                
    }

    // 判断rtk是否恢复正常（正在经历跳变）
    void Estimater::CheckRtkRecorved()
    {
        if(!mbLastRtkSlipped)
        {
            return;
        }
        
        RtkSlipInfo &rtkSlipInfo = mRtkSlipInfos.back();
        double slip_timelen = rtkSlipInfo.t1 - rtkSlipInfo.t0;

        // 先考虑rtk跳变持续的时长大于阈值（5分钟）的情况，是否还考虑持续的轨迹长度？
        double max_time_len = 5 * 60;
        if(slip_timelen > max_time_len && rtkSlipInfo.trajLen > max_time_len * 0.125)
        {
            double traj_len_thre = 5.0;
            double dT_min_thre = 10.0;
            if(CheckRtkRecorvedSelf(traj_len_thre, dT_min_thre))
            {
                mbLastRtkSlipped = false;
            }

            return;
        }

        // 判断递推的位置与rtk位置的差值
        // 先找到最新的关联了gps数据的state
        bool find = false;
        for(int i=mStateQueue.size()-1; i>=0; i--)
        {
            double t = mStateQueue.at(i).t;
            if(t < mSimpleGpsQueue.back().t - 5.0)
            {
                //Logger("rtkslipinfo.txt") << "CheckRtkRecorved failed 10 for t : " 
                    // << t << ", " << mSimpleGpsQueue.back().t  << ", " 
                    // << mSimpleGpsQueue.back().t - t << std::endl;

                return;
            }

            if(mStateQueue.at(i).bGpsProcessed && mStateQueue.at(i).gpsOdom != nullptr)
            {
                Eigen::Vector2d dr_gpspos = mStateQueue.at(i).gpsPos - mStateQueue.at(i).gpsR * mPgv.topRows(2);
                Eigen::Vector2d dpos = dr_gpspos - mStateQueue.at(i).gpsOdom->pos.topRows(2);

                double d_thre = 0.5 + rtkSlipInfo.trajLen * 0.05;
                if(dpos.norm() < d_thre)
                {
                    double traj_len_thre = rtkSlipInfo.trajLen * 0.1;
                    if(traj_len_thre < 0.5)
                    {
                        traj_len_thre = 0.5;
                    }
                    if(traj_len_thre > 3.0)
                    {
                        traj_len_thre = 3.0;
                    }
                    double dT_min_thre = 10 * traj_len_thre/5.0;
                    
                    if(CheckRtkRecorvedSelf(traj_len_thre, dT_min_thre))
                    {
                        mbLastRtkSlipped = false;
                    }
                }
                else
                {
                    //Logger("rtkslipinfo.txt") << "CheckRtkRecorved failed 11 for dpos.norm : " 
                        // << dpos.norm() << ", " << d_thre << std::endl;
                }

                find = true;
                return;
            }
        }

    }

    /* 
    * 从rtk数据本身判断是否恢复了，即Rtk位置是否能从跳变状态恢复到正常状态
    * 常规判断方法超过一定时间（15分钟）没有成功判断恢复的情况下才调用这个函数
    */
    bool Estimater::CheckRtkRecorvedSelf(double traj_len_thre, double dT_min_thre)
    {
        double vel_thre = 0.125;
        double max_dT = traj_len_thre / vel_thre;
        
        // 1. 检查数据个数、轨迹长度
        int size = mSimpleGpsQueue.size();
        if(size < 100)
        {
            //Logger("rtkslipinfo.txt") << "CheckRtkRecorvedSelf failed 1 for size : " << size << std::endl;

            return false;
        }

        double traj_len = mSimpleGpsQueue.back().trajLen - mSimpleGpsQueue.front().trajLen;
        if(traj_len <= traj_len_thre)
        {
            //Logger("rtkslipinfo.txt") << "CheckRtkRecorvedSelf failed 2 for traj_len : " << traj_len << std::endl;

            return false;
        }

        // 2. 找到距离最后一个gps的轨迹长度满足要求的gps数据下标
        int start_indx = -1;
        for(int i=0; i<size; i++)
        {
            int indx = size-1 - i;
            traj_len = mSimpleGpsQueue.back().trajLen - mSimpleGpsQueue.at(indx).trajLen;
            double dt = mSimpleGpsQueue.back().t - mSimpleGpsQueue.at(indx).t;
            if(traj_len >= traj_len_thre && dt > dT_min_thre)
            {
                start_indx = indx;
                break;
            }

            // 如果时间长度超过max_dT（说明速度太小），则提前跳出循环
            if(dt > max_dT)
            {
                break;
            }
        }

        if(start_indx < 0 || size-start_indx < 10)
        {
            //Logger("rtkslipinfo.txt") << "CheckRtkRecorvedSelf failed 3 for size-start_indx : " 
                // << size-start_indx << std::endl;

            return false;
        }

        // 3. 检查轨迹长度和由轨迹长度计算的速度（均值）
        traj_len = mSimpleGpsQueue.back().trajLen - mSimpleGpsQueue.at(start_indx).trajLen;
        double dt = mSimpleGpsQueue.back().t - mSimpleGpsQueue.at(start_indx).t;
        double vel = traj_len / dt;
        if(traj_len < traj_len_thre || vel < vel_thre)
        {
            //Logger("rtkslipinfo.txt") << "CheckRtkRecorvedSelf failed 4 for traj_len or vel : " 
                // << traj_len << ", " << vel << std::endl;

            return false;
        }

        // 检查每个数据是否为固定解，且前后没有出现跳变
        double vel_sum = 0;
        for(int i=start_indx; i<size; i++)
        {
            if(mSimpleGpsQueue.at(i).fs != 4)
            {
                //Logger("rtkslipinfo.txt") << "CheckRtkRecorvedSelf failed 5 for loc_type : " 
                    // << mSimpleGpsQueue.at(i).fs << std::endl;

                return false;
            }

            // 判断前后是否跳变
            if(i > start_indx)
            {
                double dt = mSimpleGpsQueue.at(i).t - mSimpleGpsQueue.at(i-1).t;
                if(dt > 0.6)
                {
                    //Logger("rtkslipinfo.txt") << "CheckRtkRecorvedSelf failed 6 for dt : " 
                        // << dt << std::endl;

                    return false;
                }

                double vel_0 = mSimpleGpsQueue.at(i-1).speed;
                double vel_1 = mSimpleGpsQueue.at(i).speed;

                if(fabs(vel_0) > 0.75 || fabs(vel_1) > 0.75)
                {
                    //Logger("rtkslipinfo.txt") << "CheckRtkRecorvedSelf failed 7 for vel_0/vel_1 : " 
                        // << vel_0 << ", " << vel_1 << std::endl;

                    return false;
                }

                double dx = mSimpleGpsQueue.at(i).x - mSimpleGpsQueue.at(i-1).x;
                double dy = mSimpleGpsQueue.at(i).y - mSimpleGpsQueue.at(i-1).y;
                double d = sqrt(dx*dx + dy*dy);
                
                double s = (fabs(vel_0) + fabs(vel_1)) / 2 * dt;
                if(d > s * 1.25 + 0.3)
                {
                    //Logger("rtkslipinfo.txt") << "CheckRtkRecorvedSelf failed 8 for s, d : " 
                        // << s << ", " << d << std::endl;

                    return false;
                }
            }

            vel_sum += fabs(mSimpleGpsQueue.at(i).speed);
        }

        // 检查平均运动速度
        vel = vel_sum / (size - start_indx);
        if(vel < vel_thre) 
        {
            //Logger("rtkslipinfo.txt") << "CheckRtkRecorvedSelf failed 9 for vel_mean : " 
                // << vel << std::endl;

            return false;
        }

        //Logger("rtkslipinfo.txt") << "CheckRtkRecorvedSelf ok " << std::endl;

        // 到这里说明恢复正常了
        return true;
    }


    /* 
    * ----------------- 处理打滑相关的函数 -----------------
    */
   
    // 如果检测到打滑，数据回退到打滑前time_len重新计算
    bool Estimater::OnSlideDetected(double slide_t, double time_len)
    {
        // 1. 如果rtk/dr还没对齐，不做处理
        if(!mbDrInited || !mbGpsAigned)
        {
            return false;
        }

        /* ------------- 2. 获取rtk速度 -------------
        */
        // 2.1 rtk数据个数
        if(mGpsOdomQueue.size() < 2)
        {
            return false;
        }
         
        // 2.2 先找到时间t的前后的两个rtk数据
        int rtk_indx0 = -1;
        int rtk_indx1 = -1;
        int rtk_size = mGpsOdomQueue.size();
        for(int i=0; i<rtk_size; i++)
        {
            if(mGpsOdomQueue.at(i).t >= slide_t - time_len/2)
            {
                rtk_indx1 = i;
                break;
            }
        }
        for(int i=0; i<rtk_size; i++)
        {
            int indx = rtk_size-1 - i;
            if(mGpsOdomQueue.at(indx).t <= slide_t - time_len/2)
            {
                rtk_indx0 = indx;
                break;
            }
        }

        if(rtk_indx0 < 0 || rtk_indx1 < 0)
        {
            return false;
        }

        // 2.3 计算前后两个rtk数据的时间差，判断时间差是否合理
        double dt = mGpsOdomQueue.at(rtk_indx1).t - mGpsOdomQueue.at(rtk_indx0).t;
        if(dt > 1.5 || dt < 1e-3)
        {
            return false;
        }

        // 2.4 计算rtk速度，判断速度是否合理
        Eigen::Vector3d vel_3d = (mGpsOdomQueue.at(rtk_indx1).vel + mGpsOdomQueue.at(rtk_indx0).vel) / 2;
        double rtk_vel_x = vel_3d(0);
        double rtk_vel_y = vel_3d(1);
        double rtk_vel = sqrt(rtk_vel_x*rtk_vel_x + rtk_vel_y*rtk_vel_y);
        if(rtk_vel > 1.0)
        {
            return false;
        }

        /* -------------- 3. 计算该回退到哪，并判断相关的传感器数据是否可用 ---------------
        */
        double target_t = slide_t - time_len;

        // 如果要回退的时间长度太长，则不做处理
        if(mStateQueue.back().t - target_t > 6.0)
        {
            return false;
        }

        int imu_size = mImuQueue.size(); 
        int motion_size = mMotionstatusQueue.size(); 
        int simpstat_size = mSimpleStateQueue.size();
        int state_size = mStateQueue.size();
        
        // 3.1 先找到小于target_t的state下标
        int state_target_indx = -1;

        for(int i=0; i<state_size; i++)
        {
            int indx = state_size - 1 - i;
            if(mStateQueue.at(indx).t < target_t)
            {
                state_target_indx = indx;
                break;
            }
        }
        if(state_target_indx < 0)
        {
            return false;
        }

        // 3.2 再找与state_target_indx对应的 imu、motionstatus、simplestate的下标
        double t_tmp = mStateQueue.at(state_target_indx).t;
        int imu_target_indx = -1;        
        int simpstat_target_indx = -1;
        int motion_target_indx = -1;
        // 找imu indx
        for(int i=0; i<imu_size; i++)
        {
            int indx = imu_size - 1 - i;
            if(mImuQueue.at(indx).t == t_tmp)
            {
                imu_target_indx = indx;
                break;
            }
        }
        // 找 simplestate indx
        for(int i=0; i<simpstat_size; i++)
        {
            int indx = simpstat_size - 1 - i;
            if(mSimpleStateQueue.at(indx).t == t_tmp)
            {
                simpstat_target_indx = indx;
                break;
            }
        }
        // 找 motionstatus indx
        for(int i=0; i<motion_size; i++)
        {
            int indx = motion_size - 1 - i;
            if(mMotionstatusQueue.at(indx).t == t_tmp)
            {
                motion_target_indx = indx;
                break;
            }
        }

        // 如果数据找不全（不具备回退的条件），则不做处理
        if(imu_target_indx < 0 || simpstat_target_indx < 0 || motion_target_indx < 0)
        {
            std::cout << "OnSlideDetected error [imu_target_indx, simpstat_target_indx, motion_target_indx] : " 
                << imu_target_indx << ", " << simpstat_target_indx << ", " << motion_target_indx << std::endl;
            return false;
        }

        /* --------------------- 4. 下面开始重新计算 ---------------------
        */
        // 4.1-1 清空mMotionstatusQueue后面的数据
        for(int i=motion_target_indx+1; i<motion_size; i++)
        {
            mMotionstatusQueue.pop_back();
        }

        // 4.1-2 重新计算motionstatus
        for (int i = imu_target_indx + 1; i <= mCalcToImuIndx; i++)
        {
            double t = mImuQueue.at(i).t;

            MotionStatus motionStatus;
            CalcMotionstatus(t, motionStatus);
            // clink::Logger("motionStatus.txt") << "motionStatus : " << motionStatus.t << ", " << static_cast<int>(motionStatus.status) << std::endl;
            mMotionstatusQueue.push_back(motionStatus);
        }

        // 4.2-1 用rtk速度替换轮速：时间范围 [slide_t - time_len, slide_t]
        for(int i=0, iend=mWheelOdometerQueue.size(); i<iend; i++)
        {
            WheelOdometer &wheelOdometer = mWheelOdometerQueue.at(i);
            if(wheelOdometer.t < slide_t - time_len - 0.05)
            {
                continue;
            }
            if(wheelOdometer.t > slide_t + 0.05)
            {
                break;
            }

            Eigen::Vector3d vel_3d;
            if(CalcWheelVelWithRtk(wheelOdometer.t, vel_3d))
            {
                wheelOdometer.vel_3d_from_rtk = vel_3d;
                wheelOdometer.use_rtk_vel = true;
            }

        }

        // 4.2-2. 清除mSimpleStateQueue/mStateQueue后面的数据
        for(int i=simpstat_target_indx+1; i<simpstat_size; i++)
        {
            mSimpleStateQueue.pop_back();
        }
        for(int i=state_target_indx+1; i<state_size; i++)
        {
            mStateQueue.pop_back();
        }

        // 重新做DR推算
        int imuStartIndx = imu_target_indx + 1;
        RecalcDeadReckoning(imuStartIndx);

        // 重新处理State关联的Gps数据
        RecalcProcessStateGpsSmooth();

        return true;
    }

    // 重新做DR推算，打滑判断后的处理
    void Estimater::RecalcDeadReckoning(int imuStartIndx)
    {
        for (int i = imuStartIndx; i <= mCalcToImuIndx; i++)
        {
            IMU::Point &imuPre = mImuQueue.at(i - 1); // 获取之前时刻的imu数据
            IMU::Point &imuCur = mImuQueue.at(i);     // 获取 当前时刻的imu数据
            double cur_t = imuCur.t;

            EKF_WHEELPLUS::State &drStatePre = mStateQueue.back();
            EKF_WHEELPLUS::State drState;
            drState.t = cur_t;
            drState.x3_t = imuCur.x3_t;

            // 提前获取到cur_t时刻的轮速
            WheelOdometer wheelOdometer;
            int wheelIndx = getWheelodometerIndx(cur_t, 0.15);
            if (wheelIndx < 0)
            {
                std::cout << "Can't get wheeelodometer indx in DeadReckoning [time, min_t, max_t] : "
                          << cur_t << " " << mWheelOdometerQueue.front().t << " " << mWheelOdometerQueue.back().t << std::endl;
            }
            if (wheelIndx <= 0)
            {
                wheelIndx = 1;
            }
            wheelOdometer = mWheelOdometerQueue.at(wheelIndx);

            // DR计算：静止/运动两种情况
            double dt = imuCur.t - imuPre.t;
            if (isStopInRecent1s(cur_t))
            {
                // 静止
                drState.pos = drStatePre.pos;
                drState.vel = Eigen::Vector3d::Zero();
                drState.R = drStatePre.R;
                drState.ba = drStatePre.ba;
                drState.bg = drStatePre.bg;

                // 计算预积分
                Eigen::Vector3d atti = getAttitudeFromR(drState.R);
                Preintegrate(dt, 0, atti(2), EnMotionStatus::STOP, mPreintegInfo);

                // 更新陀螺零偏
                StopImuInfo stopImuInfo;
                if (GetImustopinfoOntime(cur_t, 1e-3, stopImuInfo))
                {
                    drState.bg = stopImuInfo.gyro_bias_mean;
                }
            }
            else
            {
                // 运动
                Eigen::Vector3f gyroRaw = (imuPre.w + imuCur.w) * 0.5;
                Eigen::Vector3d gyro = gyroRaw.cast<double>() - drStatePre.bg;
                // 姿态更新
                Eigen::Vector3d r = gyro * dt;
                Eigen::AngleAxisd rv(r.norm(), r.normalized());
                Eigen::Matrix3d incrR(rv);
                drState.R = drStatePre.R * incrR;

                double d;
                if(wheelOdometer.use_rtk_vel)
                {
                    // 速度
                    drState.vel = drState.R * wheelOdometer.vel_3d_from_rtk;

                    // 位置更新
                    Eigen::Vector3d vehicle_d(wheelOdometer.vel_3d_from_rtk * dt);
                    drState.pos = drStatePre.pos + (drState.R + drStatePre.R) * 0.5 * vehicle_d;
                    d = (wheelOdometer.vel_3d_from_rtk * dt).norm();
                }
                else
                {
                    // 速度
                    Eigen::Vector3d vel(0, wheelOdometer.vel * mWheelplusSF, 0);
                    drState.vel = drState.R * vel;

                    // 位置更新
                    double dl = mWheelOdometerQueue.at(wheelIndx).d_l;
                    double dr = mWheelOdometerQueue.at(wheelIndx).d_r;
                    double dT = mWheelOdometerQueue.at(wheelIndx).t - mWheelOdometerQueue.at(wheelIndx).pre_t;
                    d = (dl + dr) * 0.5 * dt / dT;
                    Eigen::Vector3d vehicle_d(0, d, 0);
                    drState.pos = drStatePre.pos + (drState.R + drStatePre.R) * 0.5 * vehicle_d * mWheelplusSF;
                }

                drState.ba = drStatePre.ba;
                drState.bg = drStatePre.bg;

                // 计算预积分
                Eigen::Vector3d atti = getAttitudeFromR(drState.R);
                Preintegrate(dt, d, atti(2), EnMotionStatus::MOVE, mPreintegInfo);
            }

            // 修正pitch/roll
            static double last_pr_calib_time = 0;
            if (cur_t - last_pr_calib_time > 0.1 - 0.005)
            {
                Eigen::Vector2d state_atti_mean;
                Eigen::Vector3d accel_mean;
                Eigen::Vector3d gyro_mean;
                Eigen::Vector3d accel_std;
                double smooth_timelen = 0.5;

                if (CalcImuMeanAndStdInfos(cur_t, smooth_timelen, accel_mean, gyro_mean, accel_std) && CalcMeanStatePitchroll(cur_t, smooth_timelen, state_atti_mean))
                {
                    CalibPitchRollWithAccel(wheelOdometer, drState, state_atti_mean, accel_mean, gyro_mean, accel_std);
                }

                last_pr_calib_time = cur_t;
            }

            // DR结果添加到队列：避免添加距离很近的轨迹点
            int drStateSize = mStateQueue.size();
            if (drStateSize < 2)
            {
                drState.bPreintegred = true;
                drState.preintegInfo = mPreintegInfo;
                mStateQueue.push_back(drState);
            }
            else
            {
                Eigen::Vector3d dpos = mStateQueue.at(drStateSize - 1).pos - mStateQueue.at(drStateSize - 2).pos;
                double d = dpos.topRows(2).norm();
                Eigen::Vector3d atti1 = getAttitudeFromR(mStateQueue.at(drStateSize - 1).R);
                Eigen::Vector3d atti0 = getAttitudeFromR(mStateQueue.at(drStateSize - 2).R);
                double dyaw = atti1(2) - atti0(2);
                while (dyaw > M_PI)
                {
                    dyaw -= 2 * M_PI;
                }
                while (dyaw < -M_PI)
                {
                    dyaw += 2 * M_PI;
                }
                double dt = mStateQueue.at(drStateSize-1).t - mStateQueue.at(drStateSize-2).t;
                // 相邻点之间的距离小于阈值，且角度变化小于阈值
                if (d < 0.2 && fabs(dyaw) < 15 * M_PI / 180 && dt < 3.0)
                {
                    EKF_WHEELPLUS::State &state = mStateQueue.back();
                    state.t = drState.t;
                    state.x3_t = drState.x3_t;
                    state.pos = drState.pos;
                    state.vel = drState.vel;
                    state.R = drState.R;
                    state.ba = drState.ba;
                    state.bg = drState.bg;
                    state.bPreintegred = true;
                    for (int k = 0; k < mPreintegInfo.ve_dt.size(); k++)
                    {
                        Preintegrate(mPreintegInfo.ve_dt[k], mPreintegInfo.ve_d[k], mPreintegInfo.ve_yaw[k],
                                     mPreintegInfo.ve_motionStatus[k], state.preintegInfo);
                    }
                }
                else
                {
                    drState.bPreintegred = true;
                    drState.preintegInfo = mPreintegInfo;
                    mStateQueue.push_back(drState);

                    mbAddNewDr = true;
                }
            }

            if(mStateQueue.size() > 10)
            {
                bool combined = false;
                for(int i=0; i<3; i++)
                {
                    int indx = mStateQueue.size() - 7 - i;
                    Eigen::Vector3d dpos_1 = mStateQueue.at(indx+1).pos - mStateQueue.at(indx).pos;
                    Eigen::Vector3d datti_1 = getAttitudeFromR(mStateQueue.at(indx+1).R) - getAttitudeFromR(mStateQueue.at(indx).R);
                    double dyaw1 = datti_1(2);
                    while(dyaw1 > M_PI)
                    {
                        dyaw1 -= 2 * M_PI;
                    }
                    while(dyaw1 < -M_PI)
                    {
                        dyaw1 += 2 * M_PI;
                    }

                    Eigen::Vector3d dpos_2 = mStateQueue.at(indx).pos - mStateQueue.at(indx-1).pos;
                    Eigen::Vector3d datti_2 = getAttitudeFromR(mStateQueue.at(indx).R) - getAttitudeFromR(mStateQueue.at(indx-1).R);
                    double dyaw2 = datti_2(2);
                    while(dyaw2 > M_PI)
                    {
                        dyaw2 -= 2 * M_PI;
                    }
                    while(dyaw2 < -M_PI)
                    {
                        dyaw2 += 2 * M_PI;
                    }

                    // 删掉indx对应的state
                    if(dpos_1.norm() < 1.0e-3 && dyaw1 < 1.0e-4 && dpos_2.norm() < 1.0e-3 && dyaw2 < 1.0e-4)
                    {
                        std::deque<EKF_WHEELPLUS::State, Eigen::aligned_allocator<EKF_WHEELPLUS::State>> tmpqueue = 
                            mStateQueue;

                        mStateQueue.clear();
                        for(int i=0; i<tmpqueue.size(); i++)
                        {
                            if(i != indx)
                            {
                                mStateQueue.push_back(tmpqueue.at(i));
                            }
                        }
                        combined = true;
                    }

                    if(combined)
                    {
                        break;
                    }
                }

            }


            EKF_WHEELPLUS::State temp_state;  
            {
                // std::lock_guard<std::mutex> lock(mMutexSensordata);

                EKF_WHEELPLUS::State &lastest_state = mStateQueue.back();
                lastest_state.Dr_Position = lastest_state.pos;
                Eigen::Vector3d dr_atti = getAttitudeFromR(lastest_state.R);
                lastest_state.Dr_Attitude = dr_atti;
                // 计算drState在Gps坐标系下的位姿
                if (mbGpsAigned)
                {
                    lastest_state.bAligned = mbGpsAigned;
                    lastest_state.gpsPos = mT_Gps_Dr.block<2, 2>(0, 0) * lastest_state.pos.topRows(2) + mT_Gps_Dr.block<2, 1>(0, 2);
                    Eigen::Vector3d atti = getAttitudeFromR(lastest_state.R);
                    lastest_state.gpsR = mT_Gps_Dr.block<2, 2>(0, 0) * Yaw2R(atti(2));

                    lastest_state.Position.topRows(2) = mT_Dr_Gps_fixed_inv.block<2, 2>(0, 0) * lastest_state.gpsPos + mT_Dr_Gps_fixed_inv.block<2, 1>(0, 2);
                    lastest_state.Position.z() = 0.0;
                    Eigen::Matrix2d R = mT_Dr_Gps_fixed_inv.block<2, 2>(0, 0) * lastest_state.gpsR;

                    lastest_state.Attitude.x() = atti.x();       // pitch
                    lastest_state.Attitude.y() = atti.y();       // roll
                    lastest_state.Attitude.z() = getYawFromR(R); // yaw

                    Eigen::Matrix3d R_temp = mR_EN * Attitude2R(lastest_state.Attitude);
                    Eigen::Vector3d Atti_end = getAttitudeFromR(R_temp);
                    lastest_state.Attitude.x() = Atti_end.x();
                    lastest_state.Attitude.y() = Atti_end.y();
                    lastest_state.Attitude.z() = Atti_end.z();

                    // clink::Logger("yaw_debug.txt") << lastest_state.t << " " << mT_Dr_Gps_fixed_inv(0, 0) << " " << mT_Dr_Gps_fixed_inv(0, 1) << " " << mT_Dr_Gps_fixed_inv(1, 0) << " " << mT_Dr_Gps_fixed_inv(1, 1) << " " << atti.z() << " " << lastest_state.Attitude.z() << std::endl;
                }
                temp_state = lastest_state;
            }
            ResetPreintegInfo(temp_state);

            EKF_WHEELPLUS::SimpleState simpleState;
            simpleState.t = temp_state.t;
            simpleState.pos = temp_state.pos;
            simpleState.R = temp_state.R;
            simpleState.gpsPos = temp_state.gpsPos;
            simpleState.gpsR = temp_state.gpsR;
            mSimpleStateQueue.push_back(simpleState);

        }
    }

    /*
     * 重新处理State关联的Gps数据，打滑判断后的处理：
     * 1、如果待处理的State在mStateQueue中的索引为indx, 则取[indx-1, indx+1]范围内(对应[t1,t2])的Gps数据做平滑
     * 2、先确定能处理到哪个State（calc_to_state_indx）
     *   1.1 因为mStateQueue里面最后一个数据是动态更新的，不可用。
     *   1.2 倒数第二个数据已经稳定了，但考虑到平滑参考点应该位于中间，因此最多算到倒数第三个数据（indx = size-3）
     *   1.3 再根据最新的Gps时刻调整，要求indx+1对应的时间戳 <= 最新的Gps时间戳
     */
    void Estimater::RecalcProcessStateGpsSmooth()
    {    
        int state_size = mStateQueue.size();

        int calc_to_state_indx = state_size - 3;

        // 再根据最新的Gps时刻调整，要求indx+1对应的时间戳 <= 最新的Gps时间戳
        double latest_gpst = mGpsOdomQueue.back().t;
        int gps_ready_state_indx = -1;
        for (int i = state_size - 1; i >= 0; i--)
        {
            if (mStateQueue.at(i).t <= latest_gpst)
            {
                gps_ready_state_indx = i;
                break;
            }
        }
        if (gps_ready_state_indx < 2)
        {
            return;
        }
        if (calc_to_state_indx > gps_ready_state_indx - 1)
        {
            calc_to_state_indx = gps_ready_state_indx - 1;
        }
        // 将所有距离倒数第二个状态 GPS_DATA_MAX_DELAY 之前的全部状态置为已经处理
        double ref_t = mStateQueue.at(state_size - 2).t;
        for (int i = state_size - 1; i >= 0; i--)
        {
            EKF_WHEELPLUS::State &state = mStateQueue.at(i);
            if (state.bGpsProcessed)
            {
                break;
            }

            // if (ref_t - state.t >= GPS_DATA_MAX_DELAY)
            if (ref_t - state.t >= 10.0)
            {
                state.bGpsProcessed = true;
                mLaseGpsProcessedTime = state.t;
            }
        }

        // 再确定calc_from_state_indx
        int calc_from_state_indx = -1;
        for (int i = state_size - 1; i >= 0; i--)
        {
            if (mStateQueue.at(i).bGpsProcessed)
            {
                calc_from_state_indx = i + 1;
                break;
            }
        }
        if (calc_from_state_indx < 1)
        {
            calc_from_state_indx = 1;
        }
        // 下面开始处理
        for (int i = calc_from_state_indx; i <= calc_to_state_indx; i++)
        {
            EKF_WHEELPLUS::State &state = mStateQueue.at(i);
            double t1 = mStateQueue.at(i - 1).t;
            double t2 = mStateQueue.at(i + 1).t;
            GpsOdometry gpsOdomSmooth;
            if (SmoothGps(t1, t2, state.t, gpsOdomSmooth))
            {
                state.gpsOdom = std::make_shared<GpsOdometry>(gpsOdomSmooth);
            }
            state.bGpsProcessed = true;
            mLaseGpsProcessedTime = state.t;
        }
    }   

    // 根据rtk速度计算轮速（后轴中心的速度）
    bool Estimater::CalcWheelVelWithRtk(double t, Eigen::Vector3d &vel_3d)
    {
        if(mImuQueue.size() < 100 || mGpsOdomQueue.size() < 2 || mStateQueue.size() < 2)
        {
            return false;
        }

        // 找到与t最接近的imu数据、state数据、rtk数据 的下标
        int imu_size = mImuQueue.size();
        int rtk_size = mGpsOdomQueue.size();
        int state_size = mStateQueue.size();

        // 找imu数据下标
        int imu_target_indx = -1;
        double dt_min = 1e10;
        for(int i=0; i<imu_size; i++)
        {
            int indx = imu_size - 1 - i;
            double dt = mImuQueue.at(indx).t - t;
            if(fabs(dt) > dt_min)
            {
                break;
            }
            imu_target_indx = indx;
            dt_min = fabs(dt);
        }
        if(imu_target_indx < 0 || imu_target_indx < 10)
        {
            return false;
        }

        // 找state数据下标
        int state_target_indx = -1;
        dt_min = 1e10;
        for(int i=0; i<state_size; i++)
        {
            int indx = state_size - 1 - i;
            double dt = mStateQueue.at(indx).t - t;
            if(fabs(dt) > dt_min)
            {
                break;
            }
            state_target_indx = indx;
            dt_min = fabs(dt);
        }
        if(state_target_indx < 0)
        {
            return false;
        }

        // 找rtk数据下标
        int rtk_target_indx = -1;
        dt_min = 1e10;
        for(int i=0; i<rtk_size; i++)
        {
            int indx = rtk_size - 1 - i;
            double dt = mGpsOdomQueue.at(indx).t - t;
            if(fabs(dt) > dt_min)
            {
                break;
            }
            rtk_target_indx = indx;
            dt_min = fabs(dt);
        }
        if(rtk_target_indx < 0)
        {
            return false;
        }

        // 计算角速度，取imu_target_indx最近0.1秒内的imu数据
        Eigen::Vector3d angular_vel_sum = Eigen::Vector3d::Zero();
        for(int i=imu_target_indx-10; i<imu_target_indx; i++)
        {
            angular_vel_sum += mImuQueue.at(i).w.cast<double>();
        }
        Eigen::Vector3d angular_vel = angular_vel_sum / 10;
        angular_vel -= mStateQueue.at(state_target_indx).bg;

        // 把rtk速度从世界系分解到body系
        Eigen::Matrix3d Rwb = mat2dToMat3d(mStateQueue.at(state_target_indx).gpsR);
        Eigen::Vector3d rtk_vel_w = mGpsOdomQueue.at(rtk_target_indx).vel;
        Eigen::Vector3d rtk_vel_b = Rwb.transpose() * rtk_vel_w;
        vel_3d = rtk_vel_b + EKF_WHEELPLUS::getSkewMatrix(angular_vel) * (mPgv);

        // Logger("CalcWheelVelWithRtk.txt") << t << ", " << rtk_vel_w.transpose() << ", " 
        //     << rtk_vel_b.transpose() << ", " << vel_3d.transpose() << std::endl;

        return true;
    } 

    void Estimater::UpdateWheelSlipInfo(int slip_state, double t, double duration)
    {
        std::lock_guard<std::mutex> lock(mMutexSensordata);

        mWheelSlipInfo.cur_t = t;
        mWheelSlipInfo.blastslip = mWheelSlipInfo.bslip;
        mWheelSlipInfo.bprocessed = false;

        if(slip_state == 0) // 当前不打滑
        {
            // 但之前处理打滑状态
            if(mWheelSlipInfo.bslip)
            {
                double dt = mWheelSlipInfo.cur_t - mWheelSlipInfo.t1;
                if(dt > 3.0)
                {
                    mWheelSlipInfo.bslip = false;
                }
            }
        }
        else // 当前打滑
        {
            if(mWheelSlipInfo.bslip) // 之前处于打滑状态
            {
                mWheelSlipInfo.t1 = mWheelSlipInfo.cur_t;
            }
            else // 之前不是打滑状态
            {
                mWheelSlipInfo.t0 = mWheelSlipInfo.cur_t - duration;
                mWheelSlipInfo.t1 = mWheelSlipInfo.cur_t;
                mWheelSlipInfo.bslip = true;
            }
        }
    }

} // namespace ORB_SLAM
