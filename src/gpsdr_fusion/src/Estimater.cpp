#include "Estimater.h"


Estimater::Estimater(){

}

Estimater::~Estimater(){
    
}

void Estimater::run(){
    mbRunning = true;

    while(mbRunning){
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

        }
        else
        {
            std::this_thread::sleep_for(std::chrono::microseconds(500));
        }
    }
}

void Estimater::AddRawImu(const SENSOR_DATA::RawImu &imu){
    std::lock_guard<std::mutex> lock(mMutexSensordata);
    SENSOR_DATA::RawDataWithType rawDataWithType;
    rawDataWithType.type = SENSOR_DATA::Type::IMU;
    rawDataWithType.pRawData = std::make_shared<SENSOR_DATA::RawImu>(imu);
    mRawDataWithTypeQueue.push_back(rawDataWithType);
}

void Estimater::AddRawWheel(const SENSOR_DATA::RawWheel &wheelOdometer){
    std::lock_guard<std::mutex> lock(mMutexSensordata);
    SENSOR_DATA::RawDataWithType rawDataWithType;
    rawDataWithType.type = SENSOR_DATA::Type::WHEELODOMETER;
    rawDataWithType.pRawData = std::make_shared<SENSOR_DATA::RawWheel>(wheelOdometer);
    mRawDataWithTypeQueue.push_back(rawDataWithType);
}

void Estimater::AddRawGps(const SENSOR_DATA::RawGps &gpsOdom){
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