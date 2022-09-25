#pragma once
/* 外部依赖 */
/* 内部依赖 */
#include <include/utility/typedef.hpp>

namespace ESKF_VIO_BACKEND {
    /* 姿态解算估计结果队列中的一个元素 */
    class AttitudeEstimateItem {
    public:
        // 解算所得姿态
        Quaternion atti;
        // 当前时刻点对应的 IMU 量测值
        Vector3 accel;
        Vector3 gyro;
        // 当前时间戳
        fp64 timeStamp;
    public:
        /* 构造函数与析构函数 */
        AttitudeEstimateItem() {}
        ~AttitudeEstimateItem() {}
    };


    /* 姿态解算求解器 */
    class AttitudeEstimate {
    public:
        // 姿态解算结果维护为一个队列
        std::deque<std::shared_ptr<AttitudeEstimateItem>> items;
    public:
        /* 构造函数与析构函数 */
        AttitudeEstimate() {}
        ~AttitudeEstimate() {}
    public:
        /* 姿态解算进行一步更新 */
        bool Propate(const Vector3 &accel, const Vector3 &gyro, const fp64 timeStamp);
        /* 提取出指定时刻点附近的姿态估计结果 */
        bool GetAttitude(const fp64 timeStamp, Quaternion &atti);
    };
}