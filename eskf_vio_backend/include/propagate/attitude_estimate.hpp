#pragma once
/* 外部依赖 */
/* 内部依赖 */
#include <typedef.hpp>

namespace ESKF_VIO_BACKEND {
    /* 姿态解算估计结果队列中的一个元素 */
    class AttitudeEstimateItem {
    public:
        // 解算所得姿态
        Quaternion q_wb = Quaternion::Identity();
        // 当前时刻点对应的 IMU 量测值
        Vector3 accel = Vector3::Zero();
        Vector3 gyro = Vector3::Zero();
        // 当前时间戳
        fp64 timeStamp = 0;
    public:
        /* 构造函数与析构函数 */
        AttitudeEstimateItem() = default;
        virtual ~AttitudeEstimateItem() = default;
    };


    /* 姿态解算求解器 */
    class AttitudeEstimate {
    public:
        // 姿态解算结果维护为一个队列
        std::deque<std::shared_ptr<AttitudeEstimateItem>> items;
        // IMU bias，由 update 过程进行更新
        Vector3 bias_a = Vector3::Zero();
        Vector3 bias_g = Vector3::Zero();
        // 姿态解算的 PI 控制器，积分部分的误差累加值
        Scalar Kp = Scalar(0.1);
        Scalar Ki = Scalar(0.0001);
        Vector3 errInt = Vector3::Zero();
    public:
        /* 构造函数与析构函数 */
        AttitudeEstimate() = default;
        virtual ~AttitudeEstimate() = default;
    public:
        /* 姿态解算进行一步更新 */
        bool Propagate(const Vector3 &accel, const Vector3 &gyro, const fp64 timeStamp);
        /* 提取出指定时刻点附近的姿态估计结果 */
        bool GetAttitude(const fp64 timeStamp, const fp64 threshold, Quaternion &atti);
        /* 设置 bias */
        bool SetBias(const Vector3 &bias_a, const Vector3 &bias_g);
        /* 清除旧时刻的姿态解算结果 */
        bool CleanOldItems(const fp64 timeStamp, const Scalar threshold);
    };
}