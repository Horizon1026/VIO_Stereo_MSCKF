#pragma once
/* 外部依赖 */
/* 内部依赖 */
#include <include/utility/typedef.hpp>

namespace ESKF_VIO_BACKEND {
    /* IMU 完整状态 */
    class IMUFullState {
    public:
        // 位置
        Eigen::Matrix<Scalar, 3, 1> p_wb;
        // 姿态（名义状态与误差状态二选一）
        Eigen::Quaternion<Scalar> q_wb;
        Eigen::Matrix<Scalar, 3, 1> theta_wb;
        // 速度
        Eigen::Matrix<Scalar, 3, 1> v_wb;
        // 加速度偏差
        Eigen::Matrix<Scalar, 3, 1> bias_a;
        // 角速度偏差
        Eigen::Matrix<Scalar, 3, 1> bias_g;
        // 重力加速度
        Eigen::Matrix<Scalar, 3, 1> gravity;
    public:
        /* 构造函数与析构函数 */
        IMUFullState() {}
        ~IMUFullState() {}
        IMUFullState(const Eigen::Matrix<Scalar, 3, 1> &p_wb,
                     const Eigen::Quaternion<Scalar> &q_wb,
                     const Eigen::Matrix<Scalar, 3, 1> &v_wb,
                     const Eigen::Matrix<Scalar, 3, 1> &bias_a,
                     const Eigen::Matrix<Scalar, 3, 1> &bias_g,
                     const Eigen::Matrix<Scalar, 3, 1> &gravity) :
            p_wb(p_wb), q_wb(q_wb), v_wb(v_wb), bias_a(bias_a), bias_g(bias_g), gravity(gravity) {}
        IMUFullState(const Eigen::Matrix<Scalar, 3, 1> &p_wb,
                     const Eigen::Matrix<Scalar, 3, 1> &theta_wb,
                     const Eigen::Matrix<Scalar, 3, 1> &v_wb,
                     const Eigen::Matrix<Scalar, 3, 1> &bias_a,
                     const Eigen::Matrix<Scalar, 3, 1> &bias_g,
                     const Eigen::Matrix<Scalar, 3, 1> &gravity) :
            p_wb(p_wb), theta_wb(theta_wb), v_wb(v_wb), bias_a(bias_a), bias_g(bias_g), gravity(gravity) {}
    };


    /* IMU 运动相关状态 */
    class IMUMotionState {
    public:
        // 位置
        Eigen::Matrix<Scalar, 3, 1> p_wb;
        // 姿态
        Eigen::Quaternion<Scalar> q_wb;
        // 速度
        Eigen::Matrix<Scalar, 3, 1> v_wb;
    public:
        /* 构造函数与析构函数 */
        IMUMotionState() {}
        ~IMUMotionState() {}
        IMUMotionState(const Eigen::Matrix<Scalar, 3, 1> &p_wb,
                       const Eigen::Quaternion<Scalar> &q_wb,
                       const Eigen::Matrix<Scalar, 3, 1> &v_wb) :
            p_wb(p_wb), q_wb(q_wb), v_wb(v_wb) {}
    };

}