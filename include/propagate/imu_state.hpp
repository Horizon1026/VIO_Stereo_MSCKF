#pragma once
/* 外部依赖 */
/* 内部依赖 */
#include <include/utility/typedef.hpp>

namespace ESKF_VIO_BACKEND {
    /* 状态与噪声向量维度 */
    #define IMU_FULL_ERROR_STATE_SIZE (18)
    #define IMU_NOISE_SIZE (12)
    /* 状态向量分量索引 */
    #define INDEX_P (0)
    #define INDEX_V (3)
    #define INDEX_R (6)
    #define INDEX_BA (9)
    #define INDEX_BG (12)
    #define INDEX_G (15)
    /* 噪声向量分量索引 */
    #define INDEX_NA (0)
    #define INDEX_NG (3)
    #define INDEX_NWA (6)
    #define INDEX_NWG (9)

    /* IMU 完整状态 */
    class IMUFullState {
    public:
        // 位置
        Vector3 p_wb;
        // 速度
        Vector3 v_wb;
        // 姿态（名义状态与误差状态二选一）
        Quaternion q_wb;
        Vector3 theta_wb;
        // 加速度偏差
        Vector3 bias_a;
        // 角速度偏差
        Vector3 bias_g;
        // 重力加速度
        Vector3 gravity;
    public:
        /* 构造函数与析构函数 */
        IMUFullState() {}
        ~IMUFullState() {}
        IMUFullState(const Vector3 &p_wb,
                     const Quaternion &q_wb,
                     const Vector3 &v_wb,
                     const Vector3 &bias_a,
                     const Vector3 &bias_g,
                     const Vector3 &gravity) :
            p_wb(p_wb), v_wb(v_wb), q_wb(q_wb), bias_a(bias_a), bias_g(bias_g), gravity(gravity) {}
        IMUFullState(const Vector3 &p_wb,
                     const Vector3 &theta_wb,
                     const Vector3 &v_wb,
                     const Vector3 &bias_a,
                     const Vector3 &bias_g,
                     const Vector3 &gravity) :
            p_wb(p_wb), v_wb(v_wb), theta_wb(theta_wb), bias_a(bias_a), bias_g(bias_g), gravity(gravity) {}
    public:
        /* 状态清零 */
        void Reset(void);
    };


    /* IMU 运动相关状态 */
    class IMUMotionState {
    public:
        // 位置
        Vector3 p_wb;
        // 速度
        Vector3 v_wb;
        // 姿态
        Quaternion q_wb;
    public:
        /* 构造函数与析构函数 */
        IMUMotionState() {}
        ~IMUMotionState() {}
        IMUMotionState(const Vector3 &p_wb,
                       const Vector3 &v_wb,
                       const Quaternion &q_wb) :
            p_wb(p_wb), v_wb(v_wb), q_wb(q_wb) {}
    };

}