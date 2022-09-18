#pragma once
/* 外部依赖 */
/* 内部依赖 */
#include <include/utility/typedef.hpp>
#include <include/propagate/imu_state.hpp>
#include <include/data_manager/frame_manager.hpp>

namespace ESKF_VIO_BACKEND {
    /* IMU propagate 序列中的元素 */
    class IMUPropagateQueueItem {
    public:
        // 完整误差状态
        IMUFullState errorState;
        // 运动相关名义状态
        IMUMotionState nominalState;
        // IMU 完整状态协方差矩阵
        Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> imuCov;
        // IMU 与 Camera 的协方差矩阵
        Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> imuCamCov;
        // 当前时刻点对应的 IMU 量测值
        Eigen::Matrix<Scalar, 3, 1> accel;
        Eigen::Matrix<Scalar, 3, 1> gyro;
        // 当前时间戳
        double timeStamp;
    public:
        /* 构造函数与析构函数 */
        IMUPropagateQueueItem() {}
        ~IMUPropagateQueueItem() {}
    };


    /* IMU-Camera propagate 序列 */
    class PropagateQueue {
    public:
        // propagate 之后需要记录的变化的相关元素
        std::deque<std::shared_ptr<IMUPropagateQueueItem>> items;
        // 滑动窗口内的关键帧
        FrameManager *slidingWindow;
        // 滑动窗口内 camera pose 的协方差矩阵
        Matrix camCov;
        // 无 update 时的 IMU bias 和 gravity
        Eigen::Matrix<Scalar, 3, 1> bias_a;
        Eigen::Matrix<Scalar, 3, 1> bias_g;
        Eigen::Matrix<Scalar, 3, 1> gravity;
        // 联系 IMU 和 Camera 协方差的 Fai 矩阵
        Matrix fai;
        // IMU 的噪声
        Scalar noise_accel;
        Scalar noise_gyro;
        Scalar random_walk_accel;
        Scalar random_walk_gyro;
    public:
        /* 构造函数与析构函数 */
        PropagateQueue() {}
        ~PropagateQueue() {}
    public:
        /* 新一帧 IMU 量测数据输入，在已有 queue 的基础上进行 propagate */
        bool Propagate(const Eigen::Matrix<Scalar, 3, 1> &accel,
                       const Eigen::Matrix<Scalar, 3, 1> &gyro,
                       const fp64 timeStamp);
        /* 中值积分法 propagate 运动相关名义状态 */
        void PropagateMotionNominalState(const IMUMotionState &state_0,
                                         IMUMotionState &state_1,
                                         const Eigen::Matrix<Scalar, 3, 1> &accel_0,
                                         const Eigen::Matrix<Scalar, 3, 1> &gyro_0,
                                         const Eigen::Matrix<Scalar, 3, 1> &accel_1,
                                         const Eigen::Matrix<Scalar, 3, 1> &gyro_1,
                                         const Eigen::Matrix<Scalar, 3, 1> &bias_a,
                                         const Eigen::Matrix<Scalar, 3, 1> &bias_g,
                                         const Scalar dt);
        /* 基于离散误差状态过程方程 propagate 完整误差状态以及误差状态对应协方差矩阵 */
        void PropagateFullErrorStateCovariance(const IMUFullState &d_state_0,
                                               IMUFullState &d_state_1,
                                               const Eigen::Matrix<Scalar, 3, 1> &accel_0,
                                               const Eigen::Matrix<Scalar, 3, 1> &gyro_0,
                                               const Eigen::Matrix<Scalar, 3, 1> &accel_1,
                                               const Eigen::Matrix<Scalar, 3, 1> &gyro_1,
                                               const Eigen::Matrix<Scalar, 3, 1> &bias_a,
                                               const Eigen::Matrix<Scalar, 3, 1> &bias_g,
                                               const Scalar dt);
    };
}