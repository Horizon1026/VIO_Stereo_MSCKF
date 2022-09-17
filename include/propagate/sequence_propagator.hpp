#pragma once
/* 外部依赖 */
/* 内部依赖 */
#include <include/utility/typedef.hpp>
#include <include/propagate/imu_state.hpp>

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
    };


    /* IMU-Camera propagate 序列 */
    class PropagateQueue {
    public:
        // propagate 之后需要记录的变化的相关元素
        std::deque<std::shared_ptr<IMUPropagateQueueItem>> items;
        // items 中首个 state 中值积分时的上一时刻 IMU 量测值，以及对应的时间戳
        Eigen::Matrix<Scalar, 3, 1> accel_0;
        Eigen::Matrix<Scalar, 3, 1> gyro_0;
        double timeStamp_0;
        // 无 update 时的 IMU bias
        Eigen::Matrix<Scalar, 3, 1> bias_a;
        Eigen::Matrix<Scalar, 3, 1> bias_g;
        // 联系 IMU 和 Camera 协方差的 Fai 矩阵
        Matrix fai;
        // propagate 的起点名义状态
        std::shared_ptr<IMUFullState> start;
        // propagate 的起点时间戳
        fp64 startTimeStamp;
    public:
        /* 构造函数与析构函数 */
        PropagateQueue() {}
        ~PropagateQueue() {}
    public:
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