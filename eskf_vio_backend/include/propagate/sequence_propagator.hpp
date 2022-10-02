#pragma once
/* 外部依赖 */
/* 内部依赖 */
#include <typedef.hpp>
#include <imu_state.hpp>
#include <frame_manager.hpp>

namespace ESKF_VIO_BACKEND {
    /* IMU propagate 序列中的元素 */
    class IMUPropagateQueueItem {
    public:
        // 完整误差状态
        IMUFullState errorState;
        // 运动相关名义状态
        IMUMotionState nominalState;
        // IMU 完整状态协方差矩阵
        Eigen::Matrix<Scalar, IMU_STATE_SIZE, IMU_STATE_SIZE> imuCov;
        // IMU 与 Camera 的协方差矩阵
        Eigen::Matrix<Scalar, IMU_STATE_SIZE, Eigen::Dynamic> imuCamCov;
        // 当前时刻点对应的 IMU 量测值
        Vector3 accel;
        Vector3 gyro;
        // 当前时间戳
        fp64 timeStamp;
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
        std::shared_ptr<FrameManager> slidingWindow;
        // 滑动窗口内 camera pose 的协方差矩阵
        Matrix camCov;
        // 无 update 时的 IMU bias
        Vector3 bias_a;
        Vector3 bias_g;
        // 无 update 且 items 为空时的 motion state 初值，初值将在 eskf vio backend 初始化时从姿态解算结果中获取
        IMUMotionState initState;
        // 联系 IMU 和 Camera 协方差的 Fai 矩阵就是离散时间的 F 矩阵
        Eigen::Matrix<Scalar, IMU_STATE_SIZE, IMU_STATE_SIZE> phi;
        // 离散时间状态方程和过程噪声
        Eigen::Matrix<Scalar, IMU_STATE_SIZE, IMU_STATE_SIZE> F;
        Eigen::Matrix<Scalar, IMU_STATE_SIZE, IMU_NOISE_SIZE> G;
        Eigen::Matrix<Scalar, IMU_NOISE_SIZE, IMU_NOISE_SIZE> Q;

    public:
        /* 构造函数与析构函数 */
        PropagateQueue() {}
        ~PropagateQueue() {}
    public:
        /* 新一帧 IMU 量测数据输入，在已有 queue 的基础上进行 propagate */
        bool Propagate(const Vector3 &accel,
                       const Vector3 &gyro,
                       const fp64 timeStamp);
        /* 中值积分法 propagate 运动相关名义状态 */
        void PropagateMotionNominalState(const std::shared_ptr<IMUPropagateQueueItem> &item_0,
                                         std::shared_ptr<IMUPropagateQueueItem> &item_1,
                                         const Vector3 &bias_a,
                                         const Vector3 &bias_g,
                                         const Vector3 &gravity_w,
                                         Vector3 &midAccel,
                                         Vector3 &midGyro);
        /* 基于离散误差状态过程方程 propagate 完整误差状态以及误差状态对应协方差矩阵 */
        void PropagateFullErrorStateCovariance(const std::shared_ptr<IMUPropagateQueueItem> &item_0,
                                               std::shared_ptr<IMUPropagateQueueItem> &item_1,
                                               const Vector3 &midAccel,
                                               const Vector3 &midGyro);
        /* 重置过程方程 */
        void ResetProcessFunction(void);
        /* 误差状态合并与分裂 */
        Eigen::Matrix<Scalar, IMU_STATE_SIZE, 1> ErrorStateConvert(const IMUFullState &errorState);
        IMUFullState ErrorStateConvert(const Eigen::Matrix<Scalar, IMU_STATE_SIZE, 1> &delta_x);
        /* 初始化过程噪声矩阵 */
        void InitializeProcessNoiseMatrix(const Scalar noise_accel,
                                          const Scalar noise_gyro,
                                          const Scalar random_walk_accel,
                                          const Scalar random_walk_gyro);
    };
}