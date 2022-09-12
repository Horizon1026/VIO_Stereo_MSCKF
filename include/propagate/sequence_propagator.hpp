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
    };


    /* IMU-Camera propagate 序列 */
    class PropagateQueue {
    public:
        // propagate 之后需要记录的变化的相关元素
        std::deque<std::shared_ptr<IMUPropagateQueueItem>> items;
        // 联系 IMU 和 Camera 协方差的 Fai 矩阵
        Matrix fai;
        // propagate 的起点名义状态
        std::shared_ptr<IMUFullState> start;
        // propagate 的起点时间戳
        fp64 startTimeStamp;
        
    };
}