/* 内部依赖 */
#include <include/propagate/sequence_propagator.hpp>
/* 外部依赖 */

namespace ESKF_VIO_BACKEND {
    /* 新一帧 IMU 量测数据输入，在已有 queue 的基础上进行 propagate */
    bool PropagateQueue::Propagate(const Eigen::Matrix<Scalar, 3, 1> &accel,
                                   const Eigen::Matrix<Scalar, 3, 1> &gyro,
                                   const fp64 timeStamp) {
        // 获取 IMU 和 Cam 的状态维度
        uint32_t imuSize = 18;
        uint32_t camSize = this->slidingWindow->frames.size() * 6;

        // 如果序列为空，则构造新的起点
        if (this->items.empty()) {
            // 构造一个新的 queue item，添加到序列中，作为 propagate 的起点
            std::shared_ptr<IMUPropagateQueueItem> newItem(new IMUPropagateQueueItem());
            this->items.emplace_back(newItem);
            // 设置 propagate 起点参数
            newItem->accel = accel;
            newItem->gyro = gyro;
            newItem->timeStamp = timeStamp;
            newItem->imuCov.setZero(imuSize, imuSize);
            if (camSize > 0) {
                newItem->imuCamCov.setZero(imuSize, camSize);
                this->camCov.setZero(camSize, camSize);
                this->fai.setIdentity(imuSize, camSize);
            }
            newItem->errorState.Reset();
            // 从零开始创建的 propagate 起点，名义运动状态归零
            newItem->nominalState.p_wb.setZero();
            newItem->nominalState.q_wb.setIdentity();
            newItem->nominalState.v_wb.setZero();

            return true;
        }

        // 当序列不为空时，触发一次中值积分的 propagate 过程
        std::shared_ptr<IMUPropagateQueueItem> newItem(new IMUPropagateQueueItem());
        // TODO: 

        return true;
    }
    /* 中值积分法 propagate 运动相关名义状态 */
    void PropagateQueue::PropagateMotionNominalState(const IMUMotionState &state_0,
                                                     IMUMotionState &state_1,
                                                     const Eigen::Matrix<Scalar, 3, 1> &accel_0,
                                                     const Eigen::Matrix<Scalar, 3, 1> &gyro_0,
                                                     const Eigen::Matrix<Scalar, 3, 1> &accel_1,
                                                     const Eigen::Matrix<Scalar, 3, 1> &gyro_1,
                                                     const Eigen::Matrix<Scalar, 3, 1> &bias_a,
                                                     const Eigen::Matrix<Scalar, 3, 1> &bias_g,
                                                     const Scalar dt) {
        
    }
    /* 基于离散误差状态过程方程 propagate 完整误差状态以及误差状态对应协方差矩阵 */
    void PropagateQueue::PropagateFullErrorStateCovariance(const IMUFullState &d_state_0,
                                                           IMUFullState &d_state_1,
                                                           const Eigen::Matrix<Scalar, 3, 1> &accel_0,
                                                           const Eigen::Matrix<Scalar, 3, 1> &gyro_0,
                                                           const Eigen::Matrix<Scalar, 3, 1> &accel_1,
                                                           const Eigen::Matrix<Scalar, 3, 1> &gyro_1,
                                                           const Eigen::Matrix<Scalar, 3, 1> &bias_a,
                                                           const Eigen::Matrix<Scalar, 3, 1> &bias_g,
                                                           const Scalar dt) {
        
    }
}