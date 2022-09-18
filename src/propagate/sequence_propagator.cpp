/* 内部依赖 */
#include <include/propagate/sequence_propagator.hpp>
/* 外部依赖 */

namespace ESKF_VIO_BACKEND {
    /* 新一帧 IMU 量测数据输入，在已有 queue 的基础上进行 propagate */
    bool PropagateQueue::Propagate(const Eigen::Matrix<Scalar, 3, 1> &accel,
                                   const Eigen::Matrix<Scalar, 3, 1> &gyro,
                                   const fp64 timeStamp) {
        // 如果序列为空，则初始化运动相关名义状态
        if (this->items.empty()) {
            this->startTimeStamp = timeStamp;
            this->startNominalState.p_wb.setZero();
            this->startNominalState.v_wb.setZero();
            this->startNominalState.q_wb.setIdentity();
        }

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