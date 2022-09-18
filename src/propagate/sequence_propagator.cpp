/* 内部依赖 */
#include <include/propagate/sequence_propagator.hpp>
/* 外部依赖 */

namespace ESKF_VIO_BACKEND {
    /* 新一帧 IMU 量测数据输入，在已有 queue 的基础上进行 propagate */
    bool PropagateQueue::Propagate(const Vector3 &accel,
                                   const Vector3 &gyro,
                                   const fp64 timeStamp) {
        // 获取 IMU 和 Cam 的状态维度
        uint32_t imuSize = IMU_FULL_ERROR_STATE_SIZE;
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
            newItem->nominalState.q_wb = Quaternion(0.99875, 0.0499792, 0, 0);
            newItem->nominalState.v_wb.setZero();

            return true;
        }

        // 当序列不为空时，触发一次中值积分的 propagate 过程
        auto item_0 = this->items.back();
        std::shared_ptr<IMUPropagateQueueItem> item_1(new IMUPropagateQueueItem());
        this->items.emplace_back(item_1);
        item_1->accel = accel;
        item_1->gyro = gyro;
        item_1->timeStamp = timeStamp;
        // 中值积分递推名义状态
        this->PropagateMotionNominalState(item_0->nominalState, item_1->nominalState,
                                          item_0->accel, item_0->gyro,
                                          item_1->accel, item_1->gyro,
                                          this->bias_a, this->bias_g, this->gravity,
                                          static_cast<Scalar>(item_1->timeStamp - item_0->timeStamp));
        // 中值积分递推误差状态方程，更新 covariance
        // TODO: 

        // update fai matrix
        // TODO: 

        return true;
    }


    /* 中值积分法 propagate 运动相关名义状态 */
    void PropagateQueue::PropagateMotionNominalState(const IMUMotionState &state_0,
                                                     IMUMotionState &state_1,
                                                     const Vector3 &accel_0,
                                                     const Vector3 &gyro_0,
                                                     const Vector3 &accel_1,
                                                     const Vector3 &gyro_1,
                                                     const Vector3 &bias_a,
                                                     const Vector3 &bias_g,
                                                     const Vector3 &gravity_w,
                                                     const Scalar dt) {
        // 计算 gyro 中值，propagate 姿态
        Vector3 midGyro = Scalar(0.5) * (gyro_0 + gyro_1) - bias_g;
        Quaternion dq(Scalar(1), midGyro.x() * Scalar(0.5) * dt,
                                 midGyro.y() * Scalar(0.5) * dt,
                                 midGyro.z() * Scalar(0.5) * dt);
        state_1.q_wb = state_0.q_wb * dq;
        state_1.q_wb.normalize();
        // 计算 accel 中值，propagate 速度
        Vector3 midAccel = Scalar(0.5) * (state_0.q_wb * (accel_0 - bias_a) + state_1.q_wb * (accel_1 - bias_a)) - gravity_w;
        state_1.v_wb = state_0.v_wb + midAccel * dt;
        // propagate 位置
        state_1.p_wb = state_0.p_wb + Scalar(0.5) * (state_0.v_wb + state_1.v_wb) * dt;
    }


    /* 基于离散误差状态过程方程 propagate 完整误差状态以及误差状态对应协方差矩阵 */
    void PropagateQueue::PropagateFullErrorStateCovariance(const IMUFullState &errorState_0,
                                                           IMUFullState &errorState_1,
                                                           const Vector3 &accel_0,
                                                           const Vector3 &gyro_0,
                                                           const Vector3 &accel_1,
                                                           const Vector3 &gyro_1,
                                                           const Vector3 &bias_a,
                                                           const Vector3 &bias_g,
                                                           const Scalar dt) {
        
    }
}