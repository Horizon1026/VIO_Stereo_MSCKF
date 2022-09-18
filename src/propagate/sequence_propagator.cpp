/* 内部依赖 */
#include <include/propagate/sequence_propagator.hpp>
#include <include/utility/math_lib.hpp>
/* 外部依赖 */
// TODO: 
#include <iostream>

namespace ESKF_VIO_BACKEND {

    /* 重置过程方程 */
    void PropagateQueue::ResetProcessFunction(void) {
        this->F.setIdentity();
        this->G.setZero();
    }


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

            // 从零开始创建的 propagate 起点，名义运动状态归为初值
            newItem->nominalState.p_wb = this->initState.p_wb;
            newItem->nominalState.q_wb = this->initState.q_wb;
            newItem->nominalState.v_wb = this->initState.v_wb;

            // 重置过程方程
            this->ResetProcessFunction();

            return true;
        }

        // 当序列不为空时，触发一次中值积分的 propagate 过程
        auto item_0 = this->items.back();
        std::shared_ptr<IMUPropagateQueueItem> item_1(new IMUPropagateQueueItem());
        this->items.emplace_back(item_1);
        item_1->accel = accel;
        item_1->gyro = gyro;
        item_1->timeStamp = timeStamp;
        // 中值积分递推名义状态，同时计算出 accel 和 gyro 的中值
        Vector3 midAccel, midGyro;
        this->PropagateMotionNominalState(item_0->nominalState, item_1->nominalState,
                                          item_0->accel, item_0->gyro,
                                          item_1->accel, item_1->gyro,
                                          this->bias_a, this->bias_g, this->gravity,
                                          static_cast<Scalar>(item_1->timeStamp - item_0->timeStamp),
                                          midAccel, midGyro);

        // 中值积分递推误差状态方程，更新 covariance
        this->PropagateFullErrorStateCovariance(item_0->errorState, item_1->errorState,
                                                Matrix33(item_0->nominalState.q_wb),
                                                midAccel, midGyro,
                                                static_cast<Scalar>(item_1->timeStamp - item_0->timeStamp));

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
                                                     const Scalar dt,
                                                     Vector3 &midAccel,
                                                     Vector3 &midGyro) {
        // 计算 gyro 中值，propagate 姿态
        midGyro = Scalar(0.5) * (gyro_0 + gyro_1) - bias_g;
        Quaternion dq(Scalar(1), midGyro.x() * Scalar(0.5) * dt,
                                 midGyro.y() * Scalar(0.5) * dt,
                                 midGyro.z() * Scalar(0.5) * dt);
        state_1.q_wb = state_0.q_wb * dq;
        state_1.q_wb.normalize();
        // 计算 accel 中值，propagate 速度
        midAccel = Scalar(0.5) * (state_0.q_wb * (accel_0 - bias_a) + state_1.q_wb * (accel_1 - bias_a));
        state_1.v_wb = state_0.v_wb + (midAccel - gravity_w) * dt;
        // propagate 位置
        state_1.p_wb = state_0.p_wb + Scalar(0.5) * (state_0.v_wb + state_1.v_wb) * dt;
    }


    /* 基于离散误差状态过程方程 propagate 完整误差状态以及误差状态对应协方差矩阵 */
    void PropagateQueue::PropagateFullErrorStateCovariance(const IMUFullState &errorState_0,
                                                           IMUFullState &errorState_1,
                                                           const Matrix33 &R_wb_0,
                                                           const Vector3 &midAccel,
                                                           const Vector3 &midGyro,
                                                           const Scalar dt) {
        // midAccel = 0.5 * (R_wb0 * (accel0 - ba) + R_wb1 * (accel1 - ba))
        // midGyro = 0.5 * (gyro0 + gyro1) - bg
        // 构造离散过程方程 F 矩阵
        this->F.block<3, 3>(INDEX_P, INDEX_V) = dt * Matrix33::Identity();
        this->F.block<3, 3>(INDEX_V, INDEX_R) = - dt * SkewSymmetricMatrix(midAccel);
        this->F.block<3, 3>(INDEX_V, INDEX_BA) = - dt * R_wb_0;
        this->F.block<3, 3>(INDEX_V, INDEX_G) = dt * Matrix33::Identity();
        this->F.block<3, 3>(INDEX_R, INDEX_R) = - dt * SkewSymmetricMatrix(midGyro);
        this->F.block<3, 3>(INDEX_R, INDEX_BG) = - dt * Matrix33::Identity();

        // 构造离散过程方程 G 矩阵
        Scalar sqrt_dt = std::sqrt(dt);
        this->G.block<3, 3>(INDEX_V, INDEX_NA) = dt * R_wb_0;
        this->G.block<3, 3>(INDEX_R, INDEX_NG) = dt * Matrix33::Identity();
        this->G.block<3, 3>(INDEX_BA, INDEX_NWA) = sqrt_dt * Matrix33::Identity();
        this->G.block<3, 3>(INDEX_BG, INDEX_NWG) = sqrt_dt * Matrix33::Identity();

        // TODO: propagate delta_x with cov

    }


    /* 误差状态合并与分裂 */
    Eigen::Matrix<Scalar, IMU_FULL_ERROR_STATE_SIZE, 1> PropagateQueue::ErrorStateConvert(const IMUFullState &errorState) {
        Eigen::Matrix<Scalar, IMU_FULL_ERROR_STATE_SIZE, 1> delta_x;
        delta_x.segment<3>(INDEX_P) = errorState.p_wb;
        delta_x.segment<3>(INDEX_V) = errorState.v_wb;
        delta_x.segment<3>(INDEX_R) = errorState.theta_wb;
        delta_x.segment<3>(INDEX_BA) = errorState.bias_a;
        delta_x.segment<3>(INDEX_BG) = errorState.bias_g;
        delta_x.segment<3>(INDEX_G) = errorState.gravity;
        return delta_x;
    }


    IMUFullState PropagateQueue::ErrorStateConvert(const Eigen::Matrix<Scalar, IMU_FULL_ERROR_STATE_SIZE, 1> &delta_x) {
        IMUFullState errorState;
        errorState.p_wb = delta_x.segment<3>(INDEX_P);
        errorState.v_wb = delta_x.segment<3>(INDEX_V);
        errorState.theta_wb = delta_x.segment<3>(INDEX_R);
        errorState.bias_a = delta_x.segment<3>(INDEX_BA);
        errorState.bias_g = delta_x.segment<3>(INDEX_BG);
        errorState.gravity = delta_x.segment<3>(INDEX_G);
        return errorState;
    }


    /* 初始化过程噪声矩阵 */
    void PropagateQueue::InitializeProcessNoiseMatrix(const Scalar noise_accel,
                                                      const Scalar noise_gyro,
                                                      const Scalar random_walk_accel,
                                                      const Scalar random_walk_gyro) {
        this->Q.setIdentity();
        this->Q.diagonal().segment<3>(INDEX_NA) *= noise_accel;
        this->Q.diagonal().segment<3>(INDEX_NG) *= noise_gyro;
        this->Q.diagonal().segment<3>(INDEX_NWA) *= random_walk_accel;
        this->Q.diagonal().segment<3>(INDEX_NWG) *= random_walk_gyro;
    }
}