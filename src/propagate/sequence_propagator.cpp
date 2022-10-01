/* 内部依赖 */
#include <sequence_propagator.hpp>
#include <math_lib.hpp>
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
            newItem->imuCov.setZero();
            if (camSize > 0) {
                newItem->imuCamCov.setZero(imuSize, camSize);
                this->camCov.setZero(camSize, camSize);
                this->fai.setIdentity();
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
        this->PropagateMotionNominalState(item_0, item_1,
                                          this->bias_a, this->bias_g, this->gravity,
                                          midAccel, midGyro);

        // 中值积分递推误差状态方程，更新 covariance
        this->PropagateFullErrorStateCovariance(item_0, item_1, midAccel, midGyro);

        return true;
    }


    /* 中值积分法 propagate 运动相关名义状态 */
    void PropagateQueue::PropagateMotionNominalState(const std::shared_ptr<IMUPropagateQueueItem> &item_0,
                                                     std::shared_ptr<IMUPropagateQueueItem> &item_1,
                                                     const Vector3 &bias_a,
                                                     const Vector3 &bias_g,
                                                     const Vector3 &gravity_w,
                                                     Vector3 &midAccel,
                                                     Vector3 &midGyro) {
        Scalar dt = static_cast<Scalar>(item_1->timeStamp - item_0->timeStamp);
        // 计算 gyro 中值，propagate 姿态
        midGyro = Scalar(0.5) * (item_0->gyro + item_1->gyro) - bias_g;
        Quaternion dq = Utility::DeltaQ(midGyro * dt);
        item_1->nominalState.q_wb = item_0->nominalState.q_wb * dq;
        item_1->nominalState.q_wb.normalize();
        // 计算 accel 中值，propagate 速度
        midAccel = Scalar(0.5) * (item_0->nominalState.q_wb * (item_0->accel - bias_a) +
            item_1->nominalState.q_wb * (item_1->accel - bias_a));
        item_1->nominalState.v_wb = item_0->nominalState.v_wb + (midAccel - gravity_w) * dt;
        // propagate 位置
        item_1->nominalState.p_wb = item_0->nominalState.p_wb +
            Scalar(0.5) * (item_0->nominalState.v_wb + item_1->nominalState.v_wb) * dt;
    }


    /* 基于离散误差状态过程方程 propagate 完整误差状态以及误差状态对应协方差矩阵 */
    void PropagateQueue::PropagateFullErrorStateCovariance(const std::shared_ptr<IMUPropagateQueueItem> &item_0,
                                                           std::shared_ptr<IMUPropagateQueueItem> &item_1,
                                                           const Vector3 &midAccel,
                                                           const Vector3 &midGyro) {

        // midAccel = 0.5 * (R_wb0 * (accel0 - ba) + R_wb1 * (accel1 - ba))
        // midGyro = 0.5 * (gyro0 + gyro1) - bg
        Scalar dt = static_cast<Scalar>(item_1->timeStamp - item_0->timeStamp);
        Matrix33 R_wb_0(item_0->nominalState.q_wb);
        Matrix33 I3_dt = dt * Matrix33::Identity();
        Scalar sqrt_dt = std::sqrt(dt);
        Matrix33 I3_sqrt_dt = sqrt_dt * Matrix33::Identity();

        // 构造离散过程方程 F 矩阵
        this->F.block<3, 3>(INDEX_P, INDEX_V) = I3_dt;
        this->F.block<3, 3>(INDEX_V, INDEX_R) = - dt * Utility::SkewSymmetricMatrix(midAccel);
        this->F.block<3, 3>(INDEX_V, INDEX_BA) = - dt * R_wb_0;
        this->F.block<3, 3>(INDEX_V, INDEX_G) = - I3_dt;    // g_w > 0, propagate is -g (not +g)
        this->F.block<3, 3>(INDEX_R, INDEX_R) = - dt * Utility::SkewSymmetricMatrix(midGyro);
        this->F.block<3, 3>(INDEX_R, INDEX_BG) = - I3_dt;

        // 构造离散过程方程 G 矩阵
        this->G.block<3, 3>(INDEX_V, INDEX_NA) = dt * R_wb_0;
        this->G.block<3, 3>(INDEX_R, INDEX_NG) = I3_dt;
        this->G.block<3, 3>(INDEX_BA, INDEX_NWA) = I3_sqrt_dt;
        this->G.block<3, 3>(INDEX_BG, INDEX_NWG) = I3_sqrt_dt;

        // propagate 误差状态以及对应的 IMU 协方差矩阵
        Eigen::Matrix<Scalar, IMU_FULL_ERROR_STATE_SIZE, 1> error_x_0 = this->ErrorStateConvert(item_0->errorState);
        Eigen::Matrix<Scalar, IMU_FULL_ERROR_STATE_SIZE, 1> error_x_1 = this->F * error_x_0;
        item_1->errorState = this->ErrorStateConvert(error_x_1);
        item_1->imuCov = this->F * item_0->imuCov * this->F.transpose() + this->G * this->Q * this->G.transpose();

        // propagate IMU 与相机之间的协方差矩阵
        this->fai = this->F * this->fai;
        item_1->imuCamCov = this->fai * item_0->imuCamCov;
    }


    /* 重置过程方程 */
    void PropagateQueue::ResetProcessFunction(void) {
        this->F.setIdentity();
        this->G.setZero();
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