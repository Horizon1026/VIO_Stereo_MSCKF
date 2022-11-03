/* 内部依赖 */
#include <sequence_propagator.hpp>
#include <math_lib.hpp>
#include <log_api.hpp>
/* 外部依赖 */

namespace ESKF_VIO_BACKEND {
    /* 新一帧 IMU 量测数据输入，在已有 queue 的基础上进行 propagate */
    bool PropagateQueue::Propagate(const Vector3 &accel,
                                   const Vector3 &gyro,
                                   const fp64 timeStamp) {
        // 设置 IMU 和 Cam 的状态维度，初始化完成后，滑动窗口内应当仅仅保留 frame i
        // 虽然初始化时，窗口内肯定有两帧，但是初始化后需要进行一次 update，期间会有 state expand，因此初始化成只有一帧的情况
        const uint32_t imuSize = IMU_STATE_SIZE;
        const uint32_t camExSize = this->propagator->extrinsics.size() * 6 + 6;

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
            if (camExSize > 0) {
                newItem->imuExCamCov.setZero(imuSize, camExSize);
                this->exCamCov.setZero(camExSize, camExSize);
            }

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
                                          this->bias_a, this->bias_g, IMUFullState::gravity_w,
                                          midAccel, midGyro);
        // 中值积分递推误差状态方程，更新 covariance
        this->PropagateFullErrorStateCovariance(item_0, item_1, midAccel, midGyro);
        return true;
    }


    /* 从头开始重新 propagate */
    bool PropagateQueue::Repropagate(void) {
        for (auto it = this->items.begin(); it != this->items.end(); ++it) {
            if (std::next(it) == this->items.end()) {
                break;
            }
            auto item_0 = *it;
            auto item_1 = *std::next(it);
            // 中值积分递推名义状态，同时计算出 accel 和 gyro 的中值
            Vector3 midAccel, midGyro;
            this->PropagateMotionNominalState(item_0, item_1,
                                              this->bias_a, this->bias_g, IMUFullState::gravity_w,
                                              midAccel, midGyro);
            // 中值积分递推误差状态方程，更新 covariance
            this->PropagateFullErrorStateCovariance(item_0, item_1, midAccel, midGyro);
        }
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
        this->F.block<3, 3>(INDEX_V, INDEX_R) = - dt * R_wb_0 * Utility::SkewSymmetricMatrix(midAccel);
        this->F.block<3, 3>(INDEX_V, INDEX_BA) = - dt * R_wb_0;
        this->F.block<3, 3>(INDEX_R, INDEX_R) = Matrix33::Identity() - dt * Utility::SkewSymmetricMatrix(midGyro);
        this->F.block<3, 3>(INDEX_R, INDEX_BG) = - I3_dt;

        // 构造离散过程方程 G 矩阵
        this->G.block<3, 3>(INDEX_V, INDEX_NA) = dt * R_wb_0;
        this->G.block<3, 3>(INDEX_R, INDEX_NG) = I3_dt;
        this->G.block<3, 3>(INDEX_BA, INDEX_NWA) = I3_sqrt_dt;
        this->G.block<3, 3>(INDEX_BG, INDEX_NWG) = I3_sqrt_dt;

        // propagate 误差状态对应的 IMU 协方差矩阵
        item_1->imuCov = this->F * item_0->imuCov * this->F.transpose() + this->G * this->Q * this->G.transpose();

        // check cov
        for (uint32_t i = 0; i < item_1->imuCov.rows(); ++i) {
            if (item_1->imuCov(i, i) < 0) {
                LogError("IMU cov has items < 0 in diagnal!");
                LogDebug("diagnal of item_0->imuCov is\n" << item_0->imuCov.diagonal().transpose());
                LogDebug("diagnal of item_1->imuCov is\n" << item_1->imuCov.diagonal().transpose());
                break;
            }
        }

        // propagate IMU 与相机之间的协方差矩阵
        item_1->imuExCamCov = this->F * item_0->imuExCamCov;

        /* 协方差矩阵 propagate 过程 */
        /*
                   [ imu-imu  imu-ex  imu-cam ]
        full_cov = [ ex-imu   ex-ex   ex-cam  ]
                   [ cam-imu  cam-ex  cam-cam ]

        cov <- F * cov * F.T + G * Q * G.T  =>  [ F  0 ] * [ A    B ] * [ F.T  0 ]  =  [ F * A * F.T  F * B ]
                                                [ 0  I ]   [ B.T  C ]   [  0   I ]     [  B.T * F.T     C   ]
        
        */
    }


    /* 重置过程方程 */
    void PropagateQueue::ResetProcessFunction(void) {
        this->F.setIdentity();
        this->G.setZero();
    }


    /* 误差状态合并与分裂 */
    Eigen::Matrix<Scalar, IMU_STATE_SIZE, 1> PropagateQueue::ErrorStateConvert(const IMUFullState &errorState) {
        Eigen::Matrix<Scalar, IMU_STATE_SIZE, 1> delta_x;
        delta_x.segment<3>(INDEX_P) = errorState.p_wb;
        delta_x.segment<3>(INDEX_V) = errorState.v_wb;
        delta_x.segment<3>(INDEX_R) = errorState.theta_wb;
        delta_x.segment<3>(INDEX_BA) = errorState.bias_a;
        delta_x.segment<3>(INDEX_BG) = errorState.bias_g;
        return delta_x;
    }


    IMUFullState PropagateQueue::ErrorStateConvert(const Eigen::Matrix<Scalar, IMU_STATE_SIZE, 1> &delta_x) {
        IMUFullState errorState;
        errorState.p_wb = delta_x.segment<3>(INDEX_P);
        errorState.v_wb = delta_x.segment<3>(INDEX_V);
        errorState.theta_wb = delta_x.segment<3>(INDEX_R);
        errorState.bias_a = delta_x.segment<3>(INDEX_BA);
        errorState.bias_g = delta_x.segment<3>(INDEX_BG);
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


    /* 重置序列初始时刻点 */
    bool PropagateQueue::ResetOrigin(const fp64 timeStamp, const fp64 threshold) {
        if (this->items.empty()) {
            return false;
        }
        while ((timeStamp - this->items.front()->timeStamp) > threshold) {
            this->items.pop_front();
            if (this->items.empty()) {
                return false;
            }
        }
        LogInfo(">> Propagator reset at " << timeStamp << "s, first item time stamp is " <<
            this->items.front()->timeStamp << "s, " << this->items.size() << " items maintained.");
        return true;
    }
}