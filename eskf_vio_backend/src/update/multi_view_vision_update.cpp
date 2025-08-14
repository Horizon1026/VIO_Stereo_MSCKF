/* 内部依赖 */
#include <multi_view_vision_update.hpp>
#include <math_lib.hpp>
#include <log_api.hpp>
/* 外部依赖 */

namespace ESKF_VIO_BACKEND {
    /* 构造函数 */
    MultiViewVisionUpdate::MultiViewVisionUpdate() {
        this->features.reserve(100);
    }

    /* 执行一次 update 过程 */
    bool MultiViewVisionUpdate::Update(const fp64 timeStamp, const fp64 threshold) {
        // Step 1: 定位到 propagator 序列中对应时间戳的地方，提取对应时刻状态，清空在这之前的序列 item
        RETURN_FALSE_IF_FALSE(this->ResetPropagatorOrigin(timeStamp, threshold));
        // Step 2: 若此时滑动窗口已满，则判断次新帧是否为关键帧，确定边缘化策略。可以在此时给前端 thread 发送信号。
        RETURN_FALSE_IF_FALSE(this->DecideMargPolicy());
        // Step 3: 构造完整的协方差矩阵
        RETURN_FALSE_IF_FALSE(this->ConstructCovariance());
        // Step 4: 三角测量滑动窗口内所有特征点。已被测量过的选择迭代法，没被测量过的选择数值法。更新每一个点的三角测量质量，基于三角测量的质量，选择一定数量的特征点
        RETURN_FALSE_IF_FALSE(this->UpdateNewestFramePose());
        RETURN_FALSE_IF_FALSE(this->SelectGoodFeatures());
        if (this->features.empty()) {
            // Step 4.1: 扩展 state 的维度以及协方差矩阵，利用对应时刻的 propagate 名义状态给 frame pose 赋值
            RETURN_FALSE_IF_FALSE(this->ExpandCameraCovariance());
            // Step 4.2: 根据边缘化策略，选择跳过此步，或裁减 update 时刻点上的状态和协方差矩阵
            RETURN_FALSE_IF_FALSE(this->ReduceCameraCovariance());
            RETURN_FALSE_IF_FALSE(this->UpdateCovariance());
            // Step 4.3: 对于 propagate queue 中后续的已经存在的 items，从 update 时刻点重新 propagate，保持协方差维度一致
            RETURN_FALSE_IF_FALSE(this->propagator->Repropagate());
            return true;
        }
        // Step 5: 构造量测方程。其中包括计算雅可比、投影到左零空间、缩减维度、卡尔曼 update 误差和名义状态
        RETURN_FALSE_IF_FALSE(this->ConstructMeasurementFunction());
        RETURN_FALSE_IF_FALSE(this->UpdateState());
        // Step 3: 扩展 state 的维度以及协方差矩阵，利用对应时刻的 propagate 名义状态给 frame pose 赋值
        RETURN_FALSE_IF_FALSE(this->ExpandCameraCovariance());
        // Step 6: 根据边缘化策略，选择跳过此步，或裁减 update 时刻点上的状态和协方差矩阵
        RETURN_FALSE_IF_FALSE(this->ReduceCameraCovariance());
        RETURN_FALSE_IF_FALSE(this->UpdateCovariance());
        // Step 7: 对于 propagate queue 中后续的已经存在的 items，从 update 时刻点重新 propagate，保持协方差维度一致
        RETURN_FALSE_IF_FALSE(this->propagator->Repropagate());
        return true;
    }


    /* 定位到 propagator 序列中对应时间戳的地方，清空在这之前的序列 item */
    bool MultiViewVisionUpdate::ResetPropagatorOrigin(const fp64 timeStamp, const fp64 threshold) {
        RETURN_FALSE_IF_FALSE(this->propagator->ResetOrigin(timeStamp, threshold));
        // 清空后，this->propagator->items.front() 就是 update 对应时刻
        return true;
    }


    /* 若此时滑动窗口已满，则判断次新帧是否为关键帧，确定边缘化策略 */
    bool MultiViewVisionUpdate::DecideMargPolicy(void) {
        if (this->frameManager == nullptr) {
            this->margPolicy = NO_MARG;
            return false;
        }
        if (this->frameManager->NeedMarginalize()) {
            // 最新帧已经加入到 sliding window 中
            auto subnew = *std::next(this->frameManager->frames.rbegin());
            if (this->frameManager->IsKeyFrame(subnew)) {
                LogInfo(">> Need marg oldest key frame.");
                this->margPolicy = MARG_OLDEST;
            } else {
                LogInfo(">> Need marg subnew key frame.");
                this->margPolicy = MARG_SUBNEW;
            }
        } else {
            this->margPolicy = NO_MARG;
        }
        return true;
    }


    /* 拼接完整的协方差矩阵 */
    bool MultiViewVisionUpdate::ConstructCovariance(void) {
        // 最新帧等价于 IMU 的 pose，因此在 update 之前，不额外为他构造维度
        auto propagateItem = this->propagator->items.front();
        const uint32_t camExSize = (this->frameManager->extrinsics.size() + this->frameManager->frames.size() - 1) * 6;
        const uint32_t size = IMU_STATE_SIZE + camExSize;
        this->covariance.setZero(size, size);
        // 维度检查
        RETURN_FALSE_IF(propagateItem->imuExCamCov.cols() != camExSize);
        // 填充雅可比矩阵的既有部分
        this->covariance.topLeftCorner<IMU_STATE_SIZE, IMU_STATE_SIZE>() = propagateItem->imuCov;
        this->covariance.block(0, IMU_STATE_SIZE, IMU_STATE_SIZE, camExSize) = propagateItem->imuExCamCov;
        this->covariance.block(IMU_STATE_SIZE, 0, camExSize, IMU_STATE_SIZE) = propagateItem->imuExCamCov.transpose();
        this->covariance.block(IMU_STATE_SIZE, IMU_STATE_SIZE, camExSize, camExSize) = this->propagator->exCamCov;
        return true;
    }


    /* 利用 propagator 的结果更新最新帧的 frame pose */
    bool MultiViewVisionUpdate::UpdateNewestFramePose(void) {
        RETURN_FALSE_IF(this->frameManager->frames.empty());

        auto frame = this->frameManager->frames.back();
        auto propagateItem = this->propagator->items.front();
        // expand state，因在 frame manager 中已经扩充内存，因此此处只需要赋值
        frame->p_wb = propagateItem->nominalState.p_wb;
        frame->v_wb = propagateItem->nominalState.v_wb; // 速度不参与 update，但是需要赋值
        frame->q_wb = propagateItem->nominalState.q_wb;
        return true;
    }


    /* 利用所有观测，三角测量一个特征点 */
    bool MultiViewVisionUpdate::TrianglizeMultiFrame(const std::shared_ptr<Feature> &feature) {
        if (feature == nullptr) {
            return false;
        }
        static std::vector<Quaternion> all_q_wc(50);
        static std::vector<Vector3> all_p_wc(50);
        static std::vector<Vector2> all_norm(50);
        all_q_wc.clear();
        all_p_wc.clear();
        all_norm.clear();
        for (uint32_t i = 0; i < feature->observes.size(); ++i) {
            const uint32_t frameID = i + feature->firstFrameID;
            const auto frame = this->frameManager->GetFrame(frameID);
            const Quaternion &q_wb = frame->q_wb;
            const Vector3 &p_wb = frame->p_wb;
            for (auto it = feature->observes[i]->norms.begin(); it != feature->observes[i]->norms.end(); ++it) {
                const uint32_t cameraID = it->first;
                const Quaternion &q_bc = this->frameManager->extrinsics[cameraID].q_bc;
                const Vector3 &p_bc = this->frameManager->extrinsics[cameraID].p_bc;

                /* T_wc = T_wb * T_bc */
                /* [R_wc  t_wc] = [R_wb  t_wb]  *  [R_bc  t_bc] = [ R_wb * R_bc  R_wb * t_bc + t_wb]
                   [  0    1  ]   [  0     1 ]     [  0     1 ]   [      0                1        ] */
                /* T_wb = T_wc * T_bc.inv */
                /* [R_wb  t_wb] = [R_wc  t_wc]  *  [R_bc.t  - R_bc.t * t_bc] = [ R_wc * R_bc.t  - R_wc * R_bc.t * t_bc + t_wc]
                   [  0    1  ]   [  0     1 ]     [  0             1      ]   [      0                       1              ] */
                all_q_wc.emplace_back(Quaternion(q_wb * q_bc));
                all_p_wc.emplace_back(Vector3(q_wb * p_bc + p_wb));
                all_norm.emplace_back(it->second);
            }
        }

        if (this->trianglator->TrianglateAnalytic(all_q_wc, all_p_wc, all_norm, feature->p_w) == true) {
            feature->status = Feature::SOLVED;
            return true;
        } else {
            feature->status = Feature::UNSOLVED;
            return false;
        }
    }


    /* 三角测量在最新一帧中被追踪到的特征点。已被测量过的选择迭代法，没被测量过的选择数值法。*/
    /* 更新每一个点的三角测量质量，基于三角测量的质量，选择一定数量的特征点 */
    bool MultiViewVisionUpdate::SelectGoodFeatures(void) {
        auto frame = this->frameManager->frames.back();
        std::map<fp32, std::shared_ptr<Feature>> goodFeatures;  // <score, feature_ptr>
        for (auto it = frame->features.begin(); it != frame->features.end(); ++it) {
            auto feature = (*it).second;
            if (this->TrianglizeMultiFrame(feature) == true) {
                // TODO: 得分
                goodFeatures.insert(std::make_pair(std::rand(), feature));
            }
        }
        // 从高分到低分选择特征点，存入到 this->features 中
        this->features.clear();
        uint32_t cnt = this->maxUsedFeatures;
        for (auto it = goodFeatures.rbegin(); it != goodFeatures.rend(); ++it) {
            if (cnt) {
                this->features.emplace_back(it->second);
                --cnt;
            } else {
                break;
            }
        }
        if (this->features.empty()) {
            LogInfo(">> No good features for update.");
        } else {
            LogInfo(">> Use " << this->features.size() << " features for update.");
        }
        return true;
    }


    /* 构造量测方程。其中包括计算雅可比、投影到左零空间、缩减维度、卡尔曼 update 误差和名义状态 */
    bool MultiViewVisionUpdate::ConstructMeasurementFunction(void) {
        // Hx r
        /*  需要 update 的状态量包括以下，最新帧的 pose 就是 IMU pose
            [p_wb   v_wb   q_wb  ba  bg]  [p_bc0  q_bc0  p_bc1  q_bc1 ...]  [p_wb0  q_wb0  p_wb1  q_wb1 ...] */
        this->Hx_cols = IMU_STATE_SIZE + this->frameManager->extrinsics.size() * 6 +
            this->frameManager->frames.size() * 6 - 6;
        this->Hx_rows = 0;
        for (uint32_t i = 0; i < this->features.size(); ++i) {
            this->Hx_rows += this->features[i]->observeNum * 2 - 3;
        }
        this->Hx_r.setZero(this->Hx_rows, this->Hx_cols + 1);

        // 遍历每一个特征点，构造各自的量测方程，然后拼接到一起
        uint32_t rowsCount = 0;
        Matrix subHx_r;
        for (uint32_t i = 0; i < this->features.size(); ++i) {
            RETURN_FALSE_IF_FALSE(this->ConstructMeasurementFunction(this->features[i], subHx_r));
            const uint32_t subRow = this->features[i]->observeNum * 2 - 3;
            this->Hx_r.block(rowsCount, 0, subRow, this->Hx_cols + 1) = subHx_r;
            rowsCount += subRow;
        }
        // 对 Hx_r 的 Hx 部分进行 QR 分解，简化量测误差方程
        if (this->Hx_r.rows() > this->Hx_r.cols()) {
            Matrix &&Hx = this->Hx_r.block(0, 0, this->Hx_r.rows(), this->Hx_r.cols() - 1);
            Eigen::HouseholderQR<Matrix> QRSolver;
            QRSolver.compute(Hx);
            this->Hx_r = QRSolver.householderQ().transpose() * this->Hx_r;
            this->Hx_r.conservativeResize(this->Hx_r.cols() - 1, this->Hx_r.cols());
            this->Hx_rows = this->Hx_cols;
        }
        return true;
    }


    /* 构造一个特征点所有观测的量测方程，投影到左零空间 */
    bool MultiViewVisionUpdate::ConstructMeasurementFunction(const std::shared_ptr<Feature> &feature,
                                                             Matrix &Hx_r) {
        Matrix Hf_Hx_r;
        Hf_Hx_r.setZero(feature->observeNum * 2, this->Hx_cols + 4);

        // 遍历每一帧中的每一个观测，填充 Hf_Hx_r
        uint32_t row = 0;
        for (uint32_t i = 0; i < feature->observes.size(); ++i) {
            uint32_t frameID = i + feature->firstFrameID;
            auto frame = this->frameManager->GetFrame(frameID);
            RETURN_FALSE_IF_TRUE(frame == nullptr);
            uint32_t frameIdx = frameID - this->frameManager->frames.front()->id;
            for (auto it = feature->observes[i]->norms.begin(); it != feature->observes[i]->norms.end(); ++it) {
                uint32_t cameraIdx = it->first;
                // 提取出计算雅可比与残差的相关变量
                Vector3 &p_bc = this->frameManager->extrinsics[cameraIdx].p_bc;
                Matrix33 R_cb(this->frameManager->extrinsics[cameraIdx].q_bc.inverse());
                Vector3 &p_wb = frame->p_wb;
                Matrix33 R_bw(frame->q_wb.inverse());
                Vector3 &p_w = feature->p_w;
                Vector2 &measure = it->second;
                // 计算残差和雅可比矩阵，并填充到 Hf_Hx_r 对应位置
                Vector3 p_b = R_bw * (p_w - p_wb);
                Vector3 p_c = R_cb * (p_b - p_bc);
                if (std::fabs(p_c.z()) > ZERO) {
                    // 计算残差
                    Vector2 residual = measure - (p_c / p_c.z()).head<2>();
                    Hf_Hx_r.block<2, 1>(row, Hf_Hx_r.cols() - 1) = residual;
                    // 计算雅可比（归一化平面误差，对 p_c 求导）
                    Matrix23 jacobian_2d_3d;
                    jacobian_2d_3d << 1. / p_c(2), 0, - p_c(0) / (p_c(2) * p_c(2)),
                                      0, 1. / p_c(2), - p_c(1) / (p_c(2) * p_c(2));
                    // 计算雅可比（归一化平面误差，对 p_w 求导）
                    Matrix23 jacobian_r_p_w = jacobian_2d_3d * R_cb * R_bw;
                    Hf_Hx_r.block<2, 3>(row, 0) = jacobian_r_p_w;
                    // 计算雅可比（归一化平面误差，对 camera pose 求导）
                    uint32_t col = 0;
                    if (frameID == this->frameManager->frames.back()->id) {
                        col = 3;
                        Matrix23 jacobian_r_p_wb = jacobian_2d_3d * (- R_cb * R_bw);
                        Hf_Hx_r.block<2, 3>(row, col + INDEX_P) = jacobian_r_p_wb;
                        Matrix23 jacobian_r_R_wb = jacobian_2d_3d * R_cb * Utility::SkewSymmetricMatrix(p_b);
                        Hf_Hx_r.block<2, 3>(row, col + INDEX_R) = jacobian_r_R_wb;
                    } else {
                        col = 3 + IMU_STATE_SIZE + this->frameManager->extrinsics.size() * 6 + frameIdx * 6;
                        Matrix23 jacobian_r_p_wb = jacobian_2d_3d * (- R_cb * R_bw);
                        Hf_Hx_r.block<2, 3>(row, col) = jacobian_r_p_wb;
                        Matrix23 jacobian_r_R_wb = jacobian_2d_3d * R_cb * Utility::SkewSymmetricMatrix(p_b);
                        Hf_Hx_r.block<2, 3>(row, col + 3) = jacobian_r_R_wb;
                    }
                    // 计算雅可比（归一化平面误差，对 camera extrinsic 求导）
                    col = 3 + IMU_STATE_SIZE + cameraIdx * 6;
                    Matrix23 jacobian_r_p_bc = jacobian_2d_3d * (- R_cb);
                    Hf_Hx_r.block<2, 3>(row, col) = jacobian_r_p_bc;
                    Matrix23 jacobian_r_R_bc = jacobian_2d_3d * Utility::SkewSymmetricMatrix(p_c);
                    Hf_Hx_r.block<2, 3>(row, col + 3) = jacobian_r_R_bc;
                }
                row += 2;
            }
        }

        // 对 Hf_Hx_r 中的 Hf 部分进行 QR 分解，同步变化到 Hx 和 r 上
        Matrix &&Hf = Hf_Hx_r.block(0, 0, Hf_Hx_r.rows(), 3);
        Eigen::HouseholderQR<Matrix> QRSolver;
        QRSolver.compute(Hf);
        Hf_Hx_r = QRSolver.householderQ().transpose() * Hf_Hx_r;

        // 裁剪出投影结果
        Hx_r = Hf_Hx_r.block(3, 3, Hf_Hx_r.rows() - 3, Hf_Hx_r.cols() - 3);
        // TODO: 添加 Huber 核函数，调整此特征点的 scale
        // Hx_r *= scale;
        return true;
    }


    /* 更新误差状态和名义状态 */
    bool MultiViewVisionUpdate::UpdateState(void) {
        // Step 1: 执行卡尔曼滤波的 update 过程
        // 计算卡尔曼增益
        Matrix &&H = this->Hx_r.block(0, 0, this->Hx_r.rows(), this->Hx_r.cols() - 1);
        Vector &&r = this->Hx_r.block(0, this->Hx_r.cols() - 1, this->Hx_r.rows(), 1);
        Matrix PHt = this->covariance * H.transpose();
        const Scalar R = this->measureNoise * this->measureNoise;
        this->meas_covariance = H * PHt;
        this->meas_covariance.diagonal().array() += R;     // S = H * P * Ht + R
        this->K = PHt * Utility::Inverse(this->meas_covariance);    // K = P * Ht * Sinv
        // 更新误差状态向量
        this->delta_x = this->K * r;       // dx = K * r
        Matrix I_KH = - this->K * H;
        I_KH.diagonal().array() += Scalar(1);
        static Matrix newCov;
        newCov = I_KH * this->covariance;   // P = (I - K * H) * P
        // newCov = I_KH * this->covariance * I_KH.transpose() +
        //     this->K * this->K.transpose() * R;     // P = (I - K * H) * P * (I - K * H).t + K * R * K.t

        // check cov
        for (uint32_t i = 0; i < newCov.rows(); ++i) {
            if (newCov(i, i) < 0) {
                LogError("Full cov has items < 0 in diagnal!");
                LogDebug("diagnal of P before update is\n" << this->covariance.diagonal().transpose());
                LogDebug("diagnal of P after update is\n" << newCov.diagonal().transpose());
                break;
            }
        }
        this->covariance = Scalar(0.5) * (newCov + newCov.transpose());

        // Step 2: 结果更新到 propagator
        // 更新名义状态
        this->propagator->items.front()->nominalState.p_wb += this->delta_x.segment<3>(INDEX_P);
        this->propagator->items.front()->nominalState.v_wb += this->delta_x.segment<3>(INDEX_V);
        this->propagator->items.front()->nominalState.q_wb = this->propagator->items.front()->nominalState.q_wb *
            Utility::DeltaQ(this->delta_x.segment<3>(INDEX_R));
        this->propagator->items.front()->nominalState.q_wb.normalize();
        // 更新 bias
        this->propagator->bias_a += this->delta_x.segment<3>(INDEX_BA);
        this->propagator->bias_g += this->delta_x.segment<3>(INDEX_BG);
        // 更新相机与 imu 之间的标定参数
        for (uint32_t i = 0; i < this->frameManager->extrinsics.size(); ++i) {
            this->frameManager->extrinsics[i].p_bc += this->delta_x.segment<3>(IMU_STATE_SIZE + i * 6);
            this->frameManager->extrinsics[i].q_bc = this->frameManager->extrinsics[i].q_bc *
                Utility::DeltaQ(this->delta_x.segment<3>(IMU_STATE_SIZE + i * 6 + 3));
            this->frameManager->extrinsics[i].q_bc.normalize();
        }
        // 更新关键帧的位姿（速度不用管）
        uint32_t idx = 0;
        const uint32_t offset = IMU_STATE_SIZE + this->frameManager->extrinsics.size() * 6;
        for (auto it = this->frameManager->frames.begin(); it != this->frameManager->frames.end(); ++it) {
            if ((*it)->id == this->frameManager->frames.back()->id) {
                (*it)->p_wb = this->propagator->items.front()->nominalState.p_wb;
                (*it)->v_wb = this->propagator->items.front()->nominalState.v_wb;
                (*it)->q_wb = this->propagator->items.front()->nominalState.q_wb;
                break;
            }
            (*it)->p_wb += this->delta_x.segment<3>(offset + idx * 6);
            (*it)->q_wb = (*it)->q_wb * Utility::DeltaQ(this->delta_x.segment<3>(offset + idx * 6 + 3));
            (*it)->q_wb.normalize();
            ++idx;
        }

        return true;
    }


    /* 扩展 state 的维度以及协方差矩阵，利用对应时刻的 propagate 名义状态给 frame pose 赋值 */
    bool MultiViewVisionUpdate::ExpandCameraCovariance(void) {
        auto frame = this->frameManager->frames.back();
        auto propagateItem = this->propagator->items.front();
        // expand covariance
        /*  用于扩充维度的雅可比矩阵，本质上就是 new state/full state 的雅可比矩阵
            full_cov = [ I(15 + 6m + 6n) ] * old_full_cov(C) * [ I(15 + 6m + 6n) ].T
                       [        J        ]                     [        J        ]
                     = [ C  ] * [ I  J.T ]
                       [ JC ]
                     = [ C     CJ.T ]
                       [ JC   JCJ.T ]

            when camera state is q_wb/p_wb, things become easy
                      p  v  theta   ba  bg  p_bc0  q_bc0  p_bc1  q_bc1  ...  Twb0  Twb1  ...
            as J is [ I  0    0     0   0     0      0      0      0    ...    0     0   ...  ]  ->  p_wb
                    [ 0  0    I     0   0     0      0      0      0    ...    0     0   ...  ]  ->  q_wb      */
        uint32_t camExSize = (this->frameManager->extrinsics.size() + this->frameManager->frames.size()) * 6;
        uint32_t size = IMU_STATE_SIZE + camExSize;
        this->covariance.conservativeResize(size, size);
        this->covariance.block(0, size - 6, size, 6).setZero();
        this->covariance.block(size - 6, 0, 6, size).setZero();
        // 构造雅可比矩阵（不需要全部构造，只需要不为零的部分就行了）
        this->expand_J.setZero(6, IMU_STATE_SIZE);
        this->expand_J.block<3, 3>(0, INDEX_P).setIdentity();
        this->expand_J.block<3, 3>(3, INDEX_R).setIdentity();
        // 定位协方差矩阵中的 imu/cam 部分
        Matrix &&imuCov = this->covariance.block(0, 0, IMU_STATE_SIZE, IMU_STATE_SIZE);
        Matrix &&imuExCamCov = this->covariance.block(0, IMU_STATE_SIZE, IMU_STATE_SIZE, camExSize - 6);
        // 填充雅可比矩阵的扩维部分
        this->covariance.block(size - 6, 0, 6, IMU_STATE_SIZE) = this->expand_J * imuCov;
        this->covariance.block(size - 6, IMU_STATE_SIZE, 6, camExSize - 6) = this->expand_J * imuExCamCov;
        this->covariance.block(0, size - 6, size - 6, 6) = this->covariance.block(size - 6, 0, 6, size - 6).transpose();
        this->covariance.block(size - 6, size - 6, 6, 6) = this->expand_J * imuCov * this->expand_J.transpose();
        return true;
    }


    /* 更新协方差矩阵到 propagator 中 */
    bool MultiViewVisionUpdate::UpdateCovariance(void) {
        uint32_t camExSize = this->covariance.cols() - IMU_STATE_SIZE;
        auto propagateItem = this->propagator->items.front();
        propagateItem->imuCov = this->covariance.block(0, 0, IMU_STATE_SIZE, IMU_STATE_SIZE);
        propagateItem->imuExCamCov = this->covariance.block(0, IMU_STATE_SIZE, IMU_STATE_SIZE, camExSize);
        this->propagator->exCamCov = this->covariance.block(IMU_STATE_SIZE, IMU_STATE_SIZE, camExSize, camExSize);
        return true;
    }


    /* 裁减 update 时刻点上的状态和协方差矩阵 */
    bool MultiViewVisionUpdate::ReduceCameraCovariance(void) {
        const uint32_t exSize = this->frameManager->extrinsics.size() * 6;
        const uint32_t camSize = this->frameManager->frames.size() * 6;
        const uint32_t camExSize = exSize + camSize;
        const uint32_t imuSize = IMU_STATE_SIZE;
        const uint32_t imuExSize = imuSize + exSize;
        const uint32_t fullSize = imuSize + camExSize;
        static Matrix temp;
        switch (this->margPolicy) {
            case NO_MARG:
                return true;
            case MARG_NEWEST:
                this->covariance.conservativeResize(fullSize - 6, fullSize - 6);
                return true;
            case MARG_SUBNEW:
                this->covariance.block(0, fullSize - 12, fullSize - 12, 6) = this->covariance.block(0, fullSize - 6, fullSize - 12, 6);
                this->covariance.block(fullSize - 12, 0, 6, fullSize - 12) = this->covariance.block(fullSize - 6, 0, 6, fullSize - 12);
                this->covariance.block(fullSize - 12, fullSize - 12, 6, 6) = this->covariance.block(fullSize - 6, fullSize - 6, 6, 6);
                this->covariance.conservativeResize(fullSize - 6, fullSize - 6);
                return true;
            case MARG_OLDEST:
                temp = this->covariance.block(0, imuExSize + 6, imuExSize, camSize - 6);
                this->covariance.block(0, imuExSize, imuExSize, camSize - 6) = temp;
                this->covariance.block(imuExSize, 0, camSize - 6, imuExSize) = temp.transpose();
                temp = this->covariance.bottomRightCorner(camSize - 6, camSize - 6);
                this->covariance.block(imuExSize, imuExSize, camSize - 6, camSize - 6) = temp;
                this->covariance.conservativeResize(fullSize - 6, fullSize - 6);
                return true;
            default:
                return false;
        }
    }
}