/* 内部依赖 */
#include <multi_view_vision_update.hpp>
#include <log_api.hpp>
/* 外部依赖 */

namespace ESKF_VIO_BACKEND {
    /* 执行一次 update 过程 */
    bool MultiViewVisionUpdate::Update(const fp64 timeStamp, const fp64 threshold) {
        // Step 1: 定位到 propagator 序列中对应时间戳的地方，提取对应时刻状态，清空在这之前的序列 item
        RETURN_FALSE_IF_FALSE(this->ResetPropagatorOrigin(timeStamp, threshold));
        // Step 2: 若此时滑动窗口已满，则判断次新帧是否为关键帧，确定边缘化策略。可以在此时给前端 thread 发送信号。
        RETURN_FALSE_IF_FALSE(this->DecideMargPolicy());
        // Step 3: 扩展 state 的维度以及协方差矩阵，利用对应时刻的 propagate 名义状态给 frame pose 赋值
        RETURN_FALSE_IF_FALSE(this->ExpandCameraStateCovariance());
        // Step 4: 三角测量滑动窗口内所有特征点。已被测量过的选择迭代法，没被测量过的选择数值法。更新每一个点的三角测量质量，基于三角测量的质量，选择一定数量的特征点
        RETURN_FALSE_IF_FALSE(this->SelectGoodFeatures(30));
        // Step 5: 构造量测方程。其中包括计算雅可比、投影到左零空间、缩减维度、卡尔曼 update 误差和名义状态
        RETURN_FALSE_IF_FALSE(this->ConstructMeasurementFunction());
        RETURN_FALSE_IF_FALSE(this->UpdateState());
        // Step 6: 根据边缘化策略，选择跳过此步，或裁减 update 时刻点上的状态和协方差矩阵
        RETURN_FALSE_IF_FALSE(this->ReduceCameraStateCovariance());
        RETURN_FALSE_IF_FALSE(this->UpdateCovariance());
        // Step 7: 对于 propagate queue 中后续的已经存在的 items，从 update 时刻点重新 propagate
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
            if (this->frameManager->IsKeyFrame(subnew)) {   // iskeyframe() TODO:
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


    /* 扩展 state 的维度以及协方差矩阵，利用对应时刻的 propagate 名义状态给 frame pose 赋值 */
    bool MultiViewVisionUpdate::ExpandCameraStateCovariance(void) {
        auto frame = this->frameManager->frames.back();
        auto propagateItem = this->propagator->items.front();
        // expand state，因在 frame manager 中已经扩充内存，因此此处只需要赋值
        frame->p_wb = propagateItem->nominalState.p_wb;
        frame->v_wb = propagateItem->nominalState.v_wb; // 速度不参与 update，但是需要赋值
        frame->q_wb = propagateItem->nominalState.q_wb;
        // expand covariance
        /*  用于扩充维度的雅可比矩阵，本质上就是 new state/full state 的雅可比矩阵
            full_cov = [ I(15 + 6m + 6n) ] * old_full_cov(C) * [ I(15 + 6m + 6n) ].T
                       [        J        ]                     [        J        ]
                     = [ C  ] * [ I  J.T ]
                       [ JC ]
                     = [ C     CJ.T ]
                       [ JC   JCJ.T ]
 
            when camera state is q_wc/p_wc, use q_bc0/p_bc0 to calculate from q_wb/p_wb
                q_wc = q_wb * q_bc0;
                p_wc = p_wb + q_wb * p_bc0
                      p  v        theta          ba  bg  p_bc0  q_bc0  p_bc1  q_bc1  ...  Twb0  Twb1  ...
            as J is [ 0  0         R_wb          0   0     0      I      0      0    ...    0     0   ...  ]  ->  q_wc(theta)
                    [ I  0  - R_wb * hat(p_bc)   0   0     I      0      0      0    ...    0     0   ...  ]  ->  p_wc

            when camera state is q_wb/p_wb, things become easy
                      p  v  theta   ba  bg  p_bc0  q_bc0  p_bc1  q_bc1  ...  Twb0  Twb1  ...
            as J is [ 0  0    I     0   0     0      0      0      0    ...    0     0   ...  ]  ->  q_wb
                    [ I  0    0     0   0     0      0      0      0    ...    0     0   ...  ]  ->  p_wb       */
        uint32_t camExSize = (this->frameManager->extrinsics.size() + this->frameManager->frames.size()) * 6;
        uint32_t size = IMU_STATE_SIZE + camExSize;
        this->covariance.setZero(size, size);
        // 填充雅可比矩阵的既有部分
        this->covariance.topLeftCorner<IMU_STATE_SIZE, IMU_STATE_SIZE>() = propagateItem->imuCov;
        this->covariance.block(0, IMU_STATE_SIZE, IMU_STATE_SIZE, camExSize - 6) = propagateItem->imuCamCov;
        this->covariance.block(IMU_STATE_SIZE, 0, camExSize - 6, IMU_STATE_SIZE) = propagateItem->imuCamCov.transpose();
        this->covariance.block(IMU_STATE_SIZE, IMU_STATE_SIZE, camExSize - 6, camExSize - 6) = this->propagator->camCov;
        // 构造雅可比矩阵（不需要全部构造，只需要不为零的部分就行了）
        this->expand_J.setZero(6, IMU_STATE_SIZE);
        this->expand_J.block(3, INDEX_P, 3, 3).setIdentity();
        this->expand_J.block(0, INDEX_R, 3, 3).setIdentity();
        // 填充雅可比矩阵的扩维部分
        this->covariance.block(size - 6, 0, 6, IMU_STATE_SIZE) = this->expand_J * propagateItem->imuCov;
        this->covariance.block(size - 6, IMU_STATE_SIZE, 6, camExSize - 6) = this->expand_J * propagateItem->imuCamCov;
        this->covariance.block(0, size - 6, size - 6, 6) = this->covariance.block(size - 6, 0, 6, size - 6).transpose();
        this->covariance.block(size - 6, size - 6, 6, 6) = this->expand_J * propagateItem->imuCov * this->expand_J.transpose();
        return true;
    }


    /* 三角测量在最新一帧中被追踪到的特征点。已被测量过的选择迭代法，没被测量过的选择数值法。*/
    /* 更新每一个点的三角测量质量，基于三角测量的质量，选择一定数量的特征点 */
    bool MultiViewVisionUpdate::SelectGoodFeatures(const uint32_t num) {
        auto frame = this->frameManager->frames.back();
        std::map<fp32, std::shared_ptr<Feature>> goodFeatures;  // <score, feature_ptr>
        for (auto it = frame->features.begin(); it != frame->features.end(); ++it) {
            if ((*it).second->status == Feature::SOLVED) {
                // 迭代法三角测量
                // TODO: 三角测量成功后，连同打分塞到 goodFeatures
            } else {
                // 解析法三角测量
                // TODO：同上
            }
        }
        // 从高分到低分选择特征点，存入到 this->features 中
        this->features.clear();
        uint32_t cnt = num;
        for (auto it = goodFeatures.begin(); it != goodFeatures.end(); ++it) {
            if (cnt) {
                this->features.emplace_back(it->second);
                --cnt;
            } else {
                break;
            }
        }
        return true;
    }


    /* 构造量测方程。其中包括计算雅可比、投影到左零空间、缩减维度、卡尔曼 update 误差和名义状态 */
    bool MultiViewVisionUpdate::ConstructMeasurementFunction(void) {
        /*  需要 update 的状态量包括：
            p_wb   v_wb   q_wb  ba  bg  p_bc0  q_bc0  p_bc1  q_bc1 ... p_wb0  q_wb0  p_wb1  q_wb1 ... */
        this->Hx_cols = IMU_STATE_SIZE + this->frameManager->extrinsics.size() * 6 +
            this->frameManager->frames.size() * 6;
        this->Hx_rows = 0;
        for (uint32_t i = 0; i < this->features.size(); ++i) {
            this->Hx_rows += this->features[i]->observeNum;
        }
        this->Hx_r.setZero(this->Hx_rows, this->Hx_cols + 1);
        // 遍历每一个特征点，构造各自的量测方程，然后拼接到一起
        uint32_t rowsCount = 0;
        for (uint32_t i = 0; i < this->features.size(); ++i) {
            Matrix &&subHx_r = this->Hx_r.block(rowsCount, 0, this->features[i]->observeNum * 2 - 3, this->Hx_cols);
            RETURN_FALSE_IF_FALSE(this->ConstructMeasurementFunction(this->features[i], subHx_r));
            rowsCount += this->features[i]->observeNum * 2 - 3;
        }
        // 对 Hx_r 的 Hx 部分进行 QR 分解，简化量测方程
        // TODO:
        return true;
    }


    /* 构造一个特征点所有观测的量测方程，投影到左零空间 */
    bool MultiViewVisionUpdate::ConstructMeasurementFunction(const std::shared_ptr<Feature> &feature,
                                                             Matrix &Hx_r) {
        Matrix Hx_Hf_r;
        Hx_Hf_r.setZero(feature->observeNum * 2, this->Hx_cols + 4);

        // 遍历每一帧中的每一个观测，填充 Hx_Hf_r
        uint32_t rowsCount = 0;
        for (uint32_t i = 0; i < feature->observes.size(); ++i) {
            uint32_t frameID = i + feature->firstFrameID;
            auto frame = this->frameManager->GetFrame(frameID);
            RETURN_FALSE_IF_TRUE(frame == nullptr);
            for (auto it = feature->observes[i]->norms.begin(); it != feature->observes[i]->norms.end(); ++it) {
                // 提取出计算雅可比与残差的相关变量
                Vector3 &p_bc = this->frameManager->extrinsics[it->first].p_bc;
                Matrix33 R_bc(this->frameManager->extrinsics[it->first].q_bc);
                Vector3 &p_wb = frame->p_wb;
                Matrix33 R_wb(frame->q_wb);
                Vector3 &p_w = feature->p_w;
                Vector2 &norm = it->second;
                // 计算残差和雅可比，并填充到对应位置
                // TODO:
                rowsCount += 2;
            }
        }
        // 对 Hx_Hf_r 中的 Hf 部分进行 QR 分解，同步变化到 Hx 和 r 上
        // TODO:

        // 裁剪出投影结果
        Hx_r.block(0, 0, Hx_r.rows(), Hx_r.cols() - 1) = Hx_Hf_r.block(3, 0, Hx_Hf_r.rows() - 3, Hx_Hf_r.cols() - 4);
        Hx_r.block(0, Hx_r.cols() - 1, Hx_r.rows(), 1) = Hx_Hf_r.block(3, 0, Hx_Hf_r.rows() - 3, 1);
        return true;
    }


    /* 更新误差状态和名义状态 */
    bool MultiViewVisionUpdate::UpdateState(void) {
        // TODO:
        return true;
    }


    /* 更新协方差矩阵到 propagator 中 */
    bool MultiViewVisionUpdate::UpdateCovariance(void) {
        auto propagateItem = this->propagator->items.front();
        uint32_t camExSize = this->covariance.cols() - IMU_STATE_SIZE;
        propagateItem->imuCamCov = this->covariance.block(0, IMU_STATE_SIZE, IMU_STATE_SIZE, camExSize);
        this->propagator->camCov = this->covariance.block(IMU_STATE_SIZE, IMU_STATE_SIZE, camExSize, camExSize);
        return true;
    }


    /* 裁减 update 时刻点上的状态和协方差矩阵 */
    bool MultiViewVisionUpdate::ReduceCameraStateCovariance(void) {
        uint32_t exSize = this->frameManager->extrinsics.size() * 6;
        uint32_t camSize = this->frameManager->frames.size() * 6;
        uint32_t camExSize = exSize + camSize;
        uint32_t imuSize = IMU_STATE_SIZE;
        Matrix temp;
        switch (this->margPolicy) {
            case NO_MARG:
                return true;
            case MARG_NEWEST:
                this->covariance.conservativeResize(imuSize + camExSize - 6, imuSize + camExSize - 6);
                return true;
            case MARG_SUBNEW:
                this->covariance.block(0, camExSize - 12, camExSize - 12, 6) = this->covariance.block(0, camExSize - 6, camExSize - 12, 6);
                this->covariance.block(camExSize - 12, 0, 6, camExSize - 12) = this->covariance.block(camExSize - 6, 0, 6, camExSize - 12);
                this->covariance.block(camExSize - 12, camExSize - 12, 6, 6) = this->covariance.block(camExSize - 6, camExSize - 6, 6, 6);
                this->covariance.conservativeResize(imuSize + camExSize - 6, imuSize + camExSize - 6);
                return true;
            case MARG_OLDEST:
                temp = this->covariance.block(0, imuSize + exSize + 6, imuSize + exSize, camSize - 6);
                this->covariance.block(0, imuSize + exSize, imuSize + exSize, camSize - 6) = temp;
                this->covariance.block(imuSize + exSize, 0, camSize - 6, imuSize + exSize) = temp.transpose();
                temp = this->covariance.bottomRightCorner(camSize - 6, camSize - 6);
                this->covariance.block(imuSize + exSize, imuSize + exSize, camSize - 6, camSize - 6) = temp;
                this->covariance.conservativeResize(imuSize + camExSize - 6, imuSize + camExSize - 6);
                return true;
            default:
                return false;
        }
    }
}