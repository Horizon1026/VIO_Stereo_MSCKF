/* 内部依赖 */
#include <multi_view_vision_update.hpp>
#include <log_api.hpp>
/* 外部依赖 */

namespace ESKF_VIO_BACKEND {
    /* 执行一次 update 过程 */
    bool MultiViewVisionUpdate::Update(const fp64 timeStamp, const fp64 threshold) {
        // Step 1: 定位到 propagator 序列中对应时间戳的地方，提取对应时刻状态，清空在这之前的序列 item
        RETURN_IF_FALSE(this->ResetPropagatorOrigin(timeStamp, threshold));
        // Step 2: 若此时滑动窗口已满，则判断次新帧是否为关键帧，确定边缘化策略。可以在此时给前端 thread 发送信号。
        RETURN_IF_FALSE(this->DecideMargPolicy());
        // Step 3: 扩展 state 的维度以及协方差矩阵，利用对应时刻的 propagate 名义状态给 frame pose 赋值
        RETURN_IF_FALSE(this->ExpandCameraStateCovariance());
        // Step 4: 三角测量滑动窗口内所有特征点。已被测量过的选择迭代法，没被测量过的选择数值法。更新每一个点的三角测量质量，基于三角测量的质量，选择一定数量的特征点
        RETURN_IF_FALSE(this->SelectGoodFeatures(30));
        // Step 5: 构造量测方程。其中包括计算雅可比、投影到左零空间、缩减维度、卡尔曼 update 误差和名义状态
        RETURN_IF_FALSE(this->ConstructMeasurementFunction());
        RETURN_IF_FALSE(this->UpdateState());
        // Step 6: 根据边缘化策略，选择跳过此步，或裁减 update 时刻点上的状态和协方差矩阵
        RETURN_IF_FALSE(this->ReduceCameraStateCovariance());
        // Step 7: 对于 propagate queue 中后续的已经存在的 items，从 update 时刻点重新 propagate
        RETURN_IF_FALSE(this->propagator->Repropagate());
        return true;
    }


    /* 定位到 propagator 序列中对应时间戳的地方，清空在这之前的序列 item */
    bool MultiViewVisionUpdate::ResetPropagatorOrigin(const fp64 timeStamp, const fp64 threshold) {
        RETURN_IF_FALSE(this->propagator->ResetOrigin(timeStamp, threshold));
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
                this->margPolicy = MARG_OLDEST;
            } else {
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
        // expand state，因在 frame manager 中已经扩充内存，因此此处就是赋值
        frame->p_wb = propagateItem->nominalState.p_wb;
        frame->v_wb = propagateItem->nominalState.v_wb;
        frame->q_wb = propagateItem->nominalState.q_wb;
        // expand covariance
        /*
            用于扩充维度的雅可比矩阵，本质上就是 new state/full state 的雅可比矩阵
            full_cov = [ I(15 + 6m + 6n) ] * old_full_cov * [ I(15 + 6m + 6n) ].T
                       [        J        ]                  [        J        ]
 
            when camera state is q_wc/p_wc, use q_bc0/p_bc0 to calculate from q_wb/p_wb
                q_wc = q_wb * q_bc0;
                p_wc = p_wb + q_wb * p_bc0
                      p  v        theta          ba  bg  q_bc0  p_bc0  q_bc1  p_bc1  ...  Twb0  Twb1  ...
            as J is [ 0  0         R_wb          0   0     I      0      0      0    ...    0     0   ...  ]  ->  q_wc(theta)
                    [ I  0  - R_wb * hat(p_bc)   0   0     0      I      0      0    ...    0     0   ...  ]  ->  p_wc

            when camera state is q_wb/p_wb, things become easy
                      p  v  theta   ba  bg  q_bc0  p_bc0  q_bc1  p_bc1  ...  Twb0  Twb1  ...
            as J is [ 0  0    I     0   0     0      0      0      0    ...    0     0   ...  ]  ->  q_wb
                    [ I  0    0     0   0     0      0      0      0    ...    0     0   ...  ]  ->  p_wb

        */
        // TODO:
        return true;
    }


    /* 三角测量滑动窗口内所有特征点。已被测量过的选择迭代法，没被测量过的选择数值法。更新每一个点的三角测量质量，基于三角测量的质量，选择一定数量的特征点 */
    bool MultiViewVisionUpdate::SelectGoodFeatures(const uint32_t num) {
        this->features.clear();

        // TODO:
        return true;
    }


    /* 构造量测方程。其中包括计算雅可比、投影到左零空间、缩减维度、卡尔曼 update 误差和名义状态 */
    bool MultiViewVisionUpdate::ConstructMeasurementFunction(void) {
        // TODO:
        return true;
    }


    /* 更新误差状态和名义状态 */
    bool MultiViewVisionUpdate::UpdateState(void) {
        // TODO:
        return true;
    }


    /* 裁减 update 时刻点上的状态和协方差矩阵 */
    bool MultiViewVisionUpdate::ReduceCameraStateCovariance(void) {
        switch (this->margPolicy) {
            case NO_MARG:
                return true;
            case MARG_NEWEST:
                return false;
            case MARG_OLDEST:
                // TODO:

                return true;
            case MARG_SUBNEW:
                // TODO:

                return true;
            default:
                return false;
        }
    }
}