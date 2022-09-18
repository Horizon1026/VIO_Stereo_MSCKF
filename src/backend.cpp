/* 内部依赖 */
#include <include/backend.hpp>
/* 外部依赖 */

namespace ESKF_VIO_BACKEND {
    /* 输入一帧 IMU 数据 */
    bool Backend::GetIMUMessage(const std::shared_ptr<IMUMessage> &newImuMeas) {
        this->dataloader.PushIMUMessage(newImuMeas);
        return true;
    }


    /* 输入一帧 Features 追踪数据 */
    bool Backend::GetFeaturesMessage(const std::shared_ptr<FeaturesMessage> &newFeatMeas) {
        this->dataloader.PushFeaturesMessage(newFeatMeas);
        return true;
    }


    /* 单步运行 */
    bool Backend::RunOnce(void) {
        bool res = true;
        // 提取一个数据，可能是单个 IMU 数据，也可能是 IMU 和 features 的捆绑数据
        std::shared_ptr<CombinedMessage> msg;
        res = this->dataloader.PopOneMessage(msg);
        if (res == false) {
            return false;
        }
        // msg->Information();

        // 当存在 IMU 量测输入时
        if (msg->imuMeas.size() > 0) {
            if (msg->imuMeas.front() != nullptr) {
                res = this->propagator.Propagate(msg->imuMeas.front()->accel,
                                            msg->imuMeas.front()->gyro,
                                            msg->imuMeas.front()->timeStamp);
            }
        }

        // 当存在特征点追踪结果输入时
        if (msg->featMeas != nullptr) {
            res = this->UpdateFeatureFrameManager(msg->featMeas);
            // TODO: 
            // update
            // re propagate from update point to newest time
        }

        // 当滑动窗口数量达到上限，则丢弃一帧以及相关特征点
        if (this->frameManager.NeedMarginalize() == true) {
            this->MarginalizeFeatureFrameManager(MARG_OLDEST);
            // TODO:
            // covariance expand or cut
        }

        return res;
    }


    /* 输出最新 Propagate 点估计 */
    bool Backend::PublishPropagateState(IMUFullState &state) {
        if (this->propagator.items.empty()) {
            return false;
        }
        state.p_wb = this->propagator.items.back()->nominalState.p_wb;
        state.q_wb = this->propagator.items.back()->nominalState.q_wb;
        state.v_wb = this->propagator.items.back()->nominalState.v_wb;
        state.v_wb = this->propagator.items.back()->nominalState.v_wb;
        state.v_wb = this->propagator.items.back()->nominalState.v_wb;
        state.bias_a = this->propagator.bias_a;
        state.bias_g = this->propagator.bias_g;
        state.gravity = this->propagator.gravity;
        return true;
    }


    /* 输出最新 Update 点估计 */
    bool Backend::PublishUpdateState(IMUFullState &state) {
        if (this->propagator.items.empty()) {
            return false;
        }
        state.p_wb = this->propagator.items.front()->nominalState.p_wb;
        state.q_wb = this->propagator.items.front()->nominalState.q_wb;
        state.v_wb = this->propagator.items.front()->nominalState.v_wb;
        state.bias_a = this->propagator.bias_a;
        state.bias_g = this->propagator.bias_g;
        state.gravity = this->propagator.gravity;
        return true;
    }


    /* 重置 */
    void Backend::Reset(void) {

    }


    /* 将新输入的 feature message 更新到特征点管理器和帧管理器中 */
    bool Backend::UpdateFeatureFrameManager(const std::shared_ptr<FeaturesMessage> &featMeas) {
        if (featMeas == nullptr) {
            return false;
        }
        // 添加新帧，新帧 frame id 由 frame manager 给定
        std::shared_ptr<Frame> newFrame(new Frame(featMeas->timeStamp));
        this->frameManager.AddNewFrame(newFrame);

        // 特征点追踪结果添加到 feature manager
        std::vector<std::shared_ptr<Feature>> newObserveFeatures;
        this->featureManager.AddNewFeatures(featMeas->ids, featMeas->observes,
            newFrame->id, newObserveFeatures);

        // 新帧关联特征点追踪结果
        newFrame->AddFeatures(newObserveFeatures);

        // TODO: 
        // this->featureManager.Information();
        // this->frameManager.Information();
        return true;
    }


    /* 基于 marg 策略调整特征点管理器和帧管理器 */
    bool Backend::MarginalizeFeatureFrameManager(MargPolicy policy) {
        switch (policy) {
            case MARG_NEWEST:
                // 剔除最新帧以及相关特征点
                this->featureManager.RemoveByFrameID(this->frameManager.frames.back()->id, true);
                this->frameManager.RemoveFrame(this->frameManager.frames.size() - 1);
                break;
            case MARG_OLDEST:
                // 剔除最旧帧以及相关特征点
                this->featureManager.RemoveByFrameID(this->frameManager.frames.front()->id, false);
                this->frameManager.RemoveFrame(0);
                break;
            case MARG_SUBNEW:
                // 剔除次新帧以及相关特征点
                this->featureManager.RemoveByFrameID(this->frameManager.frames.back()->id - 1, true);
                this->frameManager.RemoveFrame(this->frameManager.frames.size() - 2);
                break;
            default:
                return false;
        }
        return true;
    }


    /* 设置相机与 IMU 之间的外参 */
    bool Backend::SetExtrinsic(const std::vector<Quaternion> &q_bc,
        const std::vector<Vector3> &p_bc) {
        if (q_bc.size() != p_bc.size()) {
            return false;
        } else {
            this->q_bc = q_bc;
            this->p_bc = p_bc;
            return true;
        }
    }
}