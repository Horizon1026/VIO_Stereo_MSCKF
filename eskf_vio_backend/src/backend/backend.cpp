/* 内部依赖 */
#include <backend.hpp>
#include <log_api.hpp>
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

        // 当存在 IMU 量测输入时
        if (msg->imuMeas.size() > 0) {
            if (msg->imuMeas.front() != nullptr) {
                // 姿态解算求解器始终保持工作
                res = this->attitudeEstimator.Propagate(msg->imuMeas.front()->accel,
                                                        msg->imuMeas.front()->gyro,
                                                        msg->imuMeas.front()->timeStamp);
                // 状态递推器仅在完成初始化之后工作
                if (this->status != NEED_INIT) {
                    res = this->propagator.Propagate(msg->imuMeas.front()->accel,
                                                     msg->imuMeas.front()->gyro,
                                                     msg->imuMeas.front()->timeStamp);
                }
            }
        }

        // 当存在特征点追踪结果输入时
        if (msg->featMeas != nullptr) {
            // 更新特征点管理器和帧管理器
            res = this->UpdateFeatureFrameManager(msg->featMeas);

            if (this->status == NEED_INIT) {
                // 尝试初始化，成功后状态会变成 INITIALIZED
                LogInfo(">> Start initialization.");
                bool res = this->Initialize();
                // 如果初始化失败，可能是帧数不够，也可能是过程出错
                if (res == false) {
                    LogInfo(">> Initialization failed.");
                    // 调整滑动窗口使其仅剩下一帧
                    while (this->frameManager.frames.size() > 1) {
                        this->MarginalizeFeatureFrameManager(MARG_OLDEST);
                    }
                } else {
                    this->status = INITIALIZED;
                    LogInfo(">> Initialization succeed.");
                }
            }
            
            if (this->status == INITIALIZED) {
                // 在已经完成初始化的情况下，进行一次 update 的流程
                LogInfo(">> Start vision update.");
                bool res = this->visionUpdator.Update(msg->featMeas->timeStamp, this->dataloader.imuPeriod);
                // 如果 update 失败，需要重新初始化
                if (res == false) {
                    LogInfo(">> Vision update failed.");
                    this->status = NEED_INIT;
                    // 调整滑动窗口使其仅剩下一帧
                    while (this->frameManager.frames.size() > 1) {
                        this->MarginalizeFeatureFrameManager(MARG_OLDEST);
                    }
                } else {
                    LogInfo(">> Vision update succeed.");
                    // 基于边缘化策略，调整特征点管理器和关键帧管理器的管理内容
                    this->MarginalizeFeatureFrameManager(this->visionUpdator.margPolicy);
                    // 更新姿态解算求解器的 bias
                    this->attitudeEstimator.bias_a = this->propagator.bias_a;
                    this->attitudeEstimator.bias_g = this->propagator.bias_g;
                }
            }

            // 清空 attitude estimator 中记录的 imu 历史量测数据和姿态估计结果，仅保留最新两帧之间的
            if (this->frameManager.frames.size() > 1) {
                fp64 timeStamp = (*this->frameManager.frames.rbegin())->timeStamp;
                this->attitudeEstimator.CleanOldItems(timeStamp, this->dataloader.imuPeriod);
            }
        }

        return res;
    }


    /* 单步运行的测试 */
    bool Backend::RunOnceTest(void) {
        bool res = true;
        // 提取一个数据，可能是单个 IMU 数据，也可能是 IMU 和 features 的捆绑数据
        std::shared_ptr<CombinedMessage> msg;
        res = this->dataloader.PopOneMessage(msg);
        if (res == false) {
            return false;
        }

        return true;
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
        return true;
    }


    /* 重置 */
    void Backend::Reset(void) {
        this->status = NEED_INIT;
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
            case NO_MARG:
                break;
            default:
                return false;
        }
        return true;
    }
}