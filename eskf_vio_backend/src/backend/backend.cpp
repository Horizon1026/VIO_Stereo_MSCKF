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
                    // Step e1: 调整滑动窗口使其仅剩下一帧
                    while (this->frameManager.frames.size() > 1) {
                        this->MarginalizeFeatureFrameManager(MARG_OLDEST);
                    }
                    // Step e2: 清空 attitude estimator 中记录的 imu 历史量测数据和姿态估计结果
                    this->attitudeEstimator.CleanOldItems(this->frameManager.frames.back()->timeStamp,
                                                          this->dataloader.imuPeriod);
                } else {
                    this->status = INITIALIZED;
                    LogInfo(">> Initialization succeed.");
                }
            }
            
            if (this->status == INITIALIZED) {
                // 在已经完成初始化的情况下，进行一次 update 的流程
                this->visionUpdator.Update(msg->featMeas->timeStamp, this->dataloader.imuPeriod);
                // Step 1: 定位到 propagator 序列中对应时间戳的地方，提取对应时刻状态，清空在这之前的序列 item

                // Step 2: 若此时滑动窗口已满，则判断最新帧是否为关键帧，确定边缘化策略。可以在此时给前端 thread 发送信号。

                // Step 3: 扩展 state 的维度以及协方差矩阵，利用对应时刻的 propagate 名义状态给 frame pose 赋值

                // Step 4: 三角测量滑动窗口内所有特征点。已被测量过的选择迭代法，没被测量过的选择数值法。更新每一个点的三角测量质量。

                // Step 5: 基于三角测量的质量，选择一定数量的特征点来构造量测方程。其中包括计算雅可比、投影到左零空间、缩减维度、卡尔曼 update 误差和名义状态

                // Step 6: 根据边缘化策略，选择跳过此步，或裁减 update 时刻点上的状态和协方差矩阵

                // Step 7: 对于 propagate queue 中后续的已经存在的 items，从 update 时刻点重新 propagate

                // 基于边缘化策略，调整特征点管理器和关键帧管理器的管理内容
                this->MarginalizeFeatureFrameManager(this->visionUpdator.margPolicy);
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