#include <include/backend.hpp>

namespace ESKF_VIO_BACKEND {
    /* 后端优化器读取配置并初始化 */
    bool Backend::Initialize(const std::string &configFile) {
        return true;
    }


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
        this->dataloader.PopOneMessage(msg);
        // msg->Information();

        // 当存在特征点追踪结果输入时
        if (msg->featMeas != nullptr) {
            res = this->UpdateFeatureFrameManager(msg->featMeas);
        }

        return res;
    }


    /* 输出最新 Propagate 点估计 */
    bool Backend::PublishPropagateState(IMUFullState &state) {
        return true;
    }


    /* 输出最新 Update 点估计 */
    bool Backend::PublishUpdataState(IMUFullState &state) {
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
        this->featureManager.Information();
        return true;
    }
}