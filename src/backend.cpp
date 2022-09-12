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
        std::shared_ptr<CombinedMessage> msg;
        this->dataloader.PopOneMessage(msg);
        msg->Information();
        return true;
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
}