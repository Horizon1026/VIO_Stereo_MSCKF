#include <include/data_manager/feature_manager.hpp>

namespace ESKF_VIO_BACKEND {
    /* 为此特征点添加一个观测 */
    void Feature::AddNewObserve(const std::shared_ptr<FeatureObserve> &newObserve) {
        if (newObserve->norms.size() == 0) {
            return;
        }
        this->observes.emplace_back(newObserve);
    }

    /* 获取指定 FrameID 和 CameraID 的归一化平面坐标观测 */
    bool Feature::GetNorm(const uint32_t frameID, const uint32_t cameraID, Eigen::Matrix<Scalar, 2, 1> &norm) {
        uint32_t idx = frameID - this->firstFrameID;
        if (idx < this->observes.size()) {
            if (this->observes[idx]->GetNorm(cameraID, norm) == true) {
                return true;
            }
        }
        return false;
    }

    /* 获取最后一个观测到此特征点的 Frame ID */
    uint32_t Feature::FinalFrameID(void) {
        return this->firstFrameID + this->observes.size() - 1;
    }

    /* 添加前端提供的特征点最新追踪结果信息 */
    bool FeatureManager::AddNewFeatures(const std::vector<uint32_t> &ids,
        const std::vector<Eigen::Matrix<Scalar, 2, 1>> &norms,
        const uint32_t frameID) {
        if (ids.size() != norms.size()) {
            return false;
        }
        
        return true;
    }
    
    /* 移除指定关键帧所观测到的特征点。但此关键帧之后的观测帧 ID 会依次向前偏移 */
    void FeatureManager::RemoveByFrameID(const uint32_t frameID, bool offset) {

    }

    /* 移除指定 ID 的特征点 */
    void FeatureManager::RemoveByID(const uint32_t landmarkID) {

    }

}