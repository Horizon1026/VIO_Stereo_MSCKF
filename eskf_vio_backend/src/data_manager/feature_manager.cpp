/* 内部依赖 */
#include <feature_manager.hpp>
#include <log_api.hpp>
/* 外部依赖 */

namespace ESKF_VIO_BACKEND {
    /* 为此特征点添加一个观测 */
    void Feature::AddNewObserve(const std::shared_ptr<FeatureObserve> &newObserve) {
        if (newObserve->norms.size() == 0) {
            return;
        }
        this->observes.emplace_back(newObserve);
        this->observeNum += newObserve->norms.size();
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


    /* 打印出当前特征点的信息 */
    void Feature::Information(void) {
        LogInfo(">> Feature id " << this->id << " is observed in frame [" << this->firstFrameID <<
            ", " << this->FinalFrameID() << "], observe num is " << this->observeNum);
        for (uint32_t i = 0; i < this->observes.size(); ++i) {
            for (auto it = this->observes[i]->norms.begin(); it != this->observes[i]->norms.end(); ++it) {
                LogInfo("     frame " << this->firstFrameID + i << " camera " << it->first << " observe [" <<
                    it->second.transpose() << "]");
            }
        }
    }


    /* 添加前端提供的特征点最新追踪结果信息，返回有所变动的特征点 */
    bool FeatureManager::AddNewFeatures(const std::vector<uint32_t> &ids,
        const std::vector<std::shared_ptr<FeatureObserve>> &newObserves,
        const uint32_t frameID,
        std::vector<std::shared_ptr<Feature>> &changedFeatures) {
        changedFeatures.clear();
        if (ids.size() != newObserves.size()) {
            return false;
        }
        changedFeatures.reserve(ids.size());
        for (uint32_t i = 0; i < ids.size(); ++i) {
            auto it = this->features.find(ids[i]);
            if (it != this->features.end()) {
                // 对应特征点已经存在时，直接添加观测
                it->second->AddNewObserve(newObserves[i]);
                changedFeatures.emplace_back(it->second);
            } else {
                // 对应特征点不存在时，构造新的特征点
                std::vector<std::shared_ptr<FeatureObserve>> observes;
                observes.emplace_back(newObserves[i]);
                std::shared_ptr<Feature> newFeature(new Feature(ids[i], frameID, observes));
                this->features.insert(std::make_pair(newFeature->id, newFeature));
                changedFeatures.emplace_back(newFeature);
            }
        }
        return true;
    }


    /* 移除指定关键帧所观测到的特征点。但此关键帧之后的观测帧 ID 会依次向前偏移 */
    void FeatureManager::RemoveByFrameID(const uint32_t frameID, bool offset) {
        // 记录失去所有观测的特征点的 ID，预先分配内存
        std::vector<uint32_t> needDelete;
        needDelete.reserve(this->features.size() / 2);

        // 根据 offset 决定关键帧的 ID 是否发生了偏移（是否进行了调整）
        if (offset) {
            // 滑动窗口内的帧 ID 发生了调整，对应着 marg subnew 的情况
            // 需要删除的观测一般位于中间，直接删除此观测即可
            // 如果需要删除的观测是第一个观测，则删除之后首次观测不变
            // 如果需要删除的观测是最后一个观测，直接删除即可
            for (auto &item : this->features) {
                auto &feature = item.second;
                if (feature->firstFrameID <= frameID && frameID <= feature->FinalFrameID()) {
                    // 如果此特征点只有这一个观测，则准备删除
                    if (feature->observes.size()  == 1) {
                        feature->observes.clear();
                        needDelete.emplace_back(feature->id);
                        continue;
                    }
                    // 如果此特征点有多个观测，则删除对应观测
                    feature->observeNum -= feature->observes[frameID - feature->firstFrameID]->norms.size();
                    for (uint32_t i = frameID - feature->firstFrameID; i < feature->observes.size() - 1; ++i) {
                        feature->observes[i] = feature->observes[i + 1];
                    }
                    feature->observes.resize(feature->observes.size() - 1);
                } else if (frameID < feature->firstFrameID) {
                    // 如果需要删除的观测，在这个特征点的所有观测之前，需要调整其首次观测
                    --feature->firstFrameID;
                }
            }
        } else {
            // 滑动窗口内的帧 ID 没有发生调整，对应着 marg oldest 的情况
            // 只需要关注 feature 的首次观测帧。去掉首次观测帧的观测之后，首次观测帧的 ID 需要加一
            for (auto &item : this->features) {
                auto &feature = item.second;
                if (feature->firstFrameID == frameID) {
                    // 如果此特征点只有这一个观测，则准备删除
                    if (feature->observes.size()  == 1) {
                        feature->observes.clear();
                        needDelete.emplace_back(feature->id);
                        continue;
                    }
                    // 如果此特征点有多个观测，则删除对应观测
                    feature->observeNum -= feature->observes[0]->norms.size();
                    for (uint32_t i = 0; i < feature->observes.size() - 1; ++i) {
                        feature->observes[i] = feature->observes[i + 1];
                    }
                    feature->observes.resize(feature->observes.size() - 1);
                    // 调整首帧观测的 ID
                    ++feature->firstFrameID;
                }
            }
        }

        // 剔除掉失去所有观测的特征点
        for (uint32_t i = 0; i < needDelete.size(); ++i) {
            this->features.erase(needDelete[i]);
        }
    }


    /* 移除指定 ID 的特征点 */
    void FeatureManager::RemoveByID(const uint32_t featureID) {
        this->features.erase(featureID);
    }


    /* 打印出当前管理的所有特征点的信息 */
    void FeatureManager::Information(void) {
        if (this->features.empty()) {
            LogInfo(">> Feature manager has no featuers.");
        } else {
            for (auto it = this->features.begin(); it != this->features.end(); ++it) {
                it->second->Information();
            }
            LogInfo("");
        }
    }

}