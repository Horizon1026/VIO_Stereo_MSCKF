/* 内部依赖 */
#include <include/data_manager/frame_manager.hpp>
/* 外部依赖 */
#if STD_COUT_INFO
    #include <iostream>
#endif

namespace ESKF_VIO_BACKEND {
    /* 初始化关键帧 */
    bool Frame::Initialize(const uint32_t id, const double timeStamp) {
        this->features.clear();
        this->q_wb.setIdentity();
        this->p_wb.setZero();
        this->v_wb.setZero();
        this->id = id;
        this->timeStamp = timeStamp;
        return true;
    }


    /* 为此帧添加一个特征点关联 */
    bool Frame::AddFeature(const std::shared_ptr<Feature> &newFeature) {
        this->features.insert(std::make_pair(newFeature->id, newFeature));
        return true;
    }


    /* 为此帧添加一组特征点关联 */
    bool Frame::AddFeatures(const std::vector<std::shared_ptr<Feature>> &newFeatures) {
        bool res = true;
        for (uint32_t i = 0; i < newFeatures.size(); ++i) {
            res = this->AddFeature(newFeatures[i]);
            if (res == false) {
                return false;
            }
        }
        return true;
    }


    /* 提取当前帧与某一帧之间的共视特征点 */
    std::vector<std::shared_ptr<Feature>> Frame::GetCovisibleFeatures(const std::shared_ptr<Frame> &target) {
        std::vector<std::shared_ptr<Feature>> ret;
        ret.reserve(std::min(this->features.size(), target->features.size()));
        for (auto it = this->features.begin(); it != this->features.end(); ++it) {
            if (target->features.find(it->first) != target->features.end()) {
                ret.emplace_back(it->second);
            }
        }
        return ret;
    }


    /* 打印出此帧所有信息 */
    void Frame::Information(void) {
    #if STD_COUT_INFO
        std::cout << ">> Frame id " << this->id << " at time " << this->timeStamp << "s observes " <<
            this->features.size() << " features.\n";
        std::cout << "     q_wb is [" << this->q_wb.w() << ", " << this->q_wb.x() << ", " <<
            this->q_wb.y() << ", " << this->q_wb.z() << "] (s, v)\n";
        std::cout << "     p_wb is [" << this->p_wb.transpose() << "]\n";
        std::cout << "     v_wb is [" << this->v_wb.transpose() << "]\n";
    #endif
    }


    /* 帧管理器初始化 */
    bool FrameManager::Initialize(const uint32_t maxWindowSize) {
        this->frames.clear();
        this->maxWindowSize = maxWindowSize;
        return true;
    }


    /* 增加新一帧，检查滑窗内 ID 的合法性，并返回新加入帧的 ID */
    bool FrameManager::AddNewFrame(const std::shared_ptr<Frame> &newFrame) {
        // 设置 new frame 的 ID 和 pose 初值
        if (this->frames.empty()) {
            newFrame->id = FIRST_FRAME_ID;
            newFrame->q_wb.setIdentity();
            newFrame->p_wb.setZero();
            newFrame->v_wb.setZero();
        } else {
            newFrame->id = this->frames.back()->id + 1;
            newFrame->q_wb = this->frames.back()->q_wb;
            newFrame->p_wb = this->frames.back()->p_wb;
            newFrame->v_wb = this->frames.back()->v_wb;
        }
        this->frames.emplace_back(newFrame);

        // 检查 frame ID 是否为递增 1 的等差数列
        uint32_t idx = this->frames.front()->id;
        for (auto it = this->frames.begin(); it != this->frames.end(); ++it) {
            if ((*it)->id != idx) {
                return false;
            }
            ++idx;
        }
        return true;
    }


    /* 移除一帧，给定待移动帧在滑动窗口内的索引 (0 -> max window size - 1) */
    bool FrameManager::RemoveFrame(const uint32_t localIndex) {
        if (localIndex >= this->frames.size()) {
            return false;
        }
        auto removeTarget = this->frames.begin();
        // 如果被移除的不是最旧帧，则后续帧的 ID 需要调整
        if (localIndex > 0) {
            uint32_t cnt = localIndex;
            while (cnt) {
                ++removeTarget;
                --cnt;
            }
            auto needAdjustID = removeTarget;
            while (needAdjustID != this->frames.end()) {
                --(*needAdjustID)->id;
                ++needAdjustID;
            }
        }
        this->frames.erase(removeTarget);
        return true;
    }


    /* 提取某一个关键帧的指针 */
    std::shared_ptr<Frame> FrameManager::GetFrame(const uint32_t frameID) {
        if (this->frames.empty()) {
            return nullptr;
        }
        if (frameID < this->frames.front()->id || frameID > this->frames.back()->id) {
            return nullptr;
        }
        for (auto it = this->frames.begin(); it != this->frames.end(); ++it) {
            if ((*it)->id == frameID) {
                return *it;
            }
        }
        return nullptr;
    }


    /* 获取滑动窗口内最大的 frame ID */
    uint32_t FrameManager::GetMaxFrameID(void) {
        if (this->frames.empty()) {
            return 0;
        } else {
            return this->frames.back()->id;
        }
    }


    /* 判断滑动窗口是否需要进行边缘化操作 */
    bool FrameManager::NeedMarginalize(void) {
        if (this->frames.size() >= this->maxWindowSize) {
            return true;
        } else {
            return false;
        }
    }


    /* 打印出滑动窗口内所有关键帧的信息 */
    void FrameManager::Information(void) {
    #if STD_COUT_INFO
        if (this->frames.empty()) {
            std::cout << ">> Frame manager has no featuers." << std::endl;
        } else {
            for (auto it = this->frames.begin(); it != this->frames.end(); ++it) {
                (*it)->Information();
            }
            std::cout << std::endl;
        }
    #endif
    }
}