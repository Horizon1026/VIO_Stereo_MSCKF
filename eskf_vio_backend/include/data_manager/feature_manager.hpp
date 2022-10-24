#pragma once
/* 外部依赖 */
/* 内部依赖 */
#include <data_typedef.hpp>

namespace ESKF_VIO_BACKEND {
    /* 管理一个特征点 */
    class Feature {
    public:
        // 此特征点的全局索引
        const uint32_t id;
        // 首次观测到此特征点的关键帧的全局索引
        uint32_t firstFrameID;
        // 此特征点在以第 firstFrameID 帧为起始的连续几帧中的观测
        std::vector<std::shared_ptr<FeatureObserve>> observes;
        // 观测到此特征点的 camera pose 数量
        uint32_t observeNum;
        // 此特征点在世界坐标系中的位置
        Vector3 p_w;
        // 此特征点的状态
        enum Status {
            UNSOLVED = 1,   // 尚未被三角化
            SOLVED,         // 三角化成功
            ERROR           // 三角化失败
        } status = UNSOLVED;
    public:
        /* 构造函数与析构函数 */
        explicit Feature(const uint32_t id,
                const uint32_t firstFrameID,
                const std::vector<std::shared_ptr<FeatureObserve>> &observes) :
            id(id), firstFrameID(firstFrameID), observes(observes) {
            this->observeNum = observes.front()->norms.size();
        }
        ~Feature() {}
    public:
        /* 为此特征点添加一个观测 */
        void AddNewObserve(const std::shared_ptr<FeatureObserve> &newObserve);
        /* 获取指定 FrameID 和 CameraID 的归一化平面坐标观测 */
        bool GetNorm(const uint32_t frameID, const uint32_t cameraID, Eigen::Matrix<Scalar, 2, 1> &norm);
        /* 获取最后一个观测到此特征点的 Frame ID */
        uint32_t FinalFrameID(void);
        /* 打印出当前特征点的信息 */
        void Information(void);
    };


    /* 管理 localmap 中所有特征点 */
    class FeatureManager {
    public:
        // 所有 localmap 中的特征点
        std::unordered_map<uint32_t, std::shared_ptr<Feature>> features;
    public:
        /* 构造函数与析构函数 */
        FeatureManager() {}
        ~FeatureManager() {}
    public:
        /* 添加前端提供的特征点最新追踪结果信息，返回有所变动的特征点 */
        bool AddNewFeatures(const std::vector<uint32_t> &ids,
            const std::vector<std::shared_ptr<FeatureObserve>> &newObserves,
            const uint32_t frameID,
            std::vector<std::shared_ptr<Feature>> &changedFeatures);
        /* 移除指定关键帧所观测到的特征点。但此关键帧之后的观测帧 ID 会依次向前偏移 */
        void RemoveByFrameID(const uint32_t frameID, bool offset);
        /* 移除指定 ID 的特征点 */
        void RemoveByID(const uint32_t landmarkID);
        /* 打印出当前管理的所有特征点的信息 */
        void Information(void);
    };
}