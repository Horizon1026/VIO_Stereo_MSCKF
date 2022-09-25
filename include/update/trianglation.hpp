#pragma once
/* 外部依赖 */
/* 内部依赖 */
#include <include/utility/typedef.hpp>
#include <include/data_manager/feature_manager.hpp>
#include <include/data_manager/frame_manager.hpp>

namespace ESKF_VIO_BACKEND {
    class Trianglator {
    public:
        // 指向特征点管理器与关键帧管理器的指针
        FeatureManager *featureManager;
        FrameManager *frameManager;
    public:
        /* 构造函数与析构函数 */
        Trianglator() {}
        ~Trianglator() {}
    public:
        /* 在一帧内，利用多目观测结果来三角测量一个特征点 */
        bool TrianglateMultiView(const std::shared_ptr<Feature> &feature);
        /* 在多帧内，利用单目观测结果来三角测量一个特征点 */
        bool TrianglateMultiFrame(const std::shared_ptr<Feature> &feature);
    };
}