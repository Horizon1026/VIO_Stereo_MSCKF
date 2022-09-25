#pragma once
/* 外部依赖 */
/* 内部依赖 */
#include <include/utility/typedef.hpp>
#include <include/propagate/sequence_propagator.hpp>
#include <include/data_manager/feature_manager.hpp>
#include <include/data_manager/frame_manager.hpp>
#include <include/update/trianglation.hpp>

namespace ESKF_VIO_BACKEND {
    /* 多目视觉观测更新管理器 */
    class MultiViewVisionUpdate {
    public:
        // 指向相关管理器或工具的指针
        FeatureManager *featureManager;
        FrameManager *frameManager;
        Trianglator *trianglator;
        // 量测方程拼接矩阵 [Hx | Hp | r]，用于原始拼接、左零空间投影
        Matrix Hx_Hp_r;

    public:
        /* 构造函数与析构函数 */
        MultiViewVisionUpdate() {}
        ~MultiViewVisionUpdate() {}
    public:
        /* 选择出指定数量的具备较好视差的特征点 */
        bool SelectGoodFeatures(const uint32_t num, std::vector<std::shared_ptr<Feature>> &features);
        /* 构造量测方程 */
        bool ConstrunctMeasurementFunction(std::vector<std::shared_ptr<Feature>> &features);
    };
}