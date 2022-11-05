/* 外部依赖 */
#pragma once
/* 内部依赖 */
#include <typedef.hpp>
#include <sequence_propagator.hpp>
#include <feature_manager.hpp>
#include <frame_manager.hpp>
#include <trianglation.hpp>
#include <perspective_n_point.hpp>

namespace ESKF_VIO_BACKEND {
    /* 多目视觉观测更新管理器 */
    class MultiViewVisionUpdate {
    public:
        // 指向相关管理器或工具的指针
        PropagateQueue *propagator = nullptr;
        FeatureManager *featureManager = nullptr;
        FrameManager *frameManager = nullptr;
        Trianglator *trianglator = nullptr;
        PnPSolver *pnpSolver = nullptr;
        // 量测噪声（使用时默认为对角矩阵 R ）
        Scalar measureNoise = Scalar(0);
        // 最多使用多少特征点去 update
        uint32_t maxUsedFeatures = 20;
        // 边缘化策略
        MargPolicy margPolicy = NO_MARG;

    private:
        // 用于构造量测方程的特征点
        std::vector<std::shared_ptr<Feature>> features;
        // propagator 预测的，以及 update 过程输出的 errorState 结果
        // [p_wb  v_wb  q_wb  ba  bg]  [p_bc0  q_bc0  p_bc1  q_bc1 ...]  [p_wb0  q_wb0  p_wb1  q_wb1 ...]
        Vector delta_x;
        // 状态的协方差矩阵
        Matrix covariance;
        // 用于状态扩维的雅可比矩阵
        Matrix expand_J;
        // 量测误差的协方差矩阵
        Matrix meas_covariance;
        // 所有特征点共同构造出来的量测方程 [Hx | r]
        Matrix Hx_r;
        // 每个特征点投影到左零空间的子量测方程的量测维度之和
        uint32_t Hx_cols = 0;
        uint32_t Hx_rows = 0;
        // ESKF 中的卡尔曼增益
        Matrix K;

    public:
        /* 构造函数与析构函数 */
        MultiViewVisionUpdate();
        virtual ~MultiViewVisionUpdate() = default;
    public:
        /* 执行一次 update 过程 */
        bool Update(const fp64 timeStamp, const fp64 threshold);
    private:
        /* 定位到 propagator 序列中对应时间戳的地方，清空在这之前的序列 item */
        bool ResetPropagatorOrigin(const fp64 timeStamp, const fp64 threshold);
        /* 若此时滑动窗口已满，则判断次新帧是否为关键帧，确定边缘化策略 */
        bool DecideMargPolicy(void);
        /* 拼接完整的协方差矩阵 */
        bool ConstructCovariance(void);
        /* 利用 propagator 的结果更新最新帧的 frame pose */
        bool UpdateNewestFramePose(void);
        /* 利用所有观测，三角测量一个特征点 */
        bool TrianglizeMultiFrame(const std::shared_ptr<Feature> &feature);
        /* 三角测量在最新一帧中被追踪到的特征点。已被测量过的选择迭代法，没被测量过的选择数值法。*/
        /* 更新每一个点的三角测量质量，基于三角测量的质量，选择一定数量的特征点 */
        bool SelectGoodFeatures(void);
        /* 构造量测方程。其中包括计算雅可比、投影到左零空间、缩减维度、卡尔曼 update 误差和名义状态 */
        bool ConstructMeasurementFunction(void);
        /* 构造一个特征点所有观测的量测方程，投影到左零空间 */
        bool ConstructMeasurementFunction(const std::shared_ptr<Feature> &feature,
                                          Matrix &Hx_r);
        /* 更新误差状态和名义状态 */
        bool UpdateState(void);
        /* 扩展 state 的维度以及协方差矩阵，利用对应时刻的 propagate 名义状态给 frame pose 赋值 */
        bool ExpandCameraCovariance(void);
        /* 更新协方差矩阵到 propagator 中 */
        bool UpdateCovariance(void);
        /* 裁减 update 时刻点上的状态和协方差矩阵 */
        bool ReduceCameraCovariance(void);
    };
}