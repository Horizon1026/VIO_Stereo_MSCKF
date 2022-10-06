#pragma once
/* 外部依赖 */
/* 内部依赖 */
#include <data_loader.hpp>
#include <feature_manager.hpp>
#include <frame_manager.hpp>
#include <sequence_propagator.hpp>
#include <attitude_estimate.hpp>
#include <multi_view_vision_update.hpp>
#include <trianglation.hpp>
#include <perspective_n_point.hpp>

namespace ESKF_VIO_BACKEND {
    /* ESKF VIO 后端控制器 */
    class Backend {
    private:
        /* 数据管理相关 */
        DataLoader dataloader;                  // 数据加载器，控制数据流按时间戳顺序输入
        FeatureManager featureManager;          // 视觉特征点管理器
        FrameManager frameManager;              // 视觉关键帧管理器
        
        /* 序列递推与观测更新相关 */
        AttitudeEstimate attitudeEstimator;     // 姿态解算求解器
        PropagateQueue propagator;              // IMU 状态递推过程管理器
        Trianglator trianglator;                // 视觉特征点三角测量求解器
        PnPSolver pnpSolver;                    // PnP 求解器
        MultiViewVisionUpdate visionUpdator;    // 视觉 Update 过程管理器
    
    public:
        // 当前 Backend 的状态
        Status status = NEED_INIT;
    public:
        /* 构造函数与析构函数 */
        Backend() {}
        ~Backend() {}
    /* 对外接口 interface */
    public:
        /* 后端优化器读取配置并初始化 */
        bool Initialize(const std::string &configPath);
        /* 输入一帧 IMU 数据 */
        bool GetIMUMessage(const std::shared_ptr<IMUMessage> &newImuMeas);
        /* 输入一帧 Features 追踪数据 */
        bool GetFeaturesMessage(const std::shared_ptr<FeaturesMessage> &newFeatMeas);
        /* 单步运行 */
        bool RunOnce(void);
        /* 单步运行的测试 */
        bool RunOnceTest(void);
        /* 输出最新 Propagate 点估计 */
        bool PublishPropagateState(IMUFullState &state);
        /* 输出最新 Update 点估计 */
        bool PublishUpdateState(IMUFullState &state);
        /* 重置 */
        void Reset(void);

    /* 对内接口 interface */
    private:
        /* 从 txt 文件中读取一个矩阵 */
        bool LoadMatrix(const std::string &matrixFile, const uint32_t rows, const uint32_t cols, Matrix &mat);
        /* 将新输入的 feature message 更新到特征点管理器和帧管理器中 */
        bool UpdateFeatureFrameManager(const std::shared_ptr<FeaturesMessage> &featMeas);
        /* 基于 marg 策略调整特征点管理器和帧管理器 */
        bool MarginalizeFeatureFrameManager(MargPolicy policy);
    
    /* 对内接口 interface 初始化过程相关 */
    private:
        /* 尝试进行初始化 */
        bool Initialize(void);
        /* 基于某一帧的多目测量结果进行三角化 */
        bool TrianglizeMultiView(const std::shared_ptr<Frame> &frame);
        /* 基于三角化成功的点，估计某一帧的位姿 */
        bool EstimateFramePose(const std::shared_ptr<Frame> &frame);
        /* 从指定时刻点开始，初始化序列化 propagator，递推到最新时刻 */
        bool InitializePropagator(const IMUMotionState &initState,
                                  const fp64 startTime);
    };

}