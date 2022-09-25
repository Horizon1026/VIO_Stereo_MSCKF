#pragma once
/* 外部依赖 */
/* 内部依赖 */
#include <include/data_manager/data_loader.hpp>
#include <include/data_manager/feature_manager.hpp>
#include <include/data_manager/frame_manager.hpp>
#include <include/propagate/sequence_propagator.hpp>
#include <include/propagate/attitude_estimate.hpp>
#include <include/update/multi_view_vision_update.hpp>
#include <include/update/trianglation.hpp>

namespace ESKF_VIO_BACKEND {
    /* ESKF VIO 后端控制器 */
    class Backend {
    private:
        /* 数据管理相关 */
        DataLoader dataloader;
        FeatureManager featureManager;
        FrameManager frameManager;
        // 每一个 camera 和 IMU 之间的相对位姿，脚标即为对应 camera 的 ID
        std::vector<Quaternion> q_bc;
        std::vector<Vector3> p_bc;
        
        /* 序列递推与观测更新相关 */
        AttitudeEstimate attitudeEstimator;
        PropagateQueue propagator;
        Trianglator trianglator;
        MultiViewVisionUpdate visionUpdator;


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
        /* 设置相机与 IMU 之间的外参 */
        bool SetExtrinsic(const std::vector<Quaternion> &q_bc,
            const std::vector<Vector3> &p_bc);
    };

}