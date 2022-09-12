#pragma once
/* 外部依赖 */
/* 内部依赖 */
#include <include/data_manager/data_loader.hpp>
#include <include/propagate/sequence_propagator.hpp>

namespace ESKF_VIO_BACKEND {
    /* ESKF VIO 后端控制器 */
    class Backend {
    private:
        // 参数配置
        // TODO:
        // 数据管理
        DataLoader dataloader;
        
        // 序列递推
        PropagateQueue queue;
        // 观测更新
        // TODO:
    public:
        /* 构造函数与析构函数 */
        Backend() {}
        ~Backend() {}
    /* 对外接口 interface */
    public:
        /* 后端优化器读取配置并初始化 */
        bool Initialize(const std::string &configFile);
        /* 输入一帧 IMU 数据 */
        bool GetIMUMessage(const std::shared_ptr<IMUMessage> &newImuMeas);
        /* 输入一帧 Features 追踪数据 */
        bool GetFeaturesMessage(const std::shared_ptr<FeaturesMessage> &newFeatMeas);
        /* 单步运行 */
        bool RunOnce(void);
        /* 输出最新 Propagate 点估计 */
        bool PublishPropagateState(IMUFullState &state);
        /* 输出最新 Update 点估计 */
        bool PublishUpdataState(IMUFullState &state);
        /* 重置 */
        void Reset(void);
    };

}