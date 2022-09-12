#pragma once
/* 外部依赖 */
/* 内部依赖 */
#include <include/utility/typedef.hpp>

/* 后端的数据加载器仅仅支持特征点追踪结果，以及IMU量测信息 */
namespace ESKF_VIO_BACKEND {
    /* 某一个特征点于同一时刻在多个相机中的观测 */
    class FeatureObserve {
    public:
        // 特征点在对应 camera ID 中的归一化平面坐标观测
        std::unordered_map<uint32_t, Eigen::Matrix<Scalar, 2, 1>> uv;
    public:
        FeatureObserve() {}
        ~FeatureObserve() {}
    public:
        /* 清空保存的数据 */
        void Clear(void);
    };


    /* 特征点追踪结果数据类型定义，保存一帧双目图像的追踪结果 */
    class FeaturesMessage {
    public:
        // 特征点的索引
        std::vector<uint32_t> ids;
        // 归一化平面坐标观测
        std::vector<std::shared_ptr<FeatureObserve>> observes;
        // 标志位
        std::vector<uint8_t> flag;
        // 时间戳（单位 s）
        fp64 timeStamp;
    public:
        FeaturesMessage() {}
        ~FeaturesMessage() {}
        FeaturesMessage(const std::vector<uint32_t> &ids,
                        const std::vector<std::shared_ptr<FeatureObserve>> &observes,
                        const std::vector<uint8_t> &flag,
                        const fp64 &timeStamp);
    public:
        /* 清空保存的数据 */
        void Clear(void);
    };


    /* IMU 量测数据类型定义，保存一次 IMU 量测数据 */
    class IMUMessage {
    public:
        // body 系角速度量测（单位 rad/s）
        Eigen::Matrix<Scalar, 3, 1> gyro;
        // body 系加速度量测（单位 m/s^2）
        Eigen::Matrix<Scalar, 3, 1> acc;
        // 时间戳（单位 s）
        fp64 timeStamp;
    public:
        IMUMessage() {}
        ~IMUMessage() {}
        IMUMessage(const Eigen::Matrix<Scalar, 3, 1> &gyro,
                   const Eigen::Matrix<Scalar, 3, 1> &acc,
                   const fp64 &timeStamp);
    };


    /* 一帧特征点追踪结果和一系列 IMU 量测数据 */
    class CombinedMessage {
    public:
        std::shared_ptr<FeaturesMessage> featMeas;
        std::vector<std::shared_ptr<IMUMessage>> imuMeas;
    public:
        CombinedMessage() {}
        ~CombinedMessage() {}
        CombinedMessage(const std::shared_ptr<FeaturesMessage> &featMeas,
                        const std::shared_ptr<IMUMessage> &imuMeas);
        CombinedMessage(const std::shared_ptr<FeaturesMessage> &featMeas,
                        const std::vector<std::shared_ptr<IMUMessage>> &imuMeas);
    public:
        /* 清空保存的数据 */
        void Clear(void);
    };
}