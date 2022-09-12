#pragma once
/* 外部依赖 */
#include <vector>
#include <list>
#include <memory>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
/* 内部依赖 */
#include <include/utility/typedef.hpp>

/* 后端的数据加载器仅仅支持特征点追踪结果，以及IMU量测信息 */
namespace VIOBackend {
    /* 特征点追踪结果数据类型定义，保存一帧双目图像的追踪结果 */
    class FeaturesMessage {
    public:
        // 特征点的索引
        std::vector<uint32_t> ids;
        // 左目归一化平面坐标
        std::vector<Eigen::Vector2f> left;
        // 右目归一化平面坐标（单目时此项为空）
        std::vector<Eigen::Vector2f> right;
        // 标志位
        std::vector<uint8_t> flag;
        // 时间戳（单位 s）
        fp64 timeStamp;
    public:
        FeaturesMessage() {}
        ~FeaturesMessage() {}
        FeaturesMessage(const std::vector<uint32_t> &ids,
                        const std::vector<Eigen::Vector2f> &left,
                        const std::vector<Eigen::Vector2f> &right,
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
        Eigen::Vector3f gyro;
        // body 系加速度量测（单位 m/s^2）
        Eigen::Vector3f acc;
        // 时间戳（单位 s）
        fp64 timeStamp;
    public:
        IMUMessage() {}
        ~IMUMessage() {}
        IMUMessage(const Eigen::Vector3f &gyro,
                   const Eigen::Vector3f &acc,
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