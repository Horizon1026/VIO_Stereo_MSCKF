#pragma once
/* 外部依赖 */
/* 内部依赖 */
#include <include/data_manager/data_typedef.hpp>

namespace ESKF_VIO_BACKEND {
    /* 数据加载器负责管理数据的输入，并按时间戳顺序提供给后续接口 */
    /* 主要是为了控制数据输入流，尽量按照时间戳顺序输出数据 */
    class DataLoader {
    private:
        // 特征点追踪数据和 IMU 量测数据
        std::deque<std::shared_ptr<FeaturesMessage>> featMeas;
        std::deque<std::shared_ptr<IMUMessage>> imuMeas;
        // 时间戳起点
        double timeStampOffset;
    public:
        /* 构造函数与析构函数 */
        DataLoader() { this->Clear(); }
        ~DataLoader() {}
    public:
        /* 清空缓冲区 */
        void Clear(void);
        /* 输入一帧特征点追踪结果 */
        bool PushFeaturesMessage(const std::shared_ptr<FeaturesMessage> &newFeatMeas);
        /* 输入一刻 IMU 量测结果 */
        bool PushIMUMessage(const std::shared_ptr<IMUMessage> &newImuMeas);
        /* 按时间戳输出下一个数据 */
        bool PopOneMessage(std::shared_ptr<CombinedMessage> &output);
        /* 仅当 features message 存在时，按时间戳输出下一捆数据 */
        bool PopPackedMessage(std::shared_ptr<CombinedMessage> &output);
    };

}