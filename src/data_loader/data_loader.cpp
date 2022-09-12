#include <include/data_loader/data_loader.hpp>

namespace VIOBackend {

    /* 清空缓冲区 */
    void DataLoader::Clear(void) {
        this->featMeas.clear();
        this->imuMeas.clear();
        this->timeStampOffset = static_cast<fp64>(-1);
    }


    /* 输入一帧特征点追踪结果 */
    bool DataLoader::PushFeaturesMessage(const std::shared_ptr<FeaturesMessage> &newFeatMeas) {
        // 检查 newFeatMeas 中数据是否合规
        if (newFeatMeas->ids.size() != newFeatMeas->left.size() ||
            newFeatMeas->ids.size() != newFeatMeas->flag.size()) {
            return false;
        }
        // 校准时间戳
        if (this->timeStampOffset < static_cast<fp64>(0)) {
            this->timeStampOffset = newFeatMeas->timeStamp;
        }
        newFeatMeas->timeStamp -= this->timeStampOffset;
        this->featMeas.emplace_back(newFeatMeas);
        return true;
    }


    /* 输入一刻 IMU 量测结果 */
    bool DataLoader::PushIMUMessage(const std::shared_ptr<IMUMessage> &newImuMeas) {
        // 校准时间戳
        if (this->timeStampOffset < static_cast<fp64>(0)) {
            this->timeStampOffset = newImuMeas->timeStamp;
        }
        newImuMeas->timeStamp -= this->timeStampOffset;
        this->imuMeas.emplace_back(newImuMeas);
        return true;
    }


    /* 按时间戳输出下一个数据 */
    bool DataLoader::PopOneMessage(std::shared_ptr<CombinedMessage> &output) {
        output->Clear();
        // 如果队列都不为空，则输出时间戳最旧的数据
        if (!this->imuMeas.empty() && !this->featMeas.empty()) {
            auto &imu = this->imuMeas.front();
            auto &feat = this->featMeas.front();
            if (feat->timeStamp < imu->timeStamp) {
                output->featMeas = feat;
                this->featMeas.pop_front();
            } else {
                output->imuMeas.emplace_back(imu);
                this->imuMeas.pop_front();
            }
            return true;
        }
        // 否则直接输出不为空的队列中时间戳最旧的
        if (!this->imuMeas.empty()) {
            output->imuMeas.emplace_back(this->imuMeas.front());
            this->imuMeas.pop_front();
            return true;
        } else if (!this->featMeas.empty()) {
            output->featMeas = this->featMeas.front();
            this->featMeas.pop_front();
            return true;
        }
        return false;
    }


    /* 仅当 features message 存在时，按时间戳输出下一捆数据 */
    bool DataLoader::PopPackedMessage(std::shared_ptr<CombinedMessage> &output) {
        output->Clear();
        // 如果队列为空，则返回 false
        if (this->imuMeas.empty() || this->featMeas.empty()) {
            return false;
        }
        // 如果当前 IMU 的最新量测的时间戳还没有超过队列中的首帧 features，则需要继续等待 IMU
        if (this->imuMeas.back()->timeStamp <= this->featMeas.front()->timeStamp) {
            return false;
        }
        // 如果首帧 features 来了之后才有有效的 IMU 消息，则丢弃初始 features，因为无法捆绑有效的 IMU 量测
        if (this->imuMeas.front()->timeStamp >= this->featMeas.front()->timeStamp) {
            this->featMeas.pop_front();
            return false;
        }
        // 通过上述条件，则提取出队列中的首帧 features 
        output->featMeas = this->featMeas.front();
        this->featMeas.pop_front();
        // 提取出与此 features 对应时间段的 IMU 量测序列
        output->imuMeas.reserve(20);
        while (this->imuMeas.front()->timeStamp <= output->featMeas->timeStamp) {
            output->imuMeas.emplace_back(this->imuMeas.front());
            this->imuMeas.pop_front();
        }
        // 如果最新一帧 IMU 量测的时间戳和 features 时间戳不一致，则线性插值一个 IMU 量测
        if (std::fabs(output->imuMeas.back()->timeStamp - output->featMeas->timeStamp) > static_cast<fp64>(1e-6)) {
            // 采用线性插值的方法，计算 features 时间戳对应位置的 IMU 量测
            std::shared_ptr<IMUMessage> mid(new IMUMessage());
            mid->timeStamp = output->featMeas->timeStamp;
            std::shared_ptr<IMUMessage> left = output->imuMeas.back();
            std::shared_ptr<IMUMessage> right = this->imuMeas.front();
            double rate = (mid->timeStamp - left->timeStamp) / (right->timeStamp - left->timeStamp);
            mid->gyro = left->gyro * (1 - rate) + right->gyro * rate;
            mid->acc = left->acc * (1 - rate) + right->acc * rate;
            // 添加时间戳与 features 时间戳同步的 IMU 量测
            output->imuMeas.emplace_back(mid);
            // 给下一帧相机量测设置起点
            this->imuMeas.push_front(mid);
        } else {
            // 给下一帧相机量测设置起点
            this->imuMeas.push_front(output->imuMeas.back());
        }
        return true;
    }
}