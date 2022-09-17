#include <include/data_manager/data_typedef.hpp>
#if STD_COUT_INFO
    #include <iostream>
#endif

namespace ESKF_VIO_BACKEND {
    /* 清空保存的数据 */
    void FeatureObserve::Clear(void) {
        this->norms.clear();
    }


    /* 获取指定 camera ID 的观测 */
    bool FeatureObserve::GetNorm(const uint32_t cameraID, Eigen::Matrix<Scalar, 2, 1> &norm) {
        auto it = this->norms.find(cameraID);
        if (it == this->norms.end()) {
            return false;
        } else {
            norm = (*it).second;
            return true;
        }
    }


    /* 带参数的构造函数 */
    FeaturesMessage::FeaturesMessage(const std::vector<uint32_t> &ids,
                                     const std::vector<std::shared_ptr<FeatureObserve>> &observes,
                                     const std::vector<uint8_t> &flag,
                                     const fp64 &timeStamp) :
        ids(ids), observes(observes), flag(flag), timeStamp(timeStamp) {
        if (this->ids.size() != this->observes.size()) {
            this->Clear();
            return;
        }
    }


    /* 清空保存的数据 */
    void FeaturesMessage::Clear(void) {
        this->flag.clear();
        this->ids.clear();
        this->observes.clear();
        this->timeStamp = static_cast<fp64>(0);
    }


    /* 自我打印保存信息 */
    void FeaturesMessage::Information(void) {
    #if STD_COUT_INFO
        std::cout << ">> Features Message at time stamp " << this->timeStamp << "s:\n";
        for (uint32_t i = 0; i < this->ids.size(); ++i) {
            std::cout << "     feature id " << this->ids[i] << " is observed in:\n";
            for (auto it = this->observes[i]->norms.begin(); it != this->observes[i]->norms.end(); ++it) {
                std::cout << "       camera id " << it->first << " [" << it->second.transpose() << "]\n";
            }
        }
    #endif
    }


    /* 带参数的构造函数 */
    IMUMessage::IMUMessage(const Eigen::Matrix<Scalar, 3, 1> &gyro,
                           const Eigen::Matrix<Scalar, 3, 1> &acc,
                           const fp64 &timeStamp) :
        gyro(gyro), acc(acc), timeStamp(timeStamp) {}

    
    /* 自我打印保存信息 */
    void IMUMessage::Information(void) {
    #if STD_COUT_INFO
        std::cout << ">> IMU Message at time stamp " << this->timeStamp << "s:";
        std::cout << "     gyro [" << this->gyro.transpose() << "]\n";
        std::cout << "     acc  [" << this->acc.transpose() << "]\n";
    #endif
    }


    /* 带参数的构造函数 */
    CombinedMessage::CombinedMessage(const std::shared_ptr<FeaturesMessage> &featMeas,
                                     const std::shared_ptr<IMUMessage> &imuMeas) :
        featMeas(featMeas) {
        this->imuMeas.clear();
        this->imuMeas.emplace_back(imuMeas);
    }
    CombinedMessage::CombinedMessage(const std::shared_ptr<FeaturesMessage> &featMeas,
                                     const std::vector<std::shared_ptr<IMUMessage>> &imuMeas) :
        featMeas(featMeas), imuMeas(imuMeas) {}


    /* 清空保存的数据 */
    void CombinedMessage::Clear(void) {
        this->imuMeas.clear();
        this->featMeas = nullptr;
    }


    /* 自我打印保存信息 */
    void CombinedMessage::Information(void) {
    #if STD_COUT_INFO
        if (this->featMeas != nullptr) {
            this->featMeas->Information();
        }
        for (uint32_t i = 0; i < this->imuMeas.size(); ++i) {
            this->imuMeas[i]->Information();
        }
    #endif
    }
}