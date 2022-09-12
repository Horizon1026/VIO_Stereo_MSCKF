#include <include/data_loader/data_typedef.hpp>

namespace ESKF_VIO_BACKEND {
    /* 清空保存的数据 */
    void FeatureObserve::Clear(void) {
        this->uv.clear();
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


    /* 带参数的构造函数 */
    IMUMessage::IMUMessage(const Eigen::Matrix<Scalar, 3, 1> &gyro,
                           const Eigen::Matrix<Scalar, 3, 1> &acc,
                           const fp64 &timeStamp) :
        gyro(gyro), acc(acc), timeStamp(timeStamp) {}


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
}