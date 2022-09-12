#include <include/data_loader/data_typedef.hpp>

namespace VIOBackend {
    /* 带参数的构造函数 */
    FeaturesMessage::FeaturesMessage(const std::vector<uint32_t> &ids,
                                     const std::vector<Eigen::Vector2f> &left,
                                     const std::vector<Eigen::Vector2f> &right,
                                     const std::vector<uint8_t> &flag,
                                     const fp64 &timeStamp) :
        ids(ids), left(left), right(right), flag(flag), timeStamp(timeStamp) {
        if (this->ids.size() != this->left.size() || this->ids.size() != this->flag.size()) {
            this->Clear();
            return;
        }
        if (this->left.size() != this->right.size()) {
            this->right.clear();
        }
    }


    /* 清空保存的数据 */
    void FeaturesMessage::Clear(void) {
        this->flag.clear();
        this->ids.clear();
        this->left.clear();
        this->right.clear();
        this->timeStamp = static_cast<fp64>(0);
    }


    /* 带参数的构造函数 */
    IMUMessage::IMUMessage(const Eigen::Vector3f &gyro,
                           const Eigen::Vector3f &acc,
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