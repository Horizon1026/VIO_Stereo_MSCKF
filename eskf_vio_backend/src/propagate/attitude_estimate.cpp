/* 内部依赖 */
#include <attitude_estimate.hpp>
#include <imu_state.hpp>
#include <math_lib.hpp>
#include <log_api.hpp>
/* 外部依赖 */

namespace ESKF_VIO_BACKEND {
    /* 姿态解算进行一步更新 */
    bool AttitudeEstimate::Propagate(const Vector3 &accel, const Vector3 &gyro, const fp64 timeStamp) {
        if (this->items.empty()) {
            /* 如果队列为空，则利用 accel 来给一个 attitude 初值 */
            Vector3 g_imu = accel - this->bias_a;
            Vector3 g_word = IMUFullState::gravity_w;
            // 首先需要检查 accel 的模长是否和重力加速度一致，不一致的话不能初始化
            if (std::fabs(g_word.norm() - g_imu.norm()) > fp64(0.04)) {
                LogInfo(">> Attitude estimator: imu accel norm " << g_imu.norm() <<
                    " is not equal to gravity, attitude estimator init failed.");
                return false;
            }
            // 将重力加速度向量转化为旋转矩阵
            Vector3 g_cross = g_imu.cross(g_word);
            Scalar norm = g_cross.norm();
            Vector3 u = g_cross / norm;
            Scalar theta = std::atan2(norm, g_imu.transpose() * g_word);
            Vector3 dq_v = u * theta;
            Quaternion q_wb = Utility::DeltaQ(dq_v).normalized();
            // 构造新的 item
            std::shared_ptr<AttitudeEstimateItem> newItem(new AttitudeEstimateItem());
            this->items.emplace_back(newItem);
            newItem->accel = accel - this->bias_a;
            newItem->gyro = gyro - this->bias_g;
            newItem->q_wb = q_wb.normalized();
            newItem->timeStamp = timeStamp;
            LogInfo(">> Attitude estimator init successfully.");
        } else {
            /* 在上一步的基础上 propagate 姿态 */
            auto item_0 = this->items.back();
            if (item_0->timeStamp > timeStamp) {
                return false;
            }
            std::shared_ptr<AttitudeEstimateItem> item_1(new AttitudeEstimateItem());
            this->items.emplace_back(item_1);
            item_1->accel = accel - this->bias_a;
            item_1->gyro = gyro - this->bias_g;
            item_1->timeStamp = timeStamp;
            
            // 利用加速度观测，修正角速度中值
            Vector3 acc_b_measure = accel / accel.norm();
            Vector3 acc_b_target = item_0->q_wb.inverse() * IMUFullState::gravity_w / IMUFullState::gravity_w.norm();
            Vector3 err = acc_b_measure.cross(acc_b_target);
            Vector3 gyrMid = Scalar(0.5) * (item_0->gyro + item_1->gyro);
            this->errInt += err * this->Ki;
            Vector3 gyrCorrect = gyrMid + err * this->Kp + this->errInt;

            // 更新姿态
            Quaternion dq = Utility::DeltaQ(- gyrCorrect * static_cast<Scalar>(item_1->timeStamp - item_0->timeStamp));
            item_1->q_wb = item_0->q_wb * dq;
            item_1->q_wb.normalize();
        }
        return true;
    }


    /* 提取出指定时刻点附近的姿态估计结果 */
    bool AttitudeEstimate::GetAttitude(const fp64 timeStamp, const fp64 threshold, Quaternion &atti) {
        if (this->items.empty()) {
            return false;
        }
        // 最大的时间戳也小于目标时间戳
        if (this->items.back()->timeStamp <= timeStamp) {
            if (timeStamp - this->items.back()->timeStamp < threshold) {
                atti = this->items.back()->q_wb;
                return true;
            } else {
                return false;
            }
        }

        // 最小的时间戳也大于目标时间戳
        if (this->items.size() > 1 && this->items.front()->timeStamp >= timeStamp) {
            if (this->items.front()->timeStamp - timeStamp < threshold) {
                atti = this->items.front()->q_wb;
                return true;
            } else {
                return false;
            }
        }

        // 从后往前找
        for (auto it = this->items.rbegin(); it != this->items.rend(); ++it) {
            // 找到的 it 是时间戳小于目标的第一个元素，且不可能是首尾两个元素
            if ((*it)->timeStamp <= timeStamp) {
                const auto prevIt = std::prev(it);
                const fp64 prevDiff = (*prevIt)->timeStamp - timeStamp;
                const fp64 diff = timeStamp - (*it)->timeStamp;
                if (prevDiff < diff && prevDiff < threshold) {
                    atti = (*prevIt)->q_wb;
                    return true;
                } else if (diff < threshold) {
                    atti = (*it)->q_wb;
                    return true;
                }
                break;
            }
        }
        return false;
    }


    /* 设置 bias */
    bool AttitudeEstimate::SetBias(const Vector3 &bias_a, const Vector3 &bias_g) {
        this->bias_a = bias_a;
        this->bias_g = bias_g;
        return true;
    }


    /* 清除旧时刻的姿态解算结果 */
    bool AttitudeEstimate::CleanOldItems(const fp64 timeStamp, const Scalar threshold) {
        if (this->items.empty()) {
            return true;
        }
        while (Scalar(timeStamp - this->items.front()->timeStamp) > threshold) {
            this->items.pop_front();
        }
        LogInfo(">> Attitude estimator reset at " << timeStamp << "s, " << this->items.size() <<
            " items maintained.");
        return true;
    }
}