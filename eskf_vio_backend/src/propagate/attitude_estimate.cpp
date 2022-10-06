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
            if (std::fabs(g_word.norm() - g_imu.norm()) > fp64(0.05)) {
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
            Quaternion dq = Utility::DeltaQ(gyrCorrect * static_cast<Scalar>(item_1->timeStamp - item_0->timeStamp));
            item_1->q_wb = item_0->q_wb * dq;
            item_1->q_wb.normalize();
        }
        return true;
    }


    /* 提取出指定时刻点附近的姿态估计结果 */
    bool AttitudeEstimate::GetAttitude(const fp64 timeStamp, const fp64 threshold, Quaternion &atti) {
        // 从后往前找
        for (auto it = this->items.rbegin(); it != this->items.rend(); ++it) {
            if ((*it)->timeStamp <= timeStamp) {
                if (it == this->items.rbegin()) {
                    // 如果这个 item 就是遍历到的第一个，相差不大就可以返回
                    if (std::fabs((*it)->timeStamp - timeStamp) < threshold) {
                        atti = (*it)->q_wb;
                        return true;
                    } else {
                        return false;
                    }
                } else {
                    // 如果不是遍历的第一个，那么和他上一个作比较，选择相近的
                    auto it_pre = std::prev(it);
                    if (std::fabs((*it_pre)->timeStamp - timeStamp) < std::fabs((*it)->timeStamp - timeStamp)) {
                        atti = (*it_pre)->q_wb;
                        return true;
                    } else {
                        atti = (*it)->q_wb;
                        return true;
                    }
                }
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
        return true;
    }
}