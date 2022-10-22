/* 内部依赖 */
#include <perspective_n_point.hpp>
#include <math_lib.hpp>
#include <log_api.hpp>
/* 外部依赖 */

namespace ESKF_VIO_BACKEND {
    /* 使用所有输入的点进行估计，输入 pose 为初值 */
    bool PnPSolver::EstimatePose(const std::vector<Vector3> &pts_3d,
                                 const std::vector<Vector2> &pts_2d,
                                 Quaternion &q_wc,
                                 Vector3 &p_wc) {
        if (pts_3d.size() != pts_2d.size() || pts_3d.empty()) {
            return false;
        }
        Matrix66 H;
        Vector6 b;
        Vector3 p_c;
        Vector2 norm;
        Vector2 residual;
        Matrix26 jacobian;
        Vector6 dx;
        uint32_t iter = this->maxIterateTimes;
        while (iter) {
            H.setZero();
            b.setZero();

            // 遍历每一个 feature，填充高斯牛顿方程 Hx = b
            for (uint32_t i = 0; i < pts_3d.size(); ++i) {
                // 计算临时变量
                p_c = q_wc.inverse() * (pts_3d[i] - p_wc);
                if (std::fabs(p_c.z()) < 1e-6) {
                    continue;
                }
                Scalar invDep = Scalar(1) / p_c.z();
                Scalar invDep2 = invDep * invDep;
                // 计算残差和雅可比
                norm << p_c(0) / p_c(2), p_c(1) / p_c(2);
                residual = norm - pts_2d[i];
                Matrix23 jacobian_2d_3d;
                jacobian_2d_3d << invDep, 0, - p_c(0) * invDep2,
                                  0, invDep, - p_c(1) * invDep2;
                jacobian.block<2, 3>(0, 0) = jacobian_2d_3d * (- Matrix33(q_wc.inverse()));
                jacobian.block<2, 3>(0, 3) = jacobian_2d_3d * Utility::SkewSymmetricMatrix(p_c);
                // 填充高斯牛顿方程
                H += jacobian.transpose() * jacobian;
                b += - jacobian.transpose() * residual;
            }

            // 求解高斯牛顿增量方程
            dx = H.ldlt().solve(b);
            Scalar norm_dx = dx.norm();
            LogDebug("delta_x is [" << norm_dx << "] " << dx.transpose());
            if (std::isnan(norm_dx) == true) {
                return false;
            }

            // 更新此次迭代结果
            p_wc += dx.head<3>();
            q_wc = (q_wc * Utility::DeltaQ(dx.tail<3>())).normalized();

            // 提前终止迭代条件
            if (norm_dx < this->maxNormDeltaX) {
                break;
            }
            --iter;
        }
        return true;
    }


    /* 采用 RANSAC 方法，挑选输入的点进行估计，输入 pose 为初值 */
    bool PnPSolver::EstimatePoseRANSAC(const std::vector<Vector3> &pts_3d,
                                       const std::vector<Vector2> &pts_2d,
                                       Quaternion &q_wc,
                                       Vector3 &p_wc) {
        return true;
    }


    /* 基于 Huber 和函数，抑制 outliers，使用所有输入的点进行估计，输入 pose 为初值 */
    bool PnPSolver::EstimatePoseKernel(const std::vector<Vector3> &pts_3d,
                                       const std::vector<Vector2> &pts_2d,
                                       Quaternion &q_wc,
                                       Vector3 &p_wc) {
        if (pts_3d.size() != pts_2d.size() || pts_3d.empty()) {
            return false;
        }
        Matrix66 H;
        Vector6 b;
        Vector3 p_c;
        Vector2 norm;
        Vector2 residual;
        Matrix26 jacobian;
        Vector6 dx;
        uint32_t iter = this->maxIterateTimes;
        while (iter) {
            H.setZero();
            b.setZero();

            // 遍历每一个 feature，填充高斯牛顿方程 Hx = b
            for (uint32_t i = 0; i < pts_3d.size(); ++i) {
                // 计算临时变量
                p_c = q_wc.inverse() * (pts_3d[i] - p_wc);
                if (std::fabs(p_c.z()) < 1e-6) {
                    continue;
                }
                Scalar invDep = Scalar(1) / p_c.z();
                Scalar invDep2 = invDep * invDep;
                // 计算残差和雅可比
                norm << p_c(0) / p_c(2), p_c(1) / p_c(2);
                residual = norm - pts_2d[i];
                Matrix23 jacobian_2d_3d;
                jacobian_2d_3d << invDep, 0, - p_c(0) * invDep2,
                                  0, invDep, - p_c(1) * invDep2;
                jacobian.block<2, 3>(0, 0) = jacobian_2d_3d * (- Matrix33(q_wc.inverse()));
                jacobian.block<2, 3>(0, 3) = jacobian_2d_3d * Utility::SkewSymmetricMatrix(p_c);
                // 计算核函数权重
                Scalar r_norm = residual.norm();
                Scalar kernel = this->Cauchy(Scalar(1), r_norm);
                // 填充高斯牛顿方程
                H += jacobian.transpose() * jacobian * kernel;
                b += - jacobian.transpose() * residual * kernel;
            }

            // 求解高斯牛顿增量方程
            dx = H.ldlt().solve(b);
            Scalar norm_dx = dx.norm();
            LogDebug("delta_x is [" << norm_dx << "] " << dx.transpose());
            if (std::isnan(norm_dx) == true) {
                return false;
            }

            // 更新此次迭代结果
            p_wc += dx.head<3>();
            q_wc = (q_wc * Utility::DeltaQ(dx.tail<3>())).normalized();

            // 提前终止迭代条件
            if (norm_dx < this->maxNormDeltaX) {
                break;
            }
            --iter;
        }
        return true;
    }
}