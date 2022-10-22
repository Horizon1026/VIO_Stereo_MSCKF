/* 内部依赖 */
#include <perspective_n_point.hpp>
#include <math_lib.hpp>
#include <log_api.hpp>
/* 外部依赖 */
#include <set>

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


    /* P3P，使用 3 对输入的点进行估计，输入 pose 为初值 */
    bool PnPSolver::EstimatePoseP3P(const std::vector<Vector3> &pts_3d,
                                    const std::vector<Vector2> &pts_2d,
                                    Quaternion &q_wc,
                                    Vector3 &p_wc) {
        if (pts_3d.size() != pts_2d.size() || pts_3d.size() < 3) {
            return false;
        }
        // TODO:

        return true;
    }


    /* 采用 RANSAC 方法，挑选输入的点进行估计，输入 pose 为初值 */
    bool PnPSolver::EstimatePoseRANSAC(const std::vector<Vector3> &pts_3d,
                                       const std::vector<Vector2> &pts_2d,
                                       Quaternion &q_wc,
                                       Vector3 &p_wc) {
        if (pts_3d.size() != pts_2d.size() || pts_3d.empty()) {
            return false;
        }
        uint32_t iter = this->ransacParam.maxIterateTimes;
        Quaternion best_q_wc = q_wc;
        Vector3 best_p_wc = p_wc;
        uint32_t best_score = 0;
        uint32_t score = 0;

        // RANSAC 循环
        std::set<uint32_t> indice;
        std::vector<Vector3> subPts3d;
        std::vector<Vector2> subPts2d;
        subPts3d.reserve(3);
        subPts2d.reserve(3);
        while (iter) {
            // 随机选择三对 3D-2D
            indice.clear();
            subPts3d.clear();
            subPts2d.clear();
            while (indice.size() < 3) {
                indice.insert(std::rand() % pts_3d.size());
            }
            for (auto it = indice.begin(); it != indice.end(); ++it) {
                subPts3d.emplace_back(pts_3d[*it]);
                subPts2d.emplace_back(pts_2d[*it]);
            }
            // 代入 PnP 模型，计算模型参数（这里如果能换成 P3P 模型会更好）
            q_wc = best_q_wc;
            p_wc = best_p_wc;
            this->EstimatePose(subPts3d, subPts2d, q_wc, p_wc);
            // 将模型代入所有点对，计算得分
            score = 0;
            for (uint32_t i = 0; i < pts_3d.size(); ++i) {
                Vector3 p_c = q_wc.inverse() * (pts_3d[i] - p_wc);
                Vector2 r = Vector2(p_c(0) / p_c(2), p_c(1) / p_c(2)) - pts_2d[i];
                if (r.squaredNorm() < this->ransacParam.inlierThres) {
                    ++score;
                }
            }
            // 若得分更高，则更新模型参数
            if (score > best_score) {
                best_score = score;
                best_q_wc = q_wc;
                best_p_wc = p_wc;
            }
            // 提前终止迭代
            if (Scalar(score) / Scalar(pts_3d.size()) > this->ransacParam.inlierRateThres) {
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
                                       Vector3 &p_wc,
                                       std::vector<bool> &isInlier) {
        RETURN_FALSE_IF_FALSE(this->EstimatePoseRANSAC(pts_3d, pts_2d, q_wc, p_wc));
        isInlier.resize(pts_3d.size(), true);
        for (uint32_t i = 0; i < pts_3d.size(); ++i) {
            Vector3 p_c = q_wc.inverse() * (pts_3d[i] - p_wc);
            Vector2 r = Vector2(p_c(0) / p_c(2), p_c(1) / p_c(2)) - pts_2d[i];
            if (r.squaredNorm() >= this->ransacParam.inlierThres) {
                isInlier[i] = false;
            }
        }
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