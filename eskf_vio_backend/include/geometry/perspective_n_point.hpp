#pragma once
/* 外部依赖 */
/* 内部依赖 */
#include <typedef.hpp>

namespace ESKF_VIO_BACKEND {
    /* PnP 问题求解器 */
    class PnPSolver {
    public:
        /* 相关参数 */
        // 最大迭代次数
        uint32_t maxIterateTimes = 10;
        // delta_x 的收敛判断阈值
        Scalar maxNormDeltaX = Scalar(1e-6);

    public:
        /* 构造函数与析构函数 */
        PnPSolver() {}
        ~PnPSolver() {}
    public:
        /* 使用所有输入的点进行估计，输入 pose 为初值 */
        bool EstimatePose(const std::vector<Vector3> &pts_3d,
                          const std::vector<Vector2> &pts_2d,
                          Quaternion &q_wc,
                          Vector3 &p_wc);
        /* 采用 RANSAC 方法，挑选输入的点进行估计，输入 pose 为初值 */
        bool EstimatePoseRANSAC(const std::vector<Vector3> &pts_3d,
                                const std::vector<Vector2> &pts_2d,
                                Quaternion &q_wc,
                                Vector3 &p_wc);
        /* 基于核函数，抑制 outliers，使用所有输入的点进行估计，输入 pose 为初值 */
        bool EstimatePoseKernel(const std::vector<Vector3> &pts_3d,
                                const std::vector<Vector2> &pts_2d,
                                Quaternion &q_wc,
                                Vector3 &p_wc);

    private:
        /* Huber 核权重 */
        inline Scalar Huber(Scalar param, Scalar x) {
            Scalar huber = Scalar(1);
            if (x > param) {
                huber = Scalar(2) * std::sqrt(x) * param - param * param;
                huber /= x;
            }
            return huber;
        }

        /* Cauchy 核权重 */
        inline Scalar Cauchy(Scalar param, Scalar x) {
            Scalar cauchy = Scalar(1);
            Scalar param2 = param * param;
            if (x > param) {
                cauchy = param2 * std::log(Scalar(1) / param2 + Scalar(1));
                cauchy /= x;
            }
            return cauchy;
        }
    };
}