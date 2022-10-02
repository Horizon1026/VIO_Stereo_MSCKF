#pragma once
/* 外部依赖 */
/* 内部依赖 */
#include <typedef.hpp>

namespace ESKF_VIO_BACKEND {
    /* PnP 问题求解器 */
    class PnPSolver {
    public:
        // 相关参数
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
        /* 基于 Huber 和函数，抑制 outliers，使用所有输入的点进行估计，输入 pose 为初值 */
        bool EstimatePoseHuber(const std::vector<Vector3> &pts_3d,
                               const std::vector<Vector2> &pts_2d,
                               Quaternion &q_wc,
                               Vector3 &p_wc);
    };
}