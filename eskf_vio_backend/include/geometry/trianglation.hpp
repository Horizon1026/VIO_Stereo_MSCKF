#pragma once
/* 外部依赖 */
/* 内部依赖 */
#include <typedef.hpp>

namespace ESKF_VIO_BACKEND {
    class Trianglator {
    public:
        /* 解析法三角化需要的参数 */
        struct AnalyticTemp {
            Matrix A;
        } analyticTemp;
    public:
        /* 构造函数与析构函数 */
        Trianglator() = default;
        virtual ~Trianglator() = default;
    public:
        /* 数值解析法三角测量特征点 */
        bool TrianglateAnalytic(const std::vector<Quaternion> &q_wc,
                                const std::vector<Vector3> &p_wc,
                                const std::vector<Vector2> &norm,
                                Vector3 &p_w);

        /* 高斯牛顿迭代法三角测量特征点 */
        bool TrianglateIterative(const std::vector<Quaternion> &q_wc,
                                 const std::vector<Vector3> &p_wc,
                                 const std::vector<Vector2> &norm,
                                 Vector3 &p_w);
    public:
        Scalar ComputeResidual(const Quaternion &q_wc,
                               const Vector3 &p_wc,
                               const Vector3 &p_w,
                               const Vector2 &observe);
        void CumputeJacobian(const Quaternion &q_c0_ci,
                             const Vector3 &t_c0_ci,
                             const Vector3 &x,
                             const Vector2 &z,
                             Matrix23 &J,
                             Vector2 &r,
                             Scalar &w);

    };
}