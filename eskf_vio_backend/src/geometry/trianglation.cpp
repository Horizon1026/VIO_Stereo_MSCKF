/* 内部依赖 */
#include <sequence_propagator.hpp>
#include <math_lib.hpp>
#include <log_api.hpp>
#include<trianglation.hpp>
/* 外部依赖 */
namespace ESKF_VIO_BACKEND {

bool Trianglator::TrianglateAnalytic(const std::vector<Quaternion> &q_wc,
                                const std::vector<Vector3> &p_wc,
                                const std::vector<Vector2> &norm,
                                Vector3 &p_w)
{
    int observe_num = q_wc.size();
    // Eigen::Matrix<Scalar,observe_num *2,4> design_matrix = Eigen::Matrix<Scalar,obseve_num *2,4>::Zero();
    Matrix design_matrix;
    design_matrix.resize(observe_num * 2, 4);
    for (int i =0; i < observe_num; i++)
    {
        auto Pose =  Utility::qtToTransformMatrix(q_wc[i], p_wc[i]);
        design_matrix.row(2*i) = norm[i][0] * Pose.row(2) - Pose.row(0);
        design_matrix.row(2*i+1) = norm[i][1] * Pose.row(2) - Pose.row(1);
    }
    Eigen::Matrix<Scalar,4,1> triangulated_point;
    triangulated_point =
              design_matrix.jacobiSvd(Eigen::ComputeFullV).matrixV().rightCols<1>();//TODO: discuss if thin or full
    // 齐次向量归一化
    p_w(0) = triangulated_point(0) / triangulated_point(3);
    p_w(1) = triangulated_point(1) / triangulated_point(3);
    p_w(2) = triangulated_point(2) / triangulated_point(3);
    return true;
}

}