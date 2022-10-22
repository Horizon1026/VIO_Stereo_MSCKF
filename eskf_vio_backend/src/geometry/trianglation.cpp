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

/*measure the accuracy of the reprojection estimation*/
Scalar Trianglator::getReprojectionCost(const Quaternion& q, const Vector3& t, const Vector3& lm, const Vector2& groundtruth ) { 
  // Compute hi1, hi2, and hi3 as Equation (37).
  const Scalar& alpha = lm(0)/lm(2);
  const Scalar& beta = lm(1)/lm(2);
  const Scalar& rho = 1.0/lm(2);

  Vector3 h = q.toRotationMatrix()* Vector3(alpha, beta, 1.0) + rho*t;
  Scalar& h1 = h(0);
  Scalar& h2 = h(1);
  Scalar& h3 = h(2);

  // Predict the feature observation in ci frame.
  Vector2 z_hat(h1/h3, h2/h3);

  // Compute the residual.
  return (z_hat-groundtruth).squaredNorm();
}

}