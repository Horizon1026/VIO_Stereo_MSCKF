/* 内部依赖 */
#include <sequence_propagator.hpp>
#include <math_lib.hpp>
#include <log_api.hpp>
#include<trianglation.hpp>
/* 外部依赖 */
namespace ESKF_VIO_BACKEND {

 Vector3 Trianglator::linearTriangulation(const Eigen::Matrix<Scalar, 4, 4>& P1, const Eigen::Matrix<Scalar, 4, 4>& P2,
 const Eigen::Matrix<Scalar, 2, 1>& x1, const Eigen::Matrix<Scalar, 2, 1>& x2)
{
    // allocate memory for design matrix
    // Mat_<float> A(4, 4);
    Eigen::Matrix<Scalar, 4, 4> A;

    // create design matrix
    // first row	x1(0, i) * P1(2, :) - P1(0, :)
    A(0, 0) = x1(0) * P1(2, 0) - P1(0, 0);
    A(0, 1) = x1(0) * P1(2, 1) - P1(0, 1);
    A(0, 2) = x1(0) * P1(2, 2) - P1(0, 2);
    A(0, 3) = x1(0) * P1(2, 3) - P1(0, 3);
    // second row	x1(1, i) * P1(2, :) - P1(1, :)
    A(1, 0) = x1(1) * P1(2, 0) - P1(1, 0);
    A(1, 1) = x1(1) * P1(2, 1) - P1(1, 1);
    A(1, 2) = x1(1) * P1(2, 2) - P1(1, 2);
    A(1, 3) = x1(1) * P1(2, 3) - P1(1, 3);
    // third row	x2(0, i) * P2(3, :) - P2(0, :)
    A(2, 0) = x2(0) * P2(2, 0) - P2(0, 0);
    A(2, 1) = x2(0) * P2(2, 1) - P2(0, 1);
    A(2, 2) = x2(0) * P2(2, 2) - P2(0, 2);
    A(2, 3) = x2(0) * P2(2, 3) - P2(0, 3);
    // first row	x2(1, i) * P2(3, :) - P2(1, :)
    A(3, 0) = x2(1) * P2(2, 0) - P2(1, 0);
    A(3, 1) = x2(1) * P2(2, 1) - P2(1, 1);
    A(3, 2) = x2(1) * P2(2, 2) - P2(1, 2);
    A(3, 3) = x2(1) * P2(2, 3) - P2(1, 3);

    Eigen::JacobiSVD<Matrix> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);

    auto tmp = svd.matrixV().transpose().row(3).transpose();

    Vector3 point3x;
    point3x << tmp(0)/tmp(3), tmp(1)/tmp(3), tmp(2)/tmp(3);

    return point3x;
}

}