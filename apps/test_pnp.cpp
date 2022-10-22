/* 外部依赖 */
#include <fstream>
#include <iostream>

/* 内部依赖 */
#include <perspective_n_point.hpp>
using namespace ESKF_VIO_BACKEND;
using Scalar = ESKF_VIO_BACKEND::Scalar;


int main(int argc, char **argv) {
    // 初始化测试
    std::cout << ">> Perspective-n-Point Module Test" << std::endl;

    // 构造 3D 点云
    std::vector<Vector3> pts_3d;
    for (int i = 1; i < 10; i++) {
        for (int j = 1; j < 10; j++) {
            for (int k = 1; k < 10; k++) {
                pts_3d.emplace_back(Vector3(i, j, k * 2.0));
            }
        }
    }

    // 定义相机位姿（相对于世界坐标系）
    Matrix33 R_wc = Matrix33::Identity();
    Vector3 p_wc = Vector3(1, -3, 0);

    // 将 3D 点云通过两帧位姿映射到对应的归一化平面上，构造匹配点对
    std::vector<Vector2> pts_2d;
    for (unsigned long i = 0; i < pts_3d.size(); i++) {
        Vector3 p_c = R_wc.transpose() * (pts_3d[i] - p_wc);
        pts_2d.emplace_back(Vector2(p_c(0) / p_c(2), p_c(1) / p_c(2)));
    }

    // 随机给匹配点增加 outliers
    // std::vector<int> outliersIndex;
    // for (unsigned int i = 0; i < pts_3d.size() / 100; i++) {
    //     int idx = rand() % pts_3d.size();
    //     pts_2d[idx](0, 0) = 0;
    //     outliersIndex.emplace_back(idx);
    // }

    // 调用 pnp 接口求解问题
    Quaternion res_q_wc = Quaternion::Identity();
    Vector3 res_p_wc = Vector3::Zero();
    PnPSolver pnpSolver;
    pnpSolver.EstimatePose(pts_3d, pts_2d, res_q_wc, res_p_wc);
    std::cout << "res_q_wc is [" << res_q_wc.w() << ", " << res_q_wc.x() << ", " << res_q_wc.y() <<
        ", " << res_q_wc.z() << "]\n";
    std::cout << "res_p_wc is " << res_p_wc.transpose() << std::endl;

    return 0;
}