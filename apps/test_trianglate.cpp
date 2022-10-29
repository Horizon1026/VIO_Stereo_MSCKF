/* 外部依赖 */
#define TEST 0
#include <fstream>
#include <iostream>

/* 内部依赖 */
#include <trianglation.hpp>
#include <log_api.hpp>
using namespace ESKF_VIO_BACKEND;
using Scalar = ESKF_VIO_BACKEND::Scalar;

Trianglator solver;

bool TestTrianglateAnalytic() {
    uint32_t poseNums = 6;      // 相机数目

    Vector3 p_w{2, 2, 2};
    std::vector<Vector2> observe_vec;
    std::vector<Quaternion> q_wc_vec;
    std::vector<Vector3> p_wc_vec;

    Scalar radius = 8;
    for (uint32_t n = 0; n < poseNums; ++n) {
        Scalar theta = n * 2 * M_PI / (poseNums * 16); // 1/16 圆弧
        // 绕 z 轴 旋转
        Matrix33 R_cw;
        R_cw = Eigen::AngleAxis<Scalar>(theta, Vector3::UnitZ());
        Vector3 p_cw = Vector3(radius * cos(theta) - radius, radius * sin(theta), 1 * sin(2 * theta));
        //cameraPoses.push_back(Frame(R_cw, p_cw));
        auto p_c = R_cw * p_w + p_cw;
        observe_vec.emplace_back(p_c[0] / p_c[2], p_c[1] / p_c[2]);
        q_wc_vec.emplace_back(R_cw.transpose());
        p_wc_vec.emplace_back(- R_cw.transpose() * p_cw);
    }

    Vector3 res_p_w;
    solver.TrianglateAnalytic(q_wc_vec, p_wc_vec, observe_vec, res_p_w);
    std::cout << "TestTrianglateAnalytic :";
    std::cout << "set p_w is " << p_w.transpose() << ", res p_w is " << res_p_w.transpose() << std::endl;
    return true;
}


bool TestTrianglateIterative() {
    uint32_t poseNums = 6;      // 相机数目
    Vector3 p_w{2, 2, 2};
    std::vector<Vector2> observe_vec;
    std::vector<Quaternion> q_wc_vec;
    std::vector<Vector3> p_wc_vec;

    Scalar radius = 8;
    for (uint32_t n = 0; n < poseNums; ++n) {
        Scalar theta = n * 2 * M_PI / (poseNums * 16); // 1/16 圆弧
        // 绕 z 轴 旋转
        Matrix33 R_cw;
        R_cw = Eigen::AngleAxis<Scalar>(theta, Vector3::UnitZ());
        Vector3 p_cw = Vector3(radius * cos(theta) - radius, radius * sin(theta), 1 * sin(2 * theta));
        auto p_c = R_cw * p_w + p_cw;
        observe_vec.emplace_back(p_c[0] / p_c[2], p_c[1] / p_c[2]);
        q_wc_vec.emplace_back(R_cw.transpose());
        p_wc_vec.emplace_back(- R_cw.transpose() * p_cw);
    }

    Vector3 p_w_noise = Vector3(0.4, 0.4, 0.4);
    Vector3 res_p_w = p_w + p_w_noise;

    solver.TrianglateIterative(q_wc_vec, p_wc_vec, observe_vec, res_p_w);
    std::cout << "TestTrianglateIterative :";
    std::cout << "set p_w is " << res_p_w.transpose() << ", res p_w is " << res_p_w.transpose() << std::endl;
    return true;
}


int main(int argc, char **argv) {
    TestTrianglateAnalytic();
    TestTrianglateIterative();
    return 0;
}