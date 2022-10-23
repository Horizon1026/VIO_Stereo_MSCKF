/* 外部依赖 */
#define TEST 0
#include <fstream>
#include <iostream>

/* 内部依赖 */
#include <backend.hpp>
#include <log_api.hpp>
using namespace ESKF_VIO_BACKEND;
using Scalar = ESKF_VIO_BACKEND::Scalar;

Trianglator solver;

bool test_triangulation() {
    int poseNums = 6;      // 相机数目

    Vector3 landmark{2, 2, 2};
    std::vector<Vector2> observe_vec;
    std::vector<Quaternion> quat_vec;
    std::vector<Vector3> t_vec;

    Scalar radius = 8;
    for (int n = 0; n < poseNums; ++n) {
        Scalar theta = n * 2 * M_PI / (poseNums * 16); // 1/16 圆弧
        // 绕 z 轴 旋转
        Matrix33 R;
        R = Eigen::AngleAxis<Scalar>(theta, Vector3::UnitZ());
        Vector3 t = Vector3(radius * cos(theta) - radius, radius * sin(theta), 1 * sin(2 * theta));
        //cameraPoses.push_back(Frame(R, t));
        auto res = R * landmark + t;
        observe_vec.emplace_back(res[0]/res[2],res[1]/res[2]);
        quat_vec.emplace_back(R);
        t_vec.emplace_back(t);
        // std::cout<<"res: "<<observe_vec.back()<<std::endl;
    }

    Vector3 lm_esti;

    solver.TrianglateAnalytic(quat_vec, t_vec, observe_vec, lm_esti);
    if ((lm_esti - landmark).norm() <0.0001)
    {
        LogInfo("triangulation test passed");
        return true;
    }
    else
    {
        LogError("triangulation test not passed");
        return false;
    }
}

bool test_getReprojectionCost() {
    Vector3 landmark{2, 2, 2};
    Scalar radius = 4;
    Scalar theta = 2 * M_PI / (16); // 1/16 圆弧
    // 绕 z 轴 旋转
    Matrix33 R;
    R = Eigen::AngleAxis<Scalar>(theta, Vector3::UnitZ());
    Quaternion quat(R);
    Vector3 t = Vector3(radius * cos(theta) - radius, radius * sin(theta), 1 * sin(2 * theta));
    //cameraPoses.push_back(Frame(R, t));
    auto res = R * landmark + t;
    Vector2 proj_pt{res[0]/res[2], res[1]/res[2]};
    // Vector3 lm_init(landmark[0]/landmark[2],landmark[1]/landmark[2],1.0/landmark[2]);
    auto diff = solver.getReprojectionCost(quat,  t, landmark, proj_pt);
    if (std::abs(diff)<0.0001)
    {
        LogInfo("test_getReprojectionCost passed");
        return true;
    }
    else
    {
        LogError("test_getReprojectionCost not passed");
        return true;
    }
}

bool test_TrianglateIterative() {
    int poseNums = 6;      // 相机数目
    Vector3 landmark{2, 2, 2};
    std::vector<Vector2> observe_vec;
    std::vector<Quaternion> quat_vec;
    std::vector<Vector3> t_vec;

    Scalar radius = 8;
    for (int n = 0; n < poseNums; ++n) {
        Scalar theta = n * 2 * M_PI / (poseNums * 16); // 1/16 圆弧
        // 绕 z 轴 旋转
        Matrix33 R;
        R = Eigen::AngleAxis<Scalar>(theta, Vector3::UnitZ());
        Vector3 t = Vector3(radius * cos(theta) - radius, radius * sin(theta), 1 * sin(2 * theta));
        //cameraPoses.push_back(Frame(R, t));
        auto res = R * landmark + t;
        observe_vec.emplace_back(res[0]/res[2],res[1]/res[2]);
        quat_vec.emplace_back(R);
        t_vec.emplace_back(t);
    }

    Vector3 lm_noise = Vector3(0.4, 0.4, 0.4);
    Vector3 lm_esti = landmark + lm_noise;

    solver.TrianglateIterative(quat_vec, t_vec, observe_vec, lm_esti);
    // std::cout<<"lm_esti :"<<lm_esti.transpose()<<std::endl;
    if ((lm_esti - landmark).norm() < 1e-6) {
        LogInfo("iterative triangulation test passed");
        return true;
    } else {
        LogError("iterative triangulation test not passed");
        return false;
    }

    return true;

}


int main(int argc, char **argv) {
    test_triangulation();
    test_getReprojectionCost();
    test_TrianglateIterative();
    return 0;
}