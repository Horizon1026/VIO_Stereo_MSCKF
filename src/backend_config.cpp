/* 内部依赖 */
#include <include/backend.hpp>
/* 外部依赖 */
#include <fstream>
#include <iostream>

namespace ESKF_VIO_BACKEND {
    /* 后端优化器读取配置并初始化 */
    bool Backend::Initialize(const std::string &configPath) {
        Matrix tempMat;

        // 加载 IMU 噪声参数
        std::cout << ">> Load imu noise...\n";
        if (this->LoadMatrix(configPath + "/imu_na_ng_nwa_nwg.txt", 1, 4, tempMat) == true) {
            this->queue.noise_accel = tempMat(0, 0);
            this->queue.noise_gyro = tempMat(0, 1);
            this->queue.random_walk_accel = tempMat(0, 2);
            this->queue.random_walk_gyro = tempMat(0, 3);
            std::cout << "     imu accel noise : " << this->queue.noise_accel << "\n";
            std::cout << "     imu gyro noise : " << this->queue.noise_gyro << "\n";
            std::cout << "     imu accel random walk : " << this->queue.random_walk_accel << "\n";
            std::cout << "     imu gyro random walk : " << this->queue.random_walk_gyro << "\n";
        }

        // 加载 IMU bias 初值
        std::cout << ">> Load imu bias_a/g initial value...\n";
        if (this->LoadMatrix(configPath + "/imu_bias_a_g_init.txt", 2, 3, tempMat) == true) {
            this->queue.bias_a = tempMat.block<1, 3>(0, 0).transpose();
            this->queue.bias_g = tempMat.block<1, 3>(1, 0).transpose();
        } else {
            this->queue.bias_a.setZero();
            this->queue.bias_g.setZero();
        }
        std::cout << "     imu bias_a init : " << this->queue.bias_a.transpose() << "\n";
        std::cout << "     imu bias_g init : " << this->queue.bias_g.transpose() << "\n";

        // 加载 IMU propagate 的初值
        std::cout << ">> Load imu nominal state init value...\n";
        if (this->LoadMatrix(configPath + "/imu_init_p_wb.txt", 1, 3, tempMat) == true) {
            this->queue.initState.p_wb = tempMat.transpose();
        } else {
            this->queue.initState.p_wb.setZero();
        }
        if (this->LoadMatrix(configPath + "/imu_init_v_wb.txt", 1, 3, tempMat) == true) {
            this->queue.initState.v_wb = tempMat.transpose();
        } else {
            this->queue.initState.v_wb.setZero();
        }
        if (this->LoadMatrix(configPath + "/imu_init_q_wb.txt", 1, 4, tempMat) == true) {
            this->queue.initState.q_wb = Quaternion(tempMat(0, 0), tempMat(0, 1), tempMat(0, 2), tempMat(0, 3));
        } else {
            this->queue.initState.q_wb.setIdentity();
        }
        std::cout << "     imu init nominal p_wb : " << this->queue.initState.p_wb.transpose() << "\n";
        std::cout << "     imu init nominal v_wb : " << this->queue.initState.v_wb.transpose() << "\n";
        std::cout << "     imu init nominal q_wb : [" << this->queue.initState.q_wb.w() << ", " <<
            this->queue.initState.q_wb.x() << ", " << this->queue.initState.q_wb.y() << ", " <<
            this->queue.initState.q_wb.z() << "]\n";

        // 加载 w 系的重力加速度初值
        std::cout << ">> Load gravity in w frame...\n";
        if (this->LoadMatrix(configPath + "/gravity_in_w_init.txt", 1, 3, tempMat) == true) {
            this->queue.gravity = tempMat.transpose();
        } else {
            this->queue.gravity << 0.0, 0.0, 9.8;
        }
        std::cout << "     gravity in w frame init : " << this->queue.gravity.transpose() << "\n";

        // 加载多目相机外参
        std::cout << ">> Load multi-view camera extrinsic...\n";
        this->q_bc.clear();
        this->p_bc.clear();
        uint32_t camNum = 0;
        while (this->LoadMatrix(configPath + "/T_bc" + std::to_string(camNum) + ".txt", 4, 4, tempMat) == true) {
            Quaternion q_bc(tempMat.topLeftCorner<3, 3>());
            Vector3 p_bc(tempMat.topRightCorner<3, 1>());
            this->q_bc.emplace_back(q_bc);
            this->p_bc.emplace_back(p_bc);
            std::cout << "     camera " << camNum << " extrinsic q_bc is [" << q_bc.w() << ", " << q_bc.x() << ", " <<
                q_bc.y() << ", " << q_bc.z() << "], p_bc is [" << p_bc.transpose() << "]\n";
            ++camNum;
        }

        // 加载滑动窗口最大参数
        if (this->LoadMatrix(configPath + "/window_size.txt", 1, 1, tempMat) == true) {
            this->frameManager.maxWindowSize = tempMat(0, 0);
            std::cout << ">> Sliding window size set to " << this->frameManager.maxWindowSize << ".\n";
        }

        // 给序列管理器挂载帧管理器
        this->queue.slidingWindow = &this->frameManager;
        return true;
    }

    /* 从 txt 文件中读取一个矩阵 */
    bool Backend::LoadMatrix(const std::string &matrixFile, const uint32_t rows, const uint32_t cols, Matrix &mat) {
        std::ifstream fsMatrix;
        fsMatrix.open(matrixFile.c_str());
        if (!fsMatrix.is_open()) {
            return false;
        }
        std::string oneLine;
        mat.resize(rows, cols);
        uint32_t row = 0;
        while (std::getline(fsMatrix, oneLine) && !oneLine.empty()) {
            std::istringstream matData(oneLine);
            for (uint32_t col = 0; col < cols; ++col) {
                matData >> mat(row, col);
            }
            ++row;
        }
        return true;
    }
}