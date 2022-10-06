/* 内部依赖 */
#include <backend.hpp>
#include <log_api.hpp>
/* 外部依赖 */
#include <fstream>

namespace ESKF_VIO_BACKEND {
    /* 后端优化器读取配置并初始化 */
    bool Backend::Initialize(const std::string &configPath) {
        Matrix tempMat;
        std::string configFile;
        LogInfo(">> ESKF VIO Backend Starts reading config file at " << configPath);

        // 加载 IMU 输入频率
        LogInfo(">> Load imu input frequence...");
        if (this->LoadMatrix(configPath + "/imu_frequence.txt", 1, 1, tempMat) == true) {
            this->dataloader.imuPeriod = static_cast<fp64>(1.0 / tempMat(0, 0));
        }
        LogInfo("     imu period is " << this->dataloader.imuPeriod << "s.");

        // 加载 IMU 噪声参数
        LogInfo(">> Load imu noise...");
        if (this->LoadMatrix(configPath + "/imu_na_ng_nwa_nwg.txt", 1, 4, tempMat) == true) {
            this->propagator.InitializeProcessNoiseMatrix(tempMat(0, 0), tempMat(0, 1), tempMat(0, 2), tempMat(0, 3));
            LogInfo("     imu accel noise : " << this->propagator.Q(INDEX_NA, INDEX_NA));
            LogInfo("     imu gyro noise : " << this->propagator.Q(INDEX_NG, INDEX_NG));
            LogInfo("     imu accel random walk : " << this->propagator.Q(INDEX_NWA, INDEX_NWA));
            LogInfo("     imu gyro random walk : " << this->propagator.Q(INDEX_NWG, INDEX_NA));
        }

        // 加载 IMU bias 初值
        LogInfo(">> Load imu bias_a/g initial value...");
        if (this->LoadMatrix(configPath + "/imu_bias_a_g_init.txt", 2, 3, tempMat) == true) {
            this->propagator.bias_a = tempMat.block<1, 3>(0, 0).transpose();
            this->propagator.bias_g = tempMat.block<1, 3>(1, 0).transpose();
        } else {
            this->propagator.bias_a.setZero();
            this->propagator.bias_g.setZero();
        }
        LogInfo("     imu bias_a init : " << this->propagator.bias_a.transpose());
        LogInfo("     imu bias_g init : " << this->propagator.bias_g.transpose());

        // 加载 IMU propagate 的初值
        LogInfo(">> Load imu nominal state init value...");
        if (this->LoadMatrix(configPath + "/imu_init_p_wb.txt", 1, 3, tempMat) == true) {
            this->propagator.initState.p_wb = tempMat.transpose();
        } else {
            this->propagator.initState.p_wb.setZero();
        }
        if (this->LoadMatrix(configPath + "/imu_init_v_wb.txt", 1, 3, tempMat) == true) {
            this->propagator.initState.v_wb = tempMat.transpose();
        } else {
            this->propagator.initState.v_wb.setZero();
        }
        if (this->LoadMatrix(configPath + "/imu_init_q_wb.txt", 1, 4, tempMat) == true) {
            this->propagator.initState.q_wb = Quaternion(tempMat(0, 0), tempMat(0, 1), tempMat(0, 2), tempMat(0, 3));
        } else {
            this->propagator.initState.q_wb.setIdentity();
        }
        LogInfo("     imu init nominal p_wb : " << this->propagator.initState.p_wb.transpose());
        LogInfo("     imu init nominal v_wb : " << this->propagator.initState.v_wb.transpose());
        LogInfo("     imu init nominal q_wb : [" << this->propagator.initState.q_wb.w() << ", " <<
            this->propagator.initState.q_wb.x() << ", " << this->propagator.initState.q_wb.y() << ", " <<
            this->propagator.initState.q_wb.z() << "]");

        // 加载 w 系的重力加速度初值
        LogInfo(">> Load gravity in w frame...");
        if (this->LoadMatrix(configPath + "/gravity_in_w_init.txt", 1, 1, tempMat) == true) {
            IMUFullState::gravity_w << 0.0, 0.0, tempMat(0, 0);
        } else {
            IMUFullState::gravity_w << 0.0, 0.0, 9.8;
        }
        LogInfo("     gravity in w frame init : " << IMUFullState::gravity_w.transpose());

        // 加载多目相机外参
        LogInfo(">> Load multi-view camera extrinsic...");
        this->frameManager.extrinsics.clear();
        uint32_t camNum = 0;
        while (this->LoadMatrix(configPath + "/T_bc" + std::to_string(camNum) + ".txt", 4, 4, tempMat) == true) {
            Quaternion q_bc(tempMat.topLeftCorner<3, 3>());
            Vector3 p_bc(tempMat.topRightCorner<3, 1>());
            this->frameManager.extrinsics.emplace_back(Extrinsic(q_bc, p_bc));
            LogInfo("     camera " << camNum << " extrinsic q_bc is [" << q_bc.w() << ", " << q_bc.x() << ", " <<
                q_bc.y() << ", " << q_bc.z() << "], p_bc is [" << p_bc.transpose() << "]");
            ++camNum;
        }

        // 加载滑动窗口最大参数
        if (this->LoadMatrix(configPath + "/window_size.txt", 1, 1, tempMat) == true) {
            this->frameManager.maxWindowSize = tempMat(0, 0);
            LogInfo(">> Sliding window size set to " << this->frameManager.maxWindowSize << ".");
        }

        // 挂载管理器指针
        this->propagator.slidingWindow = &this->frameManager;
        this->visionUpdator.propagator = &this->propagator;
        this->visionUpdator.featureManager = &this->featureManager;
        this->visionUpdator.frameManager = &this->frameManager;
        this->visionUpdator.trianglator = &this->trianglator;
        this->visionUpdator.pnpSolver = &this->pnpSolver;
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