/* 内部依赖 */
#include <config.hpp>
#include <log_api.hpp>
/* 外部依赖 */
#include <fstream>

namespace ESKF_VIO_BACKEND {
    /* 单例模式，对外提供实例接口 */
    Configurator *Configurator::GetInstance(void) {
        static Configurator instance;
        return &instance;
    }


    /* 输入配置文件路径，自动搜寻所有配置项并记录在成员变量中 */
    bool Configurator::ReadConfigParams(const std::string &configPath) {
        Matrix tempMat;
        std::string configFile;

        // 加载 IMU 输入频率
        if (this->LoadMatrix(configPath + "/imu_frequence.txt", 1, 1, tempMat) == true) {
            this->imu.period = static_cast<Scalar>(1.0f / tempMat(0, 0));
        }

        // 加载 IMU 噪声参数
        if (this->LoadMatrix(configPath + "/imu_na_ng_nwa_nwg.txt", 1, 4, tempMat) == true) {
            this->imu.sigmaBiasAccel = tempMat(0, 0);
            this->imu.sigmaBiasGyro = tempMat(0, 1);
            this->imu.sigmaBiasAccelRandomWalk = tempMat(0, 2);
            this->imu.sigmaBiasGyroRandomWalk = tempMat(0, 3);
        }

        // 加载 IMU bias 初值
        if (this->LoadMatrix(configPath + "/imu_bias_a_g_init.txt", 2, 3, tempMat) == true) {
            this->imu.defaultBiasAccel = tempMat.block<1, 3>(0, 0).transpose();
            this->imu.defaultBiasGyro = tempMat.block<1, 3>(1, 0).transpose();
        }

        // 加载 w 系的重力加速度初值
        if (this->LoadMatrix(configPath + "/gravity_norm.txt", 1, 1, tempMat) == true) {
            this->imu.gravityNorm = tempMat(0, 0);
        }

        // 加载多目相机外参
        this->vision.default_p_bc.clear();
        this->vision.default_q_bc.clear();
        this->vision.maxCameraNum = 0;
        while (this->LoadMatrix(configPath + "/T_bc" + std::to_string(this->vision.maxCameraNum) + ".txt", 4, 4, tempMat) == true) {
            const Vector3 p_bc(tempMat.topRightCorner<3, 1>());
            const Quaternion q_bc(tempMat.topLeftCorner<3, 3>());
            this->vision.default_p_bc.emplace_back(p_bc);
            this->vision.default_q_bc.emplace_back(q_bc);
            ++this->vision.maxCameraNum;
        }

        // 加载滑动窗口最大参数
        if (this->LoadMatrix(configPath + "/window_size.txt", 1, 1, tempMat) == true) {
            this->vision.maxFrameNum = tempMat(0, 0);
        }

        // 加载视觉量测噪声
        if (this->LoadMatrix(configPath + "/vision_measure_noise.txt", 1, 1, tempMat) == true) {
            this->vision.sigmaMeasure = tempMat(0, 0);
        }
        return true;
    }


    /* 从 txt 文件中读取一个矩阵 */
    bool Configurator::LoadMatrix(const std::string &matrixFile, const uint32_t rows, const uint32_t cols, Matrix &mat) {
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