#pragma once
/* 外部依赖 */
/* 内部依赖 */
#include <typedef.hpp>

namespace ESKF_VIO_BACKEND {
    class Configurator {
    public:
        /* IMU 相关参数 */
        struct IMU {
            Scalar sigmaBiasAccel = 0.019f;
            Scalar sigmaBiasGyro = 0.015f;
            Scalar sigmaBiasAccelRandomWalk = 0.001f;
            Scalar sigmaBiasGyroRandomWalk = 0.0001f;

            Scalar gravityNorm = 9.8f;

            Vector3 defaultBiasAccel = Vector3::Zero();
            Vector3 defaultBiasGyro = Vector3::Zero();

            Scalar period = 0.005f;
        } imu;

        /* 视觉观测相关参数 */
        struct Vision {
            Scalar sigmaMeasure = 0.0021739f;

            uint32_t maxFrameNum = 6;
            uint32_t maxFeatureNumForUpdate = 30;
            uint32_t maxCameraNum = 2;

            uint32_t minKeyframeTrackedFeatureNum = 25;
            Scalar minKeyframeMeanParallax = 0.2f;
            Scalar minKeyframeTranslation = 0.3f;

            std::vector<Quaternion> default_q_bc;
            std::vector<Vector3> default_p_bc;
        } vision;

        /* Backend 通用参数 */
        struct General {
            bool useVisionUpdate = true;
        } general;

    private:
        /* 构造函数与析构函数 */
        Configurator() = default;
        virtual ~Configurator() = default;

    public:
        /* 单例模式，对外提供实例接口 */
        static Configurator *GetInstance(void);
        /* 输入配置文件路径，自动搜寻所有配置项并记录在成员变量中 */
        bool ReadConfigParams(const std::string &configPath);
        /* 输出所有配置信息 */
        void Information(void);

    private:
        /* 从 txt 文件中读取矩阵 */
        bool LoadMatrix(const std::string &matrixFile, const uint32_t rows, const uint32_t cols, Matrix &mat);
    };
}