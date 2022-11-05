/* 内部依赖 */
#include <backend.hpp>
#include <log_api.hpp>
/* 外部依赖 */

namespace ESKF_VIO_BACKEND {
    /* 后端优化器读取配置并初始化 */
    bool Backend::ConfigParams(const std::string &configPath) {
        // 获取配置参数
        this->config = Configurator::GetInstance();
        RETURN_FALSE_IF_FALSE(this->config->ReadConfigParams(configPath));
        this->config->Information();

        // 从配置器中载入参数
        this->dataloader.imuPeriod = this->config->imu.period;
        this->propagator.InitializeProcessNoiseMatrix(this->config->imu.sigmaBiasAccel,
                                                      this->config->imu.sigmaBiasGyro,
                                                      this->config->imu.sigmaBiasAccelRandomWalk,
                                                      this->config->imu.sigmaBiasGyroRandomWalk);
        this->propagator.bias_a = this->config->imu.defaultBiasAccel;
        this->propagator.bias_g = this->config->imu.defaultBiasGyro;
        IMUFullState::gravity_w << 0.0, 0.0, this->config->imu.gravityNorm;

        this->frameManager.extrinsics.clear();
        for (uint32_t i = 0; i < this->config->vision.default_p_bc.size(); ++i) {
            this->frameManager.extrinsics.emplace_back(Extrinsic(
                this->config->vision.default_q_bc[i],
                this->config->vision.default_p_bc[i]));
        }
        this->visionUpdator.measureNoise = this->config->vision.sigmaVision;
        this->visionUpdator.maxUsedFeatures = this->config->vision.maxFeatureNumForUpdate;

        this->frameManager.maxWindowSize = this->config->vision.maxFrameNum;
        this->frameManager.minKeyframeMeanParallax = this->config->vision.minKeyframeMeanParallax;
        this->frameManager.minKeyframeTrackedFeatureNum = this->config->vision.minKeyframeTrackedFeatureNum;
        this->frameManager.maxKeyframeTranslation = this->config->vision.maxKeyframeTranslation;

        // 挂载管理器指针
        this->propagator.propagator = &this->frameManager;
        this->visionUpdator.propagator = &this->propagator;
        this->visionUpdator.featureManager = &this->featureManager;
        this->visionUpdator.frameManager = &this->frameManager;
        this->visionUpdator.trianglator = &this->trianglator;
        this->visionUpdator.pnpSolver = &this->pnpSolver;
        return true;
    }
}