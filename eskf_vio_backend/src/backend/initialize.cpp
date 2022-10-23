/* 内部依赖 */
#include <backend.hpp>
#include <log_api.hpp>
/* 外部依赖 */

namespace ESKF_VIO_BACKEND {
    /* 尝试进行初始化 */
    bool Backend::Initialize(void) {
        // 在没有进行初始化时，仅在存在连续两帧观测时才进行初始化
        if (this->frameManager.frames.size() >= 2) {
            auto frame_i = this->frameManager.frames.front();
            auto frame_j = *std::next(this->frameManager.frames.begin());
            // Step 1: 从 attitude estimator 中拿取此帧对应时刻的姿态估计结果，赋值给首帧的 q_wb
            RETURN_FALSE_IF_FALSE(this->attitudeEstimator.GetAttitude(frame_i->timeStamp,
                                                                      this->dataloader.imuPeriod,
                                                                      frame_i->q_wb));
            // Step 2: 首帧观测为 multi-view 时，首帧位置设置为原点，进行首帧内的多目三角测量，得到一些特征点的 p_w
            frame_i->p_wb.setZero();
            RETURN_FALSE_IF_FALSE(this->TrianglizeMultiView(frame_i));
            // Step 3: 利用这些特征点在下一帧中的观测，通过重投影 PnP 迭代，估计出下一帧基于首帧参考系 w 系的相对位姿 p_wb 和 q_wb
            RETURN_FALSE_IF_FALSE(this->EstimateFramePose(frame_j));
            // Step 4: 两帧位置对时间进行差分，得到两帧在 w 系中的速度估计，至此首帧对应时刻点的 q_wb 和 v_wb 都已经确定
            frame_j->v_wb = (frame_j->p_wb - frame_i->p_wb) / Scalar(frame_j->timeStamp - frame_i->timeStamp);
            frame_i->v_wb = frame_j->v_wb;
            // Step 5: 初始化序列递推求解器
            //         首帧位置设置为原点，将首帧位姿和速度赋值给 propagator 的 initState
            //         从 attitude estimator 提取出从首帧时刻点开始，到最新时刻的 imu 量测，输入到 propagator 让他递推到最新时刻
            IMUMotionState initState(frame_i->p_wb, frame_i->v_wb, frame_i->q_wb);
            RETURN_FALSE_IF_FALSE(this->InitializePropagator(initState, frame_i->timeStamp));
            // 初始化过程成功，打印初始化结果
            LogInfo(">> Initialize at time stamp " << frame_i->timeStamp << "s:");
            LogInfo("     first item time stamp is " << this->propagator.items.front()->timeStamp << "s");
            LogInfo("     init p_wb is [" << frame_i->p_wb.transpose() << "]");
            LogInfo("     init v_wb is [" << frame_i->v_wb.transpose() << "]");
            LogInfo("     init q_wb is [" << frame_i->q_wb.w() << ", " << frame_i->q_wb.x() << ", " <<
                frame_i->q_wb.y() << ", " << frame_i->q_wb.z() << "]");
            return true;
        } else {
            return false;
        }
    }



    /* 基于某一帧的多目测量结果进行三角化 */
    bool Backend::TrianglizeMultiView(const std::shared_ptr<Frame> &frame) {
        if (frame == nullptr) {
            return false;
        }
        Quaternion &q_wb = frame->q_wb;
        Vector3 &p_wb = frame->p_wb;

        // 从 frame->features 中挑选点进行三角化，结果将保存在 feature manager 中
        for (auto it = frame->features.begin(); it != frame->features.end(); ++it) {
            auto feature = it->second;
            std::vector<Quaternion> all_q_wc;
            std::vector<Vector3> all_p_wc;
            std::vector<Vector2> all_norm;
            all_q_wc.reserve(20);
            all_p_wc.reserve(20);
            all_norm.reserve(20);

            auto multiview = feature->observes[frame->id - feature->firstFrameID]->norms;
            for (auto it = multiview.begin(); it != multiview.end(); ++it) {
                Quaternion &q_bc = this->frameManager.extrinsics[it->first].q_bc;
                Vector3 &p_bc = this->frameManager.extrinsics[it->first].p_bc;
                /* T_wc = T_wb * T_bc */
                /* [R_wc  t_wc] = [R_wb  t_wb]  *  [R_bc  t_bc] = [ R_wb * R_bc  R_wb * t_bc + t_wb]
                   [  0    1  ]   [  0     1 ]     [  0     1 ]   [      0                1        ] */
                /* T_wb = T_wc * T_bc.inv */
                /* [R_wb  t_wb] = [R_wc  t_wc]  *  [R_bc.t  - R_bc.t * t_bc] = [ R_wc * R_bc.t  - R_wc * R_bc.t * t_bc + t_wc]
                   [  0    1  ]   [  0     1 ]     [  0             1      ]   [      0                       1              ] */
                all_q_wc.emplace_back(Quaternion(q_wb * q_bc));
                all_p_wc.emplace_back(Vector3(q_wb * p_bc + p_wb));
                all_norm.emplace_back(multiview.find(it->first)->second);
            }
            if (this->trianglator.TrianglateAnalytic(all_q_wc, all_p_wc, all_norm, feature->p_w) == true) {
                feature->status = Feature::SOLVED;
                LogDebug("feature " << feature->id << " is solved, p_w is " << feature->p_w.transpose());
            } else {
                feature->status = Feature::UNSOLVED;
                LogDebug("feature " << feature->id << " is not solved.");
            }
        }
        return true;
    }


    /* 基于三角化成功的点，估计某一帧的位姿 */
    bool Backend::EstimateFramePose(const std::shared_ptr<Frame> &frame) {
        if (frame == nullptr) {
            return false;
        }
        // TODO: 从 frame->features 中挑选三角化成功的点，输入到 pnp solver 中
        return true;
    }


    /* 初始化序列化 propagator */
    bool Backend::InitializePropagator(const IMUMotionState &initState,
                                       const fp64 startTime) {
        // 首帧位置设置为原点，将首帧位姿和速度赋值给 propagator 的 initState
        this->propagator.initState.p_wb = initState.p_wb;
        this->propagator.initState.v_wb = initState.v_wb;
        this->propagator.initState.q_wb = initState.q_wb;
        // 从 attitude estimator 提取出从首帧时刻点开始，到最新时刻的 imu 量测
        auto it = this->attitudeEstimator.items.begin();
        while (std::fabs((*it)->timeStamp - startTime) > this->dataloader.imuPeriod &&
               it != this->attitudeEstimator.items.end()) {
            ++it;
        }
        RETURN_FALSE_IF(it == this->attitudeEstimator.items.end());
        // 依次输入到 propagator 让他递推到最新时刻
        while (it != this->attitudeEstimator.items.end()) {
            this->propagator.Propagate((*it)->accel, (*it)->gyro, (*it)->timeStamp);
            ++it;
        }
        return true;
    }
}