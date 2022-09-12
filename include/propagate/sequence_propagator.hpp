#pragma once
/* 外部依赖 */
/* 内部依赖 */
#include <include/utility/typedef.hpp>
#include <include/propagate/imu_state.hpp>

namespace ESKF_VIO_BACKEND {
    /* IMU-Camera propagate 序列 */
    class PropagateQueue {
        std::deque<std::shared_ptr<IMUPropagateQueueItem>> items;
    };
}