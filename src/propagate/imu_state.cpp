/* 内部依赖 */
#include <include/propagate/imu_state.hpp>
/* 外部依赖 */

namespace ESKF_VIO_BACKEND {
    /* 状态清零 */
    void IMUFullState::Reset(void) {
        this->p_wb.setZero();
        this->q_wb.setIdentity();
        this->theta_wb.setZero();
        this->v_wb.setZero();
        this->bias_a.setZero();
        this->bias_g.setZero();
        this->gravity.setZero();
    }
}