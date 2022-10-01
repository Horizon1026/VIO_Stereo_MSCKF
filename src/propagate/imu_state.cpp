/* 内部依赖 */
#include <imu_state.hpp>
/* 外部依赖 */

namespace ESKF_VIO_BACKEND {
    /* 定义静态变量，重力加速度默认为 9.8 */
    Vector3 IMUFullState::gravity_w = Vector3(0.0, 0.0, 9.8);

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