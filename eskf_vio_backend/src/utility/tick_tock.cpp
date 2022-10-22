/* 内部依赖 */
#include <tick_tock.hpp>
/* 外部依赖 */

namespace ESKF_VIO_BACKEND {
    TickTockTimer::TickTockTimer() {
        this->timeStamp = std::chrono::system_clock::now();
    }
    TickTockTimer::~TickTockTimer() {}

    Scalar TickTockTimer::TickTock() {
        auto newTimeStamp = std::chrono::system_clock::now();
        std::chrono::duration<double> diff = newTimeStamp - this->timeStamp;
        this->timeStamp = newTimeStamp;
        return Scalar(diff.count() * 1000.0);
    }


}