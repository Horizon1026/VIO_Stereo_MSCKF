#pragma once
/* 外部依赖 */
#include <ctime>
#include <cstdlib>
#include <chrono>
/* 内部依赖 */
#include <typedef.hpp>

namespace ESKF_VIO_BACKEND {
    class TickTockTimer {
    private:
        std::chrono::time_point<std::chrono::system_clock> timeStamp;
    public:
        /* 构造函数与析构函数 */
        TickTockTimer();
        ~TickTockTimer();
    public:
        Scalar TickTock();
    };
}