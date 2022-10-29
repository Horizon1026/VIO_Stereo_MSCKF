#pragma once
/* 外部依赖 */
#include <typedef.hpp>
/* 内部依赖 */

namespace ESKF_VIO_BACKEND {
    template<typename T>
    class TimeStampQueue {
    private:
        std::map<fp64, std::shared_ptr<T>> items;
    public:
        /* 构造函数与析构函数 */
        TimeStampQueue() = default;
        virual ~TimeStampQueue() = default;
    public:
        
    };
}