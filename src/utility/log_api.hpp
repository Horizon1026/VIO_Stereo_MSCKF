#pragma once
/* 外部依赖 */
#include <iostream>
/* 内部依赖 */
#include <typedef.hpp>

/* 是否允许 std::cout 形式的 log 输出 */
#define STD_COUT_INFO (1)

namespace ESKF_VIO_BACKEND {
#if STD_COUT_INFO
    #define LogInfo(...) std::cout << __VA_ARGS__ << std::endl
#else
    #define LogInfo(...)
#endif
}