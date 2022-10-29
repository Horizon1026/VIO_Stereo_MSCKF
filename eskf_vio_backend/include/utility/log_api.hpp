#pragma once
/* 外部依赖 */
#include <iostream>
#include <iomanip>
/* 内部依赖 */
#include <typedef.hpp>

/* 是否允许 std::cout 形式的 log 输出 */
#define STD_COUT_INFO (1)

namespace ESKF_VIO_BACKEND {
#if STD_COUT_INFO
    // #define LogFixPercision()
    #define LogFixPercision() std::cout << std::fixed << std::setprecision(2)
    #define LogInfo(...)  std::cout << __VA_ARGS__ << std::endl
    #define LogDebug(...) std::cout << "[Debug] " << __VA_ARGS__ << std::endl
    #define LogError(...) std::cout << "[Error] " << __VA_ARGS__ << std::endl
#else
    #define LogFixPercision()
    #define LogInfo(...)
    #define LogDebug(...)
    #define LogError(...)
#endif
}