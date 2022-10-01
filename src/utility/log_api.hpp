#pragma once
/* 内部依赖 */
#include <typedef.hpp>

/* 是否允许 std::cout 形式的 log 输出 */
#define STD_COUT_INFO (1)

/* 外部依赖 */
#if STD_COUT_INFO
    #include <iostream>
#endif

namespace ESKF_VIO_BACKEND {
#if STD_COUT_INFO
#define Log(...) std::cout << __VA_ARGS__ << std::endl
#else
#define Log(...) ;
#endif
}