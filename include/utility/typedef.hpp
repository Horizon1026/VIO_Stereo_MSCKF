#pragma once
#include <vector>
#include <map>
#include <unordered_map>
#include <list>
#include <memory>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

/* 所有类型命名都定义在命名空间内 */
namespace ESKF_VIO_BACKEND {
    using uint8_t = unsigned char;
    using uint16_t = unsigned short;
    using uint32_t = unsigned int;
    using uint64_t = unsigned long;
    using int8_t = char;
    using int16_t = short;
    using int32_t = int;
    using int64_t = long;
    using fp32 = float;
    using fp64 = double;

    using Scalar = float;
}