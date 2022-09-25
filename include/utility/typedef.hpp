#pragma once
/* 外部依赖 */
#include <vector>
#include <unordered_map>
#include <deque>
#include <memory>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
/* 内部依赖 */

/* 是否允许 std::cout 形式的 log 输出 */
#define STD_COUT_INFO (1)

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
    using Matrix = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>;
    using Quaternion = Eigen::Quaternion<Scalar>;
    using Matrix33 = Eigen::Matrix<Scalar, 3, 3>;
    using Matrix44 = Eigen::Matrix<Scalar, 4, 4>;
    using Vector3 = Eigen::Matrix<Scalar, 3, 1>;

    // 边缘化策略定义
    enum MargPolicy {
        MARG_OLDEST = 1,
        MARG_NEWEST,
        MARG_SUBNEW
    };

    // 状态估计器的 status 定义
    enum Status {
        NEED_INIT = 1,
        INITIALIZING,
        INITIALIZED
    };
}