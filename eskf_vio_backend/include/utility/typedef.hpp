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
    using Vector2 = Eigen::Matrix<Scalar, 3, 1>;

    #define RETURN_IF_FALSE(...) if (__VA_ARGS__ == false) { return false; }
    #define RETURN_IF_TRUE(...) if (__VA_ARGS__ == true) { return true; }
    #define RETURN_FALSE_IF_EQUAL(...) if (__VA_ARGS__) { return false; }
    #define RETURN_IF_EQUAL(...) if (__VA_ARGS__) { return; }

    // 边缘化策略定义
    enum MargPolicy {
        MARG_OLDEST = 1,
        MARG_NEWEST,
        MARG_SUBNEW,
        NO_MARG
    };

    // 状态估计器的 status 定义
    enum Status {
        NEED_INIT = 1,
        INITIALIZED
    };
}