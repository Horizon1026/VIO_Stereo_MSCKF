# cmake工程最低版本为3.10
cmake_minimum_required( VERSION 3.10 )
# cmake工程名
project(Test_eskf_vio_backend)
# 设置编译类型为发布
set(CMAKE_BUILD_TYPE "Release")
# 添加c++ 17标准的支持
set(CMAKE_CXX_FLAGS "-std=c++17 -O2")
# 设置 executable 文件输出路径
set(EXECUTABLE_OUTPUT_PATH ../output)

# 添加 eskf_vio_backend 的 so 库对应头文件
set(ESKF_VIO_BACKEND_INCLUDE ${CMAKE_SOURCE_DIR}/eskf_vio_backend)
include_directories(${ESKF_VIO_BACKEND_INCLUDE}/include)
include_directories(${ESKF_VIO_BACKEND_INCLUDE}/include/data_manager)
include_directories(${ESKF_VIO_BACKEND_INCLUDE}/include/geometry)
include_directories(${ESKF_VIO_BACKEND_INCLUDE}/include/propagate)
include_directories(${ESKF_VIO_BACKEND_INCLUDE}/include/update)
include_directories(${ESKF_VIO_BACKEND_INCLUDE}/include/utility)
# 寻找Eigen库并添加其路径
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${ESKF_VIO_BACKEND_INCLUDE}/cmake_modules/")
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# 添加源文件（非库内）
aux_source_directory(${CMAKE_SOURCE_DIR}/apps SRC_LIST)
# 编译可执行文件
add_executable(test_eskf_vio_backend
    ${SRC_LIST}
)
# 链接到库
target_link_libraries(test_eskf_vio_backend
    ${CMAKE_SOURCE_DIR}/eskf_vio_backend/output/lib_eskf_vio_backend.so
)