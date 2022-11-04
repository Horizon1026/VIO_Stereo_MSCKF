# cmake工程最低版本为3.10
cmake_minimum_required( VERSION 3.10 )
# cmake工程名
project(Make_lib_eskf_vio_backend)
# 设置编译类型为发布
set(CMAKE_BUILD_TYPE "Release")
# 添加c++ 17标准的支持
set(CMAKE_CXX_FLAGS "-std=c++17 -O2")
# 设置 lib 文件输出路径
set(LIBRARY_OUTPUT_PATH ../output)

# 寻找Eigen库并添加其路径
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake_modules/")
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories("/usr/include/eigen3")
# 寻找 TBB 库
# find_package( TBB REQUIRED )
# 寻找 Pangolin 库并添加其路径
# find_package( Pangolin REQUIRED )
# include_directories( ${Pangolin_INCLUDE_DIRS} )

# 添加头文件路径
include_directories(${CMAKE_SOURCE_DIR}/include/backend)
include_directories(${CMAKE_SOURCE_DIR}/include/config)
include_directories(${CMAKE_SOURCE_DIR}/include/data_manager)
include_directories(${CMAKE_SOURCE_DIR}/include/data_manager/vision_data)
include_directories(${CMAKE_SOURCE_DIR}/include/geometry)
include_directories(${CMAKE_SOURCE_DIR}/include/propagate)
include_directories(${CMAKE_SOURCE_DIR}/include/update)
include_directories(${CMAKE_SOURCE_DIR}/include/utility)
# 添加源文件
aux_source_directory(${CMAKE_SOURCE_DIR}/src/backend                    BACKEND)
aux_source_directory(${CMAKE_SOURCE_DIR}/src/config                     CONFIG)
aux_source_directory(${CMAKE_SOURCE_DIR}/src/data_manager               DATA_MANAGER)
aux_source_directory(${CMAKE_SOURCE_DIR}/src/data_manager/vision_data   VISION_DATA)
aux_source_directory(${CMAKE_SOURCE_DIR}/src/geometry                   GEOMETRY)
aux_source_directory(${CMAKE_SOURCE_DIR}/src/propagate                  PROPAGATE)
aux_source_directory(${CMAKE_SOURCE_DIR}/src/update                     UPDATE)
aux_source_directory(${CMAKE_SOURCE_DIR}/src/utility                    UTILITY)
# 生成库
add_library(_eskf_vio_backend SHARED
    ${BACKEND}
    ${CONFIG}
    ${DATA_MANAGER}
    ${VISION_DATA}
    ${GEOMETRY}
    ${PROPAGATE}
    ${UPDATE}
    ${UTILITY}
)