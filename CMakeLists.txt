# cmake工程最低版本为3.10
cmake_minimum_required( VERSION 3.10 )
# cmake工程名
project(ESKF_Estimator)
# 设置编译类型为发布
set(CMAKE_BUILD_TYPE "Debug")
# 添加c++ 17标准的支持
set(CMAKE_CXX_FLAGS "-std=c++17 -g")
# 寻找Eigen库并添加其路径
include_directories( "/usr/include/eigen3" )
# 寻找 TBB 库
# find_package( TBB REQUIRED )
# 寻找 Pangolin 库并添加其路径
# find_package( Pangolin REQUIRED )
# include_directories( ${Pangolin_INCLUDE_DIRS} )
# 添加路径
include_directories( ${PROJECT_SOURCE_DIR} )
# 添加源文件
aux_source_directory(${PROJECT_SOURCE_DIR}/src/data_manager DATA_MANAGER)
aux_source_directory(${PROJECT_SOURCE_DIR}/src/propagate    PROPAGATE)
aux_source_directory(${PROJECT_SOURCE_DIR}/src/update       UPDATE)
aux_source_directory(${PROJECT_SOURCE_DIR}/src/utility      UTILITY)
aux_source_directory(${PROJECT_SOURCE_DIR}/src              BACKEND)
# 链接源文件
add_executable( ESKF_Estimator
    apps/test_eskf_vio_backend.cpp
    ${DATA_MANAGER}
    ${PROPAGATE}
    ${UPDATE}
    ${UTILITY}
    ${BACKEND}
)

# 为可执行文件链接相关的库
target_link_libraries( ESKF_Estimator
    # pangolin GL GLEW    # pangolin和OpenGL的库
    # TBB::tbb
)