# cmake工程最低版本为3.10
cmake_minimum_required( VERSION 3.10 )
# cmake工程名
project(Test_eskf_vio_backend)

# 切换 Release 或者 Debug
set(CMAKE_MODE_RELEASE TRUE)
if (CMAKE_MODE_RELEASE)
    # 设置编译类型为发布
    set(CMAKE_BUILD_TYPE "Release")
    # 添加c++ 17标准的支持
    set(CMAKE_CXX_FLAGS "-std=c++17 -O2")
else()
    # 设置编译类型为调试
    set(CMAKE_BUILD_TYPE "Debug")
    # 添加c++ 17标准的支持
    set(CMAKE_CXX_FLAGS "-std=c++17 -g")
endif()
# 设置 executable 文件输出路径
set(EXECUTABLE_OUTPUT_PATH ../output)

# 切换选择编译库方式 / 直接编译方式
set(MAKE_LIB FALSE)

# 选择编译库的方式
if (MAKE_LIB)
    # 添加 eskf_vio_backend 的 so 库对应头文件
    set(ESKF_VIO_BACKEND_INCLUDE ${CMAKE_SOURCE_DIR}/eskf_vio_backend)
    include_directories(${ESKF_VIO_BACKEND_INCLUDE}/include/backend)
    include_directories(${ESKF_VIO_BACKEND_INCLUDE}/include/data_manager)
    include_directories(${ESKF_VIO_BACKEND_INCLUDE}/include/data_manager/vision_data)
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

# 选择直接编译方式
else()
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
    set(ESKF_VIO_BACKEND_INCLUDE ${CMAKE_SOURCE_DIR}/eskf_vio_backend)
    include_directories(${ESKF_VIO_BACKEND_INCLUDE}/include/backend)
    include_directories(${ESKF_VIO_BACKEND_INCLUDE}/include/data_manager)
    include_directories(${ESKF_VIO_BACKEND_INCLUDE}/include/data_manager/vision_data)
    include_directories(${ESKF_VIO_BACKEND_INCLUDE}/include/geometry)
    include_directories(${ESKF_VIO_BACKEND_INCLUDE}/include/propagate)
    include_directories(${ESKF_VIO_BACKEND_INCLUDE}/include/update)
    include_directories(${ESKF_VIO_BACKEND_INCLUDE}/include/utility)
    # 添加源文件
    aux_source_directory(${ESKF_VIO_BACKEND_INCLUDE}/src/backend                    BACKEND)
    aux_source_directory(${ESKF_VIO_BACKEND_INCLUDE}/src/data_manager               DATA_MANAGER)
    aux_source_directory(${ESKF_VIO_BACKEND_INCLUDE}/src/data_manager/vision_data   VISION_DATA)
    aux_source_directory(${ESKF_VIO_BACKEND_INCLUDE}/src/geometry                   GEOMETRY)
    aux_source_directory(${ESKF_VIO_BACKEND_INCLUDE}/src/propagate                  PROPAGATE)
    aux_source_directory(${ESKF_VIO_BACKEND_INCLUDE}/src/update                     UPDATE)
    aux_source_directory(${ESKF_VIO_BACKEND_INCLUDE}/src/utility                    UTILITY)
    # 生成可执行文件
    add_executable(test_eskf_vio_backend
        ${CMAKE_SOURCE_DIR}/apps/test_eskf_vio_backend.cpp
        ${BACKEND}
        ${DATA_MANAGER}
        ${VISION_DATA}
        ${GEOMETRY}
        ${PROPAGATE}
        ${UPDATE}
        ${UTILITY}
    )
    # 生成可执行文件
    add_executable(test_pnp
        ${CMAKE_SOURCE_DIR}/apps/test_pnp.cpp
        ${GEOMETRY}
        ${UTILITY}
    )
    # 生成可执行文件
    add_executable(test_trianglate
        ${CMAKE_SOURCE_DIR}/apps/test_trianglate.cpp
        ${GEOMETRY}
        ${UTILITY}
    )

endif()