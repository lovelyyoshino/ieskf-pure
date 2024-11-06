cmake_minimum_required(VERSION 3.0.2)
project(ieskf_slam)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -B/usr/lib/gold-ld/")

#SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
#SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")

set(CMAKE_CXX_STANDARD 17)
add_definitions(-DPROJ_DIR="${CMAKE_CURRENT_SOURCE_DIR}")

find_package(Eigen3)
INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR})

find_package(OpenMP QUIET)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}   ${OpenMP_C_FLAGS}")

# 设置编译器选项
if(CMAKE_COMPILER_IS_GNUCXX)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native") # 将编译器标志设置为使用本地CPU指令
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -malign-double") # 将双精度数据对齐到8字节边界
endif()
# 定义EIGEN_MAKE_ALIGNED_OPERATOR_NEW宏
add_definitions(-DEIGEN_MAKE_ALIGNED_OPERATOR_NEW)

find_package(PCL 1.13.0 REQUIRED)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(-DMP_EN)
add_definitions(-DMP_PROC_NUM=8)

# YAML CPP
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)

include_directories(
        include
        ${PCL_INCLUDE_DIRS}
        ${YAML_CPP_INCLUDEDIR}
)
SET(IESKF_SLAM_FILE
        src/ieskf.cpp
        src/rect_map_manager.cpp
)

add_executable(${PROJECT_NAME} src/ieskf_node.cpp
        ${IESKF_SLAM_FILE}
)

target_link_libraries(${PROJECT_NAME}
        ${PCL_LIBRARIES}
        ${YAML_CPP_LIBRARIES}
        ${Boost_LIBRARIES}
#        pthread
)