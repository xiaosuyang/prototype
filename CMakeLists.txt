cmake_minimum_required(VERSION 3.10.0) # CMake install : https://cmake.org/download/
project(Master)

# 指定目标系统
set(CMAKE_SYSTEM_NAME Linux)
# 指定目标平台
set(CMAKE_SYSTEM_PROCESSOR arm)

set(CMAKE_FIND_ROOT_PATH "${CMAKE_CURRENT_SOURCE_DIR}/gcc-linaro-4.9.4-2017.01-x86_64_arm-linux-gnueabihf")

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)

set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)

set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
# 指定交叉编译工具链的根路径
set(CROSS_CHAIN_PATH "${CMAKE_CURRENT_SOURCE_DIR}/gcc-linaro-4.9.4-2017.01-x86_64_arm-linux-gnueabihf")
# 指定C编译器
set(CMAKE_C_COMPILER "${CROSS_CHAIN_PATH}/bin/arm-linux-gnueabihf-gcc")
# 指定C++编译器
set(CMAKE_CXX_COMPILER "${CROSS_CHAIN_PATH}/bin/arm-linux-gnueabihf-g++")

set(CMAKE_CXX_STANDARD 14)


find_package(Eigen3 REQUIRED)
message("Eigen 库目录" " ${EIGEN3_INCLUDE_DIR}")
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/output/include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/output/lib)
file(GLOB INCLUDE_DIRS "include/*")
list(APPEND INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/include")
message("包含目录：" "${INCLUDE_DIRS}")
include_directories(${INCLUDE_DIRS}
"${EIGEN3_INCLUDE_DIR}")



file(GLOB_RECURSE SRC_FILE "*.cpp")
file(GLOB_RECURSE BUILD_CPP_FILES "build/*.cpp")
list(REMOVE_ITEM SRC_FILE ${BUILD_CPP_FILES})
foreach(file ${SRC_FILE})
    message(STATUS "${file}")
endforeach()

add_executable(master ${SRC_FILE})

link_directories("/home/debian/output/lib")


target_link_libraries(master ${CMAKE_CURRENT_SOURCE_DIR}/output/lib/libethercat.so -pthread)
