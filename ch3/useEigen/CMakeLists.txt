cmake_minimum_required(VERSION 3.19)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
project(useEigen)

set(CMAKE_CXX_STANDARD 14)

add_executable(useEigen
        eigenMatrix.cpp)
