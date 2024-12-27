cmake_minimum_required(VERSION 3.19)
project(trajectoryError)

set(CMAKE_CXX_STANDARD 17)
find_package(Eigen3 REQUIRED)
include_directories(${Eigen3_INCLUDE_DIR})

find_package(Pangolin REQUIRED)
include_directories(${Pangolin_INCLUDE_DIR})

find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIR})

add_executable(trajectoryError trajectoryError.cpp)

target_link_libraries(trajectoryError ${Pangolin_LIBRARIES})
