cmake_minimum_required(VERSION 3.19)
project(stereoVision)

set(CMAKE_CXX_STANDARD 14)
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIR})
find_package(Pangolin REQUIRED)
include_directories(${Pangolin_INCLUDE_DIR})
find_package(Eigen3 REQUIRED)
include_directories(${Eigen_INCLUDE_DIR})

add_executable(stereoVision stereoVision.cpp)
target_link_libraries(stereoVision ${OpenCV_LIBS})
target_link_libraries(stereoVision ${Pangolin_LIBRARIES})

