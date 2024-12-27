cmake_minimum_required(VERSION 3.19)
project(jointMap)

set(CMAKE_CXX_STANDARD 17)
find_package(Eigen3 REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(Pangolin REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})

find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})


add_executable(jointMap jointMap.cpp)
target_link_libraries(jointMap ${OpenCV_LIBS})
target_link_libraries(jointMap ${Pangolin_LIBRARIES})
target_link_libraries(jointMap ${Sophus_LIBRARIES})
