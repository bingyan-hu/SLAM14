cmake_minimum_required(VERSION 3.19)
project(useSophus)

set(CMAKE_CXX_STANDARD 17)
find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIR})

add_executable(useSophus useSophus.cpp)
target_link_libraries(useSophus ${Sophus_LIBRARIES})
# target_link_libraries(useSophus Sophus::Sophus)