cmake_minimum_required(VERSION 3.16)

project(rgfd)

list(APPEND CMAKE_PREFIX_PATH "/mnt/c/Users/Lenovo/Desktop/eigen/bench/btl/libs/")
find_package (Eigen3 3.3 REQUIRED NO_MODULE)


add_library(plane plane.cpp)
add_executable(main main.cpp)
target_link_libraries(main plane Eigen3::Eigen)