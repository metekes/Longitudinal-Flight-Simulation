cmake_minimum_required(VERSION 3.16)

project(JEDSY)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)

add_library(plane plane.cpp)
add_executable(main main.cpp)
target_link_libraries(main plane Eigen3::Eigen)