cmake_minimum_required(VERSION 3.0.2)
project(MyProject)

# set(CMAKE_INSTALL_PREFIX ${CMAKE_BINARY_DIR}/install CACHE PATH "Install prefix")

# 设置全局变量（子目录可以访问）
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


# 添加子目录（按顺序）
add_subdirectory(src/turn_on_robot ${CMAKE_BINARY_DIR}/turn_on_robot)

add_subdirectory(src/global_planner ${CMAKE_BINARY_DIR}/global_planner)

add_subdirectory(src/local_planner ${CMAKE_BINARY_DIR}/local_planner)

add_subdirectory(src/motion_control ${CMAKE_BINARY_DIR}/motion_control)

message(STATUS "Project configuration complete")
