cmake_minimum_required(VERSION 3.20)

project(mc_autotune
    VERSION 0.1.0
    LANGUAGES CXX
)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_subdirectory(pid_design)
add_subdirectory(filter)
add_subdirectory(system_identification)
