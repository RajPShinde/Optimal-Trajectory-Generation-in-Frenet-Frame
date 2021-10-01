cmake_minimum_required(VERSION 3.2.1)
project (Optimal Trajectory Planner in Frenet Frame)

# Add project cmake modules to path.
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake)

include(CMakeToolsHelpers OPTIONAL)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 14)

add_executable(planner src/main.cpp src/optimalTrajectoryPlanner.cpp src/polynomial.cpp include/optimalTrajectoryPlanner.hpp include/polynomial.hpp include/frenetPath.hpp)
include_directories(
    ${CMAKE_SOURCE_DIR}/include
)
