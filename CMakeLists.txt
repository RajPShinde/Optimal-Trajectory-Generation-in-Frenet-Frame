cmake_minimum_required(VERSION 3.2.1)
project (Optimal_Trajectory_Planner_in_Frenet_Frame)

# Add project cmake modules to path.
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

include(CMakeToolsHelpers OPTIONAL)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 14)

add_executable(planner src/main.cpp src/optimalTrajectoryPlanner.cpp src/polynomial.cpp include/optimalTrajectoryPlanner.hpp include/polynomial.hpp include/frenetPath.hpp)
target_link_libraries(planner ${Boost_LIBRARIES} ${OpenCV_LIBS})
include_directories(
    ${CMAKE_SOURCE_DIR}/include
    ${Boost_INCLUDE_DIRS}
)
