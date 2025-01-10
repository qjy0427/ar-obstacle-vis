cmake_minimum_required(VERSION 3.25)
project(ar-obstacle-vis)

set(CMAKE_CXX_STANDARD 17)

find_package(PCL 1.2 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})

find_package(catkin REQUIRED COMPONENTS roscpp tf)
include_directories(${catkin_INCLUDE_DIRS})

add_subdirectory(octomap)
add_subdirectory(octovis)

add_executable(octomap_example main.cpp)
target_link_libraries(octomap_example octomap ${PCL_LIBRARIES} ${catkin_LIBRARIES})
