cmake_minimum_required(VERSION 3.2.0)
project(using_pcd)

set(CMAKE_CXX_FLAGS "-std=c++14 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED
        heightmap_core
)
find_package(camel-euclid-vector REQUIRED)

catkin_package(
        CATKIN_DEPENDS
        heightmap_core
)

include_directories(
        ${catkin_INCLUDE_DIRS}
)
add_executable(using_pcd usingPCD.cpp)
target_link_libraries(using_pcd ${catkin_LIBRARIES} camel-euclid-vector)