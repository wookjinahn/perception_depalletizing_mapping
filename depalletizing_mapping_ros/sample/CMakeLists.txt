cmake_minimum_required(VERSION 3.2.0)
project(using_msgs)

set(CMAKE_CXX_FLAGS "-std=c++14 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
        roscpp
        sensor_msgs
        heightmap_core
        heightmap_ros
        heightmap_msgs
)

find_package(camel-euclid-vector REQUIRED)

catkin_package(
        INCLUDE_DIRS
#            include
        LIBRARIES
            {PROJECT_NAME}
        CATKIN_DEPENDS
            roscpp
            sensor_msgs
            heightmap_core
            heightmap_ros
            heightmap_msgs
        DEPENDS
)

include_directories(
        ${catkin_INCLUDE_DIRS}
)

add_executable(using_msgs using_msgs.cpp)

target_link_libraries(using_msgs ${catkin_LIBRARIES} camel-euclid-vector)
