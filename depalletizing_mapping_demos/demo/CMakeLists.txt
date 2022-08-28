cmake_minimum_required(VERSION 3.2.0)
project(depalletizing_demo)

set(CMAKE_CXX_FLAGS "-std=c++14 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
        roscpp
        sensor_msgs
        tf
        depalletizing_mapping_base
        depalletizing_mapping_core
        depalletizing_mapping_ros
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
        tf
        depalletizing_mapping_base
        depalletizing_mapping_core
        depalletizing_mapping_ros
        DEPENDS
)

include_directories(
        ${catkin_INCLUDE_DIRS}
)

add_executable(depalletizing_demo depalletizing_demo.cpp)

target_link_libraries(depalletizing_demo ${catkin_LIBRARIES})
