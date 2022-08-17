cmake_minimum_required(VERSION 3.2.0)
project(depalletizing_mapping_ros)

set(CMAKE_CXX_FLAGS "-std=c++14 ${CMAKE_CXX_FLAGS}")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

## Find catkin macros and libraries
find_package(catkin REQUIRED
        roscpp
        sensor_msgs
        tf
        depalletizing_mapping_base
        depalletizing_mapping_core
        depalletizing_mapping_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
        INCLUDE_DIRS
            include
        LIBRARIES
            ${PROJECT_NAME}
        CATKIN_DEPENDS
            roscpp
            sensor_msgs
            tf
            depalletizing_mapping_base
            depalletizing_mapping_core
            depalletizing_mapping_msgs
        DEPENDS
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
        include
        SYSTEM
            ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
        src/MapDataNodeROS.cpp
        src/MapTreeNodeROS.cpp
)

target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
)

#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(
        TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(
        DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.hpp"
)