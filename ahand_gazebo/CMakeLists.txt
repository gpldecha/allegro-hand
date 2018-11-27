cmake_minimum_required(VERSION 2.8.3)
project(ahand_gazebo)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED ahand_hw gazebo_ros)
find_package(gazebo REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES ahand_hw_gazebo_plugin
  CATKIN_DEPENDS ahand_hw gazebo_ros
  DEPENDS
    gazebo
)

###########
## Build ##
###########

set(INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)


## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${INCLUDE_DIR}
)

link_directories(${GAZEBO_LIBRARY_DIRS})

## Sim Interface
add_library(ahand_hw_gazebo_plugin src/ahand_hw_gazebo_plugin.cpp include/ahand_gazebo/ahand_hw_gazebo.hpp)
target_link_libraries(ahand_hw_gazebo_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
