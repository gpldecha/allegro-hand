cmake_minimum_required(VERSION 2.8.3)
project(ahand_hw)

add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  controller_manager
  roscpp
  control_msgs
  trajectory_msgs
  actionlib
  pluginlib
  kdl_parser
  transmission_interface
  urdf
  control_toolbox
  joint_limits_interface
  gazebo_ros
  ahand_driver
)

find_package(gazebo REQUIRED)

catkin_package(
 INCLUDE_DIRS include
 CATKIN_DEPENDS
  hardware_interface
  controller_manager
  roscpp
  control_msgs
  trajectory_msgs
  pluginlib
  kdl_parser
  transmission_interface
  urdf
  control_toolbox
  joint_limits_interface
  gazebo_ros
  ahand_driver
 LIBRARIES
    ahand_hw
    ahand_hw_can
    ahand_hw_gazebo_plugin
  DEPENDS
    gazebo
)


###########
## Build ##
###########


message(STATUS "gazebo include: ${GAZEBO_INCLUDE_DIRS}")
message(STATUS "gazebo lib:     ${GAZEBO_LIBRARY_DIRS}")

set(INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)
set(HDR_FILES  ${INCLUDE_DIR}/ahand_hw/ahand_hw.h
               ${INCLUDE_DIR}/ahand_hw/ahand_filters.h
                )

include_directories(
  include/
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${INCLUDE_DIR}
)

link_directories(${GAZEBO_LIBRARY_DIRS})


## AHAND HW BASE CLASS

add_library(ahand_hw src/ahand_hw.cpp src/ahand_filters.cpp ${HDR_FILES})
target_link_libraries(ahand_hw ${catkin_LIBRARIES})


## REAL
add_executable(ahand_hw_can_node src/ahand_hw_can_node.cpp  include/ahand_hw/ahand_hw_can.hpp)
add_dependencies(ahand_hw_can_node ahand_hw ahand_driver)
target_link_libraries(ahand_hw_can_node ahand_hw ahand_driver  ${catkin_LIBRARIES} )


## Sim Interface
add_library(ahand_hw_gazebo_plugin src/ahand_hw_gazebo_plugin.cpp include/ahand_hw/ahand_hw_gazebo.hpp)
add_dependencies(ahand_hw_gazebo_plugin ahand_hw)

target_link_libraries(ahand_hw_gazebo_plugin ahand_hw ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
