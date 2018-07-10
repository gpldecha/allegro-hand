cmake_minimum_required(VERSION 2.8.3)
project(ahand_hw)

find_package(catkin REQUIRED COMPONENTS
  control_toolbox
  controller_interface
  controller_manager
  hardware_interface
  realtime_tools
  joint_limits_interface
  roscpp
  tf
  urdf
  cmake_modules
  pluginlib
  kdl_parser
  transmission_interface
  gazebo_ros
  std_msgs
  ahand_driver
)

add_definitions (-std=c++11)

catkin_package(
 INCLUDE_DIRS include
 CATKIN_DEPENDS 
    control_toolbox
    controller_interface
    controller_manager
    hardware_interface
    realtime_tools
    joint_limits_interface
    roscpp
    tf
    urdf
    cmake_modules
    pluginlib
    kdl_parser
    transmission_interface
    gazebo_ros
    std_msgs
    ahand_driver
  LIBRARIES ahand_hw ahand_hw_can ahand_hw_gazebo_plugin
)

###########
## Build ##
###########

set(INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)
set(HDR_FILES  ${INCLUDE_DIR}/ahand_hw/ahand_hw.h)

include_directories(${catkin_INCLUDE_DIRS} ${INCLUDE_DIR})

## AHAND HW BASE CLASS

add_library(ahand_hw src/ahand_hw.cpp ${HDR_FILES})
target_link_libraries(ahand_hw ${catkin_LIBRARIES})


## REAL
add_executable(ahand_hw_can_node src/ahand_hw_can_node.cpp  include/ahand_hw/ahand_hw_can.hpp)
add_dependencies(ahand_hw_can_node ahand_hw ahand_driver)
target_link_libraries(ahand_hw_can_node ahand_hw ahand_driver  ${catkin_LIBRARIES} )

#add_executable(ahand_hw_can_node src/ahand_hw_can_node.cpp)
#add_dependencies(ahand_hw_can_node ahand_hw_can)
#target_link_libraries(ahand_hw_can_node ahand_hw_can)

## SIMULATION
find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
add_library(ahand_hw_gazebo_plugin  src/ahand_hw_gazebo_plugin.cpp include/ahand_hw/ahand_hw_gazebo.hpp)

add_dependencies(ahand_hw_gazebo_plugin ahand_hw)
target_link_libraries(ahand_hw_gazebo_plugin ahand_hw ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
