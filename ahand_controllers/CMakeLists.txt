cmake_minimum_required(VERSION 2.8.3)
project(ahand_controllers)

add_compile_options(-std=c++11)

find_package(orocos_kdl REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  controller_interface
  control_msgs
  forward_command_controller
  control_toolbox
  realtime_tools
  urdf
  roscpp
  kdl_parser
  message_generation
  cmake_modules
  bhand
)


include_directories(include/ahand_controllers ${orocos_kdl_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})


link_directories(${catkin_LIBRARY_DIRS})
link_directories(${orocos_kdl_LIBRARY_DIRS})


find_library(KDL_LIBRARY REQUIRED NAMES orocos-kdl HINTS ${orocos_kdl_LIBRARY_DIRS})
include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

add_message_files(
   FILES
   bhand.msg
)

add_service_files(
  FILES
  Command.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

generate_dynamic_reconfigure_options(
  cfg/gains_pd_param.cfg
)


# Declare catkin package
catkin_package(
  DEPENDS orocos_kdl
  CATKIN_DEPENDS
    controller_interface
    control_msgs
    control_toolbox
    realtime_tools
    urdf
    forward_command_controller
    kdl_parser
    message_runtime
    bhand
  INCLUDE_DIRS include
  LIBRARIES ahand_controllers
)



#########
# BUILD #
#########

set(INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)

set(SRC_FILES
        src/motion/CDDynamics.cpp
        src/controllers/bhand_controller.cpp
        src/controllers/joint_impedance_controller.cpp
        src/ros_controller.cpp
        src/state_publisher.cpp)

set(INC_FILES
        ${INCLUDE_DIR}/ahand_controllers/motion/CDDynamics.h
        ${INCLUDE_DIR}/ahand_controllers/controllers/ahand_types.h
        ${INCLUDE_DIR}/ahand_controllers/controllers/base_controller.h
        ${INCLUDE_DIR}/ahand_controllers/controllers/bhand_controller.h
        ${INCLUDE_DIR}/ahand_controllers/controllers/joint_impedance_controller.h)

add_library(ahand_controllers ${SRC_FILES} ${INC_FILES}
        ${INCLUDE_DIR}/utils/filters.h
        ${INCLUDE_DIR}/utils/utils.h)


target_link_libraries(ahand_controllers ${catkin_LIBRARIES} kdl_parser ${orocos_kdl_LIBRARIES})

add_dependencies(ahand_controllers ${PROJECT_NAME}_gencfg)
add_dependencies(ahand_controllers ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


install(FILES ahand_controllers_plugins.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})





