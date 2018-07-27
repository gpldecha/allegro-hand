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
)


include_directories(include/ahand_controllers ${orocos_kdl_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})


link_directories(${catkin_LIBRARY_DIRS})
link_directories(${orocos_kdl_LIBRARY_DIRS})


find_library(KDL_LIBRARY REQUIRED NAMES orocos-kdl HINTS ${orocos_kdl_LIBRARY_DIRS})
include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

add_message_files(
   FILES
   PoseRPY.msg
   RPY.msg
   MultiPriorityTask.msg
   PID.msg
   PIDgains.msg
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
  INCLUDE_DIRS include
  LIBRARIES ahand_controllers
)



#########
# BUILD #
#########

set(INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)

add_library(ahand_controllers
        src/ros_controller.cpp include/ahand_controllers/ros_controller.h
        src/empty_controller.cpp include/ahand_controllers/empty_controller.h
        src/pd_controller.cpp include/ahand_controllers/pd_controller.h
        src/state_publisher.cpp include/ahand_controllers/state_publisher.h
        ${INCLUDE_DIR}/utils/filters.h
        ${INCLUDE_DIR}/utils/utils.h
)


target_link_libraries(ahand_controllers ${catkin_LIBRARIES} kdl_parser ${orocos_kdl_LIBRARIES})

add_dependencies(ahand_controllers ${PROJECT_NAME}_gencfg)

add_executable(test_load tests/test_load.cpp)

add_dependencies(ahand_controllers ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})



# Install
install(DIRECTORY include/ahand_controllers/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS ahand_controllers
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(FILES ahand_controllers_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})





