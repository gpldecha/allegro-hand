cmake_minimum_required(VERSION 2.8.3)
project(allegro_hand_controllers)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
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


include_directories(include/allegro_hand_controllers ${orocos_kdl_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})


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

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
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
  LIBRARIES ${PROJECT_NAME}
  )

add_library(${PROJECT_NAME}
        # src/task_inverse_kinematics.cpp include/allegro_hand_controllers/task_inverse_kinematics.h
        src/ahand_joint_state_controller.cpp include/allegro_hand_controllers/ahand_joint_state_controller.h

)


target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} kdl_parser ${orocos_kdl_LIBRARIES})



# Install
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(FILES allegro_hand_controllers_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})


