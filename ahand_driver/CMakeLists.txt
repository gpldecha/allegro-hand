cmake_minimum_required(VERSION 2.8.3)
project(ahand_driver)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    can_api
    roscpp
    std_msgs
)

find_package(Eigen3 REQUIRED)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ahand_driver
  CATKIN_DEPENDS
  can_api
  roscpp
  std_msgs
  #  DEPENDS system_lib
)

###########
## Build ##
###########

set(INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)

include_directories(${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR} include)

add_library(ahand_driver src/ahandDriver.cpp ${INCLUDE_DIR}/ahand_driver/ahandDriver.h ${INCLUDE_DIR}/ahand_driver/rDeviceAllegroHandCANDef.h)
target_link_libraries(ahand_driver ${catkin_LIBRARIES})
add_dependencies(ahand_driver ${catkin_EXPORTED_TARGETS})