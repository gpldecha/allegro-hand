cmake_minimum_required(VERSION 2.8.3)
project(ahand_driver)


find_package(catkin REQUIRED COMPONENTS
	roscpp
 	std_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ahand_driver
  CATKIN_DEPENDS
  roscpp
  std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

set(INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)
set(HDR_FILES  ${INCLUDE_DIR}/ahand_driver/ahandDriver.h
               ${INCLUDE_DIR}/ahand_driver/canAPI.h
               ${INCLUDE_DIR}/ahand_driver/rDeviceAllegroHandCANDef.h
)

include_directories(${catkin_INCLUDE_DIRS} include)

add_library(ahand_driver src/ahandDriver.cpp src/canAPI.cpp ${HDR_FILES})
target_link_libraries(ahand_driver BHand pcanbasic pthread)
